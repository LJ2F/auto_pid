from __future__ import annotations
from dataclasses import asdict, replace
import ctypes as ct
import http.client
import json
from pathlib import Path
import socket
import struct
from contextlib import contextmanager
import uuid
import threading
import unittest
from unittest.mock import patch

import numpy as np
from pid_agent.workbench.analysis import Objective, frequency_response, score
from pid_agent.workbench.device import Device, DeviceError, NativeLink, SerialLink, TCPLink, TrialAborted
from pid_agent.workbench.protocol import COMMAND, HEADER, Parser, crc16, encode
from pid_agent.workbench.runner import Runner
from pid_agent.workbench.schema import Configuration, PID, Plan
from pid_agent.workbench.search import Bound, GaussianProcess, Search, Space
from pid_agent.workbench.server import create_server
from pid_agent.workbench.service import Workbench

TEST_TEMP = Path(__file__).resolve().parents[1]/'.test-tmp'
TEST_TEMP.mkdir(exist_ok=True)
@contextmanager
def test_directory():
    # Keep artifacts in the workspace; Python 3.14's mode-0700 temporary ACLs
    # cannot be reopened by the restricted Windows test process.
    directory = TEST_TEMP/('workbench-'+uuid.uuid4().hex)
    directory.mkdir()
    yield str(directory)


def request(**changes):
    return dict(configuration=Configuration().to_dict(),plan=asdict(Plan()),action="tune",
                parameters=[dict(name="inner.kp",lower=0,upper=5),dict(name="inner.ki",lower=0,upper=5)],
                max_trials=10,max_seconds=180,**changes)


class ProtocolTests(unittest.TestCase):
    def test_crc_vector_and_c_encoder(self):
        link=NativeLink()
        lib=link.lib
        lib.pt_encode.argtypes=[ct.c_uint16,ct.c_void_p,ct.c_size_t,ct.c_void_p,ct.c_size_t]
        lib.pt_encode.restype=ct.c_size_t
        self.assertEqual(crc16(b"123456789"),0x4b37)
        for size in (0,1,7,24,108,168,500,1014):
            body=bytes(i%256 for i in range(size)); out=ct.create_string_buffer(1024)
            n=lib.pt_encode(COMMAND,body,len(body),out,len(out))
            self.assertEqual(out.raw[:n],encode(body))

    def test_fragment_corruption_resynchronization(self):
        a=encode(b"abc");bad=bytearray(a);bad[8]^=1
        p=Parser();frames=[]
        stream=b"junk\xaa\x55\xff\xff"+bytes(bad)+a+a
        for i in range(0,len(stream),3):frames.extend(p.feed(stream[i:i+3],1.0))
        self.assertEqual([f.body for f in frames],[b"abc",b"abc"])
        self.assertLessEqual(len(p.buffer),1024)

    def test_partial_expiry(self):
        p=Parser();p.feed(encode(b"x")[:5],1)
        self.assertEqual(p.feed(encode(b"y"),2)[0].body,b"y")

    def test_replayed_configuration_ack_is_idempotent(self):
        link=NativeLink();d=Device(link)
        packet=encode(HEADER.pack(1,3,0,d.session,d.request+1,0,d.revision+1,d.boot_id)+Configuration().pack())
        link.write(packet);first=link.queue.popleft();link.write(packet);second=link.queue.popleft()
        self.assertEqual(first,second)
        d.request+=1;d.status();self.assertEqual(d.revision,1)

    def test_stale_session_and_boot_rejected(self):
        link=NativeLink();d=Device(link);old=d.session
        d.status(hello=True);self.assertNotEqual(old,d.session)
        for session,boot in ((old,d.boot_id),(d.session,d.boot_id^1)):
            packet=encode(HEADER.pack(1,2,0,session,d.request+1,0,d.revision,boot))
            link.write(packet);h=HEADER.unpack_from(Parser().feed(link.queue.popleft())[0].body)
            self.assertEqual(h[2],3)


class DeviceTests(unittest.TestCase):
    def setUp(self):self.d=Device(NativeLink())
    def tearDown(self):self.d.close()

    def test_step_and_scoring(self):
        trace=self.d.run(Configuration(),Plan(),threading.Event())
        result=score(trace,Objective())
        self.assertEqual(len(trace.samples),500)
        self.assertTrue(np.isfinite(result['cost']))
        self.assertEqual(trace.configuration,Configuration())

    def test_cancel_before_start_sends_no_commands(self):
        event=threading.Event();event.set();before=self.d.request
        with self.assertRaises(TrialAborted):self.d.run(Configuration(),Plan(),event)
        self.assertEqual(self.d.request,before)

    def test_open_requires_confirmation(self):
        with self.assertRaises(ValueError):self.d.run(Configuration(),Plan(open_loop=1),threading.Event())

    def test_physical_limit_abort(self):
        with self.assertRaises(TrialAborted) as e:
            self.d.run(Configuration(),Plan(y_max=(.01,10)),threading.Event())
        self.assertEqual(e.exception.reason,'limit')

    def test_outer_range_checked_during_inner_test(self):
        cfg=Configuration((PID(),PID()))
        with self.assertRaises(TrialAborted) as e:
            self.d.run(cfg,Plan(y_max=(10,.001)),threading.Event())
        self.assertEqual(e.exception.reason,'limit')

    def test_watchdog_and_overflow(self):
        self.d.configure(Configuration());self.d.trial+=1
        self.d.exchange(4,Plan(timeout_ms=100).pack())
        self.d.link.time+=.2;self.d.link.advance(1)
        self.assertEqual(self.d.status()['reason'],'timeout')
        self.d.exchange(7);self.d.trial+=1
        self.d.exchange(4,Plan(timeout_ms=5000,decimation=1).pack())
        self.d.link.advance(70)
        self.assertEqual(self.d.status()['reason'],'overflow')

    def test_short_scoring_window_rejected(self):
        tr=self.d.run(Configuration(),Plan(duration=1,settle=.99,decimation=1),threading.Event())
        with self.assertRaises(ValueError):score(tr,Objective())

    def test_missing_samples_rejected(self):
        tr=self.d.run(Configuration(),Plan(),threading.Event());tr.samples.pop(40)
        with self.assertRaises(ValueError):score(tr,Objective())

    def test_negative_step(self):
        tr=self.d.run(Configuration(),Plan(amplitude=-1),threading.Event())
        m=score(tr,Objective());self.assertGreaterEqual(m['overshoot'],0);self.assertTrue(np.isfinite(m['cost']))

    def test_open_frequency_response_matches_plant(self):
        p=Plan(wave=1,open_loop=1,amplitude=.2,frequency_count=1,frequencies=(1,),duration=8.5)
        tr=self.d.run(Configuration(),p,threading.Event(),open_loop_confirmed=True)
        point=frequency_response(tr,1)
        expected=1/(1+2j*np.pi*.6)*np.exp(-2j*np.pi*.03)
        actual=complex(point['real'],point['imag'])
        self.assertLess(abs(actual-expected)/abs(expected),.08)

    def test_outer_open_keeps_inner_closed(self):
        cfg=Configuration((PID(kp=1,ki=2),PID(kind='P',kp=0)))
        p=Plan(wave=1,open_loop=1,active_loop=1,amplitude=.2,frequencies=(1,),duration=8.5)
        tr=self.d.run(cfg,p,threading.Event(),open_loop_confirmed=True)
        self.assertGreater(max(abs(r['actuator']) for r in tr.samples),.1)
        self.assertLess(max(abs(r['input']) for r in tr.samples),.201)
        self.assertTrue(frequency_response(tr,1)['snr']>3)


class SearchTests(unittest.TestCase):
    def setUp(self):self.space=Space(Configuration(),[Bound('inner.kp',0,5),Bound('inner.ki',0,5)])
    def test_lock_and_float32(self):
        cfg=self.space.decode([.371,.612]);original=asdict(Configuration().loops[0]);actual=asdict(cfg.loops[0])
        for key in original:
            if key not in ('kp','ki'):self.assertEqual(original[key],actual[key])
        self.assertEqual(Configuration.unpack(cfg.pack()),cfg)

    def test_inactive_or_protection_parameter_rejected(self):
        for name in ('inner.kd','inner.output_max'):
            with self.assertRaises(ValueError):Space(Configuration(),[Bound(name,0,10)])

    def test_duplicate_effect_rejected(self):
        cfg=Configuration((PID(kind='PID',flags=514),))
        with self.assertRaises(ValueError):Space(cfg,[Bound('inner.kd',0,2),Bound('inner.ffj',0,2)])

    def test_noisy_gp_covariance(self):
        gp=GaussianProcess().fit([[0],[0],[1],[1]],[1,1.1,2,2.2])
        mean,cov=gp.predict([[0],[.5],[1]],True)
        self.assertTrue(np.isfinite(mean).all());self.assertGreater(np.linalg.eigvalsh(cov).min(),-1e-8)

    def test_all_searchers_finite_bounded_and_improve(self):
        for method in ('pattern','bayesian','cmaes'):
            with self.subTest(method=method):
                s=Search(self.space,method,4)
                def cost(cfg):return float(np.sum((self.space.encode(cfg)-.5)**2))
                initial=cost(self.space.base);s.tell(self.space.base,initial,-.1)
                for _ in range(18):
                    cfg=s.ask();x=self.space.encode(cfg);self.assertTrue(np.all((x>=0)&(x<=1)))
                    s.tell(cfg,cost(cfg),-.1)
                self.assertLess(cost(s.incumbent()),initial)
                if method=='cmaes':self.assertGreater(s.generation,0)

    def test_invalid_does_not_enter_model(self):
        s=Search(self.space);s.tell(self.space.base,status='invalid')
        self.assertFalse(s.observations);self.assertFalse(s.rejected)

    def test_repeated_incumbent_uses_mean(self):
        s=Search(self.space);a=self.space.base;b=self.space.decode([.5,.5])
        s.tell(a,0,-.1);s.tell(a,10,-.1);s.tell(b,2,-.1)
        self.assertEqual(s.incumbent(),b)


class RunnerTests(unittest.TestCase):
    def test_open_and_closed_identification_to_validation(self):
        for mode in ('open_loop','closed_loop'):
            with self.subTest(mode=mode), test_directory() as tmp:
                d=Device(NativeLink())
                r=request(start_mode=mode,open_loop_confirmed=True,plant_class='stable')
                r['max_trials']=15;r['max_seconds']=300
                r['configuration']=Configuration((PID(kp=0 if mode=='open_loop' else 1,ki=0 if mode=='open_loop' else 2),)).to_dict()
                report=Runner(d,r,threading.Event(),lambda *_:None,Path(tmp)).run()
                self.assertTrue(report['validated'],report.get('error'))
                self.assertEqual(len(report['frequency_response']),5)
                if mode=='closed_loop':self.assertEqual(report['records'][0]['purpose'],'baseline')
                d.close()

    def test_model_tuning_validates_and_restores(self):
        with test_directory() as tmp:
            d=Device(NativeLink());original=d.configuration
            report=Runner(d,request(start_mode='model',model={}),threading.Event(),lambda *_:None,Path(tmp)).run()
            self.assertTrue(report['validated'],report.get('error'))
            self.assertTrue(report['restored_original']);self.assertEqual(d.configuration,original)
            self.assertEqual(len(report['validation']),3)
            self.assertTrue((Path(tmp)/'report.json').exists());d.close()

    def test_outer_parameters_in_inner_trial_rejected(self):
        d=Device(NativeLink());r=request();r['configuration']=Configuration((PID(),PID())).to_dict();r['parameters']=[dict(name='outer.kp',lower=0,upper=5)]
        with self.assertRaises(ValueError):Runner(d,r,threading.Event(),lambda *_:None,Path('.test-tmp'))
        d.close()

    def test_cancelled_job_never_starts_trial(self):
        with test_directory() as tmp:
            d=Device(NativeLink());e=threading.Event();e.set()
            report=Runner(d,request(),e,lambda *_:None,Path(tmp)).run()
            self.assertEqual(report['status'],'cancelled');self.assertEqual(report['trials_used'],0);d.close()

    def test_infrastructure_abort_not_parameter_failure(self):
        with test_directory() as tmp:
            d=Device(NativeLink())
            with patch.object(d,'run',side_effect=TrialAborted('overflow',[])):
                report=Runner(d,request(),threading.Event(),lambda *_:None,Path(tmp)).run()
            self.assertEqual(report['status'],'failed');self.assertEqual(report['records'][0]['status'],'invalid');d.close()


class TransportAndHTTPTests(unittest.TestCase):
    def test_tcp_client_handshake_and_configuration(self):
        listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1)
        errors=[]
        def peer():
            native=NativeLink()
            try:
                conn,_=listener.accept()
                with conn:
                    while data:=conn.recv(4096):
                        native.write(data)
                        while native.queue:
                            packet=native.queue.popleft()
                            for i in range(0,len(packet),7):conn.sendall(packet[i:i+7])
            except Exception as e:errors.append(e)
            finally:native.close()
        t=threading.Thread(target=peer);t.start()
        d=Device(TCPLink('127.0.0.1',listener.getsockname()[1]));d.configure(Configuration());d.close()
        t.join(3);listener.close();self.assertFalse(t.is_alive());self.assertFalse(errors)

    def test_serial_transport_uses_sdk_packets(self):
        class Port:
            def __init__(self,**_):self.native=NativeLink();self.buffer=b''
            @property
            def in_waiting(self):
                while self.native.queue:self.buffer+=self.native.queue.popleft()
                return len(self.buffer)
            def write(self,data):self.native.write(data);return len(data)
            def read(self,n):data,self.buffer=self.buffer[:n],self.buffer[n:];return data
            def close(self):self.native.close()
        with patch('serial.Serial',Port):
            d=Device(SerialLink('MOCK',115200));d.configure(Configuration());self.assertEqual(d.configuration,Configuration());d.close()

    def test_serial_bandwidth_rejected_before_start(self):
        class Port:
            def __init__(self,**_):pass
            def close(self):pass
        d=Device(NativeLink())
        with patch('serial.Serial',Port):
            serial=SerialLink('MOCK',1200);original=d.link;d.link=serial
            with self.assertRaises(ValueError):d.run(Configuration(),Plan(),threading.Event())
            d.link=original;d.close()

    def test_http_session_auth_and_workflow(self):
        with test_directory() as tmp:
            service=Workbench(Path(tmp));server=create_server(service,port=0)
            t=threading.Thread(target=server.serve_forever);t.start()
            conn=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=5)
            def call(method,path,body=None,token=None,origin=None):
                headers={'Content-Type':'application/json'}
                if token:headers['X-Workbench-Token']=token
                if origin:headers['Origin']=origin
                conn.request(method,path,json.dumps(body) if body is not None else None,headers)
                r=conn.getresponse();return r.status,r.read()
            try:
                status,body=call('GET','/api/session');self.assertEqual(status,200);token=json.loads(body)['token']
                self.assertEqual(call('POST','/api/connect',{'transport':'simulation'})[0],403)
                self.assertEqual(call('GET','/api/session',origin='http://evil.invalid')[0],403)
                self.assertEqual(call('GET','/app.js')[0],200)
                self.assertEqual(call('POST','/api/connect',{'transport':'simulation'},token)[0],200)
                r=request(start_mode='model',model={})
                self.assertEqual(call('POST','/api/start',r,token)[0],200)
                service.worker.join(15);self.assertFalse(service.worker.is_alive())
                self.assertTrue(service.snapshot()['report']['validated'])
                for kind in ('json','csv','c'):self.assertEqual(call('GET','/api/export?format='+kind)[0],200)
            finally:
                conn.close();server.shutdown();server.server_close();t.join();service.close()


class EnhancementTests(unittest.TestCase):
    def test_c_sdk_noise_resynchronization(self):
        link = NativeLink()
        d = Device(link)
        # Verify device is working
        d.status()
        # Feed garbage noise directly into link, followed by a valid exchange
        noisy_stream = b"random_noise_without_header\xaa\x12\x34\x56" + b"\x00" * 50
        link.write(noisy_stream)
        # Now normal exchange should succeed seamlessly despite the preceding noise
        info = d.status()
        self.assertEqual(info["state"], 0)  # PT_IDLE
        d.close()

    def test_cooldown_and_alternating_steps(self):
        with test_directory() as tmp:
            d = Device(NativeLink())
            req = request(cooldown_seconds=0.01, alternating_steps=True)
            runner = Runner(d, req, threading.Event(), lambda *_: None, Path(tmp))
            report = runner.run()
            self.assertIn(report["status"], ("validated", "validation_failed", "complete"))
            # Check recorded trials: search/validation amplitudes should alternate signs
            search_plans = [r["plan"] for r in report["records"] if r["purpose"] in ("search", "validation")]
            if len(search_plans) >= 2:
                # Should have both positive and negative amplitudes
                amps = [p["amplitude"] for p in search_plans]
                self.assertTrue(any(a > 0 for a in amps))
                self.assertTrue(any(a < 0 for a in amps))
            d.close()


if __name__=='__main__':unittest.main()
