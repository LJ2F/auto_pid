#include "pt_sdk.h"
#include <math.h>
#include <string.h>

#define PID_FLOATS(X) X(kp) X(ki) X(kd) X(ffj) X(ffb) X(integral_limit) X(deadband) \
    X(integral_a) X(integral_b) X(output_tau) X(derivative_tau) X(output_min) \
    X(output_max) X(aw_gain) X(blocked_error) X(blocked_time)
static uint32_t get32(const uint8_t *p) {
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static void put32(uint8_t *p,uint32_t v) {
    p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;
}
static float getf(const uint8_t *p) { uint32_t u=get32(p);float f;memcpy(&f,&u,4);return f; }
static void putf(uint8_t *p,float f) { uint32_t u;memcpy(&u,&f,4);put32(p,u); }
static float clamp(float v,float lo,float hi) { return fminf(hi,fmaxf(lo,v)); }

static void config_read(pt_config *c,const uint8_t *b) {
    unsigned n; c->loop_count=get32(b);c->outer_divider=get32(b+4);b+=8;
    for(n=0;n<2;++n) {
        pt_pid_config *p=&c->loop[n];
        p->kind=get32(b);p->form=get32(b+4);p->flags=get32(b+8);p->antiwindup=get32(b+12);b+=16;
#define READ_FIELD(name) p->name=getf(b);b+=4;
        PID_FLOATS(READ_FIELD)
#undef READ_FIELD
    }
}
static void config_write(const pt_config *c,uint8_t *b) {
    unsigned n;put32(b,c->loop_count);put32(b+4,c->outer_divider);b+=8;
    for(n=0;n<2;++n) {
        const pt_pid_config *p=&c->loop[n];
        put32(b,p->kind);put32(b+4,p->form);put32(b+8,p->flags);put32(b+12,p->antiwindup);b+=16;
#define WRITE_FIELD(name) putf(b,p->name);b+=4;
        PID_FLOATS(WRITE_FIELD)
#undef WRITE_FIELD
    }
}
static void plan_read(pt_plan *p,const uint8_t *b) {
    unsigned i;
    p->wave=get32(b);p->open_loop=get32(b+4);p->active_loop=get32(b+8);
    p->decimation=get32(b+12);p->frequency_count=get32(b+16);p->timeout_ms=get32(b+20);b+=24;
    p->dt=getf(b);p->duration=getf(b+4);p->settle=getf(b+8);p->reference=getf(b+12);
    p->amplitude=getf(b+16);p->bias=getf(b+20);b+=24;
    p->y_min[0]=getf(b);p->y_min[1]=getf(b+4);p->y_max[0]=getf(b+8);p->y_max[1]=getf(b+12);
    p->output_min=getf(b+16);p->output_max=getf(b+20);p->max_slew=getf(b+24);b+=28;
    for(i=0;i<8;++i)p->frequencies[i]=getf(b+4*i);
}
static int plan_valid(const pt_runtime *rt,const pt_plan *p) {
    const float v[]={p->dt,p->duration,p->settle,p->reference,p->amplitude,p->bias,
        p->y_min[0],p->y_min[1],p->y_max[0],p->y_max[1],p->output_min,p->output_max,p->max_slew};
    unsigned i;
    for(i=0;i<sizeof(v)/sizeof(v[0]);++i)if(!isfinite(v[i]))return 0;
    for(i=0;i<8;++i)if(!isfinite(p->frequencies[i]))return 0;
    if(p->wave>2 || p->open_loop>1 || p->active_loop>=rt->config.loop_count ||
       p->decimation<1 || p->decimation>10000 || p->frequency_count>8 ||
       p->timeout_ms<100 || p->timeout_ms>5000 || p->dt<0.000001f || p->dt>1 ||
       fabsf(p->dt-rt->control_period)>rt->control_period*0.001f ||
       p->duration<10*p->dt || p->duration>3600 || p->settle<0 || p->settle>=p->duration ||
       p->y_min[0]>=p->y_max[0] || p->y_min[1]>=p->y_max[1] ||
       p->output_min>=p->output_max || p->output_min<rt->hard_output_min ||
       p->output_max>rt->hard_output_max || p->max_slew<0) return 0;
    if(p->open_loop && !p->active_loop && (p->bias-fabsf(p->amplitude)<p->output_min ||
                        p->bias+fabsf(p->amplitude)>p->output_max)) return 0;
    if(p->open_loop && p->active_loop &&
       (p->bias-fabsf(p->amplitude)<rt->config.loop[1].output_min ||
        p->bias+fabsf(p->amplitude)>rt->config.loop[1].output_max)) return 0;
    if(p->wave) {
        if(!p->frequency_count || (p->wave==1 && p->frequency_count!=1))return 0;
        for(i=0;i<p->frequency_count;++i)
            if(p->frequencies[i]<=0 || p->frequencies[i]*p->dt*p->decimation>0.1f)return 0;
    }
    return 1;
}

void pt_runtime_init(pt_runtime *rt,uint32_t boot_id,pt_send_fn send,pt_exit_fn exit,void *user) {
    memset(rt,0,sizeof(*rt));rt->boot_id=boot_id;rt->send=send;rt->exit=exit;rt->user=user;
    rt->command=PT_COMMAND;rt->config.loop_count=1;rt->config.outer_divider=1;
    rt->control_period=0.001f;rt->hard_output_min=-1;rt->hard_output_max=1;
    pt_pid_defaults(&rt->config.loop[0]);pt_pid_defaults(&rt->config.loop[1]);
}
size_t pt_runtime_size(void) { return sizeof(pt_runtime); }
uint32_t pt_runtime_state(const pt_runtime *rt) { return rt->state; }
void pt_runtime_allow_open_loop(pt_runtime *rt,int allowed) { if(rt->state!=PT_RUNNING)rt->allow_open_loop=allowed?1u:0u; }
void pt_runtime_set_hardware(pt_runtime *rt,float period,float minimum,float maximum) {
    if(rt->state!=PT_RUNNING && isfinite(period)&&period>0&&isfinite(minimum)&&isfinite(maximum)&&minimum<maximum) {
        rt->control_period=period;rt->hard_output_min=minimum;rt->hard_output_max=maximum;
    }
}
void pt_runtime_abort(pt_runtime *rt,uint32_t reason) {
    if(rt->state!=PT_RUNNING)return;
    rt->reason=reason;rt->state=reason==PT_OK?PT_COMPLETE:PT_ABORTED;
    if(rt->exit)rt->exit(rt->user,reason);
}

static void sample_write(const pt_sample *s,uint8_t *b) {
    put32(b,s->index);b+=4;
#define SAMPLE_FIELD(name) putf(b,s->name);b+=4;
    SAMPLE_FIELD(time) SAMPLE_FIELD(reference) SAMPLE_FIELD(measurement) SAMPLE_FIELD(input)
    SAMPLE_FIELD(actuator) SAMPLE_FIELD(perturbation) SAMPLE_FIELD(inner_reference)
    SAMPLE_FIELD(inner_measurement) SAMPLE_FIELD(outer_measurement) SAMPLE_FIELD(controller_output)
    SAMPLE_FIELD(p) SAMPLE_FIELD(i) SAMPLE_FIELD(d)
#undef SAMPLE_FIELD
    put32(b,s->flags);
}

static void dispatch(pt_runtime *rt,const uint8_t *frame,size_t size,uint32_t now) {
    const uint8_t *b=frame+8,*data=b+24;
    uint8_t op,*out=rt->tx+8;uint32_t session,req,trial,revision,status=PT_OK;
    size_t payload=size-10,extra=0;int cache=0;
    if(payload<24 || b[0]!=PT_VERSION || b[2] || b[3])return;
    op=b[1];session=get32(b+4);req=get32(b+8);trial=get32(b+12);revision=get32(b+16);
    if(!session || !req || op<PT_HELLO || op>PT_CLEAR)return;
    if(op==PT_HELLO && rt->reply_size && rt->reply[9]==(PT_HELLO|0x80) &&
       size==rt->previous_request_size && !memcmp(frame,rt->previous_request,size)) {
        if(rt->send)rt->send(rt->user,rt->reply,rt->reply_size);
        return;
    }
    if(op!=PT_HELLO && get32(b+20)!=rt->boot_id)status=PT_SESSION;
    else if(session==rt->session && req==rt->request && rt->reply_size) {
        if(size==rt->previous_request_size && !memcmp(frame,rt->previous_request,size)) {
            rt->last_contact_ms=now;if(rt->send)rt->send(rt->user,rt->reply,rt->reply_size);return;
        }
        status=PT_SEQUENCE;
    } else if(op==PT_HELLO) {
        if(rt->state==PT_RUNNING)status=PT_BUSY;
        else if(payload!=24)status=PT_BAD_REQUEST;
        else if(rt->session_counter==UINT32_MAX)status=PT_SEQUENCE;
        else {
            ++rt->session_counter;
            rt->session=rt->session_counter;rt->request=0;rt->reply_size=0;
        }
    } else if(session!=rt->session)status=PT_SESSION;
    else if(req<=rt->request)status=PT_SEQUENCE;
    if(status==PT_OK) {
        cache=1;rt->last_contact_ms=now;
        if(op==PT_CONFIGURE) {
            pt_config config;
            if(rt->state==PT_RUNNING)status=PT_BUSY;
            else if(payload!=24+PT_CONFIG_BYTES)status=PT_BAD_REQUEST;
            else if(revision!=rt->revision+1u)status=PT_REVISION;
            else {
                config_read(&config,data);
                if(config.loop_count<1||config.loop_count>2||config.outer_divider<1||config.outer_divider>1000||
                   !pt_pid_validate(&config.loop[0])||!pt_pid_validate(&config.loop[1])||
                   config.loop[0].output_min<rt->hard_output_min||config.loop[0].output_max>rt->hard_output_max)
                    status=PT_BAD_REQUEST;
                else {rt->config=config;rt->revision=revision;rt->configured=1;}
            }
        } else if(op==PT_START) {
            pt_plan plan;
            if(rt->state==PT_RUNNING || rt->sample_count)status=PT_BUSY;
            else if(payload!=24+PT_PLAN_BYTES || !trial || !rt->configured || !rt->exit)status=PT_BAD_REQUEST;
            else if(revision!=rt->revision)status=PT_REVISION;
            else {
                plan_read(&plan,data);
                if(!plan_valid(rt,&plan))status=PT_BAD_REQUEST;
                else if(fmaxf(plan.output_min,rt->config.loop[0].output_min)>=
                        fminf(plan.output_max,rt->config.loop[0].output_max))status=PT_BAD_REQUEST;
                else if(plan.open_loop && !rt->allow_open_loop)status=PT_OPEN_DENIED;
                else {
                    rt->plan=plan;rt->trial=trial;rt->tick=rt->read_at=rt->sample_count=0;
                    rt->elapsed=rt->actuator=rt->outer_output=0;rt->reason=PT_OK;rt->state=PT_RUNNING;
                    pt_pid_init(&rt->pid[0],&rt->config.loop[0],0);
                    pt_pid_init(&rt->pid[1],&rt->config.loop[1],0);
                    rt->pid[0].config.output_min=fmaxf(plan.output_min,rt->config.loop[0].output_min);
                    rt->pid[0].config.output_max=fminf(plan.output_max,rt->config.loop[0].output_max);
                }
            }
        } else if(op==PT_STOP) {
            if(payload!=24)status=PT_BAD_REQUEST;
            else pt_runtime_abort(rt,PT_CANCELLED);
        } else if(op==PT_CLEAR) {
            if(payload!=24)status=PT_BAD_REQUEST;
            else if(rt->state==PT_RUNNING)status=PT_BUSY;
            else {rt->sample_count=rt->read_at=0;rt->state=PT_IDLE;rt->reason=PT_OK;}
        } else if(payload!=24)status=PT_BAD_REQUEST;
    }
    out[0]=PT_VERSION;out[1]=(uint8_t)(op|0x80);out[2]=(uint8_t)(status>>8);out[3]=(uint8_t)status;
    put32(out+4,op==PT_HELLO && status==PT_OK?rt->session:session);
    put32(out+8,req);put32(out+12,rt->trial);put32(out+16,rt->revision);
    put32(out+20,rt->boot_id);
    if(status==PT_OK && (op==PT_HELLO || op==PT_STATUS)) {
        put32(out+24,rt->boot_id);put32(out+28,rt->state);put32(out+32,rt->reason);
        put32(out+36,rt->sample_count);put32(out+40,rt->tick);put32(out+44,rt->allow_open_loop);
        putf(out+48,rt->control_period);putf(out+52,rt->hard_output_min);putf(out+56,rt->hard_output_max);
        config_write(&rt->config,out+60);extra=36+PT_CONFIG_BYTES;
    } else if(status==PT_OK && op==PT_READ) {
        uint32_t n=rt->sample_count,i,max=(PT_FRAME_MAX-46u)/PT_SAMPLE_BYTES;
        if(n>max)n=max;
        put32(out+24,rt->state);put32(out+28,rt->reason);put32(out+32,n);
        for(i=0;i<n;++i) {
            sample_write(&rt->samples[rt->read_at],out+36+i*PT_SAMPLE_BYTES);
            rt->read_at=(rt->read_at+1)%PT_SAMPLE_CAPACITY;
        }
        rt->sample_count-=n;extra=12+n*PT_SAMPLE_BYTES;
    }
    size=pt_encode(rt->command,out,24+extra,rt->tx,sizeof(rt->tx));
    if(cache) {
        memcpy(rt->reply,rt->tx,size);rt->reply_size=size;rt->request=req;
        rt->previous_request_size=payload+10;memcpy(rt->previous_request,frame,payload+10);
    }
    if(rt->send)rt->send(rt->user,rt->tx,size);
}

void pt_runtime_receive(pt_runtime *rt,const uint8_t *data,size_t size,uint32_t now) {
    pt_parser *p=&rt->parser;size_t i;
    if(p->used && (uint32_t)(now-p->last_byte_ms)>250)p->used=0;
    for(i=0;i<size;++i) {
        if(p->used==sizeof(p->bytes)){memmove(p->bytes,p->bytes+1,--p->used);}
        p->bytes[p->used++]=data[i];p->last_byte_ms=now;
        while(p->used>=2) {
            size_t len;uint16_t cmd,crc;
            if(p->bytes[0]!=0xAA||p->bytes[1]!=0x55)goto discard;
            if(p->used<4)break;
            len=((size_t)p->bytes[2]<<8)|p->bytes[3];
            if(len<10||len>sizeof(p->bytes))goto discard;
            if(p->used<len)break;
            crc=(uint16_t)(((uint16_t)p->bytes[6]<<8)|p->bytes[7]);
            if(p->bytes[len-2]!=0x55||p->bytes[len-1]!=0xAA||pt_frame_crc(p->bytes,len)!=crc)goto discard;
            cmd=(uint16_t)(((uint16_t)p->bytes[4]<<8)|p->bytes[5]);
            if(cmd==rt->command)dispatch(rt,p->bytes,len,now);
            p->used-=len;memmove(p->bytes,p->bytes+len,p->used);continue;
discard:
            --p->used;memmove(p->bytes,p->bytes+1,p->used);
        }
    }
}

float pt_runtime_tick(pt_runtime *rt,float y0,float y1,uint32_t now) {
    pt_plan *p=&rt->plan;float signal=0,r,inner_r,base=0,input=0,output,measure;
    uint32_t flags=0;unsigned i;pt_sample *s;pt_pid *selected;
    if(rt->state!=PT_RUNNING)return rt->actuator;
    if((uint32_t)(now-rt->last_contact_ms)>p->timeout_ms){pt_runtime_abort(rt,PT_TIMEOUT);return rt->actuator;}
    if(!isfinite(y0)||!isfinite(y1)){pt_runtime_abort(rt,PT_NONFINITE);return rt->actuator;}
    if(y0<p->y_min[0]||y0>p->y_max[0]||
       (rt->config.loop_count==2 && (y1<p->y_min[1]||y1>p->y_max[1]))) {
        pt_runtime_abort(rt,PT_LIMIT);return rt->actuator;
    }
    rt->elapsed=rt->tick*p->dt;
    if(rt->elapsed>=p->duration){pt_runtime_abort(rt,PT_OK);return rt->actuator;}
    if(rt->elapsed>=p->settle) {
        if(!p->wave)signal=p->amplitude;
        else for(i=0;i<p->frequency_count;++i)
            signal+=p->amplitude/p->frequency_count*sinf(6.28318530718f*p->frequencies[i]*(rt->elapsed-p->settle));
    }
    r=p->reference+((!p->open_loop && !p->wave)?signal:0);
    inner_r=r;measure=p->active_loop?y1:y0;selected=&rt->pid[p->active_loop];
    if(p->open_loop && !p->active_loop){base=p->bias;output=base+signal;input=output;}
    else {
        if(p->active_loop) {
            if(!p->open_loop && rt->tick%rt->config.outer_divider==0)
                rt->outer_output=pt_pid_step(&rt->pid[1],r,y1,p->dt*rt->config.outer_divider);
            base=p->open_loop?p->bias:rt->outer_output;
            inner_r=clamp(base+((p->wave||p->open_loop)?signal:0),rt->config.loop[1].output_min,rt->config.loop[1].output_max);
            if(inner_r<=rt->config.loop[1].output_min || inner_r>=rt->config.loop[1].output_max)flags|=1;
            input=inner_r;
        }
        output=pt_pid_step(&rt->pid[0],inner_r,y0,p->dt);
        if(!p->active_loop){base=output;output+=(p->wave?signal:0);}
    }
    if(rt->pid[0].fault||rt->pid[1].fault){pt_runtime_abort(rt,rt->pid[0].fault?rt->pid[0].fault:rt->pid[1].fault);return rt->actuator;}
    if(!isfinite(output)){pt_runtime_abort(rt,PT_NONFINITE);return rt->actuator;}
    if(output<=p->output_min || output>=p->output_max ||
       ((!p->open_loop || p->active_loop) && (rt->pid[0].output<=rt->pid[0].config.output_min ||
                         rt->pid[0].output>=rt->pid[0].config.output_max)))flags|=1;
    output=clamp(output,p->output_min,p->output_max);
    if(p->max_slew>0 && fabsf(output-rt->actuator)>p->max_slew*p->dt) {
        pt_runtime_abort(rt,PT_SLEW);return rt->actuator;
    }
    rt->actuator=output;if(!p->active_loop)input=output;
    if(rt->tick%p->decimation==0) {
        if(rt->sample_count==PT_SAMPLE_CAPACITY){pt_runtime_abort(rt,PT_OVERFLOW);return output;}
        s=&rt->samples[(rt->read_at+rt->sample_count)%PT_SAMPLE_CAPACITY];
        memset(s,0,sizeof(*s));s->index=rt->tick;s->time=rt->elapsed;s->reference=r;s->measurement=measure;
        s->input=input;s->actuator=output;s->perturbation=(p->open_loop||p->wave)?signal:0;
        s->inner_reference=inner_r;s->inner_measurement=y0;s->outer_measurement=y1;s->controller_output=base;
        s->p=selected->p_term;s->i=selected->i_term;s->d=selected->d_term;s->flags=flags;++rt->sample_count;
    }
    ++rt->tick;return output;
}
