'use strict';
const $ = id => document.getElementById(id);
const clone = value => JSON.parse(JSON.stringify(value));
let token = '', metadata, configuration, lastState = {}, cursor = 0, trace = [], traceTrial = null, polling = false;
let bounds = [{}, {}], loopIndex = 0, sessionBase = '';
const featureNames = {integral_limit:'积分限幅', derivative_on_measurement:'微分先行（测量）', trapezoidal_integral:'梯形积分', proportional_on_measurement:'比例先行（测量）', output_filter:'输出滤波', variable_integral:'变速积分 / 积分分离', derivative_filter:'微分滤波', blocked_detection:'堵转检测', measurement_feedforward:'测量前馈 FFJ / FFB'};
const states = {idle:'空闲', running:'运行中', stopping:'正在停止', complete:'完成', infeasible:'约束未通过', invalid:'数据无效', aborted:'保护退出', cancelled:'已停止', failed:'失败', validated:'复验通过', validation_failed:'复验未通过', budget_exhausted:'预算耗尽'};
const stages = {baseline:'基准', identification:'辨识', search:'搜索', validation:'独立复验', manual:'手动试验'};
const number = id => { const v = Number($(id).value); if (!$(id).value.trim() || !Number.isFinite(v)) throw Error(`${id} 需要有限数字`); return v; };
const fieldNumber = e => {if(!e.value.trim()||!Number.isFinite(Number(e.value)))throw Error('参数和边界需要有限数字，不能留空');return Number(e.value);};
const fmt = (n, digits=4) => n == null ? '—' : Number(n).toPrecision(digits);
function message(text, error=false) { $('message').hidden = !text; $('message').textContent = text; $('message').className = error ? 'error' : 'note'; }
function safe(action) { return async () => { try { await action(); } catch(e) { message(e.message, true); } }; }
function base() { return $('bridge').value.trim().replace(/\/$/, ''); }
async function api(path, body) {
  const res = await fetch(base()+path, {method:body === undefined?'GET':'POST', headers:body === undefined?{}:{'Content-Type':'application/json','X-Workbench-Token':token}, body:body === undefined?undefined:JSON.stringify(body)});
  const data = await res.json(); if (!res.ok) throw Error(data.error || `HTTP ${res.status}`); return data;
}
async function session() { const data = await api('/api/session'); token=data.token; metadata=data.schema; sessionBase=base(); if(!configuration) { configuration=clone(metadata.default_configuration); configuration.loops.push(clone(configuration.loops[0])); renderController(); } }
function activeParameters(pid) {
  const names = new Set(['kp','deadband']);
  if(pid.kind.includes('I')) { names.add('ki'); if(pid.flags&1 || pid.antiwindup==='clamp') names.add('integral_limit'); if(pid.flags&32) { names.add('integral_a'); names.add('integral_b'); } if(pid.antiwindup==='backcalc') names.add('aw_gain'); }
  if(pid.kind.includes('D')) { names.add('kd'); if(pid.flags&64) names.add('derivative_tau'); }
  if(pid.flags&16) names.add('output_tau'); if(pid.flags&512) { names.add('ffj'); names.add('ffb'); } return names;
}
function input(value, type='number') { const e=document.createElement('input'); e.type=type; if(type==='number') e.step='any'; e.value=value; return e; }
function cell(row, value) { const e=document.createElement('td'); if(value instanceof Node) e.append(value); else e.textContent=value; row.append(e); return e; }
function capture() {
  if(!configuration) return;
  const pid=configuration.loops[loopIndex];
  pid.kind=$('kind').value; pid.form=$('form').value; pid.antiwindup=$('antiwindup').value;
  pid.flags=[...$('features').querySelectorAll('input:checked')].reduce((v,e)=>v|Number(e.value),0);
  for(const row of $('parameters').children) { const n=row.dataset.name; pid[n]=fieldNumber(row.querySelector('[data-value]')); bounds[loopIndex][n]={selected:row.querySelector('[data-select]').checked, lower:fieldNumber(row.querySelector('[data-lower]')), upper:fieldNumber(row.querySelector('[data-upper]')), scale:row.querySelector('select').value}; }
  for(const e of $('protected-parameters').querySelectorAll('input')) pid[e.dataset.name]=fieldNumber(e);
  configuration.outer_divider=number('outer-divider');
}
function renderController() {
  const p=configuration.loops[loopIndex];
  $('kind').replaceChildren(...metadata.kinds.map(k=>new Option(k,k))); $('kind').value=p.kind; $('form').value=p.form; $('antiwindup').value=p.antiwindup;
  $('outer-divider').value=configuration.outer_divider;
  $('features').replaceChildren();
  for(const [name,bit] of Object.entries(metadata.features)) { const l=document.createElement('label'); l.className='inline'; const e=input(bit,'checkbox'); e.checked=Boolean(p.flags&bit); e.onchange=()=>{capture();renderController();}; l.append(e,document.createTextNode(featureNames[name])); $('features').append(l); }
  $('parameters').replaceChildren(); const available=activeParameters(p);
  for(const [name,info] of Object.entries(metadata.parameters)) {
    const b=bounds[loopIndex][name] || {selected:['kp','ki'].includes(name),lower:p[name]<0?p[name]*3:0,upper:Math.max(Math.abs(p[name])*3, ['kp','ki','kd'].includes(name)?5:1),scale:'linear'};
    const row=document.createElement('tr'); row.dataset.name=name; const enabled=available.has(name);
    const check=input('','checkbox'); check.dataset.select=''; check.checked=b.selected&&enabled; check.disabled=!enabled; cell(row,check);
    const label=cell(row,`${name} · ${info.label}`); label.title=info.unit;
    for(const [key,v] of [['value',p[name]],['lower',b.lower],['upper',b.upper]]) { const e=input(v); e.dataset[key]=''; e.disabled=!enabled; cell(row,e); }
    const scale=document.createElement('select'); scale.append(new Option('线性','linear'),new Option('对数','log')); scale.value=b.scale; scale.disabled=!enabled; cell(row,scale); if(!enabled) row.style.opacity='.45'; $('parameters').append(row);
  }
  $('protected-parameters').replaceChildren();
  for(const [name,title] of Object.entries({output_min:'控制器输出下界',output_max:'控制器输出上界',blocked_error:'堵转误差阈值',blocked_time:'堵转持续时间 / s'})) { const l=document.createElement('label'); l.textContent=title; const e=input(p[name]); e.dataset.name=name; l.append(e); $('protected-parameters').append(l); }
}
function getConfiguration() { capture(); return {loops:clone(configuration.loops.slice(0,$('cascade').checked?2:1)),outer_divider:configuration.outer_divider}; }
function loadConfiguration(cfg) { configuration=clone(cfg); $('cascade').checked=cfg.loops.length===2; if(configuration.loops.length===1) configuration.loops.push(clone(configuration.loops[0])); loopIndex=0; $('edit-loop').value='0'; bounds=[{},{}]; cascadeChanged(); renderController(); }
function cascadeChanged() { const on=$('cascade').checked; for(const id of ['edit-loop','active-loop']) { $(id).options[1].disabled=!on; if(!on) $(id).value='0'; } if(!on) loopIndex=0; }
function plan() { return { ...clone(metadata.default_plan), dt:number('dt'), duration:number('duration'), settle:number('settle'), reference:number('reference'), amplitude:number('amplitude'), bias:number('bias'), active_loop:number('active-loop'), decimation:number('decimation'), timeout_ms:number('timeout'), y_min:[number('y-min-0'),number('y-min-1')], y_max:[number('y-max-0'),number('y-max-1')], output_min:number('out-min'),output_max:number('out-max'),max_slew:number('slew') }; }
function request(action) {
  const cfg=getConfiguration(), p=plan(), selected=[];
  cfg.loops.forEach((pid,i)=>{ for(const [name,b] of Object.entries(bounds[i])) if(b.selected&&activeParameters(pid).has(name)) selected.push({name:(i?'outer.':'inner.')+name,lower:b.lower,upper:b.upper,scale:b.scale}); });
  const objective={max_overshoot:number('max-overshoot'),max_steady_error:number('max-steady'),max_saturation:number('max-saturation')};
  for(const key of ['tracking','overshoot','steady','effort','saturation']) objective[key]=number('weight-'+key);
  const mode=$('start-mode').value;
  if(action==='identify'&&!['open_loop','closed_loop'].includes(mode)) throw Error('仅辨识请选择“闭环辨识”或“受限开环辨识”起步方式');
  return {action, configuration:cfg, plan:p, validation_plans:[{...p,amplitude:number('validation-amplitude')}], objective, parameters:selected, start_mode:mode, method:$('method').value,max_trials:number('max-trials'),max_seconds:number('max-seconds'),seed:1,frequencies_hz:$('frequencies').value.split(/[,，\s]+/).filter(Boolean).map(Number),identification_amplitude:number('ident-amplitude'),phase_margin:number('phase-margin'),model:{family:$('model-family').value,gain:number('model-gain'),tau:number('model-tau'),delay:number('model-delay'),damping:number('model-damping')},plant_class:$('plant-class').value,open_loop_confirmed:$('open-confirmed').checked,limits_confirmed:$('limits-confirmed').checked};
}
function updateControls(s) {
  const busy=Boolean(s.active), connected=s.connected;
  for(const id of ['connect','ports']) $(id).disabled=busy;
  for(const id of ['disconnect','load-device','apply','run-tuning','run-experiment','run-identification']) $(id).disabled=busy||!connected;
  $('stop').disabled=!busy;
  $('connection-status').textContent=connected?(s.transport==='simulation'?'仿真已连接':s.transport.toUpperCase()+' 已连接'):'未连接';
  $('task-status').textContent=states[s.status]||s.status;
  for(const id of ['export-json','export-csv']) $(id).disabled=busy||!s.report;
  $('export-c').disabled=busy||!s.report?.validated;
  $('apply-best').disabled=busy||!connected||!s.report?.validated;
  $('hardware-confirmation').hidden=!connected||s.transport==='simulation';
  if(s.device) { $('dt').value=s.device.dt; $('device-detail').textContent=`${s.transport==='simulation'?'仿真':'设备'} · 周期 ${fmt(s.device.dt)} s · Boot ${s.device.boot_id}`; }
}
function renderResults(s) {
  $('trials').textContent=s.records.length;
  const last=[...s.records].reverse().find(r=>r.metrics)?.metrics;
  if(last) { $('score').textContent=fmt(last.score); $('overshoot').textContent=(last.overshoot*100).toFixed(1)+'%'; $('steady').textContent=(last.steady_error*100).toFixed(1)+'%'; }
  $('history').replaceChildren();
  for(const r of [...s.records].reverse()) { const tr=document.createElement('tr'); const pid=r.configuration.loops[r.plan.active_loop]; for(const v of [r.number,stages[r.purpose]||r.purpose,(states[r.status]||r.status)+(r.reason?' · '+r.reason:''),[pid.kp,pid.ki,pid.kd].map(x=>fmt(x,3)).join(' / '),fmt(r.metrics?.cost),r.metrics?(r.metrics.overshoot*100).toFixed(1)+'%':'—',r.metrics?(r.metrics.steady_error*100).toFixed(1)+'%':'—']) cell(tr,v); $('history').append(tr); }
  $('frequency-table').replaceChildren(); for(const p of s.frequency_response) { const tr=document.createElement('tr'); for(const key of ['frequency','magnitude','phase_deg','snr','window_mismatch']) cell(tr,fmt(p[key])); $('frequency-table').append(tr); }
  if(s.current_trial) $('trial-label').textContent=`试验 ${s.current_trial.number} · ${stages[s.current_trial.purpose]||s.current_trial.purpose}`;
  if(s.report) { const r=s.report; $('report-message').textContent=`${r.simulated?'【仿真结果】':'【设备结果】'}${states[r.status]||r.status}。${r.error||''} ${r.validated?`验证代价 ${fmt(r.validation_cost_mean)} ± ${fmt(r.validation_cost_std)}；${r.validation_scope==='held_out_conditions'?'使用独立验证工况':'相同工况独立重复'}。通过约束不等于已证明优于基准。`:''} ${r.restored_original?'已恢复试验前配置。':'配置恢复未确认：'+(r.restore_error||'')} ${r.best?'候选：'+r.best.loops.map((p,i)=>`${i?'外环':'内环'} Kp=${fmt(p.kp)}, Ki=${fmt(p.ki)}, Kd=${fmt(p.kd)}`).join('；'):''}`; }
  else if(s.active) $('report-message').textContent='任务运行中。只有完整且有效的试验参与评分；当前候选尚未通过最终复验。';
  if(s.error) message(s.error,true);
}
function draw() {
  const canvas=$('chart'), rect=canvas.getBoundingClientRect(), ratio=window.devicePixelRatio||1;
  canvas.width=Math.max(1,Math.round(rect.width*ratio)); canvas.height=Math.round(330*ratio);
  const ctx=canvas.getContext('2d'); ctx.scale(ratio,ratio); const w=rect.width,h=330;
  ctx.fillStyle='#fff';ctx.fillRect(0,0,w,h);ctx.font='11px sans-serif';
  if(!trace.length) {ctx.fillStyle='#7a8696';ctx.fillText('完成连接并运行试验后显示响应曲线',40,145);return;}
  const end=Math.max(...trace.map(r=>r.time),.01);
  function plot(keys,colors,top,height) {
    let low=Infinity,high=-Infinity; for(const r of trace) for(const k of keys) {low=Math.min(low,r[k]);high=Math.max(high,r[k]);} const pad=Math.max((high-low)*.1,.05);low-=pad;high+=pad;
    const x=t=>48+t/end*Math.max(1,w-68), y=v=>top+height-(v-low)/(high-low)*height;
    ctx.strokeStyle='#e9edf3';ctx.fillStyle='#79879a'; for(let i=0;i<=4;i++){const v=low+(high-low)*i/4;ctx.beginPath();ctx.moveTo(48,y(v));ctx.lineTo(w-20,y(v));ctx.stroke();ctx.fillText(v.toFixed(2),2,y(v)+3);}
    keys.forEach((k,i)=>{ctx.strokeStyle=colors[i];ctx.lineWidth=1.6;ctx.beginPath();const stride=Math.max(1,Math.floor(trace.length/Math.max(1,w*2)));trace.forEach((r,j)=>{if(j%stride&&j!==trace.length-1)return;if(!j)ctx.moveTo(x(r.time),y(r[k]));else ctx.lineTo(x(r.time),y(r[k]));});ctx.stroke();});
  }
  plot(['reference','measurement'],['#5d85c4','#0c9685'],15,170);plot(['actuator'],['#d7903e'],220,80);
  ctx.fillStyle='#79879a';for(let i=0;i<=4;i++)ctx.fillText((end*i/4).toFixed(2)+' s',48+(w-68)*i/4-10,322);
}
async function poll() {
  if(polling||!token||sessionBase!==base())return;polling=true;
  try {const s=await api('/api/state?after='+cursor);if(lastState.active&&!s.active)message('');lastState=s;cursor=s.cursor;for(const r of s.samples){if(traceTrial!==r.trial){trace=[];traceTrial=r.trial;}trace.push(r);}if(trace.length>12000)trace=trace.slice(-12000);updateControls(s);renderResults(s);draw();}
  catch(e){message('本地服务连接中断：'+e.message,true);}finally{polling=false;}
}
async function ports(){const r=await api('/api/ports');$('serial-port').replaceChildren(...r.ports.map(p=>new Option(`${p.port} · ${p.description}`,p.port)));}
function modeChanged(){const mode=$('start-mode').value;$('model-settings').hidden=mode!=='model';$('open-settings').hidden=mode!=='open_loop';}
$('bridge').value=location.protocol==='http:'&&['localhost','127.0.0.1'].includes(location.hostname)?location.origin:'http://127.0.0.1:8766';
$('transport').onchange=()=>{const t=$('transport').value;for(const id of ['serial-field','baud-field','ports'])$(id).hidden=t!=='serial';for(const id of ['tcp-host-field','tcp-port-field'])$(id).hidden=t!=='tcp';$('simulation-note').hidden=t!=='simulation';};
$('connect').onclick=safe(async()=>{await session();const t=$('transport').value;const p={transport:t};if(t==='serial'){p.port=$('serial-port').value;p.baudrate=number('baud');}if(t==='tcp'){p.host=$('tcp-host').value.trim();p.port=number('tcp-port');}const info=await api('/api/connect',p);loadConfiguration(info.configuration);message('连接成功；尚未启动试验。');await poll();});
$('ports').onclick=safe(async()=>{await session();await ports();});
$('disconnect').onclick=safe(async()=>{await api('/api/disconnect',{});await poll();});
$('load-device').onclick=safe(async()=>{loadConfiguration(lastState.device.configuration);message('已载入最近一次设备确认的配置。');});
$('apply').onclick=safe(async()=>{await api('/api/apply',{configuration:getConfiguration()});message('配置已应用到 RAM，未启动试验。');await poll();});
$('stop').onclick=safe(async()=>{await api('/api/stop',{});message('已请求停止，等待设备退出与配置恢复。');await poll();});
for(const [id,action] of [['run-tuning','tune'],['run-experiment','experiment'],['run-identification','identify']])$(id).onclick=safe(async()=>{await api('/api/start',request(action));trace=[];traceTrial=null;message('任务已开始。');await poll();$('results').scrollIntoView({behavior:'smooth'});});
$('apply-best').onclick=safe(async()=>{await api('/api/apply',{configuration:lastState.report.best});loadConfiguration(lastState.report.best);message('已应用复验通过的参数；未写入 Flash。');await poll();});
for(const kind of ['json','csv','c'])$('export-'+kind).onclick=safe(async()=>{const res=await fetch(base()+'/api/export?format='+kind);if(!res.ok)throw Error((await res.json()).error);const url=URL.createObjectURL(await res.blob());const a=document.createElement('a');a.href=url;a.download='pid-'+(lastState.job_id||'result')+'.'+(kind==='c'?'h':kind);a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('edit-loop').onchange=safe(()=>{capture();loopIndex=number('edit-loop');renderController();});
$('cascade').onchange=safe(()=>{capture();cascadeChanged();renderController();});
for(const id of ['kind','form','antiwindup'])$(id).onchange=safe(()=>{capture();renderController();});
$('start-mode').onchange=modeChanged;
for(const [key,label,value] of [['tracking','跟踪误差权重',.55],['overshoot','超调权重',.2],['steady','稳态误差权重',.15],['effort','输出变化权重',.05],['saturation','饱和权重',.05]]){const l=document.createElement('label');l.textContent=label;const e=input(value);e.id='weight-'+key;e.min='0';l.append(e);$('weights').append(l);}
window.addEventListener('resize',draw);modeChanged();draw();
for(const nav of document.querySelectorAll('.nav'))nav.addEventListener('click',()=>{document.querySelector('.nav.active')?.classList.remove('active');nav.classList.add('active');});
safe(async()=>{await session();await poll();})().then(()=>setInterval(poll,500));
