#include "pt_sdk.h"
#include <math.h>
#include <string.h>

static float limit(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

void pt_pid_defaults(pt_pid_config *c) {
    memset(c, 0, sizeof(*c));
    c->kind=PT_PI; c->antiwindup=PT_AW_CONDITIONAL;
    c->integral_limit=10.0f; c->integral_a=1.0f;
    c->output_min=-1.0f; c->output_max=1.0f;
    c->output_tau=0.01f; c->derivative_tau=0.01f;
    c->aw_gain=1.0f; c->blocked_error=1.0f; c->blocked_time=1.0f;
}

int pt_pid_validate(const pt_pid_config *c) {
    const float v[]={c->kp,c->ki,c->kd,c->ffj,c->ffb,c->integral_limit,c->deadband,
        c->integral_a,c->integral_b,c->output_tau,c->derivative_tau,c->output_min,
        c->output_max,c->aw_gain,c->blocked_error,c->blocked_time};
    size_t i;
    for(i=0;i<sizeof(v)/sizeof(v[0]);++i) if(!isfinite(v[i])) return 0;
    return c->kind<=PT_PID && c->form<=PT_INCREMENTAL && c->antiwindup<=PT_AW_BACKCALC
        && !(c->flags & ~767u) && c->output_min<c->output_max
        && c->integral_limit>=0 && c->deadband>=0 && c->integral_a>0 && c->integral_b>=0
        && c->output_tau>=0 && c->derivative_tau>=0 && c->aw_gain>=0
        && c->blocked_error>0 && c->blocked_time>0;
}

void pt_pid_init(pt_pid *p, const pt_pid_config *c, float bias) {
    memset(p,0,sizeof(*p)); p->config=*c; p->bias=bias;
    p->output=p->accumulator=bias;
}

float pt_pid_step(pt_pid *p, float r, float y, float dt) {
    const pt_pid_config *c=&p->config;
    float e, pv, di=0, dv=0, ff=0, inew, raw, clipped, filtered, dy;
    int has_i=c->kind==PT_PI || c->kind==PT_PID;
    int has_d=c->kind==PT_PD || c->kind==PT_PID;
    if(!isfinite(r)||!isfinite(y)||!isfinite(dt)||dt<=0||!pt_pid_validate(c)) {
        p->fault=PT_NONFINITE; return p->output;
    }
    e=r-y; dy=p->initialized?(y-p->previous_measurement)/dt:0;
    pv=c->kp*((c->flags&PT_P_MEASUREMENT)?-y:e);
    if(has_i) {
        di=c->ki*dt*((c->flags&PT_TRAPEZOID)&&p->initialized ? (e+p->previous_error)*0.5f:e);
        if((c->flags&PT_VARIABLE_INTEGRAL) && e*p->integral>0) {
            di*=limit((c->integral_a+c->integral_b-fabsf(e))/c->integral_a,0,1);
        }
    }
    if(has_d && p->initialized) dv=c->kd*((c->flags&PT_D_MEASUREMENT)?-dy:(e-p->previous_error)/dt);
    if(c->flags&PT_D_FILTER) dv=p->derivative+dt/(c->derivative_tau+dt)*(dv-p->derivative);
    if(c->flags&PT_FEEDFORWARD) ff=c->ffj*dy+c->ffb*y;
    inew=has_i?p->integral+di:0;
    if((c->flags&PT_INTEGRAL_LIMIT)||c->antiwindup==PT_AW_CLAMP)
        inew=limit(inew,-c->integral_limit,c->integral_limit);
    raw=c->form==PT_POSITIONAL ? p->bias+pv+inew+dv+ff :
        p->accumulator+(pv-p->previous_p)+(inew-p->integral)+(dv-p->derivative)+(ff-p->previous_ff);
    clipped=limit(raw,c->output_min,c->output_max);
    if(has_i && c->antiwindup==PT_AW_CONDITIONAL && (raw-clipped)*(inew-p->integral)>0) {
        raw-=inew-p->integral; inew=p->integral;
    } else if(has_i && c->antiwindup==PT_AW_BACKCALC) {
        float correction=c->aw_gain*(clipped-raw)*dt;
        inew+=correction; raw+=correction;
    }
    if((c->flags&PT_INTEGRAL_LIMIT)||c->antiwindup==PT_AW_CLAMP) {
        float bounded=limit(inew,-c->integral_limit,c->integral_limit);
        raw+=bounded-inew; inew=bounded;
    }
    filtered=(c->flags&PT_OUTPUT_FILTER)?p->output+dt/(c->output_tau+dt)*(raw-p->output):raw;
    if(!isfinite(raw)||!isfinite(filtered)||!isfinite(inew)||!isfinite(dv)) {
        p->fault=PT_NONFINITE; return p->output;
    }
    p->output=limit(filtered,c->output_min,c->output_max);
    /* Store the algebraic sum, not clipped output: clipping the P/D history
     * creates a permanent offset when a saturated P controller returns to zero.
     * Integral windup is handled explicitly above, identically in both forms. */
    p->accumulator=raw;
    p->integral=inew; p->derivative=dv;
    if(c->deadband>0 && fabsf(e)<=c->deadband) {
        p->integral=0; p->output=limit(0,c->output_min,c->output_max);
        p->accumulator=p->bias+pv+dv+ff;
    }
    if((c->flags&PT_BLOCK_DETECT) && fabsf(e)>c->blocked_error &&
       (p->output>=0.95f*c->output_max || p->output<=0.95f*c->output_min)) {
        p->blocked_elapsed+=dt;
        if(p->blocked_elapsed>=c->blocked_time) p->fault=PT_BLOCKED;
    } else p->blocked_elapsed=0;
    p->previous_error=e; p->previous_measurement=y; p->previous_p=pv; p->previous_ff=ff;
    p->initialized=1; p->p_term=pv; p->i_term=p->integral; p->d_term=dv;
    return p->output;
}
