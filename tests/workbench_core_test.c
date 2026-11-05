#include "pt_sdk.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float a,float b) { assert(fabsf(a-b)<0.0001f); }
int main(void) {
    pt_pid_config c;pt_pid p,q;unsigned i;
    const float sequence[]={10,0,0,0.1f,1,-1,0};
    pt_pid_defaults(&c);c.kind=PT_P;c.kp=2;c.output_min=-5;c.output_max=5;
    pt_pid_init(&p,&c,0);c.form=PT_INCREMENTAL;pt_pid_init(&q,&c,0);
    for(i=0;i<sizeof(sequence)/sizeof(sequence[0]);++i)
        near(pt_pid_step(&p,sequence[i],0,.01f),pt_pid_step(&q,sequence[i],0,.01f));
    near(p.output,0);
    c.deadband=.2f;pt_pid_init(&q,&c,0);c.form=PT_POSITIONAL;pt_pid_init(&p,&c,0);
    for(i=0;i<sizeof(sequence)/sizeof(sequence[0]);++i)
        near(pt_pid_step(&p,sequence[i],0,.01f),pt_pid_step(&q,sequence[i],0,.01f));
    c.deadband=0;c.flags=PT_P_MEASUREMENT;pt_pid_init(&p,&c,0);
    near(pt_pid_step(&p,10,1,.1f),-2);near(pt_pid_step(&p,20,1,.1f),-2);
    c.kind=PT_PI;c.kp=0;c.ki=1;c.flags=PT_TRAPEZOID;pt_pid_init(&p,&c,0);
    near(pt_pid_step(&p,1,0,.1f),.1f);near(pt_pid_step(&p,3,0,.1f),.3f);
    c.kind=PT_PD;c.kd=1;c.flags=PT_D_MEASUREMENT;pt_pid_init(&p,&c,0);
    near(pt_pid_step(&p,0,0,.1f),0);near(pt_pid_step(&p,2,0,.1f),0);
    near(pt_pid_step(&p,2,.1f,.1f),-1);
    c.kind=PT_P;c.flags=PT_FEEDFORWARD;c.ffj=2;c.ffb=3;pt_pid_init(&p,&c,0);
    near(pt_pid_step(&p,0,.1f,.1f),.3f);near(pt_pid_step(&p,0,.2f,.1f),2.6f);
    c.flags=0;c.kind=PT_PI;c.ki=100;c.kp=2;c.antiwindup=PT_AW_CONDITIONAL;pt_pid_init(&p,&c,0);
    for(i=0;i<100;i++)pt_pid_step(&p,10,0,.01f);
    near(p.integral,0);near(pt_pid_step(&p,0,0,.01f),0);
    c.kind=PT_P;c.kp=1;c.flags=PT_OUTPUT_FILTER;c.output_tau=.1f;pt_pid_init(&p,&c,0);
    near(pt_pid_step(&p,1,0,.1f),.5f);near(pt_pid_step(&p,1,0,.1f),.75f);
    pt_pid_step(&p,NAN,0,.01f);assert(p.fault==PT_NONFINITE);
    {
        unsigned kind,aw,step;
        for(kind=PT_P;kind<=PT_PID;++kind)for(aw=PT_AW_NONE;aw<=PT_AW_BACKCALC;++aw) {
            pt_pid_defaults(&c);c.kind=kind;c.antiwindup=aw;c.kp=1.2f;c.ki=.7f;c.kd=.03f;
            c.flags=PT_INTEGRAL_LIMIT|PT_TRAPEZOID|PT_D_FILTER|PT_OUTPUT_FILTER|PT_FEEDFORWARD;
            c.ffj=.01f;c.ffb=.1f;c.output_min=-2;c.output_max=2;
            pt_pid_init(&p,&c,0);c.form=PT_INCREMENTAL;pt_pid_init(&q,&c,0);
            for(step=0;step<200;++step) {
                float r=step<50?4.0f:(step<150?-1.0f:0.0f),y=sinf(step*.03f);
                near(pt_pid_step(&p,r,y,.01f),pt_pid_step(&q,r,y,.01f));
            }
            assert(p.fault==0 && q.fault==0);
        }
    }
    assert(pt_crc16((const uint8_t*)"123456789",9)==0x4B37u);
    printf("PID properties and CRC passed; runtime bytes=%lu\n",(unsigned long)pt_runtime_size());
    return 0;
}
