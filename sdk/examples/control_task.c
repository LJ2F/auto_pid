/* Compile this adapter with your project-provided hooks. No HAL/RTOS dependency. */
#include "pt_sdk.h"
static pt_runtime tuner;

/* Implement these hooks with bounded, nonblocking queues and your actuator policy. */
extern void board_tx_copy(const uint8_t *bytes,size_t size);
extern void board_leave_test(uint32_t reason);
extern void board_apply_output(float output);
extern size_t board_rx_pop(uint8_t *bytes,size_t capacity);
extern void board_normal_control(void);

static void send_copy(void *user,const uint8_t *bytes,size_t size) {
    (void)user;board_tx_copy(bytes,size);
}
static void leave_test(void *user,uint32_t reason) {
    (void)user;board_leave_test(reason);
}
void tuning_init(uint32_t unique_boot_id,float actual_period,float output_min,float output_max) {
    pt_runtime_init(&tuner,unique_boot_id,send_copy,leave_test,0);
    pt_runtime_set_hardware(&tuner,actual_period,output_min,output_max);
    /* Optional only after integration-specific assessment:
       pt_runtime_allow_open_loop(&tuner,1); */
}
void tuning_control_tick(float inner_measurement,float outer_measurement,uint32_t now_ms) {
    uint8_t rx[128];size_t count;unsigned batch;
    /* Bound transport work per control tick. ISR and network tasks enqueue only. */
    for(batch=0;batch<4;++batch) {
        count=board_rx_pop(rx,sizeof(rx));
        if(!count)break;
        pt_runtime_receive(&tuner,rx,count,now_ms);
    }
    {
        uint32_t before=tuner.state;
        float output=pt_runtime_tick(&tuner,inner_measurement,outer_measurement,now_ms);
        if(tuner.state==PT_RUNNING)board_apply_output(output);
        /* Never overwrite an exit action on the tick which leaves the experiment. */
        else if(before!=PT_RUNNING)board_normal_control();
    }
}
