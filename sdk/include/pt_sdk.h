#ifndef PT_SDK_H
#define PT_SDK_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#ifndef PT_FRAME_MAX
#define PT_FRAME_MAX 1024u
#endif
#ifndef PT_SAMPLE_CAPACITY
#define PT_SAMPLE_CAPACITY 64u
#endif
#define PT_COMMAND 0x7D50u
#define PT_VERSION 1u
#define PT_CONFIG_BYTES 168u
#define PT_PLAN_BYTES 108u
#define PT_SAMPLE_BYTES 60u
#if PT_FRAME_MAX < 238u || PT_FRAME_MAX > 65535u
#error "PT_FRAME_MAX must be between 238 and 65535 bytes"
#endif
#if PT_SAMPLE_CAPACITY < 1u
#error "PT_SAMPLE_CAPACITY must be positive"
#endif

enum { PT_P, PT_PI, PT_PD, PT_PID };
enum { PT_POSITIONAL, PT_INCREMENTAL };
enum { PT_AW_NONE, PT_AW_CLAMP, PT_AW_CONDITIONAL, PT_AW_BACKCALC };
enum {
    PT_INTEGRAL_LIMIT=1, PT_D_MEASUREMENT=2, PT_TRAPEZOID=4,
    PT_P_MEASUREMENT=8, PT_OUTPUT_FILTER=16, PT_VARIABLE_INTEGRAL=32,
    PT_D_FILTER=64, PT_BLOCK_DETECT=128, PT_FEEDFORWARD=512
};
enum { PT_IDLE, PT_RUNNING, PT_COMPLETE, PT_ABORTED };
enum {
    PT_OK, PT_BAD_REQUEST, PT_BUSY, PT_SESSION, PT_REVISION, PT_LIMIT,
    PT_TIMEOUT, PT_CANCELLED, PT_NONFINITE, PT_OVERFLOW, PT_BLOCKED,
    PT_OPEN_DENIED, PT_SEQUENCE, PT_SLEW
};
enum { PT_HELLO=1, PT_STATUS, PT_CONFIGURE, PT_START, PT_READ, PT_STOP, PT_CLEAR };

typedef struct {
    uint32_t kind, form, flags, antiwindup;
    float kp, ki, kd, ffj, ffb, integral_limit, deadband, integral_a, integral_b;
    float output_tau, derivative_tau, output_min, output_max, aw_gain;
    float blocked_error, blocked_time;
} pt_pid_config;

typedef struct {
    pt_pid_config config;
    float integral, derivative, output, accumulator, previous_p, previous_ff;
    float previous_error, previous_measurement, blocked_elapsed, bias;
    uint32_t initialized, fault;
    float p_term, i_term, d_term;
} pt_pid;

typedef struct {
    uint32_t loop_count, outer_divider;
    pt_pid_config loop[2]; /* inner first, optional outer second */
} pt_config;

typedef struct {
    uint32_t wave, open_loop, active_loop, decimation, frequency_count, timeout_ms;
    float dt, duration, settle, reference, amplitude, bias;
    float y_min[2], y_max[2]; /* wire order min0,min1,max0,max1 */
    float output_min, output_max, max_slew;
    float frequencies[8]; /* Hz, summed with equal amplitudes */
} pt_plan;

typedef struct {
    uint32_t index;
    float time, reference, measurement, input, actuator, perturbation;
    float inner_reference, inner_measurement, outer_measurement, controller_output;
    float p, i, d;
    uint32_t flags;
} pt_sample;

/** TX callback must COPY bytes before returning. Never retain for asynchronous DMA. */
typedef void (*pt_send_fn)(void *user, const uint8_t *data, size_t size);
/** Required integration hook: stop/brake/transfer control as appropriate to the plant. */
typedef void (*pt_exit_fn)(void *user, uint32_t reason);

typedef struct {
    uint8_t bytes[PT_FRAME_MAX];
    size_t used;
    uint32_t last_byte_ms;
} pt_parser;

typedef struct {
    pt_config config;
    pt_pid pid[2];
    pt_plan plan;
    pt_parser parser;
    pt_sample samples[PT_SAMPLE_CAPACITY];
    uint32_t read_at, sample_count, tick, state, reason, session, request, trial, revision;
    uint32_t boot_id, last_contact_ms, allow_open_loop, configured, session_counter;
    uint16_t command;
    float elapsed, actuator, outer_output, control_period, hard_output_min, hard_output_max;
    uint8_t reply[PT_FRAME_MAX], previous_request[PT_FRAME_MAX], tx[PT_FRAME_MAX];
    size_t reply_size, previous_request_size;
    pt_send_fn send;
    pt_exit_fn exit;
    void *user;
} pt_runtime;

void pt_pid_defaults(pt_pid_config *config);
int pt_pid_validate(const pt_pid_config *config);
void pt_pid_init(pt_pid *pid, const pt_pid_config *config, float bias);
float pt_pid_step(pt_pid *pid, float reference, float measurement, float dt);
uint16_t pt_crc16(const uint8_t *data, size_t size);
uint16_t pt_frame_crc(const uint8_t *frame, size_t size);
size_t pt_encode(uint16_t command, const uint8_t *body, size_t size, uint8_t *out, size_t capacity);

/** All calls belong to ONE control/task context. ISR/network threads enqueue RX only.
 * boot_id must change after reset. allow_open_loop defaults to false; the application
 * may enable it only where the plant/fixture permits open-loop tests. */
void pt_runtime_init(pt_runtime *rt, uint32_t boot_id, pt_send_fn send, pt_exit_fn exit, void *user);
void pt_runtime_receive(pt_runtime *rt, const uint8_t *data, size_t size, uint32_t now_ms);
/** Call on every control tick, including idle. Apply returned output ONLY while
 * state == PT_RUNNING. On all exits, the exit callback owns the actuator action. */
float pt_runtime_tick(pt_runtime *rt, float inner_measurement, float outer_measurement, uint32_t now_ms);
void pt_runtime_abort(pt_runtime *rt, uint32_t reason);
size_t pt_runtime_size(void);
uint32_t pt_runtime_state(const pt_runtime *rt);
void pt_runtime_allow_open_loop(pt_runtime *rt, int allowed);
void pt_runtime_set_hardware(pt_runtime *rt, float period, float output_min, float output_max);

#ifdef __cplusplus
}
#endif
#endif
