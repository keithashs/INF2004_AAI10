#include "ir_linefollow.h"
#include "motor.h"

static inline int is_black(int raw)
{
    return IR_BLACK_IS_LOW ? (raw == 0) : (raw == 1);
}

void ir_init(void)
{
    gpio_init(IR_LEFT_PIN);
    gpio_set_dir(IR_LEFT_PIN, GPIO_IN);
    //gpio_pull_up(IR_LEFT_PIN);

    gpio_init(IR_RIGHT_PIN);
    gpio_set_dir(IR_RIGHT_PIN, GPIO_IN);
    //gpio_pull_up(IR_RIGHT_PIN);
}

int ir_read_left_raw(void)
{
    return gpio_get(IR_LEFT_PIN);
}

int ir_read_right_raw(void)
{
    return gpio_get(IR_RIGHT_PIN);
}

int ir_left_is_black(void)
{
    return is_black(ir_read_left_raw());
}

int ir_right_is_black(void)
{
    return is_black(ir_read_right_raw());
}

// ======== LINE FOLLOWING SENSOR FUNCTIONS ========
void init_line_adc(void)
{
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2);
}

uint16_t read_line_adc(void)
{
    return adc_read();
}

// ======== PID CONTROLLER FUNCTIONS ========
void pid_init(PIDController *pid, float kp, float ki, float kd)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_compute(PIDController *pid, float error, float dt)
{
    // Apply deadband to ignore small errors
    if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
        error = 0.0f;
    }
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    
    const float max_integral = MAX_STEER_CORRECTION / (pid->ki + 0.0001f);
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Compute PID output
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // Clamp output to maximum correction
    if (output > MAX_STEER_CORRECTION) output = MAX_STEER_CORRECTION;
    if (output < -MAX_STEER_CORRECTION) output = -MAX_STEER_CORRECTION;
    
    return output;
}

// ======== HELPER FUNCTIONS ========
int clamp_pwm_left(int pwm)
{
    if (pwm < PWM_MIN_LEFT) return PWM_MIN_LEFT;
    if (pwm > PWM_MAX_LEFT) return PWM_MAX_LEFT;
    return pwm;
}

int clamp_pwm_right(int pwm)
{
    if (pwm < PWM_MIN_RIGHT) return PWM_MIN_RIGHT;
    if (pwm > PWM_MAX_RIGHT) return PWM_MAX_RIGHT;
    return pwm;
}