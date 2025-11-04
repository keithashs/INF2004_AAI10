#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"   // for distances/speeds used by PID task

// =================== Config tweaks ===================
#define RIGHT_INVERTED 0

#ifndef PWM_TOP
#define PWM_TOP ((PWM_MAX_LEFT > PWM_MAX_RIGHT) ? PWM_MAX_LEFT : PWM_MAX_RIGHT)
#endif

static bool     use_pid_control = false;
static PIDState pid_state       = PID_DISABLED;
static float    target_speed    = MIN_SPEED;
static float    target_turn_angle = CONTINUOUS_TURN;

// ---------- helpers ----------
static inline void set_pwm(uint gpio, uint16_t level){ pwm_set_gpio_level(gpio, level); }
static inline uint16_t clampL(float v){ if(v<PWM_MIN_LEFT)v=PWM_MIN_LEFT; if(v>PWM_MAX_LEFT)v=PWM_MAX_LEFT; return (uint16_t)v; }
static inline uint16_t clampR(float v){ if(v<PWM_MIN_RIGHT)v=PWM_MIN_RIGHT; if(v>PWM_MAX_RIGHT)v=PWM_MAX_RIGHT; return (uint16_t)v; }

// Left raw
static inline void L_fwd(uint16_t p){ set_pwm(L_MOTOR_IN1,p); set_pwm(L_MOTOR_IN2,0); }
static inline void L_rev(uint16_t p){ set_pwm(L_MOTOR_IN1,0); set_pwm(L_MOTOR_IN2,p); }

// Right raw (optionally inverted)
#if RIGHT_INVERTED
static inline void R_fwd(uint16_t p){ set_pwm(R_MOTOR_IN3,0); set_pwm(R_MOTOR_IN4,p); }
static inline void R_rev(uint16_t p){ set_pwm(R_MOTOR_IN3,p); set_pwm(R_MOTOR_IN4,0); }
#else
static inline void R_fwd(uint16_t p){ set_pwm(R_MOTOR_IN3,p); set_pwm(R_MOTOR_IN4,0); }
static inline void R_rev(uint16_t p){ set_pwm(R_MOTOR_IN3,0); set_pwm(R_MOTOR_IN4,p); }
#endif

static void coast(void){ set_pwm(L_MOTOR_IN1,0); set_pwm(L_MOTOR_IN2,0); set_pwm(R_MOTOR_IN3,0); set_pwm(R_MOTOR_IN4,0); }

// ---------- API (manual/PID-neutral) ----------
void forward_motor(float pl, float pr){ L_fwd(clampL(pl)); R_fwd(clampR(pr)); }
void reverse_motor(float pl, float pr){ L_rev(clampL(pl)); R_rev(clampR(pr)); }

void turn_motor(int direction, float pl, float pr){
    uint16_t L = clampL(pl), R = clampR(pr);
    if (direction == 0/*LEFT*/)  { L_rev(L); R_fwd(R); }
    else                         { L_fwd(L); R_rev(R); }
}

void stop_motor(void){ coast(); }

// ---------- Friendly wrappers ----------
void disable_pid_control(void){ use_pid_control = false; }
void forward_motor_manual(float pl,float pr){ disable_pid_control(); forward_motor(pl,pr); }
void reverse_motor_manual(float pl,float pr){ disable_pid_control(); reverse_motor(pl,pr); }

bool turn_until_angle(float angle){
    if (angle == CONTINUOUS_TURN) return true;
    if (angle < 0.f || angle > FULL_CIRCLE) return false;
    const float target = (angle / FULL_CIRCLE) * (3.14159265358979323846f * WHEEL_TO_WHEEL_DISTANCE);
    if (target - get_average_distance() <= 0.05f) { stop_motor(); return false; }
    return true;
}

void turn_motor_manual(int direction, float angle, float pl,float pr){
    disable_pid_control();
    turn_motor(direction, pl, pr);
    if (angle != CONTINUOUS_TURN) {
        reset_encoders();
        while (turn_until_angle(angle)) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }
}
void stop_motor_manual(void){ disable_pid_control(); stop_motor(); }

void offset_move_motor(int direction, int turn, float offset){
    if (offset < 0.f) offset = 0.f; 
    if (offset > 1.f) offset = 1.f;
    int pl = PWM_MID_LEFT, pr = PWM_MID_RIGHT;
    int lspan = (PWM_MAX_LEFT  - PWM_MIN_LEFT )/2;
    int rspan = (PWM_MAX_RIGHT - PWM_MIN_RIGHT)/2;

    if (turn == 0/*LEFT*/) { pl -= lspan*offset; pr += rspan*offset; }
    else                   { pl += lspan*offset; pr -= rspan*offset; }

    if (direction == 1/*FORWARDS*/) forward_motor_manual(pl,pr);
    else                            reverse_motor_manual(pl,pr);
}

// ---------- PID interface (optional) ----------
void enable_pid_control(void){ use_pid_control = true; }
void forward_motor_pid(float s){ enable_pid_control(); target_speed = s; pid_state = PID_FWD; }
void reverse_motor_pid(float s){ enable_pid_control(); target_speed = s; pid_state = PID_REV; }
void turn_motor_pid(int dir,float s,float ang){ enable_pid_control(); target_speed=s; target_turn_angle=ang; pid_state = (dir==0)?PID_LEFT:PID_RIGHT; }
void stop_motor_pid(void){ disable_pid_control(); stop_motor(); target_speed=0.f; pid_state=PID_STOP; enable_pid_control(); }

// tiny PID util (kept same signature you used elsewhere)
float compute_pid_pwm(float target, float current, float *I, float *prev){
    const float Kp=0.f, Ki=0.f, Kd=0.f; // keep zero unless you tune
    float e = target - current; *I += e; float d = e - *prev; *prev = e;
    return Kp*e + Ki*(*I) + Kd*d;
}

// Example PID task (yields if disabled)
void pid_task(void *params){
    float iL=0.f,iR=0.f, eL=0.f,eR=0.f;
    float pwmL=PWM_MIN_LEFT, pwmR=PWM_MIN_RIGHT;
    bool jumpstarted=false;

    for(;;){
            if(!use_pid_control){ vTaskDelay(pdMS_TO_TICKS(1)); continue; }

        if (target_speed < MIN_SPEED) pid_state = PID_STOP;
        if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;

        float vL = get_left_speed();
        float vR = get_right_speed();

        // simple straightness trim from speed mismatch
        float trim = 0.10f * (vR - vL);
        pwmL += trim; pwmR -= trim;

        pwmL += compute_pid_pwm(target_speed, vL, &iL, &eL);
        pwmR += compute_pid_pwm(target_speed, vR, &iR, &eR);

        if (vL < JUMPSTART_SPEED_THRESHOLD || vR < JUMPSTART_SPEED_THRESHOLD){
            pwmL = PWM_JUMPSTART; pwmR = PWM_JUMPSTART; jumpstarted = true;
        } else {
            if (pwmL < PWM_MIN_LEFT || jumpstarted)  pwmL = PWM_MIN_LEFT;
            else if (pwmL > PWM_MAX_LEFT)            pwmL = PWM_MAX_LEFT;

            if (pwmR < PWM_MIN_RIGHT || jumpstarted) pwmR = PWM_MIN_RIGHT;
            else if (pwmR > PWM_MAX_RIGHT)           pwmR = PWM_MAX_RIGHT;

            jumpstarted = false;
        }

        switch (pid_state){
            case PID_FWD:     forward_motor(pwmL,pwmR); break;
            case PID_REV:     reverse_motor(pwmL,pwmR); break;
            case PID_LEFT:    turn_motor(0,pwmL,pwmR); reset_encoders(); pid_state = PID_TURNING; break;
            case PID_RIGHT:   turn_motor(1,pwmL,pwmR); reset_encoders(); pid_state = PID_TURNING; break;
            case PID_TURNING: turn_until_angle(target_turn_angle); break;
            case PID_STOP:    stop_motor(); break;
            case PID_DISABLED:
            default:          disable_pid_control(); break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------- hardware init ----------
void motor_pwm_init(void){
    gpio_set_function(L_MOTOR_IN1, GPIO_FUNC_PWM);
    gpio_set_function(L_MOTOR_IN2, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_IN3, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_IN4, GPIO_FUNC_PWM);

    uint slices[4] = {
        pwm_gpio_to_slice_num(L_MOTOR_IN1),
        pwm_gpio_to_slice_num(L_MOTOR_IN2),
        pwm_gpio_to_slice_num(R_MOTOR_IN3),
        pwm_gpio_to_slice_num(R_MOTOR_IN4)
    };
    for (int i=0;i<4;i++){ pwm_set_wrap(slices[i], PWM_TOP); pwm_set_clkdiv(slices[i], 125); pwm_set_enabled(slices[i], true); }

    coast();
}

void motor_init(void){
#ifdef MOTOR_STBY
    gpio_init(MOTOR_STBY); gpio_set_dir(MOTOR_STBY, GPIO_OUT); gpio_put(MOTOR_STBY, 1);
#endif
    motor_pwm_init();
    xTaskCreate(pid_task, "PID Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void motor_conditioning(void){
    printf("[MOTOR] conditioning...\n");
    stop_motor(); forward_motor(PWM_JUMPSTART, PWM_JUMPSTART); sleep_ms(15000);
    reverse_motor(PWM_JUMPSTART, PWM_JUMPSTART); sleep_ms(15000);
    stop_motor(); printf("[MOTOR] conditioning complete\n");
}