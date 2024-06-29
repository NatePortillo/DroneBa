#include <stdbool.h>

/* Defines */
#define KPROPORTIONAL   1.0 //11
#define KINTEGRAL       0.01 //0.09
#define KDERIVITIVE     .1 //10
#define SET_POINT       0.0

/* Variables and Data Structures */
typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float setpoint;
} PIDController;

/* Function Prototypes */
void PID_Init(PIDController *pid, float kp, float ki, float kd, float setpoint);
float PID_Compute(PIDController *pid, float measured_value);


