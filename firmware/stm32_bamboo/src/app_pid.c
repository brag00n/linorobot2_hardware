#include "app_pid.h"
#include "app_motion.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_usart.h"

#include "app.h"

#define PI      (3.1415926f)


motor_pid_t pid_motor[4];

// Etat "esclave" par moteur : 1 = encodeur HS, PID desactive, PWM recopie du
// partenaire de meme cote. Sur ce robot M2 (idx 1) a l'encodeur intermittent -> esclave de M3.
uint8_t pid_slaved[4] = {0};

// Cartographie physique mesuree au banc (2026-09-13, cablage final), roues en l'air :
//   M1 avant-ligneA, M2 avant-ligneB, M3 arriere-ligneA, M4 arriere-ligneB.
//   Lignes (cotes) : A = {M1(0), M3(2)}, B = {M2(1), M4(3)}.
//   Marche avant DIR = (-,+,+,-) : dans une meme ligne, l'avant et l'arriere sont
//   montes en MIROIR (M1 avant a -PWM, M3 avant a +PWM ; M2 avant a +PWM, M4 avant a
//   -PWM). Le cote gauche/droite absolu est cale au banc (IMU), pas depuis ce repere.
// Partenaire de recopie d'un moteur esclave = l'autre roue de la MEME ligne ; signe
// oppose (miroir av/ar -> DIR[i]*DIR[partner] = -1).
static const int8_t SLAVE_PARTNER[4] = { 2, 3, 0, 1 }; // 0<->2 (ligne A), 1<->3 (ligne B)
static const int8_t SLAVE_SIGN[4]    = { -1, -1, -1, -1 };

// YAW偏航角
PID pid_Yaw = {0, 0.4, 0, 0.1, 0, 0, 0};


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

// 初始化PID参数
void PID_Param_Init(void)
{
    /* 速度相关初始化参数 */
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        pid_motor[i].target_val = 0.0;
        pid_motor[i].pwm_output = 0.0;
        pid_motor[i].err = 0.0;
        pid_motor[i].err_last = 0.0;
        pid_motor[i].err_next = 0.0;
        pid_motor[i].integral = 0.0;

        pid_motor[i].Kp = PID_DEF_KP;
        pid_motor[i].Ki = PID_DEF_KI;
        pid_motor[i].Kd = PID_DEF_KD;
    }

    pid_Yaw.Proportion = PID_YAW_DEF_KP;
    pid_Yaw.Integral = PID_YAW_DEF_KI;
    pid_Yaw.Derivative = PID_YAW_DEF_KD;
    
    #if PID_ASSISTANT_EN
    float pid_temp[3] = {pid_motor[0].Kp, pid_motor[0].Ki, pid_motor[0].Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值
    #endif
}

// 设置PID参数
void PID_Set_Parm(motor_pid_t *pid, float p, float i, float d)
{
    pid->Kp = p; // 设置比例系数 P
    pid->Ki = i; // 设置积分系数 I
    pid->Kd = d; // 设置微分系数 D
}

// 设置PID的目标值
void PID_Set_Target(motor_pid_t *pid, float temp_val)
{
    pid->target_val = temp_val; // 设置当前的目标值
}

// 获取PID目标值
float PID_Get_Target(motor_pid_t *pid)
{
    return pid->target_val; // 设置当前的目标值
}

// 增量式PID计算公式
float PID_Incre_Calc(motor_pid_t *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;
    /*PID算法实现*/
    pid->pwm_output += pid->Kp * (pid->err - pid->err_next) 
                    + pid->Ki * pid->err 
                    + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
    /*传递误差*/
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    
    /*返回PWM输出值*/
    if (Motion_Get_Car_Type() == CAR_SUNRISE)
    {
        if (pid->pwm_output > (MOTOR_MAX_PULSE-MOTOR_SUNRISE_IGNORE_PULSE))
            pid->pwm_output = (MOTOR_MAX_PULSE-MOTOR_SUNRISE_IGNORE_PULSE);
        if (pid->pwm_output < (MOTOR_SUNRISE_IGNORE_PULSE-MOTOR_MAX_PULSE))
            pid->pwm_output = (MOTOR_SUNRISE_IGNORE_PULSE-MOTOR_MAX_PULSE);
    }
    else
    {
        if (pid->pwm_output > (MOTOR_MAX_PULSE-MOTOR_IGNORE_PULSE))
            pid->pwm_output = (MOTOR_MAX_PULSE-MOTOR_IGNORE_PULSE);
        if (pid->pwm_output < (MOTOR_IGNORE_PULSE-MOTOR_MAX_PULSE))
            pid->pwm_output = (MOTOR_IGNORE_PULSE-MOTOR_MAX_PULSE);
    }
    return pid->pwm_output;
}

// 位置式PID计算方式
float PID_Location_Calc(motor_pid_t *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;
  
    /* 限定闭环死区 */
    if((pid->err >= -40) && (pid->err <= 40))
    {
        pid->err = 0;
        pid->integral = 0;
    }
    
    /* 积分分离，偏差较大时去掉积分作用 */
    if (pid->err > -1500 && pid->err < 1500)
    {
        pid->integral += pid->err;    // 误差累积

        /* 限定积分范围，防止积分饱和 */
        if (pid->integral > 4000)
            pid->integral = 4000;
        else if (pid->integral < -4000)
            pid->integral = -4000;
    }

	/*PID算法实现*/
    pid->output_val = pid->Kp * pid->err + 
                      pid->Ki * pid->integral + 
                      pid->Kd * (pid->err - pid->err_last);

	/*误差传递*/
    pid->err_last = pid->err;
    
	/*返回当前实际值*/
    return pid->output_val;
}


// PID计算输出值
// 2 passes : les moteurs regules (encodeur valide) d'abord, puis les esclaves
// (encodeur HS) qui recopient le PWM de leur partenaire de meme cote AVEC inversion
// de signe (miroir av/ar). Sans ca, un moteur a encodeur mort verrait une erreur
// constante -> emballement PWM.
void PID_Calc_Motor(motor_data_t* motor)
{
    int i;
    // Passe 1 : moteurs regules
    for (i = 0; i < MAX_MOTOR; i++)
    {
        if (!pid_slaved[i])
            motor->speed_pwm[i] = PID_Incre_Calc(&pid_motor[i], motor->speed_mm_s[i]);
    }
    // Passe 2 : moteurs esclaves -> recopie du partenaire meme cote (signe oppose)
    for (i = 0; i < MAX_MOTOR; i++)
    {
        if (pid_slaved[i])
            motor->speed_pwm[i] = SLAVE_SIGN[i] * motor->speed_pwm[SLAVE_PARTNER[i]];
    }
}

// PID单独计算一条通道
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed)
{
    if (motor_id >= MAX_MOTOR) return 0; 
    return PID_Incre_Calc(&pid_motor[motor_id], now_speed);
}

// 设置PID参数，motor_id=4设置所有，=0123设置对应电机的PID参数。
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].Kp = kp;
            pid_motor[i].Ki = ki;
            pid_motor[i].Kd = kd;
        }
        DEBUG("PID Set:%.3f, %.3f, %.3f\n", kp, ki, kd);
    }
    else
    {
        pid_motor[motor_id].Kp = kp;
        pid_motor[motor_id].Ki = ki;
        pid_motor[motor_id].Kd = kd;
        DEBUG("PID Set M%d:%.3f, %.3f, %.3f\n", motor_id+1, kp, ki, kd);
    }
}

// 清除PID数据
void PID_Clear_Motor(uint8_t motor_id)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].pwm_output = 0.0;
            pid_motor[i].err = 0.0;
            pid_motor[i].err_last = 0.0;
            pid_motor[i].err_next = 0.0;
            pid_motor[i].integral = 0.0;
        }
    }
    else
    {
        pid_motor[motor_id].pwm_output = 0.0;
        pid_motor[motor_id].err = 0.0;
        pid_motor[motor_id].err_last = 0.0;
        pid_motor[motor_id].err_next = 0.0;
        pid_motor[motor_id].integral = 0.0;
    }
}

// Active/desactive le mode esclave d'un moteur (encodeur HS). En esclave, son PID
// n'est plus calcule et son PWM recopie celui du voisin de meme cote (cf PID_Calc_Motor).
void PID_Set_Motor_Slaved(uint8_t motor_id, uint8_t on)
{
    if (motor_id >= MAX_MOTOR) return;
    pid_slaved[motor_id] = on ? 1 : 0;
    PID_Clear_Motor(motor_id); // reset propre (pwm/err/integral) au basculement
}

// Retourne l'etat esclave d'un moteur (0/1).
uint8_t PID_Get_Motor_Slaved(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTOR) return 0;
    return pid_slaved[motor_id];
}

// Partenaire de recopie (roue de meme cote) d'un moteur esclave.
int8_t PID_Slave_Partner(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTOR) return -1;
    return SLAVE_PARTNER[motor_id];
}

// Signe de recopie (miroir av/ar -> -1) d'un moteur esclave.
int8_t PID_Slave_Sign(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTOR) return 1;
    return SLAVE_SIGN[motor_id];
}

// 设置PID目标速度，单位为：mm/s
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].target_val = target;
        }
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}

// 返回PID结构体数组
motor_pid_t* Pid_Get_Motor(void)
{
    return pid_motor;
}


/* 串口发送PID数据到主控上，index=1~5，1~4表示四路电机，5表示YAW */
void PID_Send_Parm_Active(uint8_t index)
{
    #define LEN        12
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
    if (index > 0 && index < 6)
    {
        data_buffer[0] = PTO_HEAD;
        data_buffer[1] = PTO_DEVICE_ID-1;
        data_buffer[2] = LEN-2; // 数量
        data_buffer[3] = FUNC_SET_MOTOR_PID; // 功能位
        data_buffer[4] = index;
        if (index == 5)
        {
            data_buffer[3] = FUNC_SET_YAW_PID; // 功能位
            data_buffer[5] = (int32_t)(pid_Yaw.Proportion * 1000) & 0xff;
            data_buffer[6] = ((int32_t)(pid_Yaw.Proportion * 1000) >> 8) & 0xff;
            data_buffer[7] = (int32_t)(pid_Yaw.Integral * 1000) & 0xff;
            data_buffer[8] = ((int32_t)(pid_Yaw.Integral * 1000) >> 8) & 0xff;
            data_buffer[9] = (int32_t)(pid_Yaw.Derivative * 1000) & 0xff;
            data_buffer[10] = ((int32_t)(pid_Yaw.Derivative * 1000) >> 8) & 0xff;
        }
        else
        {
            data_buffer[5] = (int32_t)(pid_motor[index-1].Kp * 1000) & 0xff;
            data_buffer[6] = ((int32_t)(pid_motor[index-1].Kp * 1000) >> 8) & 0xff;
            data_buffer[7] = (int32_t)(pid_motor[index-1].Ki * 1000) & 0xff;
            data_buffer[8] = ((int32_t)(pid_motor[index-1].Ki * 1000) >> 8) & 0xff;
            data_buffer[9] = (int32_t)(pid_motor[index-1].Kd * 1000) & 0xff;
            data_buffer[10] = ((int32_t)(pid_motor[index-1].Kd * 1000) >> 8) & 0xff;
        }
        for (i = 2; i < LEN-1; i++)
        {
            checknum += data_buffer[i];
        }
        data_buffer[LEN-1] = checknum;
        USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
    }
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////



// 重置偏航角的目标值
void PID_Yaw_Reset(float yaw)
{
	pid_Yaw.SetPoint = yaw;
    pid_Yaw.SumError = 0;
    pid_Yaw.LastError = 0;
    pid_Yaw.PrevError = 0;
}

// 计算偏航角的输出值
float PID_Yaw_Calc(float NextPoint)
{
	float dError, Error;
	Error = pid_Yaw.SetPoint - NextPoint;			// 偏差
	pid_Yaw.SumError += Error;						// 积分
	dError = pid_Yaw.LastError - pid_Yaw.PrevError; // 当前微分
	pid_Yaw.PrevError = pid_Yaw.LastError;
	pid_Yaw.LastError = Error;

	double omega_rad = pid_Yaw.Proportion * Error			 // 比例项
					   + pid_Yaw.Integral * pid_Yaw.SumError // 积分项
					   + pid_Yaw.Derivative * dError;		 // 微分项

	if (omega_rad > PI / 6)
		omega_rad = PI / 6;
	if (omega_rad < -PI / 6)
		omega_rad = -PI / 6;
	return omega_rad;
}

// 设置偏航角PID的参数
void PID_Yaw_Set_Parm(float kp, float ki, float kd)
{
    pid_Yaw.Proportion = kp;
    pid_Yaw.Integral = ki;
    pid_Yaw.Derivative = kd;
}

