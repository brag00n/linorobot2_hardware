#include "mav_protocol.h"

#ifdef ENABLE_MAVLINK

/* Le dialecte genere fournit les helpers inline (mavlink_parse_char,
   mavlink_msg_to_send_buffer, ...). Aucun comm_send_ch requis : on ne definit
   pas MAVLINK_USE_CONVENIENCE_FUNCTIONS et on packe dans un mavlink_message_t
   puis on serialise a la main vers l'UART DMA. */
#include "bamboo/mavlink.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp.h"
#include "bsp_usart.h"
#include "bsp_encoder.h"
#include "bsp_pwmServo.h"
#include "app.h"
#include "app_motion.h"
#include "app_pid.h"
#include "app_bat.h"
#include "app_flash.h"
#include "protocol.h"     /* Proto_Now_Ms() : timestamp ms partage */
#include "icm20948.h"
#include "bsp_mpu9250.h"

#include <string.h>

/* Gains PID vivants (globaux non statiques dans app_pid.c). */
extern motor_pid_t pid_motor[4];
extern PID pid_Yaw;

#define DEG2RAD   (0.01745329252f)   /* gyro_float est en dps -> rad/s */

/* ======================================================================= */
/*  Etat TX : un seul chemin de framing, serialise sans mutex.             */
/*  Ce projet compile FreeRTOS avec configUSE_MUTEXES=0 (pas de            */
/*  xSemaphoreCreateMutex). On serialise donc pack+framing+handoff DMA en  */
/*  suspendant l'ordonnanceur : les IT restent actives (l'ISR USART RX et  */
/*  le DMA continuent), seule la preemption entre taches est bloquee — ce  */
/*  qui suffit puisque seules vTask_Control (ACK/echos PARAM) et           */
/*  vTask_Auto_Report (telemetrie) emettent. Le garde est bref : le spin   */
/*  d'attente DMA ne bloque que si une trame precedente n'a pas fini de    */
/*  partir, ce que la cadence telemetrie (>=10 ms entre trames) evite.     */
/*  s_tx_msg/s_tx_buf sont en BSS (la RAM est large) pour ne pas charger   */
/*  les piles tres justes des taches (vTask_Control = 128 mots).           */
/* ======================================================================= */
#define MAV_TX_LOCK()    vTaskSuspendAll()
#define MAV_TX_UNLOCK()  ((void)xTaskResumeAll())

static mavlink_message_t s_tx_msg;
static uint8_t s_tx_buf[MAVLINK_MAX_PACKET_LEN];

/* Envoie le message deja packe dans s_tx_msg. A appeler garde pris. */
static void mav_send_locked(void)
{
	uint16_t len = mavlink_msg_to_send_buffer(s_tx_buf, &s_tx_msg);
	USART1_Send_ArrayU8(s_tx_buf, len);
}

/* ======================================================================= */
/*  Etat RX : l'ISR decode, la tache route.                                */
/* ======================================================================= */
static mavlink_message_t s_rx_parse;   /* etat de parsing (ISR) */
static mavlink_status_t  s_rx_status;
static mavlink_message_t s_rx_ready;    /* dernier message complet pour la tache */
static volatile uint8_t  s_rx_flag = 0; /* 1 = un message attend d'etre route */

void Mav_Receive_Byte(uint8_t byte)
{
	if (mavlink_parse_char(MAVLINK_COMM_0, byte, &s_rx_parse, &s_rx_status))
	{
		/* Trame complete + CRC OK. On ne recopie que si la tache a consomme
		   le precedent (sinon on laisse tomber, comme le chemin v1). */
		if (!s_rx_flag)
		{
			memcpy(&s_rx_ready, &s_rx_parse, sizeof(mavlink_message_t));
			s_rx_flag = 1;
		}
	}
}

/* ======================================================================= */
/*  Table de parametres (protocole PARAM_*). Tous en REAL32.               */
/* ======================================================================= */
enum {
	P_MOT1_KP, P_MOT1_KI, P_MOT1_KD,
	P_MOT2_KP, P_MOT2_KI, P_MOT2_KD,
	P_MOT3_KP, P_MOT3_KI, P_MOT3_KD,
	P_MOT4_KP, P_MOT4_KI, P_MOT4_KD,
	P_YAW_KP,  P_YAW_KI,  P_YAW_KD,
	P_WHEEL_CPR, P_WHEEL_CIRC, P_WHEEL_APB,
	P_CAR_TYPE,
	PARAM_COUNT
};

static const char * const s_param_name[PARAM_COUNT] = {
	"MOT1_KP", "MOT1_KI", "MOT1_KD",
	"MOT2_KP", "MOT2_KI", "MOT2_KD",
	"MOT3_KP", "MOT3_KI", "MOT3_KD",
	"MOT4_KP", "MOT4_KI", "MOT4_KD",
	"YAW_KP",  "YAW_KI",  "YAW_KD",
	"WHEEL_CPR", "WHEEL_CIRC", "WHEEL_APB",
	"CAR_TYPE",
};

/* Lit la valeur vivante d'un parametre par index. */
static float param_get(uint16_t idx)
{
	float cpr = 0, circ = 0, apb = 0;
	if (idx <= P_MOT4_KD)
	{
		uint8_t m = idx / 3;   /* moteur 0..3 */
		uint8_t k = idx % 3;   /* 0=Kp 1=Ki 2=Kd */
		return (k == 0) ? pid_motor[m].Kp : (k == 1 ? pid_motor[m].Ki : pid_motor[m].Kd);
	}
	switch (idx)
	{
	case P_YAW_KP: return pid_Yaw.Proportion;
	case P_YAW_KI: return pid_Yaw.Integral;
	case P_YAW_KD: return pid_Yaw.Derivative;
	case P_WHEEL_CPR:  Motion_Get_Wheel_Geom(&cpr, &circ, &apb); return cpr;
	case P_WHEEL_CIRC: Motion_Get_Wheel_Geom(&cpr, &circ, &apb); return circ;
	case P_WHEEL_APB:  Motion_Get_Wheel_Geom(&cpr, &circ, &apb); return apb;
	case P_CAR_TYPE:   return (float)Motion_Get_Car_Type();
	default: return 0;
	}
}

/* Applique une valeur (etat vivant seulement ; la persistance flash passe par
   PREFLIGHT_STORAGE). Renvoie l'index applique, ou -1 si nom inconnu. */
static int param_set_by_name(const char *name, float value)
{
	uint16_t idx;
	for (idx = 0; idx < PARAM_COUNT; idx++)
	{
		/* param_id MAVLink : jusqu'a 16 octets, non termine si plein. */
		if (strncmp(name, s_param_name[idx], 16) == 0)
			break;
	}
	if (idx >= PARAM_COUNT) return -1;

	if (idx <= P_MOT4_KD)
	{
		uint8_t m = idx / 3;
		uint8_t k = idx % 3;
		float kp = pid_motor[m].Kp, ki = pid_motor[m].Ki, kd = pid_motor[m].Kd;
		if (k == 0) kp = value; else if (k == 1) ki = value; else kd = value;
		/* gains reels -> ré-active la regulation de ce moteur */
		PID_Set_Motor_Slaved(m, 0);
		PID_Set_Motor_Parm(m, kp, ki, kd);
		return idx;
	}
	switch (idx)
	{
	case P_YAW_KP: case P_YAW_KI: case P_YAW_KD:
	{
		float kp = pid_Yaw.Proportion, ki = pid_Yaw.Integral, kd = pid_Yaw.Derivative;
		if (idx == P_YAW_KP) kp = value; else if (idx == P_YAW_KI) ki = value; else kd = value;
		PID_Yaw_Set_Parm(kp, ki, kd);
		return idx;
	}
	case P_WHEEL_CPR: case P_WHEEL_CIRC: case P_WHEEL_APB:
	{
		float cpr, circ, apb;
		Motion_Get_Wheel_Geom(&cpr, &circ, &apb);
		if (idx == P_WHEEL_CPR) cpr = value;
		else if (idx == P_WHEEL_CIRC) circ = value;
		else apb = value;
		if (cpr > 0 && circ > 0 && apb > 0)
			Motion_Set_Wheel_Geom(cpr, circ, apb);
		return idx;
	}
	case P_CAR_TYPE:
	{
		uint8_t ct = (uint8_t)value;
		if (ct < CAR_TYPE_MAX)
		{
			Motion_Set_Car_Type((car_type_t)ct);
			if (Motion_Get_Car_Type() == CAR_SUNRISE)
				PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
		}
		return idx;
	}
	default: return -1;
	}
}

/* Emet un PARAM_VALUE pour l'index donne. */
static void mav_send_param(uint16_t idx)
{
	if (idx >= PARAM_COUNT) return;
	MAV_TX_LOCK();
	mavlink_msg_param_value_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                             s_param_name[idx], param_get(idx),
	                             MAV_PARAM_TYPE_REAL32, PARAM_COUNT, idx);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

/* Ecrit tous les parametres vivants en flash (module Flash existant). */
static void params_store_to_flash(void)
{
	for (uint8_t m = 0; m < 4; m++)
		Flash_Set_PID(m, pid_motor[m].Kp, pid_motor[m].Ki, pid_motor[m].Kd);
	Flash_Set_Yaw_PID(pid_Yaw.Proportion, pid_Yaw.Integral, pid_Yaw.Derivative);

	float cpr, circ, apb;
	Motion_Get_Wheel_Geom(&cpr, &circ, &apb);
	Flash_Set_Wheel_Geom((uint16_t)(cpr + 0.5f),
	                     (uint16_t)(circ * 10.0f + 0.5f),
	                     (uint16_t)(apb * 10.0f + 0.5f));
	Flash_Set_CarType((uint8_t)Motion_Get_Car_Type());
}

/* ======================================================================= */
/*  Emetteurs de telemetrie                                                */
/* ======================================================================= */
void Mav_Send_Heartbeat(void)
{
	MAV_TX_LOCK();
	mavlink_msg_heartbeat_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                           MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
	                           MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

void Mav_Send_Sys_Status(void)
{
	/* Bat_Voltage_Z10 = tension x10 -> mV = x100. */
	uint16_t mv = (uint16_t)(Bat_Voltage_Z10() * 100);
	MAV_TX_LOCK();
	mavlink_msg_sys_status_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                            0, 0, 0,            /* sensors present/enabled/health */
	                            0,                  /* load */
	                            mv,                 /* voltage_battery (mV) */
	                            -1,                 /* current_battery (non mesure) */
	                            -1,                 /* battery_remaining (inconnu) */
	                            0, 0, 0, 0, 0, 0);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

void Mav_Send_Attitude(void)
{
	float roll = 0, pitch = 0, yaw = 0;
	float gr = 0, gp = 0, gy = 0;   /* rates rad/s */
	if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
	{
		icm20948_data_t d;
		ICM20948_Get_Data(&d);
		roll  = d.orientation[0];
		pitch = d.orientation[1];
		yaw   = d.orientation[2];
		gr = d.gyro_float[0] * DEG2RAD;
		gp = d.gyro_float[1] * DEG2RAD;
		gy = d.gyro_float[2] * DEG2RAD;
	}
	else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
	{
		roll  = MPU_Get_Roll_Now();
		pitch = MPU_Get_Pitch_Now();
		yaw   = MPU_Get_Yaw_Now();
	}
	MAV_TX_LOCK();
	mavlink_msg_attitude_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                          Proto_Now_Ms(), roll, pitch, yaw, gr, gp, gy);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

void Mav_Send_Wheel_State(void)
{
	car_data_t car;
	Motion_Get_Speed(&car);
	/* Vx,Vy en mm/s -> m/s ; Vz en mrad/s -> rad/s. */
	float vx = car.Vx / 1000.0f;
	float vy = car.Vy / 1000.0f;
	float wz = car.Vz / 1000.0f;
	MAV_TX_LOCK();
	mavlink_msg_bamboo_wheel_state_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                                    Proto_Now_Ms(), vx, vy, wz);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

void Mav_Send_Encoders(void)
{
	int enc[4] = {0};
	int32_t counts[4];
	Encoder_Get_ALL(enc);
	for (int i = 0; i < 4; i++) counts[i] = (int32_t)enc[i];
	MAV_TX_LOCK();
	mavlink_msg_bamboo_encoders_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                                 Proto_Now_Ms(), counts);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

/* ======================================================================= */
/*  Routage des commandes                                                  */
/* ======================================================================= */
static void mav_send_command_ack(uint16_t command, uint8_t result)
{
	MAV_TX_LOCK();
	mavlink_msg_command_ack_pack(MAV_SYS_ID_STM32, MAV_COMP_ID, &s_tx_msg,
	                             command, result, 0, 0,
	                             0, 0 /* target_system/component : broadcast */);
	mav_send_locked();
	MAV_TX_UNLOCK();
}

static void handle_command_long(const mavlink_message_t *msg)
{
	uint16_t cmd = mavlink_msg_command_long_get_command(msg);
	float p1 = mavlink_msg_command_long_get_param1(msg);
	uint8_t result = MAV_RESULT_ACCEPTED;

	switch (cmd)
	{
	case MAV_CMD_DO_SET_SERVO:
	{
		/* param1 = numero de servo (1..N), param2 = angle degres (0..180).
		   L'hote pilote les deux bouts : on interprete param2 en degres,
		   directement consommables par PwmServo_Set_Angle (index 0-based). */
		uint8_t servo = (uint8_t)p1;
		uint8_t angle = (uint8_t)mavlink_msg_command_long_get_param2(msg);
		if (servo >= 1) PwmServo_Set_Angle(servo - 1, angle);
		else result = MAV_RESULT_DENIED;
		break;
	}
	case MAV_CMD_PREFLIGHT_CALIBRATION:
		/* param1=1 : calibration gyro. Le DMP de l'ICM20948 auto-calibre son
		   biais ; l'action utile cote utilisateur est la re-mise a zero du
		   cap -> App_Clear_Yaw. ACCEPTED. */
		App_Clear_Yaw();
		break;
	case MAV_CMD_PREFLIGHT_STORAGE:
		if ((int)p1 == 1)      params_store_to_flash();   /* WRITE */
		else if ((int)p1 == 2) Flash_Reset_All_Value();   /* reset defauts */
		/* p1==0 (READ) : les params refletent deja l'etat vivant, no-op. */
		break;
	case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
		mav_send_command_ack(cmd, MAV_RESULT_ACCEPTED); /* ACK avant de partir */
		if ((int)p1 == 1)      Bsp_Reset_MCU();
		else if ((int)p1 == 3) Bsp_Jump_To_Bootloader();
		return;
	case MAV_CMD_USER_1:   /* reset yaw / clear yaw (ex 0x52/0xA2) */
		App_Clear_Yaw();
		break;
	case MAV_CMD_USER_2:   /* reset odometrie : aucune primitive encodeur -> non supporte */
		result = MAV_RESULT_UNSUPPORTED;
		break;
	default:
		result = MAV_RESULT_UNSUPPORTED;
		break;
	}
	mav_send_command_ack(cmd, result);
}

void Mav_Poll_Rx(uint8_t block_motion)
{
	if (!s_rx_flag) return;
	mavlink_message_t *msg = &s_rx_ready;

	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_BAMBOO_CMD_VEL:
	{
		if (!block_motion)
		{
			/* m/s -> mm/s ; rad/s -> mrad/s (unites de Motion_Ctrl). */
			int16_t vx = (int16_t)(mavlink_msg_bamboo_cmd_vel_get_vx(msg) * 1000.0f);
			int16_t vy = (int16_t)(mavlink_msg_bamboo_cmd_vel_get_vy(msg) * 1000.0f);
			int16_t wz = (int16_t)(mavlink_msg_bamboo_cmd_vel_get_wz(msg) * 1000.0f);
			if (vx == 0 && vy == 0 && wz == 0) Motion_Stop(STOP_BRAKE);
			else Motion_Ctrl(vx, vy, wz, 1);
		}
		break;
	}
	case MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM:
	{
		if (!block_motion)
		{
			int8_t pwm[4];
			mavlink_msg_bamboo_motor_pwm_get_pwm(msg, pwm);
			int16_t motor_pulse = MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE;
			if (Motion_Get_Car_Type() == CAR_SUNRISE)
				motor_pulse = MOTOR_MAX_PULSE - MOTOR_SUNRISE_IGNORE_PULSE;
			int16_t s[4];
			for (int i = 0; i < 4; i++) s[i] = (int16_t)(pwm[i] * (motor_pulse / 100.0f));
			Motion_Set_Pwm(s[0], s[1], s[2], s[3]);
		}
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	{
		s_rx_flag = 0;   /* libere le buffer RX avant la salve d'emissions */
		for (uint16_t i = 0; i < PARAM_COUNT; i++) mav_send_param(i);
		return;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	{
		int16_t pidx = mavlink_msg_param_request_read_get_param_index(msg);
		s_rx_flag = 0;
		if (pidx >= 0)
		{
			mav_send_param((uint16_t)pidx);
		}
		else
		{
			char name[17] = {0};
			mavlink_msg_param_request_read_get_param_id(msg, name);
			for (uint16_t i = 0; i < PARAM_COUNT; i++)
				if (strncmp(name, s_param_name[i], 16) == 0) { mav_send_param(i); break; }
		}
		return;
	}
	case MAVLINK_MSG_ID_PARAM_SET:
	{
		char name[17] = {0};
		mavlink_msg_param_set_get_param_id(msg, name);
		float val = mavlink_msg_param_set_get_param_value(msg);
		int idx = param_set_by_name(name, val);
		s_rx_flag = 0;
		if (idx >= 0) mav_send_param((uint16_t)idx);  /* echo de la valeur appliquee */
		return;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_command_long(msg);
		break;
	default:
		break;
	}
	s_rx_flag = 0;
}

void Mav_Init(void)
{
	s_rx_flag = 0;
}

#endif /* ENABLE_MAVLINK */
