#ifndef __MAV_PROTOCOL_H__
#define __MAV_PROTOCOL_H__

#include "stm32f10x.h"

/*
 * Transport MAVLink v2 (dialecte bamboo) pour la carte STM32.
 *
 * Chemin actif uniquement quand ENABLE_MAVLINK est defini (env PlatformIO
 * genericSTM32F103RC_mavlink). Sinon tout ce module se compile a vide et le
 * protocole binaire maison (Yahboom, cf. protocol.c) reste en place.
 *
 * Decoupage RX identique a l'existant : l'ISR USART1 injecte les octets un a
 * un (decode dans l'ISR), et le routage — qui peut ecrire en flash — se fait
 * en contexte tache (Mav_Poll_Rx appelee par vTask_Control). Le TX est
 * centralise (un seul chemin de framing) et serialise par mutex car il est
 * emis depuis deux taches (vTask_Auto_Report pour la telemetrie, vTask_Control
 * pour les ACK/echos PARAM).
 *
 * Adressage : chaque carte = un sysid distinct (STM32=1, ESP32=2, Teensy=3) ;
 * compid = MAV_COMP_ID_AUTOPILOT1 (1) ; l'hote emet en sysid 255.
 */

#define MAV_SYS_ID_STM32   (1)
#define MAV_COMP_ID        (1)   /* MAV_COMP_ID_AUTOPILOT1 */

/* Initialise le mutex TX et l'etat interne. A appeler avant de lancer les taches. */
void Mav_Init(void);

/* --- RX ---------------------------------------------------------------- */
/* Alimente le parser MAVLink depuis l'ISR USART1 (un octet). Sur trame
   complete, memorise le message pour la tache (drapeau). Ne route rien ici. */
void Mav_Receive_Byte(uint8_t byte);

/* Route le dernier message complet recu (contexte tache). block_motion=1
   bloque cmd_vel/motor_pwm (mode batterie faible). */
void Mav_Poll_Rx(uint8_t block_motion);

/* --- TX (emetteurs de telemetrie, packent + envoient) ------------------ */
void Mav_Send_Heartbeat(void);
void Mav_Send_Sys_Status(void);
void Mav_Send_Attitude(void);
void Mav_Send_Wheel_State(void);
void Mav_Send_Encoders(void);

#endif /* __MAV_PROTOCOL_H__ */
