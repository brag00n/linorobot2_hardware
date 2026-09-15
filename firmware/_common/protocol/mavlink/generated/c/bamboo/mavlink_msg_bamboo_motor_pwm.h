#pragma once
// MESSAGE BAMBOO_MOTOR_PWM PACKING

#define MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM 42002


typedef struct __mavlink_bamboo_motor_pwm_t {
 int8_t pwm[4]; /*< [%] PWM signe par moteur M1..M4, -100..100 %.*/
} mavlink_bamboo_motor_pwm_t;

#define MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN 4
#define MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN 4
#define MAVLINK_MSG_ID_42002_LEN 4
#define MAVLINK_MSG_ID_42002_MIN_LEN 4

#define MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC 239
#define MAVLINK_MSG_ID_42002_CRC 239

#define MAVLINK_MSG_BAMBOO_MOTOR_PWM_FIELD_PWM_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_MOTOR_PWM { \
    42002, \
    "BAMBOO_MOTOR_PWM", \
    1, \
    {  { "pwm", NULL, MAVLINK_TYPE_INT8_T, 4, 0, offsetof(mavlink_bamboo_motor_pwm_t, pwm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_MOTOR_PWM { \
    "BAMBOO_MOTOR_PWM", \
    1, \
    {  { "pwm", NULL, MAVLINK_TYPE_INT8_T, 4, 0, offsetof(mavlink_bamboo_motor_pwm_t, pwm) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_motor_pwm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwm [%] PWM signe par moteur M1..M4, -100..100 %.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const int8_t *pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN];

    _mav_put_int8_t_array(buf, 0, pwm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#else
    mavlink_bamboo_motor_pwm_t packet;

    mav_array_assign_int8_t(packet.pwm, pwm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
}

/**
 * @brief Pack a bamboo_motor_pwm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwm [%] PWM signe par moteur M1..M4, -100..100 %.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const int8_t *pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN];

    _mav_put_int8_t_array(buf, 0, pwm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#else
    mavlink_bamboo_motor_pwm_t packet;

    mav_array_memcpy(packet.pwm, pwm, sizeof(int8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#endif
}

/**
 * @brief Pack a bamboo_motor_pwm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pwm [%] PWM signe par moteur M1..M4, -100..100 %.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const int8_t *pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN];

    _mav_put_int8_t_array(buf, 0, pwm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#else
    mavlink_bamboo_motor_pwm_t packet;

    mav_array_assign_int8_t(packet.pwm, pwm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
}

/**
 * @brief Encode a bamboo_motor_pwm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_motor_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_motor_pwm_t* bamboo_motor_pwm)
{
    return mavlink_msg_bamboo_motor_pwm_pack(system_id, component_id, msg, bamboo_motor_pwm->pwm);
}

/**
 * @brief Encode a bamboo_motor_pwm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_motor_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_motor_pwm_t* bamboo_motor_pwm)
{
    return mavlink_msg_bamboo_motor_pwm_pack_chan(system_id, component_id, chan, msg, bamboo_motor_pwm->pwm);
}

/**
 * @brief Encode a bamboo_motor_pwm struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_motor_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_motor_pwm_t* bamboo_motor_pwm)
{
    return mavlink_msg_bamboo_motor_pwm_pack_status(system_id, component_id, _status, msg,  bamboo_motor_pwm->pwm);
}

/**
 * @brief Send a bamboo_motor_pwm message
 * @param chan MAVLink channel to send the message
 *
 * @param pwm [%] PWM signe par moteur M1..M4, -100..100 %.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_motor_pwm_send(mavlink_channel_t chan, const int8_t *pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN];

    _mav_put_int8_t_array(buf, 0, pwm, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM, buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
#else
    mavlink_bamboo_motor_pwm_t packet;

    mav_array_assign_int8_t(packet.pwm, pwm, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
#endif
}

/**
 * @brief Send a bamboo_motor_pwm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_motor_pwm_send_struct(mavlink_channel_t chan, const mavlink_bamboo_motor_pwm_t* bamboo_motor_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_motor_pwm_send(chan, bamboo_motor_pwm->pwm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM, (const char *)bamboo_motor_pwm, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_motor_pwm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const int8_t *pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_int8_t_array(buf, 0, pwm, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM, buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
#else
    mavlink_bamboo_motor_pwm_t *packet = (mavlink_bamboo_motor_pwm_t *)msgbuf;

    mav_array_assign_int8_t(packet->pwm, pwm, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_MOTOR_PWM UNPACKING


/**
 * @brief Get field pwm from bamboo_motor_pwm message
 *
 * @return [%] PWM signe par moteur M1..M4, -100..100 %.
 */
static inline uint16_t mavlink_msg_bamboo_motor_pwm_get_pwm(const mavlink_message_t* msg, int8_t *pwm)
{
    return _MAV_RETURN_int8_t_array(msg, pwm, 4,  0);
}

/**
 * @brief Decode a bamboo_motor_pwm message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_motor_pwm C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_motor_pwm_decode(const mavlink_message_t* msg, mavlink_bamboo_motor_pwm_t* bamboo_motor_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_motor_pwm_get_pwm(msg, bamboo_motor_pwm->pwm);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN;
        memset(bamboo_motor_pwm, 0, MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM_LEN);
    memcpy(bamboo_motor_pwm, _MAV_PAYLOAD(msg), len);
#endif
}
