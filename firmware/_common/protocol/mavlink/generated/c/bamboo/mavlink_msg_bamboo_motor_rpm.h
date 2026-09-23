#pragma once
// MESSAGE BAMBOO_MOTOR_RPM PACKING

#define MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM 42006


typedef struct __mavlink_bamboo_motor_rpm_t {
 uint32_t time_boot_ms; /*< [ms] Horloge carte depuis le boot.*/
 float rpm[4]; /*< [rpm] Vitesse MESUREE M1..M4 (encodeur, apres filtrage carte).*/
 float rpm_req[4]; /*< [rpm] Vitesse DEMANDEE M1..M4 (sortie de la cinematique embarquee = consigne du PID).*/
} mavlink_bamboo_motor_rpm_t;

#define MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN 36
#define MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN 36
#define MAVLINK_MSG_ID_42006_LEN 36
#define MAVLINK_MSG_ID_42006_MIN_LEN 36

#define MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC 25
#define MAVLINK_MSG_ID_42006_CRC 25

#define MAVLINK_MSG_BAMBOO_MOTOR_RPM_FIELD_RPM_LEN 4
#define MAVLINK_MSG_BAMBOO_MOTOR_RPM_FIELD_RPM_REQ_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_MOTOR_RPM { \
    42006, \
    "BAMBOO_MOTOR_RPM", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_motor_rpm_t, time_boot_ms) }, \
         { "rpm", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_bamboo_motor_rpm_t, rpm) }, \
         { "rpm_req", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_bamboo_motor_rpm_t, rpm_req) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_MOTOR_RPM { \
    "BAMBOO_MOTOR_RPM", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_motor_rpm_t, time_boot_ms) }, \
         { "rpm", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_bamboo_motor_rpm_t, rpm) }, \
         { "rpm_req", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_bamboo_motor_rpm_t, rpm_req) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_motor_rpm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param rpm [rpm] Vitesse MESUREE M1..M4 (encodeur, apres filtrage carte).
 * @param rpm_req [rpm] Vitesse DEMANDEE M1..M4 (sortie de la cinematique embarquee = consigne du PID).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *rpm, const float *rpm_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rpm, 4);
    _mav_put_float_array(buf, 20, rpm_req, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#else
    mavlink_bamboo_motor_rpm_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_assign_float(packet.rpm, rpm, 4);
    mav_array_assign_float(packet.rpm_req, rpm_req, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
}

/**
 * @brief Pack a bamboo_motor_rpm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param rpm [rpm] Vitesse MESUREE M1..M4 (encodeur, apres filtrage carte).
 * @param rpm_req [rpm] Vitesse DEMANDEE M1..M4 (sortie de la cinematique embarquee = consigne du PID).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *rpm, const float *rpm_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rpm, 4);
    _mav_put_float_array(buf, 20, rpm_req, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#else
    mavlink_bamboo_motor_rpm_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_memcpy(packet.rpm, rpm, sizeof(float)*4);
    mav_array_memcpy(packet.rpm_req, rpm_req, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#endif
}

/**
 * @brief Pack a bamboo_motor_rpm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param rpm [rpm] Vitesse MESUREE M1..M4 (encodeur, apres filtrage carte).
 * @param rpm_req [rpm] Vitesse DEMANDEE M1..M4 (sortie de la cinematique embarquee = consigne du PID).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const float *rpm,const float *rpm_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rpm, 4);
    _mav_put_float_array(buf, 20, rpm_req, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#else
    mavlink_bamboo_motor_rpm_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_assign_float(packet.rpm, rpm, 4);
    mav_array_assign_float(packet.rpm_req, rpm_req, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
}

/**
 * @brief Encode a bamboo_motor_rpm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_motor_rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_motor_rpm_t* bamboo_motor_rpm)
{
    return mavlink_msg_bamboo_motor_rpm_pack(system_id, component_id, msg, bamboo_motor_rpm->time_boot_ms, bamboo_motor_rpm->rpm, bamboo_motor_rpm->rpm_req);
}

/**
 * @brief Encode a bamboo_motor_rpm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_motor_rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_motor_rpm_t* bamboo_motor_rpm)
{
    return mavlink_msg_bamboo_motor_rpm_pack_chan(system_id, component_id, chan, msg, bamboo_motor_rpm->time_boot_ms, bamboo_motor_rpm->rpm, bamboo_motor_rpm->rpm_req);
}

/**
 * @brief Encode a bamboo_motor_rpm struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_motor_rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_motor_rpm_t* bamboo_motor_rpm)
{
    return mavlink_msg_bamboo_motor_rpm_pack_status(system_id, component_id, _status, msg,  bamboo_motor_rpm->time_boot_ms, bamboo_motor_rpm->rpm, bamboo_motor_rpm->rpm_req);
}

/**
 * @brief Send a bamboo_motor_rpm message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param rpm [rpm] Vitesse MESUREE M1..M4 (encodeur, apres filtrage carte).
 * @param rpm_req [rpm] Vitesse DEMANDEE M1..M4 (sortie de la cinematique embarquee = consigne du PID).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_motor_rpm_send(mavlink_channel_t chan, uint32_t time_boot_ms, const float *rpm, const float *rpm_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rpm, 4);
    _mav_put_float_array(buf, 20, rpm_req, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM, buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
#else
    mavlink_bamboo_motor_rpm_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_assign_float(packet.rpm, rpm, 4);
    mav_array_assign_float(packet.rpm_req, rpm_req, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
#endif
}

/**
 * @brief Send a bamboo_motor_rpm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_motor_rpm_send_struct(mavlink_channel_t chan, const mavlink_bamboo_motor_rpm_t* bamboo_motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_motor_rpm_send(chan, bamboo_motor_rpm->time_boot_ms, bamboo_motor_rpm->rpm, bamboo_motor_rpm->rpm_req);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM, (const char *)bamboo_motor_rpm, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_motor_rpm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const float *rpm, const float *rpm_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rpm, 4);
    _mav_put_float_array(buf, 20, rpm_req, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM, buf, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
#else
    mavlink_bamboo_motor_rpm_t *packet = (mavlink_bamboo_motor_rpm_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    mav_array_assign_float(packet->rpm, rpm, 4);
    mav_array_assign_float(packet->rpm_req, rpm_req, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_MOTOR_RPM UNPACKING


/**
 * @brief Get field time_boot_ms from bamboo_motor_rpm message
 *
 * @return [ms] Horloge carte depuis le boot.
 */
static inline uint32_t mavlink_msg_bamboo_motor_rpm_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rpm from bamboo_motor_rpm message
 *
 * @return [rpm] Vitesse MESUREE M1..M4 (encodeur, apres filtrage carte).
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_get_rpm(const mavlink_message_t* msg, float *rpm)
{
    return _MAV_RETURN_float_array(msg, rpm, 4,  4);
}

/**
 * @brief Get field rpm_req from bamboo_motor_rpm message
 *
 * @return [rpm] Vitesse DEMANDEE M1..M4 (sortie de la cinematique embarquee = consigne du PID).
 */
static inline uint16_t mavlink_msg_bamboo_motor_rpm_get_rpm_req(const mavlink_message_t* msg, float *rpm_req)
{
    return _MAV_RETURN_float_array(msg, rpm_req, 4,  20);
}

/**
 * @brief Decode a bamboo_motor_rpm message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_motor_rpm C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_motor_rpm_decode(const mavlink_message_t* msg, mavlink_bamboo_motor_rpm_t* bamboo_motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bamboo_motor_rpm->time_boot_ms = mavlink_msg_bamboo_motor_rpm_get_time_boot_ms(msg);
    mavlink_msg_bamboo_motor_rpm_get_rpm(msg, bamboo_motor_rpm->rpm);
    mavlink_msg_bamboo_motor_rpm_get_rpm_req(msg, bamboo_motor_rpm->rpm_req);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN;
        memset(bamboo_motor_rpm, 0, MAVLINK_MSG_ID_BAMBOO_MOTOR_RPM_LEN);
    memcpy(bamboo_motor_rpm, _MAV_PAYLOAD(msg), len);
#endif
}
