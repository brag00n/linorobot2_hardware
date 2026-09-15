#pragma once
// MESSAGE BAMBOO_WHEEL_STATE PACKING

#define MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE 42001


typedef struct __mavlink_bamboo_wheel_state_t {
 uint32_t time_boot_ms; /*< [ms] Horloge carte depuis le boot.*/
 float vx; /*< [m/s] Vitesse lineaire avant (repere chassis).*/
 float vy; /*< [m/s] Vitesse lineaire laterale (0 sur chassis differentiel/4-roues).*/
 float wz; /*< [rad/s] Vitesse angulaire de lacet.*/
} mavlink_bamboo_wheel_state_t;

#define MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN 16
#define MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN 16
#define MAVLINK_MSG_ID_42001_LEN 16
#define MAVLINK_MSG_ID_42001_MIN_LEN 16

#define MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC 202
#define MAVLINK_MSG_ID_42001_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_WHEEL_STATE { \
    42001, \
    "BAMBOO_WHEEL_STATE", \
    4, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_wheel_state_t, time_boot_ms) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bamboo_wheel_state_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bamboo_wheel_state_t, vy) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_bamboo_wheel_state_t, wz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_WHEEL_STATE { \
    "BAMBOO_WHEEL_STATE", \
    4, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_wheel_state_t, time_boot_ms) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bamboo_wheel_state_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bamboo_wheel_state_t, vy) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_bamboo_wheel_state_t, wz) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_wheel_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param vx [m/s] Vitesse lineaire avant (repere chassis).
 * @param vy [m/s] Vitesse lineaire laterale (0 sur chassis differentiel/4-roues).
 * @param wz [rad/s] Vitesse angulaire de lacet.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_wheel_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#else
    mavlink_bamboo_wheel_state_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
}

/**
 * @brief Pack a bamboo_wheel_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param vx [m/s] Vitesse lineaire avant (repere chassis).
 * @param vy [m/s] Vitesse lineaire laterale (0 sur chassis differentiel/4-roues).
 * @param wz [rad/s] Vitesse angulaire de lacet.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_wheel_state_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#else
    mavlink_bamboo_wheel_state_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#endif
}

/**
 * @brief Pack a bamboo_wheel_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param vx [m/s] Vitesse lineaire avant (repere chassis).
 * @param vy [m/s] Vitesse lineaire laterale (0 sur chassis differentiel/4-roues).
 * @param wz [rad/s] Vitesse angulaire de lacet.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_wheel_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float vx,float vy,float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#else
    mavlink_bamboo_wheel_state_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
}

/**
 * @brief Encode a bamboo_wheel_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_wheel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_wheel_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_wheel_state_t* bamboo_wheel_state)
{
    return mavlink_msg_bamboo_wheel_state_pack(system_id, component_id, msg, bamboo_wheel_state->time_boot_ms, bamboo_wheel_state->vx, bamboo_wheel_state->vy, bamboo_wheel_state->wz);
}

/**
 * @brief Encode a bamboo_wheel_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_wheel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_wheel_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_wheel_state_t* bamboo_wheel_state)
{
    return mavlink_msg_bamboo_wheel_state_pack_chan(system_id, component_id, chan, msg, bamboo_wheel_state->time_boot_ms, bamboo_wheel_state->vx, bamboo_wheel_state->vy, bamboo_wheel_state->wz);
}

/**
 * @brief Encode a bamboo_wheel_state struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_wheel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_wheel_state_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_wheel_state_t* bamboo_wheel_state)
{
    return mavlink_msg_bamboo_wheel_state_pack_status(system_id, component_id, _status, msg,  bamboo_wheel_state->time_boot_ms, bamboo_wheel_state->vx, bamboo_wheel_state->vy, bamboo_wheel_state->wz);
}

/**
 * @brief Send a bamboo_wheel_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param vx [m/s] Vitesse lineaire avant (repere chassis).
 * @param vy [m/s] Vitesse lineaire laterale (0 sur chassis differentiel/4-roues).
 * @param wz [rad/s] Vitesse angulaire de lacet.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_wheel_state_send(mavlink_channel_t chan, uint32_t time_boot_ms, float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, wz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE, buf, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
#else
    mavlink_bamboo_wheel_state_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
#endif
}

/**
 * @brief Send a bamboo_wheel_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_wheel_state_send_struct(mavlink_channel_t chan, const mavlink_bamboo_wheel_state_t* bamboo_wheel_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_wheel_state_send(chan, bamboo_wheel_state->time_boot_ms, bamboo_wheel_state->vx, bamboo_wheel_state->vy, bamboo_wheel_state->wz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE, (const char *)bamboo_wheel_state, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_wheel_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, vx);
    _mav_put_float(buf, 8, vy);
    _mav_put_float(buf, 12, wz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE, buf, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
#else
    mavlink_bamboo_wheel_state_t *packet = (mavlink_bamboo_wheel_state_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->vx = vx;
    packet->vy = vy;
    packet->wz = wz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_WHEEL_STATE UNPACKING


/**
 * @brief Get field time_boot_ms from bamboo_wheel_state message
 *
 * @return [ms] Horloge carte depuis le boot.
 */
static inline uint32_t mavlink_msg_bamboo_wheel_state_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field vx from bamboo_wheel_state message
 *
 * @return [m/s] Vitesse lineaire avant (repere chassis).
 */
static inline float mavlink_msg_bamboo_wheel_state_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vy from bamboo_wheel_state message
 *
 * @return [m/s] Vitesse lineaire laterale (0 sur chassis differentiel/4-roues).
 */
static inline float mavlink_msg_bamboo_wheel_state_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field wz from bamboo_wheel_state message
 *
 * @return [rad/s] Vitesse angulaire de lacet.
 */
static inline float mavlink_msg_bamboo_wheel_state_get_wz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a bamboo_wheel_state message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_wheel_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_wheel_state_decode(const mavlink_message_t* msg, mavlink_bamboo_wheel_state_t* bamboo_wheel_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bamboo_wheel_state->time_boot_ms = mavlink_msg_bamboo_wheel_state_get_time_boot_ms(msg);
    bamboo_wheel_state->vx = mavlink_msg_bamboo_wheel_state_get_vx(msg);
    bamboo_wheel_state->vy = mavlink_msg_bamboo_wheel_state_get_vy(msg);
    bamboo_wheel_state->wz = mavlink_msg_bamboo_wheel_state_get_wz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN;
        memset(bamboo_wheel_state, 0, MAVLINK_MSG_ID_BAMBOO_WHEEL_STATE_LEN);
    memcpy(bamboo_wheel_state, _MAV_PAYLOAD(msg), len);
#endif
}
