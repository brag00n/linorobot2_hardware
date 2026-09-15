#pragma once
// MESSAGE BAMBOO_MAG PACKING

#define MAVLINK_MSG_ID_BAMBOO_MAG 42005


typedef struct __mavlink_bamboo_mag_t {
 uint32_t time_boot_ms; /*< [ms] Horloge carte depuis le boot.*/
 float mx; /*< [uT] Composante X.*/
 float my; /*< [uT] Composante Y.*/
 float mz; /*< [uT] Composante Z.*/
} mavlink_bamboo_mag_t;

#define MAVLINK_MSG_ID_BAMBOO_MAG_LEN 16
#define MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN 16
#define MAVLINK_MSG_ID_42005_LEN 16
#define MAVLINK_MSG_ID_42005_MIN_LEN 16

#define MAVLINK_MSG_ID_BAMBOO_MAG_CRC 72
#define MAVLINK_MSG_ID_42005_CRC 72



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_MAG { \
    42005, \
    "BAMBOO_MAG", \
    4, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_mag_t, time_boot_ms) }, \
         { "mx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bamboo_mag_t, mx) }, \
         { "my", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bamboo_mag_t, my) }, \
         { "mz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_bamboo_mag_t, mz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_MAG { \
    "BAMBOO_MAG", \
    4, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_mag_t, time_boot_ms) }, \
         { "mx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bamboo_mag_t, mx) }, \
         { "my", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bamboo_mag_t, my) }, \
         { "mz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_bamboo_mag_t, mz) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_mag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param mx [uT] Composante X.
 * @param my [uT] Composante Y.
 * @param mz [uT] Composante Z.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_mag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float mx, float my, float mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MAG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, mx);
    _mav_put_float(buf, 8, my);
    _mav_put_float(buf, 12, mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#else
    mavlink_bamboo_mag_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
}

/**
 * @brief Pack a bamboo_mag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param mx [uT] Composante X.
 * @param my [uT] Composante Y.
 * @param mz [uT] Composante Z.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_mag_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float mx, float my, float mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MAG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, mx);
    _mav_put_float(buf, 8, my);
    _mav_put_float(buf, 12, mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#else
    mavlink_bamboo_mag_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MAG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#endif
}

/**
 * @brief Pack a bamboo_mag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param mx [uT] Composante X.
 * @param my [uT] Composante Y.
 * @param mz [uT] Composante Z.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_mag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float mx,float my,float mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MAG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, mx);
    _mav_put_float(buf, 8, my);
    _mav_put_float(buf, 12, mz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#else
    mavlink_bamboo_mag_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_MAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
}

/**
 * @brief Encode a bamboo_mag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_mag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_mag_t* bamboo_mag)
{
    return mavlink_msg_bamboo_mag_pack(system_id, component_id, msg, bamboo_mag->time_boot_ms, bamboo_mag->mx, bamboo_mag->my, bamboo_mag->mz);
}

/**
 * @brief Encode a bamboo_mag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_mag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_mag_t* bamboo_mag)
{
    return mavlink_msg_bamboo_mag_pack_chan(system_id, component_id, chan, msg, bamboo_mag->time_boot_ms, bamboo_mag->mx, bamboo_mag->my, bamboo_mag->mz);
}

/**
 * @brief Encode a bamboo_mag struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_mag_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_mag_t* bamboo_mag)
{
    return mavlink_msg_bamboo_mag_pack_status(system_id, component_id, _status, msg,  bamboo_mag->time_boot_ms, bamboo_mag->mx, bamboo_mag->my, bamboo_mag->mz);
}

/**
 * @brief Send a bamboo_mag message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param mx [uT] Composante X.
 * @param my [uT] Composante Y.
 * @param mz [uT] Composante Z.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_mag_send(mavlink_channel_t chan, uint32_t time_boot_ms, float mx, float my, float mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_MAG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, mx);
    _mav_put_float(buf, 8, my);
    _mav_put_float(buf, 12, mz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MAG, buf, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
#else
    mavlink_bamboo_mag_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MAG, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
#endif
}

/**
 * @brief Send a bamboo_mag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_mag_send_struct(mavlink_channel_t chan, const mavlink_bamboo_mag_t* bamboo_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_mag_send(chan, bamboo_mag->time_boot_ms, bamboo_mag->mx, bamboo_mag->my, bamboo_mag->mz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MAG, (const char *)bamboo_mag, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_MAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_mag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float mx, float my, float mz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, mx);
    _mav_put_float(buf, 8, my);
    _mav_put_float(buf, 12, mz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MAG, buf, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
#else
    mavlink_bamboo_mag_t *packet = (mavlink_bamboo_mag_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->mx = mx;
    packet->my = my;
    packet->mz = mz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_MAG, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_MAG_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_LEN, MAVLINK_MSG_ID_BAMBOO_MAG_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_MAG UNPACKING


/**
 * @brief Get field time_boot_ms from bamboo_mag message
 *
 * @return [ms] Horloge carte depuis le boot.
 */
static inline uint32_t mavlink_msg_bamboo_mag_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mx from bamboo_mag message
 *
 * @return [uT] Composante X.
 */
static inline float mavlink_msg_bamboo_mag_get_mx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field my from bamboo_mag message
 *
 * @return [uT] Composante Y.
 */
static inline float mavlink_msg_bamboo_mag_get_my(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mz from bamboo_mag message
 *
 * @return [uT] Composante Z.
 */
static inline float mavlink_msg_bamboo_mag_get_mz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a bamboo_mag message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_mag C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_mag_decode(const mavlink_message_t* msg, mavlink_bamboo_mag_t* bamboo_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bamboo_mag->time_boot_ms = mavlink_msg_bamboo_mag_get_time_boot_ms(msg);
    bamboo_mag->mx = mavlink_msg_bamboo_mag_get_mx(msg);
    bamboo_mag->my = mavlink_msg_bamboo_mag_get_my(msg);
    bamboo_mag->mz = mavlink_msg_bamboo_mag_get_mz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_MAG_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_MAG_LEN;
        memset(bamboo_mag, 0, MAVLINK_MSG_ID_BAMBOO_MAG_LEN);
    memcpy(bamboo_mag, _MAV_PAYLOAD(msg), len);
#endif
}
