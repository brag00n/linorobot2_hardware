#pragma once
// MESSAGE BAMBOO_ENCODERS PACKING

#define MAVLINK_MSG_ID_BAMBOO_ENCODERS 42003


typedef struct __mavlink_bamboo_encoders_t {
 uint32_t time_boot_ms; /*< [ms] Horloge carte depuis le boot.*/
 int32_t counts[4]; /*<  Comptage cumulatif M1..M4 (tics).*/
} mavlink_bamboo_encoders_t;

#define MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN 20
#define MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN 20
#define MAVLINK_MSG_ID_42003_LEN 20
#define MAVLINK_MSG_ID_42003_MIN_LEN 20

#define MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC 62
#define MAVLINK_MSG_ID_42003_CRC 62

#define MAVLINK_MSG_BAMBOO_ENCODERS_FIELD_COUNTS_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_ENCODERS { \
    42003, \
    "BAMBOO_ENCODERS", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_encoders_t, time_boot_ms) }, \
         { "counts", NULL, MAVLINK_TYPE_INT32_T, 4, 4, offsetof(mavlink_bamboo_encoders_t, counts) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_ENCODERS { \
    "BAMBOO_ENCODERS", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bamboo_encoders_t, time_boot_ms) }, \
         { "counts", NULL, MAVLINK_TYPE_INT32_T, 4, 4, offsetof(mavlink_bamboo_encoders_t, counts) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_encoders message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param counts  Comptage cumulatif M1..M4 (tics).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_encoders_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const int32_t *counts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t_array(buf, 4, counts, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#else
    mavlink_bamboo_encoders_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_assign_int32_t(packet.counts, counts, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_ENCODERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
}

/**
 * @brief Pack a bamboo_encoders message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param counts  Comptage cumulatif M1..M4 (tics).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_encoders_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const int32_t *counts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t_array(buf, 4, counts, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#else
    mavlink_bamboo_encoders_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_memcpy(packet.counts, counts, sizeof(int32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_ENCODERS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#endif
}

/**
 * @brief Pack a bamboo_encoders message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param counts  Comptage cumulatif M1..M4 (tics).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_encoders_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const int32_t *counts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t_array(buf, 4, counts, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#else
    mavlink_bamboo_encoders_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_assign_int32_t(packet.counts, counts, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_ENCODERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
}

/**
 * @brief Encode a bamboo_encoders struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_encoders_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_encoders_t* bamboo_encoders)
{
    return mavlink_msg_bamboo_encoders_pack(system_id, component_id, msg, bamboo_encoders->time_boot_ms, bamboo_encoders->counts);
}

/**
 * @brief Encode a bamboo_encoders struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_encoders_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_encoders_t* bamboo_encoders)
{
    return mavlink_msg_bamboo_encoders_pack_chan(system_id, component_id, chan, msg, bamboo_encoders->time_boot_ms, bamboo_encoders->counts);
}

/**
 * @brief Encode a bamboo_encoders struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_encoders_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_encoders_t* bamboo_encoders)
{
    return mavlink_msg_bamboo_encoders_pack_status(system_id, component_id, _status, msg,  bamboo_encoders->time_boot_ms, bamboo_encoders->counts);
}

/**
 * @brief Send a bamboo_encoders message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Horloge carte depuis le boot.
 * @param counts  Comptage cumulatif M1..M4 (tics).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_encoders_send(mavlink_channel_t chan, uint32_t time_boot_ms, const int32_t *counts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t_array(buf, 4, counts, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_ENCODERS, buf, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
#else
    mavlink_bamboo_encoders_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_assign_int32_t(packet.counts, counts, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_ENCODERS, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
#endif
}

/**
 * @brief Send a bamboo_encoders message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_encoders_send_struct(mavlink_channel_t chan, const mavlink_bamboo_encoders_t* bamboo_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_encoders_send(chan, bamboo_encoders->time_boot_ms, bamboo_encoders->counts);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_ENCODERS, (const char *)bamboo_encoders, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_encoders_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const int32_t *counts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t_array(buf, 4, counts, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_ENCODERS, buf, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
#else
    mavlink_bamboo_encoders_t *packet = (mavlink_bamboo_encoders_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    mav_array_assign_int32_t(packet->counts, counts, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_ENCODERS, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN, MAVLINK_MSG_ID_BAMBOO_ENCODERS_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_ENCODERS UNPACKING


/**
 * @brief Get field time_boot_ms from bamboo_encoders message
 *
 * @return [ms] Horloge carte depuis le boot.
 */
static inline uint32_t mavlink_msg_bamboo_encoders_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field counts from bamboo_encoders message
 *
 * @return  Comptage cumulatif M1..M4 (tics).
 */
static inline uint16_t mavlink_msg_bamboo_encoders_get_counts(const mavlink_message_t* msg, int32_t *counts)
{
    return _MAV_RETURN_int32_t_array(msg, counts, 4,  4);
}

/**
 * @brief Decode a bamboo_encoders message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_encoders C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_encoders_decode(const mavlink_message_t* msg, mavlink_bamboo_encoders_t* bamboo_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bamboo_encoders->time_boot_ms = mavlink_msg_bamboo_encoders_get_time_boot_ms(msg);
    mavlink_msg_bamboo_encoders_get_counts(msg, bamboo_encoders->counts);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN;
        memset(bamboo_encoders, 0, MAVLINK_MSG_ID_BAMBOO_ENCODERS_LEN);
    memcpy(bamboo_encoders, _MAV_PAYLOAD(msg), len);
#endif
}
