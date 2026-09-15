#pragma once
// MESSAGE BAMBOO_CMD_VEL PACKING

#define MAVLINK_MSG_ID_BAMBOO_CMD_VEL 42004


typedef struct __mavlink_bamboo_cmd_vel_t {
 float vx; /*< [m/s] Vitesse lineaire avant demandee.*/
 float vy; /*< [m/s] Vitesse laterale demandee (ignoree hors mecanum).*/
 float wz; /*< [rad/s] Vitesse angulaire de lacet demandee.*/
} mavlink_bamboo_cmd_vel_t;

#define MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN 12
#define MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN 12
#define MAVLINK_MSG_ID_42004_LEN 12
#define MAVLINK_MSG_ID_42004_MIN_LEN 12

#define MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC 133
#define MAVLINK_MSG_ID_42004_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_CMD_VEL { \
    42004, \
    "BAMBOO_CMD_VEL", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_bamboo_cmd_vel_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bamboo_cmd_vel_t, vy) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bamboo_cmd_vel_t, wz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_CMD_VEL { \
    "BAMBOO_CMD_VEL", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_bamboo_cmd_vel_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bamboo_cmd_vel_t, vy) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bamboo_cmd_vel_t, wz) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_cmd_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] Vitesse lineaire avant demandee.
 * @param vy [m/s] Vitesse laterale demandee (ignoree hors mecanum).
 * @param wz [rad/s] Vitesse angulaire de lacet demandee.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_cmd_vel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#else
    mavlink_bamboo_cmd_vel_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_CMD_VEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
}

/**
 * @brief Pack a bamboo_cmd_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] Vitesse lineaire avant demandee.
 * @param vy [m/s] Vitesse laterale demandee (ignoree hors mecanum).
 * @param wz [rad/s] Vitesse angulaire de lacet demandee.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_cmd_vel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#else
    mavlink_bamboo_cmd_vel_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_CMD_VEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#endif
}

/**
 * @brief Pack a bamboo_cmd_vel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx [m/s] Vitesse lineaire avant demandee.
 * @param vy [m/s] Vitesse laterale demandee (ignoree hors mecanum).
 * @param wz [rad/s] Vitesse angulaire de lacet demandee.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_cmd_vel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx,float vy,float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, wz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#else
    mavlink_bamboo_cmd_vel_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_CMD_VEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
}

/**
 * @brief Encode a bamboo_cmd_vel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_cmd_vel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_cmd_vel_t* bamboo_cmd_vel)
{
    return mavlink_msg_bamboo_cmd_vel_pack(system_id, component_id, msg, bamboo_cmd_vel->vx, bamboo_cmd_vel->vy, bamboo_cmd_vel->wz);
}

/**
 * @brief Encode a bamboo_cmd_vel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_cmd_vel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_cmd_vel_t* bamboo_cmd_vel)
{
    return mavlink_msg_bamboo_cmd_vel_pack_chan(system_id, component_id, chan, msg, bamboo_cmd_vel->vx, bamboo_cmd_vel->vy, bamboo_cmd_vel->wz);
}

/**
 * @brief Encode a bamboo_cmd_vel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_cmd_vel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_cmd_vel_t* bamboo_cmd_vel)
{
    return mavlink_msg_bamboo_cmd_vel_pack_status(system_id, component_id, _status, msg,  bamboo_cmd_vel->vx, bamboo_cmd_vel->vy, bamboo_cmd_vel->wz);
}

/**
 * @brief Send a bamboo_cmd_vel message
 * @param chan MAVLink channel to send the message
 *
 * @param vx [m/s] Vitesse lineaire avant demandee.
 * @param vy [m/s] Vitesse laterale demandee (ignoree hors mecanum).
 * @param wz [rad/s] Vitesse angulaire de lacet demandee.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_cmd_vel_send(mavlink_channel_t chan, float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, wz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_CMD_VEL, buf, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
#else
    mavlink_bamboo_cmd_vel_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_CMD_VEL, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
#endif
}

/**
 * @brief Send a bamboo_cmd_vel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_cmd_vel_send_struct(mavlink_channel_t chan, const mavlink_bamboo_cmd_vel_t* bamboo_cmd_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_cmd_vel_send(chan, bamboo_cmd_vel->vx, bamboo_cmd_vel->vy, bamboo_cmd_vel->wz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_CMD_VEL, (const char *)bamboo_cmd_vel, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_cmd_vel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx, float vy, float wz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, wz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_CMD_VEL, buf, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
#else
    mavlink_bamboo_cmd_vel_t *packet = (mavlink_bamboo_cmd_vel_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;
    packet->wz = wz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_CMD_VEL, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_CMD_VEL UNPACKING


/**
 * @brief Get field vx from bamboo_cmd_vel message
 *
 * @return [m/s] Vitesse lineaire avant demandee.
 */
static inline float mavlink_msg_bamboo_cmd_vel_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from bamboo_cmd_vel message
 *
 * @return [m/s] Vitesse laterale demandee (ignoree hors mecanum).
 */
static inline float mavlink_msg_bamboo_cmd_vel_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field wz from bamboo_cmd_vel message
 *
 * @return [rad/s] Vitesse angulaire de lacet demandee.
 */
static inline float mavlink_msg_bamboo_cmd_vel_get_wz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a bamboo_cmd_vel message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_cmd_vel C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_cmd_vel_decode(const mavlink_message_t* msg, mavlink_bamboo_cmd_vel_t* bamboo_cmd_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bamboo_cmd_vel->vx = mavlink_msg_bamboo_cmd_vel_get_vx(msg);
    bamboo_cmd_vel->vy = mavlink_msg_bamboo_cmd_vel_get_vy(msg);
    bamboo_cmd_vel->wz = mavlink_msg_bamboo_cmd_vel_get_wz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN;
        memset(bamboo_cmd_vel, 0, MAVLINK_MSG_ID_BAMBOO_CMD_VEL_LEN);
    memcpy(bamboo_cmd_vel, _MAV_PAYLOAD(msg), len);
#endif
}
