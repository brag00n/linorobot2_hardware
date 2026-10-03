#pragma once
// MESSAGE BAMBOO_HMI_TEXT PACKING

#define MAVLINK_MSG_ID_BAMBOO_HMI_TEXT 42007


typedef struct __mavlink_bamboo_hmi_text_t {
 char text[16]; /*<  Texte ASCII, 15 caracteres utiles (zero terminal si plus court).*/
} mavlink_bamboo_hmi_text_t;

#define MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN 16
#define MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN 16
#define MAVLINK_MSG_ID_42007_LEN 16
#define MAVLINK_MSG_ID_42007_MIN_LEN 16

#define MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC 148
#define MAVLINK_MSG_ID_42007_CRC 148

#define MAVLINK_MSG_BAMBOO_HMI_TEXT_FIELD_TEXT_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAMBOO_HMI_TEXT { \
    42007, \
    "BAMBOO_HMI_TEXT", \
    1, \
    {  { "text", NULL, MAVLINK_TYPE_CHAR, 16, 0, offsetof(mavlink_bamboo_hmi_text_t, text) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAMBOO_HMI_TEXT { \
    "BAMBOO_HMI_TEXT", \
    1, \
    {  { "text", NULL, MAVLINK_TYPE_CHAR, 16, 0, offsetof(mavlink_bamboo_hmi_text_t, text) }, \
         } \
}
#endif

/**
 * @brief Pack a bamboo_hmi_text message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param text  Texte ASCII, 15 caracteres utiles (zero terminal si plus court).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN];

    _mav_put_char_array(buf, 0, text, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#else
    mavlink_bamboo_hmi_text_t packet;

    mav_array_assign_char(packet.text, text, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_HMI_TEXT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
}

/**
 * @brief Pack a bamboo_hmi_text message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param text  Texte ASCII, 15 caracteres utiles (zero terminal si plus court).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN];

    _mav_put_char_array(buf, 0, text, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#else
    mavlink_bamboo_hmi_text_t packet;

    mav_array_memcpy(packet.text, text, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_HMI_TEXT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#endif
}

/**
 * @brief Pack a bamboo_hmi_text message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param text  Texte ASCII, 15 caracteres utiles (zero terminal si plus court).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN];

    _mav_put_char_array(buf, 0, text, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#else
    mavlink_bamboo_hmi_text_t packet;

    mav_array_assign_char(packet.text, text, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAMBOO_HMI_TEXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
}

/**
 * @brief Encode a bamboo_hmi_text struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_hmi_text C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bamboo_hmi_text_t* bamboo_hmi_text)
{
    return mavlink_msg_bamboo_hmi_text_pack(system_id, component_id, msg, bamboo_hmi_text->text);
}

/**
 * @brief Encode a bamboo_hmi_text struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_hmi_text C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bamboo_hmi_text_t* bamboo_hmi_text)
{
    return mavlink_msg_bamboo_hmi_text_pack_chan(system_id, component_id, chan, msg, bamboo_hmi_text->text);
}

/**
 * @brief Encode a bamboo_hmi_text struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param bamboo_hmi_text C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_bamboo_hmi_text_t* bamboo_hmi_text)
{
    return mavlink_msg_bamboo_hmi_text_pack_status(system_id, component_id, _status, msg,  bamboo_hmi_text->text);
}

/**
 * @brief Send a bamboo_hmi_text message
 * @param chan MAVLink channel to send the message
 *
 * @param text  Texte ASCII, 15 caracteres utiles (zero terminal si plus court).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bamboo_hmi_text_send(mavlink_channel_t chan, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN];

    _mav_put_char_array(buf, 0, text, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT, buf, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
#else
    mavlink_bamboo_hmi_text_t packet;

    mav_array_assign_char(packet.text, text, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT, (const char *)&packet, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
#endif
}

/**
 * @brief Send a bamboo_hmi_text message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bamboo_hmi_text_send_struct(mavlink_channel_t chan, const mavlink_bamboo_hmi_text_t* bamboo_hmi_text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_hmi_text_send(chan, bamboo_hmi_text->text);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT, (const char *)bamboo_hmi_text, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bamboo_hmi_text_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, text, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT, buf, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
#else
    mavlink_bamboo_hmi_text_t *packet = (mavlink_bamboo_hmi_text_t *)msgbuf;

    mav_array_assign_char(packet->text, text, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT, (const char *)packet, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_MIN_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_CRC);
#endif
}
#endif

#endif

// MESSAGE BAMBOO_HMI_TEXT UNPACKING


/**
 * @brief Get field text from bamboo_hmi_text message
 *
 * @return  Texte ASCII, 15 caracteres utiles (zero terminal si plus court).
 */
static inline uint16_t mavlink_msg_bamboo_hmi_text_get_text(const mavlink_message_t* msg, char *text)
{
    return _MAV_RETURN_char_array(msg, text, 16,  0);
}

/**
 * @brief Decode a bamboo_hmi_text message into a struct
 *
 * @param msg The message to decode
 * @param bamboo_hmi_text C-struct to decode the message contents into
 */
static inline void mavlink_msg_bamboo_hmi_text_decode(const mavlink_message_t* msg, mavlink_bamboo_hmi_text_t* bamboo_hmi_text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bamboo_hmi_text_get_text(msg, bamboo_hmi_text->text);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN? msg->len : MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN;
        memset(bamboo_hmi_text, 0, MAVLINK_MSG_ID_BAMBOO_HMI_TEXT_LEN);
    memcpy(bamboo_hmi_text, _MAV_PAYLOAD(msg), len);
#endif
}
