// MESSAGE GLOBAL_POS_ATT_NED PACKING

#define MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED 239

typedef struct __mavlink_global_pos_att_ned_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t lat; ///< Latitude, expressed as * 1E7
 int32_t lon; ///< Longitude, expressed as * 1E7
 int32_t alt; ///< MSL Altitude in meters, expressed as m * 100 (cm), 
 int32_t relative_alt; ///< Fused relative altitude, expressed as meters * 100.
 float quat[4]; ///< Quaternion components: x, y, z, w (0 0 0 1) is the null-rotation)
 float rollspeed; ///< Roll angular speed (rad/s)
 float pitchspeed; ///< Pitch angular speed (rad/s)
 float yawspeed; ///< Yaw angular speed (rad/s)
 int16_t vx; ///< Fused North velocity, expressed as m/s * 100
 int16_t vy; ///< Fused East velocity, expressed as m/s * 100
 int16_t vz; ///< Fused Down velocity, expressed as m/s * 100
} mavlink_global_pos_att_ned_t;

#define MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN 54
#define MAVLINK_MSG_ID_239_LEN 54

#define MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC 6
#define MAVLINK_MSG_ID_239_CRC 6

#define MAVLINK_MSG_GLOBAL_POS_ATT_NED_FIELD_QUAT_LEN 4

#define MAVLINK_MESSAGE_INFO_GLOBAL_POS_ATT_NED { \
	"GLOBAL_POS_ATT_NED", \
	12, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_pos_att_ned_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_pos_att_ned_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_pos_att_ned_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_pos_att_ned_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_pos_att_ned_t, relative_alt) }, \
         { "quat", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_global_pos_att_ned_t, quat) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_global_pos_att_ned_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_global_pos_att_ned_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_global_pos_att_ned_t, yawspeed) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_global_pos_att_ned_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_global_pos_att_ned_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_global_pos_att_ned_t, vz) }, \
         } \
}


/**
 * @brief Pack a global_pos_att_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt MSL Altitude in meters, expressed as m * 100 (cm), 
 * @param relative_alt Fused relative altitude, expressed as meters * 100.
 * @param vx Fused North velocity, expressed as m/s * 100
 * @param vy Fused East velocity, expressed as m/s * 100
 * @param vz Fused Down velocity, expressed as m/s * 100
 * @param quat Quaternion components: x, y, z, w (0 0 0 1) is the null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_pos_att_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, const float *quat, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 36, rollspeed);
	_mav_put_float(buf, 40, pitchspeed);
	_mav_put_float(buf, 44, yawspeed);
	_mav_put_int16_t(buf, 48, vx);
	_mav_put_int16_t(buf, 50, vy);
	_mav_put_int16_t(buf, 52, vz);
	_mav_put_float_array(buf, 20, quat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#else
	mavlink_global_pos_att_ned_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	mav_array_memcpy(packet.quat, quat, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
}

/**
 * @brief Pack a global_pos_att_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt MSL Altitude in meters, expressed as m * 100 (cm), 
 * @param relative_alt Fused relative altitude, expressed as meters * 100.
 * @param vx Fused North velocity, expressed as m/s * 100
 * @param vy Fused East velocity, expressed as m/s * 100
 * @param vz Fused Down velocity, expressed as m/s * 100
 * @param quat Quaternion components: x, y, z, w (0 0 0 1) is the null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_pos_att_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,const float *quat,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 36, rollspeed);
	_mav_put_float(buf, 40, pitchspeed);
	_mav_put_float(buf, 44, yawspeed);
	_mav_put_int16_t(buf, 48, vx);
	_mav_put_int16_t(buf, 50, vy);
	_mav_put_int16_t(buf, 52, vz);
	_mav_put_float_array(buf, 20, quat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#else
	mavlink_global_pos_att_ned_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	mav_array_memcpy(packet.quat, quat, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
}

/**
 * @brief Encode a global_pos_att_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_pos_att_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_pos_att_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_pos_att_ned_t* global_pos_att_ned)
{
	return mavlink_msg_global_pos_att_ned_pack(system_id, component_id, msg, global_pos_att_ned->time_boot_ms, global_pos_att_ned->lat, global_pos_att_ned->lon, global_pos_att_ned->alt, global_pos_att_ned->relative_alt, global_pos_att_ned->vx, global_pos_att_ned->vy, global_pos_att_ned->vz, global_pos_att_ned->quat, global_pos_att_ned->rollspeed, global_pos_att_ned->pitchspeed, global_pos_att_ned->yawspeed);
}

/**
 * @brief Encode a global_pos_att_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_pos_att_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_pos_att_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_pos_att_ned_t* global_pos_att_ned)
{
	return mavlink_msg_global_pos_att_ned_pack_chan(system_id, component_id, chan, msg, global_pos_att_ned->time_boot_ms, global_pos_att_ned->lat, global_pos_att_ned->lon, global_pos_att_ned->alt, global_pos_att_ned->relative_alt, global_pos_att_ned->vx, global_pos_att_ned->vy, global_pos_att_ned->vz, global_pos_att_ned->quat, global_pos_att_ned->rollspeed, global_pos_att_ned->pitchspeed, global_pos_att_ned->yawspeed);
}

/**
 * @brief Send a global_pos_att_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt MSL Altitude in meters, expressed as m * 100 (cm), 
 * @param relative_alt Fused relative altitude, expressed as meters * 100.
 * @param vx Fused North velocity, expressed as m/s * 100
 * @param vy Fused East velocity, expressed as m/s * 100
 * @param vz Fused Down velocity, expressed as m/s * 100
 * @param quat Quaternion components: x, y, z, w (0 0 0 1) is the null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_pos_att_ned_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, const float *quat, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 36, rollspeed);
	_mav_put_float(buf, 40, pitchspeed);
	_mav_put_float(buf, 44, yawspeed);
	_mav_put_int16_t(buf, 48, vx);
	_mav_put_int16_t(buf, 50, vy);
	_mav_put_int16_t(buf, 52, vz);
	_mav_put_float_array(buf, 20, quat, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, buf, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, buf, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
#else
	mavlink_global_pos_att_ned_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	mav_array_memcpy(packet.quat, quat, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_pos_att_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, const float *quat, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 36, rollspeed);
	_mav_put_float(buf, 40, pitchspeed);
	_mav_put_float(buf, 44, yawspeed);
	_mav_put_int16_t(buf, 48, vx);
	_mav_put_int16_t(buf, 50, vy);
	_mav_put_int16_t(buf, 52, vz);
	_mav_put_float_array(buf, 20, quat, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, buf, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, buf, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
#else
	mavlink_global_pos_att_ned_t *packet = (mavlink_global_pos_att_ned_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->relative_alt = relative_alt;
	packet->rollspeed = rollspeed;
	packet->pitchspeed = pitchspeed;
	packet->yawspeed = yawspeed;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	mav_array_memcpy(packet->quat, quat, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POS_ATT_NED UNPACKING


/**
 * @brief Get field time_boot_ms from global_pos_att_ned message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_global_pos_att_ned_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from global_pos_att_ned message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_pos_att_ned_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from global_pos_att_ned message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_pos_att_ned_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_pos_att_ned message
 *
 * @return MSL Altitude in meters, expressed as m * 100 (cm), 
 */
static inline int32_t mavlink_msg_global_pos_att_ned_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field relative_alt from global_pos_att_ned message
 *
 * @return Fused relative altitude, expressed as meters * 100.
 */
static inline int32_t mavlink_msg_global_pos_att_ned_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vx from global_pos_att_ned message
 *
 * @return Fused North velocity, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_pos_att_ned_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field vy from global_pos_att_ned message
 *
 * @return Fused East velocity, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_pos_att_ned_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field vz from global_pos_att_ned message
 *
 * @return Fused Down velocity, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_pos_att_ned_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field quat from global_pos_att_ned message
 *
 * @return Quaternion components: x, y, z, w (0 0 0 1) is the null-rotation)
 */
static inline uint16_t mavlink_msg_global_pos_att_ned_get_quat(const mavlink_message_t* msg, float *quat)
{
	return _MAV_RETURN_float_array(msg, quat, 4,  20);
}

/**
 * @brief Get field rollspeed from global_pos_att_ned message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_global_pos_att_ned_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field pitchspeed from global_pos_att_ned message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_global_pos_att_ned_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field yawspeed from global_pos_att_ned message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_global_pos_att_ned_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a global_pos_att_ned message into a struct
 *
 * @param msg The message to decode
 * @param global_pos_att_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_pos_att_ned_decode(const mavlink_message_t* msg, mavlink_global_pos_att_ned_t* global_pos_att_ned)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_pos_att_ned->time_boot_ms = mavlink_msg_global_pos_att_ned_get_time_boot_ms(msg);
	global_pos_att_ned->lat = mavlink_msg_global_pos_att_ned_get_lat(msg);
	global_pos_att_ned->lon = mavlink_msg_global_pos_att_ned_get_lon(msg);
	global_pos_att_ned->alt = mavlink_msg_global_pos_att_ned_get_alt(msg);
	global_pos_att_ned->relative_alt = mavlink_msg_global_pos_att_ned_get_relative_alt(msg);
	mavlink_msg_global_pos_att_ned_get_quat(msg, global_pos_att_ned->quat);
	global_pos_att_ned->rollspeed = mavlink_msg_global_pos_att_ned_get_rollspeed(msg);
	global_pos_att_ned->pitchspeed = mavlink_msg_global_pos_att_ned_get_pitchspeed(msg);
	global_pos_att_ned->yawspeed = mavlink_msg_global_pos_att_ned_get_yawspeed(msg);
	global_pos_att_ned->vx = mavlink_msg_global_pos_att_ned_get_vx(msg);
	global_pos_att_ned->vy = mavlink_msg_global_pos_att_ned_get_vy(msg);
	global_pos_att_ned->vz = mavlink_msg_global_pos_att_ned_get_vz(msg);
#else
	memcpy(global_pos_att_ned, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GLOBAL_POS_ATT_NED_LEN);
#endif
}
