#include "chassis_bridge/chassis_protocol.hpp"

#include <cstring>

namespace chassis_bridge
{
namespace
{

void append_u16_le(const uint16_t value, std::vector<uint8_t> & data)
{
  data.push_back(static_cast<uint8_t>(value & 0xFFU));
  data.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

uint16_t read_u16_le(const std::vector<uint8_t> & data, const size_t offset)
{
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8U);
}

void append_float_le(const float value, std::vector<uint8_t> & data)
{
  uint32_t raw{0};
  std::memcpy(&raw, &value, sizeof(raw));
  data.push_back(static_cast<uint8_t>(raw & 0xFFU));
  data.push_back(static_cast<uint8_t>((raw >> 8U) & 0xFFU));
  data.push_back(static_cast<uint8_t>((raw >> 16U) & 0xFFU));
  data.push_back(static_cast<uint8_t>((raw >> 24U) & 0xFFU));
}

bool read_float_le(const std::vector<uint8_t> & data, const size_t offset, float & value)
{
  if (offset + sizeof(float) > data.size()) {
    return false;
  }

  const uint32_t raw = static_cast<uint32_t>(data[offset]) |
    (static_cast<uint32_t>(data[offset + 1]) << 8U) |
    (static_cast<uint32_t>(data[offset + 2]) << 16U) |
    (static_cast<uint32_t>(data[offset + 3]) << 24U);
  std::memcpy(&value, &raw, sizeof(value));
  return true;
}

std::vector<uint8_t> encode_cmd_vel_payload(const ChassisProtocol::CmdVelPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(8);
  append_float_le(payload.vx_mps, data);
  append_float_le(payload.wz_radps, data);
  return data;
}

std::vector<uint8_t> encode_odom_payload(const ChassisProtocol::OdomFeedbackPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(28);
  append_float_le(payload.x_m, data);
  append_float_le(payload.y_m, data);
  append_float_le(payload.theta_rad, data);
  append_float_le(payload.vx_mps, data);
  append_float_le(payload.wz_radps, data);
  append_float_le(payload.left_wheel_speed_mps, data);
  append_float_le(payload.right_wheel_speed_mps, data);
  return data;
}

}  // namespace

std::vector<uint8_t> ChassisProtocol::encode_frame(
  const MessageType type, const uint16_t seq, const std::vector<uint8_t> & payload)
{
  if (payload.size() > kMaxPayloadSize) {
    return {};
  }

  std::vector<uint8_t> frame;
  frame.reserve(kHeaderSize + payload.size() + kCrcSize);
  append_u16_le(kMagic, frame);
  frame.push_back(kProtocolVersion);
  frame.push_back(static_cast<uint8_t>(type));
  append_u16_le(seq, frame);
  append_u16_le(static_cast<uint16_t>(payload.size()), frame);
  frame.insert(frame.end(), payload.begin(), payload.end());
  append_u16_le(crc16_ccitt(frame), frame);
  return frame;
}

bool ChassisProtocol::decode_frame(const std::vector<uint8_t> & frame, DecodedFrame & decoded)
{
  if (frame.size() < kHeaderSize + kCrcSize) {
    return false;
  }
  if (read_u16_le(frame, 0) != kMagic || frame[2] != kProtocolVersion) {
    return false;
  }

  const auto payload_len = static_cast<size_t>(read_u16_le(frame, 6));
  if (payload_len > kMaxPayloadSize || frame.size() != kHeaderSize + payload_len + kCrcSize) {
    return false;
  }

  std::vector<uint8_t> crc_data(frame.begin(), frame.end() - static_cast<long>(kCrcSize));
  const uint16_t expected_crc = crc16_ccitt(crc_data);
  const uint16_t actual_crc = read_u16_le(frame, frame.size() - kCrcSize);
  if (expected_crc != actual_crc) {
    return false;
  }

  const uint8_t raw_type = frame[3];
  if (raw_type != static_cast<uint8_t>(MessageType::kCmdVel) &&
    raw_type != static_cast<uint8_t>(MessageType::kOdomFeedback) &&
    raw_type != static_cast<uint8_t>(MessageType::kHeartbeat))
  {
    return false;
  }

  decoded.type = static_cast<MessageType>(raw_type);
  decoded.seq = read_u16_le(frame, 4);
  decoded.payload.assign(frame.begin() + static_cast<long>(kHeaderSize), frame.end() - 2);
  return true;
}

std::vector<uint8_t> ChassisProtocol::encode_cmd_vel(
  const uint16_t seq, const CmdVelPayload & payload)
{
  return encode_frame(MessageType::kCmdVel, seq, encode_cmd_vel_payload(payload));
}

std::vector<uint8_t> ChassisProtocol::encode_odom_feedback(
  const uint16_t seq, const OdomFeedbackPayload & payload)
{
  return encode_frame(MessageType::kOdomFeedback, seq, encode_odom_payload(payload));
}

bool ChassisProtocol::decode_cmd_vel_payload(
  const std::vector<uint8_t> & payload, CmdVelPayload & decoded)
{
  if (payload.size() != 8) {
    return false;
  }
  return read_float_le(payload, 0, decoded.vx_mps) &&
         read_float_le(payload, 4, decoded.wz_radps);
}

bool ChassisProtocol::decode_odom_feedback_payload(
  const std::vector<uint8_t> & payload, OdomFeedbackPayload & decoded)
{
  if (payload.size() != 28) {
    return false;
  }
  return read_float_le(payload, 0, decoded.x_m) &&
         read_float_le(payload, 4, decoded.y_m) &&
         read_float_le(payload, 8, decoded.theta_rad) &&
         read_float_le(payload, 12, decoded.vx_mps) &&
         read_float_le(payload, 16, decoded.wz_radps) &&
         read_float_le(payload, 20, decoded.left_wheel_speed_mps) &&
         read_float_le(payload, 24, decoded.right_wheel_speed_mps);
}

uint16_t ChassisProtocol::crc16_ccitt(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;
  for (const uint8_t byte : data) {
    crc ^= static_cast<uint16_t>(byte) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1U);
      }
    }
  }
  return crc;
}

}  // namespace chassis_bridge
