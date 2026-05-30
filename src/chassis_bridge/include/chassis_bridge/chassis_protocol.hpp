#ifndef CHASSIS_BRIDGE__CHASSIS_PROTOCOL_HPP_
#define CHASSIS_BRIDGE__CHASSIS_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace chassis_bridge
{

/// @brief Jetson 与 ESP32 底盘通信协议的二进制帧编解码工具。
class ChassisProtocol
{
public:
  static constexpr uint8_t kProtocolVersion = 1;
  static constexpr uint16_t kMagic = 0xAA55;
  static constexpr size_t kHeaderSize = 8;
  static constexpr size_t kCrcSize = 2;
  static constexpr size_t kMaxPayloadSize = 128;

  /// @brief 协议消息类型。
  enum class MessageType : uint8_t
  {
    kCmdVel = 0x01,
    kOdomFeedback = 0x02,
    kHeartbeat = 0x03,
  };

  /// @brief 速度指令 payload。
  struct CmdVelPayload
  {
    float vx_mps{0.0F};
    float wz_radps{0.0F};
  };

  /// @brief 里程计反馈 payload。
  struct OdomFeedbackPayload
  {
    float x_m{0.0F};
    float y_m{0.0F};
    float theta_rad{0.0F};
    float vx_mps{0.0F};
    float wz_radps{0.0F};
    float left_wheel_speed_mps{0.0F};
    float right_wheel_speed_mps{0.0F};
  };

  /// @brief 解码后的通用协议帧。
  struct DecodedFrame
  {
    MessageType type{MessageType::kHeartbeat};
    uint16_t seq{0};
    std::vector<uint8_t> payload;
  };

  /// @brief 编码通用协议帧。
  /// @param type 消息类型。
  /// @param seq 帧序号。
  /// @param payload payload 字节。
  /// @return 完整帧字节；payload 过长时返回空 vector。
  static std::vector<uint8_t> encode_frame(
    MessageType type, uint16_t seq, const std::vector<uint8_t> & payload);

  /// @brief 解码通用协议帧并校验 magic、版本、长度和 CRC。
  /// @param frame 完整帧字节。
  /// @param decoded 解码输出。
  /// @return 解码成功返回 true，否则返回 false。
  static bool decode_frame(const std::vector<uint8_t> & frame, DecodedFrame & decoded);

  /// @brief 编码速度指令帧。
  /// @param seq 帧序号。
  /// @param payload 速度指令 payload。
  /// @return 完整帧字节。
  static std::vector<uint8_t> encode_cmd_vel(uint16_t seq, const CmdVelPayload & payload);

  /// @brief 编码里程计反馈帧。
  /// @param seq 帧序号。
  /// @param payload 里程计反馈 payload。
  /// @return 完整帧字节。
  static std::vector<uint8_t> encode_odom_feedback(
    uint16_t seq, const OdomFeedbackPayload & payload);

  /// @brief 从 payload 字节解码速度指令。
  /// @param payload payload 字节。
  /// @param decoded 解码输出。
  /// @return 解码成功返回 true，否则返回 false。
  static bool decode_cmd_vel_payload(
    const std::vector<uint8_t> & payload, CmdVelPayload & decoded);

  /// @brief 从 payload 字节解码里程计反馈。
  /// @param payload payload 字节。
  /// @param decoded 解码输出。
  /// @return 解码成功返回 true，否则返回 false。
  static bool decode_odom_feedback_payload(
    const std::vector<uint8_t> & payload, OdomFeedbackPayload & decoded);

  /// @brief 计算 CRC16-CCITT，覆盖 header 和 payload。
  /// @param data 待计算数据。
  /// @return CRC16 值。
  static uint16_t crc16_ccitt(const std::vector<uint8_t> & data);
};

}  // namespace chassis_bridge

#endif  // CHASSIS_BRIDGE__CHASSIS_PROTOCOL_HPP_
