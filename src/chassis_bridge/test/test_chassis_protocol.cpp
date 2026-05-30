#include "chassis_bridge/chassis_protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace
{

using chassis_bridge::ChassisProtocol;

TEST(ChassisProtocolTest, EncodesAndDecodesCmdVelDataCases)
{
  const std::vector<ChassisProtocol::CmdVelPayload> test_data = {
    {0.0F, 0.0F},
    {0.25F, 1.2F},
    {-0.4F, -1.5F},
  };

  uint16_t seq = 10;
  for (const auto & input : test_data) {
    const auto frame = ChassisProtocol::encode_cmd_vel(seq, input);

    ChassisProtocol::DecodedFrame decoded_frame;
    ASSERT_TRUE(ChassisProtocol::decode_frame(frame, decoded_frame));
    EXPECT_EQ(decoded_frame.type, ChassisProtocol::MessageType::kCmdVel);
    EXPECT_EQ(decoded_frame.seq, seq);

    ChassisProtocol::CmdVelPayload decoded_payload;
    ASSERT_TRUE(ChassisProtocol::decode_cmd_vel_payload(decoded_frame.payload, decoded_payload));
    EXPECT_FLOAT_EQ(decoded_payload.vx_mps, input.vx_mps);
    EXPECT_FLOAT_EQ(decoded_payload.wz_radps, input.wz_radps);
    ++seq;
  }
}

TEST(ChassisProtocolTest, EncodesAndDecodesOdomFeedbackDataCases)
{
  const std::vector<ChassisProtocol::OdomFeedbackPayload> test_data = {
    {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},
    {1.2F, -0.3F, 0.8F, 0.25F, -0.4F, 0.2F, 0.3F},
    {-2.0F, 1.5F, -3.1F, -0.2F, 1.1F, -0.1F, 0.1F},
  };

  uint16_t seq = 100;
  for (const auto & input : test_data) {
    const auto frame = ChassisProtocol::encode_odom_feedback(seq, input);

    ChassisProtocol::DecodedFrame decoded_frame;
    ASSERT_TRUE(ChassisProtocol::decode_frame(frame, decoded_frame));
    EXPECT_EQ(decoded_frame.type, ChassisProtocol::MessageType::kOdomFeedback);
    EXPECT_EQ(decoded_frame.seq, seq);

    ChassisProtocol::OdomFeedbackPayload decoded_payload;
    ASSERT_TRUE(
      ChassisProtocol::decode_odom_feedback_payload(
        decoded_frame.payload, decoded_payload));
    EXPECT_FLOAT_EQ(decoded_payload.x_m, input.x_m);
    EXPECT_FLOAT_EQ(decoded_payload.y_m, input.y_m);
    EXPECT_FLOAT_EQ(decoded_payload.theta_rad, input.theta_rad);
    EXPECT_FLOAT_EQ(decoded_payload.vx_mps, input.vx_mps);
    EXPECT_FLOAT_EQ(decoded_payload.wz_radps, input.wz_radps);
    EXPECT_FLOAT_EQ(decoded_payload.left_wheel_speed_mps, input.left_wheel_speed_mps);
    EXPECT_FLOAT_EQ(decoded_payload.right_wheel_speed_mps, input.right_wheel_speed_mps);
    ++seq;
  }
}

TEST(ChassisProtocolTest, RejectsCorruptedCrc)
{
  auto frame = ChassisProtocol::encode_cmd_vel(1, {0.1F, 0.2F});
  ASSERT_FALSE(frame.empty());
  frame[8] ^= 0x01U;

  ChassisProtocol::DecodedFrame decoded;
  EXPECT_FALSE(ChassisProtocol::decode_frame(frame, decoded));
}

TEST(ChassisProtocolTest, RejectsInvalidPayloadSizes)
{
  ChassisProtocol::CmdVelPayload cmd;
  EXPECT_FALSE(ChassisProtocol::decode_cmd_vel_payload({1, 2, 3}, cmd));

  ChassisProtocol::OdomFeedbackPayload odom;
  EXPECT_FALSE(ChassisProtocol::decode_odom_feedback_payload(std::vector<uint8_t>(12, 0), odom));

  EXPECT_TRUE(
    ChassisProtocol::encode_frame(
      ChassisProtocol::MessageType::kHeartbeat, 1, std::vector<uint8_t>(129, 0)).empty());
}

TEST(ChassisProtocolTest, EncodesHeartbeatEmptyPayload)
{
  const auto frame = ChassisProtocol::encode_frame(
    ChassisProtocol::MessageType::kHeartbeat, 77, {});

  ChassisProtocol::DecodedFrame decoded;
  ASSERT_TRUE(ChassisProtocol::decode_frame(frame, decoded));
  EXPECT_EQ(decoded.type, ChassisProtocol::MessageType::kHeartbeat);
  EXPECT_EQ(decoded.seq, 77);
  EXPECT_TRUE(decoded.payload.empty());
}

}  // namespace
