// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include <functional>
#include <string>

#include "gtest/gtest.h"
#include "omnibot_hardware/calibration.hpp"
#include "omnibot_hardware/feetech_protocol.hpp"
#include "omnibot_hardware/hardware_buses.hpp"
#include "omnibot_hardware/kinematics.hpp"
#include "omnibot_hardware/serial_port.hpp"
#include "omnibot_hardware/tick_math.hpp"
#include "omnibot_hardware/yahboom_protocol.hpp"

namespace
{

std::string hex_of(const std::vector<uint8_t> & bytes)
{
  static const char * digits = "0123456789abcdef";
  std::string out;
  out.reserve(bytes.size() * 2);
  for (const auto byte : bytes) {
    out.push_back(digits[byte >> 4]);
    out.push_back(digits[byte & 0x0F]);
  }
  return out;
}

std::vector<uint8_t> from_hex(const std::string & text)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < text.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoi(text.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::vector<uint8_t> status_with_position(uint8_t id, uint16_t ticks)
{
  const std::vector<uint8_t> params = {
    static_cast<uint8_t>(ticks & 0xFF),
    static_cast<uint8_t>((ticks >> 8) & 0xFF),
    0,
    0,
  };
  const uint8_t len = static_cast<uint8_t>(params.size() + 2);
  unsigned sum = static_cast<unsigned>(id) + len;
  for (const auto byte : params) {
    sum += byte;
  }
  std::vector<uint8_t> packet{0xFF, 0xFF, id, len, 0};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(static_cast<uint8_t>(~sum));
  return packet;
}

class FakeSerial : public omnibot_hardware::SerialPort
{
public:
  std::vector<std::vector<uint8_t>> written;
  std::function<void(const std::vector<uint8_t> &)> on_write;

  bool open_port(const std::string & path, int, std::string & error) override
  {
    if (path.find("/dev") != std::string::npos || path.find("COM") != std::string::npos) {
      error = "fake serial refuses a device path";
      return false;
    }
    open_ = true;
    return true;
  }

  void close_port() override { open_ = false; }
  bool is_open() const override { return open_; }

  bool write_bytes(const std::vector<uint8_t> & data, std::string &) override
  {
    written.push_back(data);
    if (on_write) {
      on_write(data);
    }
    return true;
  }

  std::vector<uint8_t> read_available() override
  {
    auto out = rx_;
    rx_.clear();
    return out;
  }

  void push(const std::vector<uint8_t> & bytes) { rx_.insert(rx_.end(), bytes.begin(), bytes.end()); }

private:
  bool open_{false};
  std::vector<uint8_t> rx_;
};

void answer_reads(FakeSerial & port)
{
  port.on_write = [&port](const std::vector<uint8_t> & packet) {
    if (packet.size() >= 5 && packet[0] == 0xFF && packet[1] == 0xFF && packet[4] == 0x02) {
      port.push(status_with_position(packet[2], 2048));
    }
  };
}

omnibot_hardware::BusConfig test_config()
{
  omnibot_hardware::BusConfig config;
  config.pace_car_type = false;
  return config;
}

}  // namespace

TEST(YahboomProtocol, MotionAndCarTypeMatchThePythonFrames)
{
  EXPECT_EQ(
    hex_of(omnibot_hardware::yahboom::packet_motion(0.1, -0.2, 0.3)),
    "fffc0a1201640038ff2c01e5");
  EXPECT_EQ(hex_of(omnibot_hardware::yahboom::packet_motion(0.0, 0.0, 0.0)), "fffc0a12010000000000001d");
  EXPECT_EQ(hex_of(omnibot_hardware::yahboom::packet_set_car_type()), "fffc0415011a");
  EXPECT_EQ(omnibot_hardware::yahboom::packet_motion(0.1, 0.0, 0.0)[3], omnibot_hardware::yahboom::kFuncMotion);
}

TEST(YahboomProtocol, VelocityReportAndABadChecksum)
{
  auto good = from_hex("fffb090cfa0083ff3200c3");
  const auto parsed = omnibot_hardware::yahboom::consume_velocity(good);
  ASSERT_EQ(parsed.size(), 1u);
  EXPECT_NEAR(parsed[0].vx, 0.25, 1e-9);
  EXPECT_NEAR(parsed[0].vy, -0.125, 1e-9);
  EXPECT_NEAR(parsed[0].wz, 0.05, 1e-9);
  EXPECT_TRUE(good.empty());

  auto bad = from_hex("fffb090cfa0083ff3200c4");
  EXPECT_TRUE(omnibot_hardware::yahboom::consume_velocity(bad).empty());
}

TEST(FeetechProtocol, TorqueGoalAndReadMatchTheStsFrames)
{
  EXPECT_EQ(hex_of(omnibot_hardware::feetech::write_torque(1, true)), "ffff0104032801ce");
  EXPECT_EQ(hex_of(omnibot_hardware::feetech::write_torque(1, false)), "ffff0104032800cf");
  EXPECT_EQ(hex_of(omnibot_hardware::feetech::write_goal(1, 2048)), "ffff0105032a0008c4");
  // Two-byte Present_Position read. The bus asks for four bytes so speed is included.
  EXPECT_EQ(
    hex_of(omnibot_hardware::feetech::build(1, omnibot_hardware::feetech::kInstRead, {56, 2})),
    "ffff0104023802be");
  EXPECT_EQ(hex_of(omnibot_hardware::feetech::read_position_speed(1)), "ffff0104023804bc");
}

TEST(FeetechProtocol, StatusPositionAndSpeed)
{
  auto raw = from_hex("ffff0104000008f2");
  const auto statuses = omnibot_hardware::feetech::consume_statuses(raw);
  ASSERT_EQ(statuses.size(), 1u);
  EXPECT_EQ(statuses[0].id, 1);
  EXPECT_EQ(statuses[0].error, 0);
  ASSERT_EQ(statuses[0].params.size(), 2u);
  EXPECT_EQ(statuses[0].params[0] | (statuses[0].params[1] << 8), 2048);

  const uint16_t magnitude = 100;
  const double rpm = 100 * 0.0146;
  EXPECT_NEAR(
    omnibot_hardware::feetech::speed_to_rad_s(magnitude),
    rpm * (2.0 * 3.14159265358979323846 / 60.0), 1e-12);
  EXPECT_LT(omnibot_hardware::feetech::speed_to_rad_s(static_cast<uint16_t>(0x8000 | magnitude)), 0.0);
}

TEST(TickMath, HalfRadianMatchesArmMath)
{
  const double per_rad = omnibot_hardware::tick_math::ticks_per_rad(4096.0);
  EXPECT_EQ(omnibot_hardware::tick_math::radians_to_ticks(0.5, 2048, per_rad), 2374);
  EXPECT_NEAR(
    omnibot_hardware::tick_math::ticks_to_radians(2374, 2048, per_rad), 0.5000777, 1e-6);
  EXPECT_EQ(omnibot_hardware::tick_math::radians_to_ticks(0.0, 2048, per_rad), 2048);
  EXPECT_EQ(omnibot_hardware::tick_math::radians_to_ticks(100.0, 2048, per_rad), 4095);
  EXPECT_EQ(omnibot_hardware::tick_math::radians_to_ticks(-100.0, 2048, per_rad), 0);
}

TEST(Kinematics, BodyAndWheelsRoundTrip)
{
  const double length = omnibot_hardware::kinematics::length_sum(0.165, 0.215);
  EXPECT_NEAR(length, 0.19, 1e-12);
  omnibot_hardware::yahboom::BodyVelocity in;
  in.vx = 0.2;
  in.vy = -0.1;
  in.wz = 0.3;
  const auto wheels = omnibot_hardware::kinematics::body_to_wheels(in, 0.04, length);
  const auto out = omnibot_hardware::kinematics::wheels_to_body(wheels, 0.04, length);
  EXPECT_NEAR(out.vx, 0.2, 1e-9);
  EXPECT_NEAR(out.vy, -0.1, 1e-9);
  EXPECT_NEAR(out.wz, 0.3, 1e-9);
}

TEST(Calibration, FixtureHomeTicksMatchJointHomeTick)
{
  std::array<int, 6> ticks{};
  std::string error;
  ASSERT_TRUE(omnibot_hardware::calibration::load_home_ticks_file(OMNIBOT_CI_FIXTURE, ticks, error))
    << error;
  const std::array<int, 6> expected{100, 200, 300, 400, 500, 600};
  EXPECT_EQ(ticks, expected);
}

TEST(Calibration, HomeTicksKeyIsNotHomeTickAndMismatchFails)
{
  std::array<int, 6> ticks{};
  std::string error;
  const char * only = "home_ticks: [7, 8, 9, 10, 11, 12]\n";
  ASSERT_TRUE(omnibot_hardware::calibration::parse_home_ticks_yaml(only, ticks, error)) << error;
  EXPECT_EQ(ticks[0], 7);
  EXPECT_EQ(ticks[5], 12);

  const char * note =
    "home_ticks: [7, 8, 9, 10, 11, 12]\n"
    "joints:\n"
    "  - home_tick_note: 3\n"
    "    home_tick: 7\n"
    "  - home_tick: 8\n"
    "  - home_tick: 9\n"
    "  - home_tick: 10\n"
    "  - home_tick: 11\n"
    "  - home_tick: 12\n";
  ASSERT_TRUE(omnibot_hardware::calibration::parse_home_ticks_yaml(note, ticks, error)) << error;
  EXPECT_EQ(ticks[0], 7);

  const char * mismatch =
    "home_ticks: [1, 2, 3, 4, 5, 6]\n"
    "joints:\n"
    "  - home_tick: 1\n"
    "  - home_tick: 2\n"
    "  - home_tick: 3\n"
    "  - home_tick: 4\n"
    "  - home_tick: 5\n"
    "  - home_tick: 9\n";
  EXPECT_FALSE(omnibot_hardware::calibration::parse_home_ticks_yaml(mismatch, ticks, error));
  EXPECT_NE(error.find("does not match"), std::string::npos);
}

TEST(Serial, FakePortNeverOpensADevice)
{
  FakeSerial port;
  std::string error;
  EXPECT_FALSE(port.open_port("/dev/ttyUSB0", 115200, error));
  EXPECT_FALSE(port.is_open());
  EXPECT_TRUE(port.open_port("fake-bus", 115200, error)) << error;
}

TEST(HardwareBuses, ConfigureHoldAndWheelVelocityUseTheExistingFrames)
{
  FakeSerial base;
  FakeSerial arm;
  answer_reads(arm);
  omnibot_hardware::HardwareBuses buses(base, arm, test_config());
  std::string error;
  ASSERT_TRUE(buses.configure(error)) << error;
  ASSERT_EQ(base.written.size(), 5u);
  for (const auto & packet : base.written) {
    EXPECT_EQ(packet, omnibot_hardware::yahboom::packet_set_car_type());
  }

  ASSERT_TRUE(buses.capture_arm_hold(error)) << error;
  EXPECT_EQ(buses.hold_ticks()[0], 2048);
  EXPECT_NEAR(buses.arm_position()[0], 0.0, 1e-9);
  bool saw_goal = false;
  bool saw_torque = false;
  for (const auto & packet : arm.written) {
    if (packet == omnibot_hardware::feetech::write_goal(1, 2048)) {
      saw_goal = true;
    }
    if (packet == omnibot_hardware::feetech::write_torque(1, true)) {
      saw_torque = true;
    }
  }
  EXPECT_TRUE(saw_goal);
  EXPECT_TRUE(saw_torque);
  EXPECT_TRUE(buses.torque_on());

  base.written.clear();
  std::array<double, 4> wheels{10.0, 10.0, 10.0, 10.0};
  ASSERT_TRUE(buses.write_wheel_velocities(wheels, error)) << error;
  ASSERT_EQ(base.written.size(), 1u);
  EXPECT_EQ(base.written[0], omnibot_hardware::yahboom::packet_motion(0.4, 0.0, 0.0));
}

TEST(HardwareBuses, FailedPositionReadDoesNotCommandZero)
{
  FakeSerial base;
  FakeSerial arm;
  omnibot_hardware::HardwareBuses buses(base, arm, test_config());
  std::string error;
  EXPECT_FALSE(buses.capture_arm_hold(error));
  for (const auto & packet : arm.written) {
    EXPECT_NE(packet, omnibot_hardware::feetech::write_goal(1, 0));
    EXPECT_NE(packet[4], omnibot_hardware::feetech::kInstWrite);
  }
}

TEST(HardwareBuses, VelocityReportBecomesEncoderTicksAndStopDropsTorque)
{
  FakeSerial base;
  FakeSerial arm;
  omnibot_hardware::HardwareBuses buses(base, arm, test_config());
  std::string error;
  base.push(from_hex("fffb090cfa0083ff320000"));
  buses.poll_base(0.1);
  EXPECT_NEAR(buses.wheel_position()[0], 0.0, 1e-12);

  base.push(from_hex("fffb090cfa0083ff3200c3"));
  buses.poll_base(0.1);
  const auto expected = omnibot_hardware::kinematics::body_to_wheels(
    [] {
      omnibot_hardware::yahboom::BodyVelocity body;
      body.vx = 0.25;
      body.vy = -0.125;
      body.wz = 0.05;
      return body;
    }(),
    0.04, 0.19);
  EXPECT_NEAR(buses.wheel_velocity()[0], expected[0], 1e-9);
  EXPECT_NEAR(buses.wheel_position()[0], expected[0] * 0.1, 1e-9);

  answer_reads(arm);
  ASSERT_TRUE(buses.capture_arm_hold(error)) << error;
  base.written.clear();
  arm.written.clear();
  ASSERT_TRUE(buses.stop(error)) << error;
  EXPECT_EQ(base.written.at(0), omnibot_hardware::yahboom::packet_motion(0.0, 0.0, 0.0));
  EXPECT_FALSE(buses.torque_on());
  EXPECT_EQ(arm.written.at(0), omnibot_hardware::feetech::write_torque(1, false));
}
