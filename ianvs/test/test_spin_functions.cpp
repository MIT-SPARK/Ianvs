#include <gtest/gtest.h>
#include <ianvs/spin_functions.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace ianvs {
namespace {

using namespace std::chrono_literals;

enum class SpinMode { UntilExitRequested, WaitingForClock, ClockPresent };

class SpinShutdownTest : public testing::TestWithParam<SpinMode> {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("spin_shutdown_test");
    if (GetParam() == SpinMode::ClockPresent) {
      clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    }

    ASSERT_EQ(node_->count_publishers("/clock"), clock_ ? 1u : 0u);
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
};

TEST_P(SpinShutdownTest, ShutdownDuringCallbackReturnsNormally) {
  NodeHandle nh(*node_);
  auto callback_ran = false;
  auto timer = node_->create_wall_timer(1ms, [&]() {
    callback_ran = true;
    rclcpp::shutdown();
  });

  EXPECT_NO_THROW(spinAndWait(nh, GetParam() != SpinMode::UntilExitRequested));
  EXPECT_TRUE(callback_ran);
  EXPECT_FALSE(rclcpp::ok());
}

class ShutdownBeforeSpinExecutor : public rclcpp::executors::SingleThreadedExecutor {
 public:
  using SingleThreadedExecutor::spin_node_some;

  void spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) override {
    rclcpp::shutdown();
    SingleThreadedExecutor::spin_node_some(node);
  }
};

TEST_P(SpinShutdownTest, ShutdownBeforeSpinReturnsNormally) {
  NodeHandle nh(*node_);
  ShutdownBeforeSpinExecutor executor;
  EXPECT_NO_THROW(spinAndWait(nh, GetParam() != SpinMode::UntilExitRequested, &executor));
  EXPECT_FALSE(rclcpp::ok());
}

INSTANTIATE_TEST_SUITE_P(SpinFunctions,
                         SpinShutdownTest,
                         testing::Values(SpinMode::UntilExitRequested,
                                         SpinMode::WaitingForClock,
                                         SpinMode::ClockPresent));

}  // namespace
}  // namespace ianvs
