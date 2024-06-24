// test_sample.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class SomethingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    _node = rclcpp::Node::make_shared("test_node");
    RCLCPP_INFO(_node->get_logger(), "SetUp %x, %x", _node.get(), this);
    x_ = 5;
  }

  int x_;
  rclcpp::Node::SharedPtr _node;
};

TEST_F(SomethingTest, Add)
{
  EXPECT_EQ(10, x_ + 5);
}

TEST_F(SomethingTest, Add2)
{
  EXPECT_EQ(10, x_ + 6);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}