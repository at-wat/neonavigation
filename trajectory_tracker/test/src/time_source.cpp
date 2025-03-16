#if 0
void timeSource()
{
  rclcpp::Node nh("/");
  bool use_sim_time;
  nh.param("/use_sim_time", use_sim_time, false);
  if (!use_sim_time)
    return;

  auto pub = nh.advertise<rosgraph_msgs::msg::Clock>("clock", 1);

  ros::WallRate rate(400.0);  // 400% speed
  ros::WallTime time = ros::WallTime::now();
  while (rclcpp::ok())
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock.fromNSec(time.toNSec());
    pub.publish(clock);
    rate.sleep();
    time += ros::WallDuration(0.01);
  }
}
#endif

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;

class TimeSource : public rclcpp::Node
{
public:
  TimeSource()
    : Node("sim_time_publisher")
    , sim_time_(0)
  {
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    timer_ = this->create_wall_timer(2.5ms, std::bind(&TimeSource::publish_sim_time, this));
  }

private:
  void publish_sim_time()
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(sim_time_);
    clock_publisher_->publish(clock);
    sim_time_ += rclcpp::Duration(10ms);
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time sim_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSource>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}