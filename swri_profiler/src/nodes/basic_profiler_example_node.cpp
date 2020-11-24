#ifdef ROS2_BUILD
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;

#else
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#endif
#include <swri_profiler/profiler.h>

class BasicProfilerExampleNode
{
#ifdef ROS2_BUILD
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<rclcpp::TimerBase> init_timer_;
  std::shared_ptr<rclcpp::TimerBase> update_timer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_callback_;
#else
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  ros::WallTimer init_timer_;
  ros::Timer update_timer_;

  ros::Subscriber int_callback_;
#endif

  int fibonacci_index1_;
  int fibonacci_index2_;
  
 public:
  BasicProfilerExampleNode()
  {
#ifdef ROS2_BUILD
    nh_ = std::make_shared<rclcpp::Node>("basic_profiler_example_node");
#else
    pnh_ = ros::NodeHandle("~");
#endif
    
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
#ifdef ROS2_BUILD
    RCLCPP_INFO(nh_->get_logger(), "Starting initialization timer...");
    initialize();
#else
    ROS_INFO("Starting initialization timer...");
    init_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),
                                      &BasicProfilerExampleNode::initialize,
                                      this,                                      
                                      true);
#endif
  }

#ifdef ROS2_BUILD
  std::shared_ptr<rclcpp::Node> get_node()
  {
    return nh_;
  }
#endif

#ifdef ROS2_BUILD
  void initialize()
  {
    nh_->declare_parameter("fibonacci_index1", 30);
    nh_->declare_parameter("fibonacci_index2", 40);
    fibonacci_index1_ = nh_->get_parameter("fibonacci_index1").as_int();
    fibonacci_index2_ = nh_->get_parameter("fibonacci_index2").as_int();

    update_timer_ = nh_->create_wall_timer(100ms,
                                           std::bind(&BasicProfilerExampleNode::handleUpdateTimer, this));

    int_callback_ = nh_->create_subscription<std_msgs::msg::Int32>(
      "trigger_fibonacci",
      10,
      [this] (std_msgs::msg::Int32::SharedPtr msg) {
        handleTriggerFibonacci(msg);
      });
      //std::bind(&BasicProfilerExampleNode::handleTriggerFibonacci, this, _1));
  }
#else
  void initialize(const ros::WallTimerEvent &/*ignored*/)
  {
    pnh_.param("fibonacci_index1", fibonacci_index1_, 30);
    pnh_.param("fibonacci_index2", fibonacci_index2_, 40);

    update_timer_ = nh_.createTimer(ros::Duration(1/10.0),
                                    &BasicProfilerExampleNode::handleUpdateTimer,
                                    this);

    int_callback_ = nh_.subscribe("trigger_fibonacci", 10,
                                  &BasicProfilerExampleNode::handleTriggerFibonacci,
                                  this);
  }
#endif
  
  int superSlowFibonacciInt(int x)
  {
    if (x <= 0) {
      return 0;
    } else if (x == 1) {
      return 1;
    } else {
      return superSlowFibonacciInt(x-1) + superSlowFibonacciInt(x-2);
    }
  }

  void superSlowFibonacci(int x)
  {
    SWRI_PROFILE("super-slow-fibonacci");
#ifdef ROS2_BUILD
    RCLCPP_INFO(nh_->get_logger(),
#else
    ROS_INFO(
#endif
      "Fibonacci %d = %d", x, superSlowFibonacciInt(x));
  }

#ifdef ROS2_BUILD
  void handleUpdateTimer()
#else
  void handleUpdateTimer(const ros::TimerEvent &/*ignored*/)
#endif
  {
    SWRI_PROFILE("handle-update-timer");

    {
      SWRI_PROFILE("fibonacci-1");
      superSlowFibonacci(fibonacci_index1_);
    }

    {
      SWRI_PROFILE("fibonacci-2");
      superSlowFibonacci(fibonacci_index2_);
    }
  }

  void handleTriggerFibonacci(
#ifdef ROS2_BUILD
    std_msgs::msg::Int32::SharedPtr msg
#else
    const std_msgs::Int32ConstPtr &msg
#endif
    )
  {
    SWRI_PROFILE("handle-trigger-fibonacci");
    superSlowFibonacci(msg->data);
  }
};

int main(int argc, char **argv)
{
#ifdef ROS2_BUILD
  rclcpp::init(argc, argv);
#else
  ros::init(argc, argv, "basic_profiler_example");
#endif

  BasicProfilerExampleNode node;

#ifdef ROS2_BUILD
  rclcpp::spin(node.get_node());
#else
  ros::spin();
#endif
  
  return 0;  
}

