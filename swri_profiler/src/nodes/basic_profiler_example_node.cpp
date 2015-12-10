#include <ros/ros.h>
#include <swri_profiler/profiler.h>
#include <std_msgs/Int32.h>

class BasicProfilerExampleNode
{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  ros::WallTimer init_timer_;
  ros::Timer update_timer_;

  ros::Subscriber int_callback_;

  int fibonacci_index1_;
  int fibonacci_index2_;
  
 public:
  BasicProfilerExampleNode()
  {
    pnh_ = ros::NodeHandle("~");
    
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    ROS_INFO("Starting initialization timer...");
    init_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),
                                      &BasicProfilerExampleNode::initialize,
                                      this,                                      
                                      true);    
  }

  void initialize(const ros::WallTimerEvent &ignored)
  {
    (void)ignored;

    pnh_.param("fibonacci_index1", fibonacci_index1_, 30);
    pnh_.param("fibonacci_index2", fibonacci_index2_, 40);

    update_timer_ = nh_.createTimer(ros::Duration(1/10.0),
                                    &BasicProfilerExampleNode::handleUpdateTimer,
                                    this);

    int_callback_ = nh_.subscribe("trigger_fibonacci", 10,
                                  &BasicProfilerExampleNode::handleTriggerFibonacci,
                                  this);
  }
  
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

  int superSlowFibonacci(int x)
  {
    SWRI_PROFILE("super-slow-fibonacci");
    ROS_INFO("Fibonacci %d = %d", x, superSlowFibonacciInt(x));
  }

  void handleUpdateTimer(const ros::TimerEvent &ignored)
  {
    (void)ignored;
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

  void handleTriggerFibonacci(const std_msgs::Int32ConstPtr &msg)
  {
    SWRI_PROFILE("handle-trigger-fibonacci");
    superSlowFibonacci(msg->data);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_profiler_example");

  BasicProfilerExampleNode node;
  ros::spin();
  
  return 0;  
}

