#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
using std::placeholders::_1;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("mapping_node")
    {
      publisher_1 = this->create_publisher<std_msgs::msg::Int32MultiArray>("mymotors", 10);
      subscription_1 = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalSubscriber::topic_callback1, this, _1));
      motor_data.data.resize(6);
    }
/*{this}{
 *
} */
  private:
    void topic_callback1(sensor_msgs::msg::Joy msg)
    {

      if(msg.buttons[5]==1)
      {
        motor_data.data[0]=-255;
        motor_data.data[1]=-255;
        button5_prev=1;
        goto publishing;
      }
      else if(msg.buttons[5]==0 && button5_prev==1)
      {
        motor_data.data[0]=0;
        motor_data.data[1]=0;
        button5_prev=0;
        goto publishing;
      }
      else button5_prev=0;

      motor_data.data[0]=msg.axes[0]*255;
      motor_data.data[1]=msg.axes[1]*255;


       pitch = msg.axes[4];
       roll  = msg.axes[3];
       if(std::abs(pitch)>std::abs(roll))
       {
          motor_data.data[2]=static_cast<int32_t>(pitch*83);
          motor_data.data[3]=static_cast<int32_t>(pitch*83);
       }
       else
       {
        motor_data.data[2]=-static_cast<int32_t>(roll*83);
        motor_data.data[3]=static_cast<int32_t>(roll*83);
       }
      if(msg.axes[2]!=1 && msg.axes[5]==1)
      {
          motor_data.data[4]=(255-msg.axes[2]*255)/2;
      }
      else if (msg.axes[2]==1 && msg.axes[5]!=1) {
          motor_data.data[4]=(msg.axes[5]*255 - 255)/2;
      }
      else {
          motor_data.data[4]=0;
      }
      motor_data.data[5]=msg.axes[6]*255;

      publishing:
      publisher_1->publish(motor_data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_1;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_1;

    std_msgs::msg::Int32MultiArray motor_data;
    int button5_prev=0;
      double pitch;
      double roll;
      double mag;
      double left_bevel;
      double right_bevel;
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
