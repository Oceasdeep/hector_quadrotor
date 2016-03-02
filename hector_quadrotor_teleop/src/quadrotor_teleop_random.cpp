#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

//
enum State
{
  TAKEOFF,//linear.z > for takeoff
  FORWARD,
  TURN,
};
State g_state = TAKEOFF; 
float linear_x = 0.0;
float linear_y = 0.0;
float linear_z = 0.0;
float angular_z = 0.0;

void toogleState()
{
  if(g_state == TAKEOFF)
  {
    g_state = FORWARD;
    linear_x = (rand()%100 - 50)/100.0;
    linear_y = 0;//rand()%100/100.0;
    linear_z = 0;//(rand()%100 - 50)/1000.0;

  }else if(g_state == FORWARD)
  {
    g_state = TURN;
    angular_z = rand()%100/100.0;

  }else if(g_state == TURN)
  {
    g_state = FORWARD;
    linear_x = rand()%100/100.0;
  }
}

void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  cout << "vel: " << linear << endl; 
  twist_pub.publish(twist);
}

void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub)
{
  static int count = 0;
  count++;

  if(g_state == TAKEOFF)
  {
     
      geometry_msgs::Twist twist;
      linear_z = rand()%100/100.0;
      twist.linear.z = linear_z;
      twist_pub.publish(twist);

      if(count % 100 == 0)
      {
        toogleState();
        return;
      }

  }else if(g_state == FORWARD)
  {
    geometry_msgs::Twist twist;
    twist.linear.x = linear_x;
    twist.linear.y = linear_y;
    twist.linear.z = linear_z;
    twist_pub.publish(twist);

    if(count % 500 == 0)
    {
      toogleState();
      return;
    }
  }else if(g_state == TURN)
  {
    geometry_msgs::Twist twist;
    twist.angular.z  = angular_z;

    twist_pub.publish(twist);
    if(count % 200 == 0)
    {
      toogleState();
      return;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_teleop_random");
  ros::NodeHandle nh;
  //cmd_vel_mux/input/teleop
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));
  srand(time(0));
  ros::spin();

  return 0;
}
