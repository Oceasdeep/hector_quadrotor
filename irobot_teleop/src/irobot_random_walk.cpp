#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <string.h>
using namespace std;

const int nrobot = 10;


enum State
{
  FORWARD,
  TURN,
};

void toogleState(State& state, double & linear, double & angular)
{
  if(state == FORWARD)
  {
    state = TURN;
    angular = rand()%100/50.0;
  }else if(state == TURN)
  {
    state = FORWARD;
    linear = rand()%100/50.0;
  }
}

void timerCallback(const ros::TimerEvent&, ros::Publisher* twists_pub)
{
  static int count = 0;
  static State states[nrobot] = {FORWARD};
  static double linears[nrobot] = {0};
  static double angulars[nrobot] = {0};

  for(int i = 0; i < nrobot; i++)
  {
    if(states[i] == FORWARD)
    {
      geometry_msgs::Twist twist;
      twist.linear.x = linears[i];
      twist.linear.y = 0;
      twist.linear.z = 0;
      twists_pub[i].publish(twist);

      if(count % 500 == 0)
      {
        toogleState(states[i], linears[i], angulars[i]);
      }
    }else if(states[i] == TURN)
    {
      geometry_msgs::Twist twist;
      twist.angular.z  = angulars[i];
      twists_pub[i].publish(twist);
      if(count % 200 == 0)
      {
        toogleState(states[i], linears[i], angulars[i]);
      }
    }
  }
  
  count++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irobot_teleop_random");
  ros::NodeHandle nh;


  ros::Publisher twists_pub[nrobot];
  for(int i = 0; i < nrobot; i++)
  {
    string topic = string("irobot") + std::to_string(i) + "/cmd_vel";
    twists_pub[i] = nh.advertise<geometry_msgs::Twist>(topic, 1); 
  }

  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twists_pub));
  srand(time(0));
  ros::spin();

  return 0;
}
