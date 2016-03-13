#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

float linear_x = 0.0;
float angular_z = 0.0;

enum State{
  TAKEOFF,
  FORWARD,
};
 
State g_state = TAKEOFF; 
bool clockwise = true;

//find the farrest direction
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int min_id = 0;
  float min_dis = DBL_MAX;

  cout << "cnt : " << msg->ranges.size() << endl;
  cout << "range_min: " << msg->range_min << endl;
  cout << "range_max: " << msg->range_max << endl;
  for(int i = 0; i < msg->ranges.size(); i++)
  {
    if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) continue;
    //cout << msg->ranges[i] << " ";
    
    if(msg->ranges[i] < min_dis)
    {  
      min_id = i;
      min_dis = msg->ranges[i];
    }
  }
  
  if(min_dis < 1.2)
  {
    angular_z = 1.0;
    linear_x = 0;
  }else
  {
    angular_z = 0.0;
    linear_x = 1.0;
  }
  cout << "min_dis: " << min_dis << "  angular_z: " << angular_z << endl;
}

void laserScanCallback2(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  int min_id = 0;
  float min_dis = DBL_MAX;
  cout << "angle min: " << msg->angle_min << "  max: " << msg->angle_max << endl;
  cout << "range min: " << msg->range_min << "  max: " << msg->range_max << endl;
  //min dis in forward direction
  int id1 = (int) ((-M_PI/6 - msg->angle_min)/msg->angle_increment );
  int id2 = (int)( (M_PI/6 - msg->angle_min)/msg->angle_increment );
  for(int i = id1; i < id2; i++)
  {
    if(msg->ranges[i] <= msg->range_min || msg->ranges[i] >= msg->range_max) continue;

    if(msg->ranges[i] < min_dis)
    {  
      min_id = i;
      min_dis = msg->ranges[i];
    }
  }
    
  cout << "min dis: " << min_dis << "  angle: " << (msg->angle_min + min_id*msg->angle_increment) << endl;
  
  if(min_dis < 1.2)
  {
    float min_dis_angle = msg->angle_min + msg->angle_increment*min_id;
    float scale = 1.0;
    angular_z = 1.0;//-min_dis_angle*scale;
    linear_x = 0;
    return;
  }


  if(msg->ranges.size() <= 0) return;
  int forward_id = (int)((0 - msg->angle_min)/msg->angle_increment);
  int count = msg->ranges.size();
  float forward_dis = msg->ranges[forward_id];

  if(forward_dis < 1.5)
  {
    angular_z = 1.0;
    linear_x = 0;
  }else
  {
    angular_z = 0.0;
    linear_x = 1.0;
  }
  cout << "forward_id: " << forward_id <<  "  dis: " << forward_dis << endl;
}


void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;//
  twist.angular.z = angular;
  twist_pub.publish(twist);
}

void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub)
{
  static int count = 0;
  if(g_state == TAKEOFF)
  {
    geometry_msgs::Twist twist;
    twist.linear.z = 0.5;//
    twist_pub.publish(twist);

    if(count >= 100)
    {
      g_state = FORWARD;
    }
  }else if(g_state == FORWARD)
  {
    if(clockwise)
    {
      commandTurtle(twist_pub, linear_x, -angular_z);
    }else 
    {
      commandTurtle(twist_pub, linear_x, angular_z);
    }
    
  }
  count++;
  if(count % 300 == 0)
  {
    if(rand() % 2 == 0)
    {
      clockwise = true;
    }else 
    {
      clockwise = false;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_obstacle_avoidance");
  ros::NodeHandle nh;
  //cmd_vel_mux/input/teleop
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, laserScanCallback2);

  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));
  srand(time(0));
  ros::spin();

  return 0;
}
