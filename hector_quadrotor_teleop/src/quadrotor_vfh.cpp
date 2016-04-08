//TODO VFH algorithm for obstacle avoidance
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

// ros::Publisher twist_pub;
Matrix3d R_laser2body;
bool R_laser2body_initialized = false;

Vector3d linear;
Vector3d angular;

float linear_x = 0.0;
float angular_z = 0.0;

enum State{
  TAKEOFF,
  FORWARD,
};
 
State g_state = TAKEOFF; 
bool clockwise = true;

//todo, velocity shoudl be transformed from laser frame to body frame

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

//reference: stdr_obstacle_avoidance
void laserScanCallback3(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan scan_ = *msg;
  float linear_x = 0, linear_y = 0, angular_z = 0;

  for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
  {
    float real_dist = scan_.ranges[i];
    if(real_dist < 0.2) continue;

    linear_x -= cos(scan_.angle_min + i * scan_.angle_increment) 
      / (1.0 + real_dist * real_dist);
    linear_y -= sin(scan_.angle_min + i * scan_.angle_increment) 
      / (1.0 + real_dist * real_dist);
    angular_z -= sin(scan_.angle_min + i * scan_.angle_increment) 
      / (1.0 + real_dist * real_dist);
  }
  geometry_msgs::Twist cmd;
  
  linear_x /= scan_.ranges.size();
  linear_y /= scan_.ranges.size();
  angular_z /= scan_.ranges.size();
  
  //~ ROS_ERROR("%f %f",linear,rotational);
  // const double constant_v = 0.3;
  // if(linear_x > constant_v)
  // {
  //   linear_x = constant_v;
  // }
  // else if(linear_x < -constant_v)
  // {
  //   linear_x = -constant_v;
  // }
  // linear_x += constant_v;

  linear(0) = linear_x;
  linear(1) = linear_y;
  //angular(2) = angular_z*30;
  //cout << "linear: " << linear_x << "angular: " << angular_z << endl;
  cout << "linear: " << linear_x  << linear_y << endl;
  //cmd.linear_x.x = 0.3 + linear_x;

  // cmd.angular.z = rotational;
  // //transform the velocity to the body frame

  // twist_pub.publish(cmd);
  
}


void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;//
  twist.angular.z = angular;
  twist_pub.publish(twist);
}

void commandTurtle2(ros::Publisher twist_pub, Vector3d linear, Vector3d angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear(0);//
  twist.linear.y = linear(1);//
  twist.linear.z = linear(2);//
  twist.angular.x = angular(0);
  twist.angular.y = angular(1);
  twist.angular.z = angular(2);
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

    if(count >= 30)
    {
      g_state = FORWARD;
    }
  }else if(g_state == FORWARD)
  {
    commandTurtle2(twist_pub, R_laser2body*linear, R_laser2body*angular);    
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
  ros::Subscriber laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, laserScanCallback3);

  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));
  srand(time(0));
  ros::Time now = ros::Time::now();
  tf::TransformListener listener;
  ros::Rate loop_rate(20);
  while(ros::ok())
  {
    ros::spinOnce();

    tf::StampedTransform transform;
    try{
     //targe_frame <- source frame
      listener.waitForTransform("/base_link", "laser0_frame", 
             now, ros::Duration(3.0));
      listener.lookupTransform("/base_link", "/laser0_frame",
                   now, transform);
    }
      catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      loop_rate.sleep();
      continue;
    }
    Quaterniond qq;
    qq.w() = transform.getRotation().w();
    qq.x() = transform.getRotation().x();
    qq.y() = transform.getRotation().y();
    qq.z() = transform.getRotation().z();
    R_laser2body = qq.toRotationMatrix();
    R_laser2body_initialized = true;
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
