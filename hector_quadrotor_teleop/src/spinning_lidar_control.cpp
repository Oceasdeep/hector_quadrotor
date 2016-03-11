#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "spinning_lidar_control");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Time start = ros::Time::now();

  while(ros::ok())
  {
    tf::Quaternion q;
    double t = (ros::Time::now() - start).toSec();
    double psy = t*0.5;//0.1 rad/s
    q.setRPY(0, 0, psy);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "spinning_hokuyo_spinning_lidar_root_link", "spinning_hokuyo_spinning_lidar_spin_link"));
    loop_rate.sleep();
  }
  return 0;
};

