#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int init_tf()
{
  tf::TransformBroadcaster test;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.1,0.1,0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  test.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","base_laser"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;
  while(n.ok()){
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),ros::Time::now(),"map", "base_footprint"));
    r.sleep();
  }
}