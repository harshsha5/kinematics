#include <urdf/model.h>
#include "ros/ros.h"
#include <iostream>
#include <vector>

void print_link_details(boost::shared_ptr< const urdf::Link > L)
{ urdf::Vector3 v;
  urdf::Rotation r;
  std::vector<double> rpy(3);
  v = L->inertial->origin.position;
  r =  L->inertial->origin.rotation;
  std::cout<<"Positon x,y,z are as follows "<<v.x<<"\t"<<v.y<<"\t"<<v.z<<"\n";
  r.getRPY(rpy[0],rpy[1],rpy[2]);
  std::cout<<"Roll, Pitch, Yaw are as follows "<<rpy[0]<<"\t"<<rpy[1]<<"\t"<<rpy[2]<<"\n";  
}

void print_joint_details(boost::shared_ptr<const urdf::Joint> J)
{ urdf::Vector3 p;
  urdf::Rotation r;
  std::vector<double> rpy(3);
  std::cout<<"Name of joint is "<<J->name<<"\n";
  std::cout<<"Axis of rotation along x,y,z is "<<J->axis.x<<"\t"<<J->axis.y<<"\t"<<J->axis.z<<"\n";
  std::cout<<"Name of parent link is "<<J->parent_link_name<<"\n";
  std::cout<<"Name of child link is "<<J->child_link_name<<"\n";
  p = J->parent_to_joint_origin_transform.position;
  r = J->parent_to_joint_origin_transform.rotation;
  r.getRPY(rpy[0],rpy[1],rpy[2]);
  std::cout<<"Roll, Pitch, Yaw are as follows "<<rpy[0]<<"\t"<<rpy[1]<<"\t"<<rpy[2]<<"\n";
}

std::vector<double> getWristPose(std::vector<double> joint_angles) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
  return std::vector<double>();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  boost::shared_ptr<const urdf::Joint> J;
  boost::shared_ptr< const urdf::Link > L;
  boost::shared_ptr< const urdf::Visual > V;
  //urdf::pose P;
  urdf::Vector3 vec;
  //std::vector<double> joint_angles;
  std::vector<std::string> joint_list = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
  for(auto x: joint_list)
  {
  J = model.getJoint(x);
  print_joint_details(J);

  L = model.getLink	(J->parent_link_name);
  print_link_details(L);  
  }
  return 0;
}
