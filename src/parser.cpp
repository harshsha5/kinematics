#include <urdf/model.h>
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

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
  std::cout<<"x,y,z are as follows "<<p.x<<"\t"<<p.y<<"\t"<<p.z<<"\n";
  std::cout<<"Roll, Pitch, Yaw are as follows "<<rpy[0]<<"\t"<<rpy[1]<<"\t"<<rpy[2]<<"\n";
}

Eigen::Matrix3d get_rot_matrix(Eigen::Vector3d net)
{
  Eigen::AngleAxisd roll_angle(net[0],Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(net[1],Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(net[2],Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<double> q = pitch_angle * yaw_angle * roll_angle;

  Eigen::Matrix3d rotationMatrix = q.matrix();

  return rotationMatrix;
}

Eigen::Vector3d get_dist_vector(urdf::Vector3 p)
{
  Eigen::Vector3d v(p.x,p.y,p.z);
  return v;
}

Eigen::Matrix4d get_homo_matrix(Eigen::Vector3d d,Eigen::Matrix3d rot_matrix)
{ Eigen::Matrix4d homo_matrix;
  Eigen::Vector4d c(0,0,0,1);
  homo_matrix.block<3,3>(0, 0) = rot_matrix;
  homo_matrix.block<3,1>(0,3) = d;
  homo_matrix.block<1,4>(3,0) = c;

  return homo_matrix;
}

void print_vector(std::vector<double> v)
{
  for(int i=0;i<v.size();i++)
  {std::cout<<v[i]<<"\t";}
}

void print_vector_4d_eigen(std::vector<Eigen::Matrix4d> v)
{
  for(int i=0;i<v.size();i++)
  {std::cout<<v[i]<<"\t";}  
}

Eigen::Vector3d get_rel_trans_vector(urdf::Vector3 p1, urdf::Vector3 p2)
{
  //Finds the relative translation vector
  Eigen::Vector3d p1_vector3d(p1.x,p1.y,p1.z);
  Eigen::Vector3d p2_vector3d(p2.x,p2.y,p2.z);
  return p2_vector3d-p1_vector3d;
}

Eigen::Vector3d get_const_trans_vector(std::string s1, std::string s2,urdf::Model model)
{ boost::shared_ptr<const urdf::Joint> J;
  urdf::Vector3 p1;
  urdf::Vector3 p2;
  J = model.getJoint(s1);
  p1 = J->parent_to_joint_origin_transform.position;
  J = model.getJoint(s2);
  p2 = J->parent_to_joint_origin_transform.position;
  return get_rel_trans_vector(p1,p2);
}

urdf::Vector3 get_base_vector3(std::string s1,urdf::Model model)
{ urdf::Vector3 p1;
  boost::shared_ptr<const urdf::Joint> J;
  J = model.getJoint(s1);
  p1 = J->parent_to_joint_origin_transform.position;
  return p1;
}

std::vector<double> getWristJacobian(std::vector<double> joint_angles,urdf::Model model) 
{
  /**
  * You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
  std::vector<std::string> joint_list = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
  boost::shared_ptr<const urdf::Joint> J;
  Eigen::Matrix4d temp_homo_matrix;
  Eigen::Matrix4d net_homo_matrix = Eigen::Matrix4d::Identity();
  urdf::Vector3 p;
  urdf::Rotation r;
  std::vector<Eigen::Matrix4d> all_homo_list;
  std::vector<double> rpy(3);
  Eigen::Vector3d net;
  int count = 0;
  Eigen::Matrix3d rot_matrix;

  Eigen::MatrixXd jacob(6,joint_list.size());
  std::vector<Eigen::Vector3d> list_of_axis_angles;

  for(auto x: joint_list)
  { 
    J = model.getJoint(x);
    p = J->parent_to_joint_origin_transform.position;
    r = J->parent_to_joint_origin_transform.rotation;
    r.getRPY(rpy[0],rpy[1],rpy[2]);
    Eigen::Vector3d rpy_vector3d(rpy[0],rpy[1],rpy[2]);
    Eigen::Vector3d axis_vector3d(J->axis.x,J->axis.y,J->axis.z);
    list_of_axis_angles.push_back(axis_vector3d);
    axis_vector3d = joint_angles[count]*axis_vector3d;
    net = axis_vector3d + rpy_vector3d;
    //list_of_axis_angles.push_back(net);
    rot_matrix = get_rot_matrix(net);

    net = get_dist_vector(p);

    temp_homo_matrix = get_homo_matrix(net,rot_matrix);
    net_homo_matrix = net_homo_matrix*temp_homo_matrix;
    all_homo_list.push_back(net_homo_matrix);
    //std::cout << net_homo_matrix <<"\n"<<"\n";
    count++;  
  }

  //std::cout<<all_homo_list[4];

  for(int i=0;i<joint_list.size();i++)
  {
    Eigen::Matrix3d Rot = all_homo_list[i].block<3,3>(0,0);
    Eigen::Vector3d Dist = all_homo_list[4].block<3,1>(0,3) - all_homo_list[i].block<3,1>(0,3);
    jacob.block<3,1>(0,i) = (Rot*list_of_axis_angles[i]).cross(Dist);
    jacob.block<3,1>(3,i) = Rot*list_of_axis_angles[i];
  }

  std::cout<<(jacob);

  std::vector<double> vec(jacob.data(), jacob.data() + jacob.rows() * jacob.cols());
  return vec;
  }

std::vector<double> getWristPose(std::vector<double> joint_angles,urdf::Model model) 
{
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
  std::vector<std::string> joint_list = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
  boost::shared_ptr<const urdf::Joint> J;
  Eigen::Matrix4d temp_homo_matrix;
  Eigen::Matrix4d net_homo_matrix = Eigen::Matrix4d::Identity();
  urdf::Vector3 p;
  urdf::Rotation r;
  std::vector<double> rpy(3);
  Eigen::Vector3d net;
  int count = 0;
  Eigen::Matrix3d rot_matrix;
  for(auto x: joint_list)
  { 
    J = model.getJoint(x);
    p = J->parent_to_joint_origin_transform.position;
    r = J->parent_to_joint_origin_transform.rotation;
    r.getRPY(rpy[0],rpy[1],rpy[2]);
    Eigen::Vector3d rpy_vector3d(rpy[0],rpy[1],rpy[2]);
    Eigen::Vector3d axis_vector3d(J->axis.x,J->axis.y,J->axis.z);
    axis_vector3d = joint_angles[count]*axis_vector3d;
    net = axis_vector3d + rpy_vector3d;
    rot_matrix = get_rot_matrix(net);

    net = get_dist_vector(p);

    temp_homo_matrix = get_homo_matrix(net,rot_matrix);
    net_homo_matrix = net_homo_matrix*temp_homo_matrix;
    //std::cout << net_homo_matrix <<"\n"<<"\n";
    count++;  
  }
  //Eigen::VectorXd homo_vector = net_homo_matrix;
  //homo_vector.resize(homo_vector.cols()*homo_vector.rows(), 1);
  std::vector<double> vec(net_homo_matrix.data(), net_homo_matrix.data() + net_homo_matrix.rows() * net_homo_matrix.cols());
  return vec;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file))
  {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }

  ROS_INFO("Successfully parsed urdf file");
  boost::shared_ptr<const urdf::Joint> J;
  boost::shared_ptr< const urdf::Link > L;
  boost::shared_ptr< const urdf::Visual > V;
  std::vector<double> result;
  std::vector<double> jacobian;
  urdf::Vector3 vec;
  std::vector<double> joint_angles = {0.727,1.073,0.694,-0.244,1.302}; 
  result = getWristPose(joint_angles,model);
  jacobian = getWristJacobian(joint_angles,model);
  //std::cout<<"\n"<<"\n";
  //print_vector(result);
  return 0;
}

