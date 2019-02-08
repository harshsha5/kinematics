#include <iostream>
#include <vector>
#include <iostream>
#include <urdf/model.h>
#include "ros/ros.h"
#include <string>

using namespace std;
urdf::model my_parser(const string urdf_file)
{

  //std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file))
  {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }

ROS_INFO("Successfully parsed urdf file");
cout<<"Parser run successfuly in Forward Kinematics file";
}

/**
 * Get the wrist pose for given joint angle configuration.
 * list of 16 values which represent the joint end effector pose obtained from the end-effector transformation matrix
 * using column-major ordering.
 * @param joint_angles list of joint angles to get final end effector pose for
 * @return
 */
std::vector<double> getWristPose(std::vector<double> joint_angles) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
  return std::vector<double>();
}

/**
 * Get the wrist pose for given joint angle configuration.
 * list of 16 values which represent the joint end effector pose obtained from the end-effector transformation matrix
 * using column-major ordering.
 * @param joint_angles list of joint angles to get final end effector pose for
 * @return
 */
std::vector<double> getWristJacobian(std::vector<double> joint_angles) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
  return std::vector<double>();
}

int main() {
  return 0;
}
