#include "ros/ros.h"
#include "trac_ik_wrapper/IKHandler.h"
#include <trac_ik/trac_ik.hpp>
#include <string>

TRAC_IK::TRAC_IK *solver;

bool getPose(trac_ik_wrapper::IKHandler::Request  &req,
         trac_ik_wrapper::IKHandler::Response &res)
{
  int points = req.points;
  int poseLen = req.seed.size();

  KDL::JntArray prev(poseLen);
  std::string seedStr = "";
  for (int i = 0; i < poseLen; i++) {
    seedStr += std::to_string(req.seed[i]) + " ";
    prev(i) = req.seed[i];
  }

  for(int p = 0; p < points; p++) {
    std::string goalStr = "";

    int idx = p * 6;
    KDL::Vector goalPos(req.goal[idx],req.goal[idx+1],req.goal[idx+2]);
    //KDL::Rotation goalRot = KDL::Rotation::Quaternion(req.goal[3],req.goal[4],req.goal[5],req.goal[6]);
    KDL::Rotation goalRot = KDL::Rotation::RPY(req.goal[idx+3],req.goal[idx+4],req.goal[idx+5]);
    KDL::Frame goalPose(goalRot,goalPos);
    for (int i = 0; i < 6; i++)
      goalStr += std::to_string(req.goal[idx + i]) += " ";

    ROS_INFO("request: seed=%s goal=%s", seedStr.c_str(), goalStr.c_str());
 
 //   KDL::JntArray result;
    KDL::Vector posTol(0.01,0.01,0.01);
    float max = std::numeric_limits<float>::max();
    KDL::Vector rotTol(max,max,max);
    KDL::Twist tol(posTol, rotTol);
    int status = solver->CartToJnt(prev, goalPose, prev, tol);
    if(status < 0)
  	ROS_INFO("IK failed!");

    std::string poseStr = "";
   // float pose[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    for(int i = 0; i < poseLen; i++) {
      double val = prev.data[i];
      res.pose.push_back(val);
      poseStr += std::to_string(val) += " ";
    }
    ROS_INFO("sending back response: %s", poseStr.c_str());
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trac_ik_wrapper_server");
  ros::NodeHandle n;

  solver = new TRAC_IK::TRAC_IK("linear_actuator_link", "right_ee_link", "/robot_description",0.005,1e-5, TRAC_IK::Distance);
 // TRAC_IK::TRAC_IK solver("base_link", "right_ee_link", "/robot_description");
  /*KDL::Chain chain;
  bool valid = solver->getKDLChain(chain);
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return 0;
  }*/


  ros::ServiceServer service = n.advertiseService("trac_ik_wrapper", getPose);
  ROS_INFO("IK wrapper ready.");
  ros::spin();
  delete solver;
  return 0;
}
