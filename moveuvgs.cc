/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-07-18 02:46:53
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-08-29 06:26:24
 * @ Description: PROTOTYPE NODE FOR MOVING ASTROBEE TO POINTS IN SPACE (uvgs in the future)
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/MotionActionFeedback.h>
#include <ff_msgs/MobilityState.h>
#include <ff_msgs/AgentStateStamped.h>

#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AckCompletedStatus.h>

#include <ff_msgs/CommandArg.h>
#include <ff_msgs/CommandConstants.h>
#include <tf2_ros/transform_listener.h>
#include <ff_util/ff_flight.h>
#include <ff_common/ff_names.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>

ros::Publisher cmd_pub;

tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped tfs;
bool move_relative = true;  //DEFAULT
bool is_moving = false;
std::deque<std::vector<ff_msgs::CommandArg>> old_args;

void sendCommand(const std::string& cmd_name, const std::vector<ff_msgs::CommandArg>& args = {}) {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";
  cmd.cmd_name = cmd_name;
  cmd.cmd_id = cmd_name;
  cmd.args = args;

  cmd_pub.publish(cmd);
  ROS_INFO("Published command: %s", cmd_name.c_str());
}
//msg->mobility_state.state != ff_msgs::MobilityState::FLYING &&
//|| msg->mobility_state.state == ff_msgs::MobilityState::STOPPING
void checkMotionState(const ff_msgs::AckStamped::ConstPtr& msg) {
  if ( msg->status.status== ff_msgs::AckStatus::COMPLETED || msg->completed_status.status == ff_msgs::AckCompletedStatus::OK) {
    is_moving = false;
    ROS_INFO("Robot is no longer moving.");
  }
  else {
    is_moving = true;
  }
}

void moveRobot(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (is_moving) {
    // ROS_WARN("Robot is currently moving, new move command will not be sent.");
    return;
  }

  std::vector<ff_msgs::CommandArg> last_args(4);

  if (!old_args.empty()){
    last_args = old_args.back();
    old_args.pop_back();
  } else {
    last_args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    last_args[1].vec3d[0] = 0.0;
    last_args[1].vec3d[1] = 0.0;
    last_args[1].vec3d[2] = 0.0;
  }

  
  std::vector<ff_msgs::CommandArg> args(4);

  args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  args[0].s = move_relative ? "honey/body" : "world";

  args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
  if (move_relative) {
    args[1].vec3d[0] = msg->pose.position.x - last_args[1].vec3d[0];
    args[1].vec3d[1] = msg->pose.position.y - last_args[1].vec3d[1];
    args[1].vec3d[2] = msg->pose.position.z - last_args[1].vec3d[2];

    if ( std::abs(args[1].vec3d[0]) < 0.01 &&  std::abs(args[1].vec3d[1]) < 0.01  && std::abs(args[1].vec3d[2]) < 0.01 ) {
      return;
    }
  } else {
    try {

      ROS_INFO("Moving in absolute coordinates");
      //FIX
      tfs = tf_buffer.lookupTransform("honey/body","world",ros::Time(0));
      // tfs = tf_buffer.lookupTransform("world","honey/body",ros::Time(0));
      //TFS PROLLY WRONG
      geometry_msgs::Vector3 translation = tfs.transform.translation;

      // ROS_INFO("Honey position x:%f, y:%f, z:%f", translation.x, translation.y, translation.z);
      geometry_msgs::Quaternion rotation = tfs.transform.rotation;
      Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
      Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

      Eigen::Vector3d vector_to_move;
      vector_to_move.x() = msg->pose.position.x - translation.x;
      vector_to_move.y() = msg->pose.position.y - translation.y;
      vector_to_move.z() = msg->pose.position.z - translation.z;

     Eigen::Vector3d target_point = rotation_matrix * vector_to_move;

      args[1].vec3d[0] = target_point.x();
      args[1].vec3d[1] = target_point.y();
      args[1].vec3d[2] = target_point.z();

      // args[1].vec3d[0] = -tfs.transform.translation.x + msg->pose.position.x;
      // args[1].vec3d[1] = -tfs.transform.translation.y + msg->pose.position.y;
      // args[1].vec3d[2] = -tfs.transform.translation.z + msg->pose.position.z;
      
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
  }

  args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
  args[2].vec3d[0] = 0;
  args[2].vec3d[1] = 0;
  args[2].vec3d[2] = 0;

  args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
  if (move_relative) {
        args[3].mat33f[0] = 0.0;
        args[3].mat33f[1] = 0.0;
        args[3].mat33f[2] = 0.0;
        args[3].mat33f[3] = 1.0;
        // args[3].mat33f[0] = msg->pose.orientation.x;
        // args[3].mat33f[1] = msg->pose.orientation.y;
        // args[3].mat33f[2] = msg->pose.orientation.z;
        // args[3].mat33f[3] = msg->pose.orientation.w;
    // Eigen::Quaternionf q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    // Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();
    // std::memcpy(args[3].mat33f.data(), rotation_matrix.data(), 9 * sizeof(float));
  } else {
    //FIX
    Eigen::Quaternionf q(tfs.transform.rotation.w, tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z);
    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();
    std::memcpy(args[3].mat33f.data(), rotation_matrix.data(), 9 * sizeof(float));
  }

  // if (msg->pose.orientation.w ==1 && msg->pose.orientation.x  ==0 && msg->pose.orientation.y ==0 && msg->pose.orientation.z ==0){
  //     Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
  //     std::memcpy(args[3].mat33f.data(), identity.data(), 9 * sizeof(float));
  //     ROS_INFO("IDENTITY SENT!");

  //   }
  std::vector<ff_msgs::CommandArg> new_args(4);
  new_args[1].vec3d[0] = msg->pose.position.x;
  new_args[1].vec3d[1] = msg->pose.position.y;
  new_args[1].vec3d[2] = msg->pose.position.z;


  old_args.push_back(new_args);

  args[1].vec3d[0] = args[1].vec3d[0];
  args[1].vec3d[1] = args[1].vec3d[1];
  args[1].vec3d[2] = -args[1].vec3d[2];

  sendCommand(ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF, args);
  // is_moving = true;
  if (is_moving == false){
 ROS_INFO("=============================================================");
 ROS_INFO("Published move command to [%f, %f, %f] with orientation [qx: %f, qy: %f, qz: %f, qw: %f]",
         msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
         msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  ROS_INFO("Published move displacement of [%f, %f, %f] with a turn of [qx: %f, qy: %f, qz: %f, qw: %f]",
         args[1].vec3d[0], args[1].vec3d[1], args[1].vec3d[2],
         msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  ROS_INFO("=============================================================");

  }

}

void initializeRobot() {
  //DEBUGGER
  ROS_INFO("INTIIALIZED");
  //RESET EKF BIAS
  sendCommand(ff_msgs::CommandConstants::CMD_NAME_INITIALIZE_BIAS);

  //RESET EKF
  sendCommand(ff_msgs::CommandConstants::CMD_NAME_RESET_EKF);

  //OPS Limits
  std::vector<ff_msgs::CommandArg> op_limits(7);

  op_limits[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  op_limits[0].s = "user_profile";

  op_limits[1].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  op_limits[1].s = "nominal";  //NOMINAL FANS

  op_limits[2].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  op_limits[2].f = 0.6;  //DESIRED LINEAR VELOCITY

  op_limits[3].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  op_limits[3].f = 0.3;  //DESIRED LINEAR ACCELERATION

  op_limits[4].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  op_limits[4].f = 0.6;  //DESIRED ANGULAR VELOCITY

  op_limits[5].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  op_limits[5].f = 0.3;  //DESIRED ANGULAR ACCELERATION

  op_limits[6].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  op_limits[6].f = 0;  //DESIRED COLLISION DISTANCE

  sendCommand(ff_msgs::CommandConstants::CMD_NAME_SET_OPERATING_LIMITS, op_limits);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "moveuvgs");
  ros::NodeHandle nh;

  //MOVING RELATIVE OR NOT --> DEFAULT RELATIVE
  nh.param("move_relative", move_relative, true);

  //ROTATE FRAMES
  tf2_ros::TransformListener tf_listener(tf_buffer);

  cmd_pub = nh.advertise<ff_msgs::CommandStamped>("/honey/command", 10);

  initializeRobot();

  ros::Subscriber sub = nh.subscribe("/pose", 100, moveRobot);
  ros::Subscriber state_sub = nh.subscribe("honey/mgt/ack", 100, checkMotionState);

  //SPIN IT BABY
  ros::spin();

  return 0;
}