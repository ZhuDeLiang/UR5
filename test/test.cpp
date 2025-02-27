#include <spatial_utilities.h>
#include <timer_linux.h>
#include "ur_rtde.h"
#include <iostream>
int main() {
  URRTDE::URRTDEConfig config;
 config.robot_ip = "169.254.210.100";
 //config.robot_ip = "10.1.94.211";
  config.rtde_frequency = 500;
  config.rt_receive_priority = 90;
  config.rt_control_priority = 85;
  config.interface_priority = 80;
  config.linear_vel = 0.5;
  config.linear_acc = 0.5;
  config.servoL_lookahead_time = 0.1;
  config.servoL_gain = 600;
  config.robot_interface_config.zone_safety_mode =
      RobotSafetyMode::SAFETY_MODE_NONE;
  config.robot_interface_config.incre_safety_mode =
      RobotSafetyMode::SAFETY_MODE_NONE;
  config.robot_interface_config.operation_mode =
      RobotOperationMode::OPERATION_MODE_JOINT;
  config.robot_interface_config.max_incre_m = 0.002;      // 1 m per second
  config.robot_interface_config.max_incre_rad = 0.00628;  // 3.14 per second
  double data[] = {0.3, 0.65, -0.3, 0.4, 0.1, 0.4};
  Eigen::Map<RUT::Vector6d> map_vec(data);
  config.robot_interface_config.safe_zone = map_vec;

  URRTDE ur_rtde;
  RUT::Timer timer;
  ur_rtde.init(timer.tic(), config);
  RUT::VectorXd joint;
  ur_rtde.getJoints(joint);
  RUT::VectorXd joint_target=joint;
    // 创建引用指向该对象
   
  RUT::Vector7d pose0;
  ur_rtde.getCartesian(pose0);
  RUT::Vector7d pose_ref = pose0;
RUT::VectorXd joint1;
  timer.tic();
  {
    RUT::TimePoint t_start = ur_rtde.rtde_init_period();

    double dt = timer.toc_ms();
   /* printf("t = %f, pose: %f %f %f %f %f %f %f\n", dt, pose_ref[0], pose_ref[1],
           pose_ref[2], pose_ref[3], pose_ref[4], pose_ref[5], pose_ref[6]);*/
    ur_rtde.getJoints(joint1);
    printf("joint=%f  %f  %f  %f  %f  %f  %f", joint1[0], joint1[1], joint1[2], joint1[3], joint1[4], joint1[5]);
    //double dx = 0.1 * (1.0 - cos(1.2 * dt / 1000.0));
    //double dy = 0.1 * sin(1.2 * dt / 1000.0);
    //dy=0;
    joint_target[0] = joint[0] +0.2;
    //joint_target[1] = joint[1] + dy;
    if(ur_rtde.setJoints(joint_target))
    {
     ur_rtde.getJoints(joint1);
    printf("joint=%f  %f  %f  %f  %f  %f  %f", joint1[0], joint1[1], joint1[2], joint1[3], joint1[4], joint1[5]);
    }
    else
    {
    printf("Move false");
    }
   // if (!ur_rtde.streamCartesian(pose_ref)) {
    //  printf("streamCartesian failed\n");
   
    //  break;
   // }

  


    ur_rtde.rtde_wait_period(t_start);
  }

  return 0;
}
