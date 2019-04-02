/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_BASE_CHASSIS_H
#define ROBORTS_BASE_CHASSIS_H
#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {
/**
 * @brief ROS API for chassis module
 */
class Chassis {
 public:
  /**
   * @brief Constructor of chassis including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  Chassis(std::shared_ptr<roborts_sdk::Handle> handle);

  /**
   * @brief Destructor of chassis
   */
  ~Chassis();

 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();

  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();
//LX ADD 0402

  void ChassisAckInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_ack_push> chassis_info);

  void ChassisFaultCodeInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_fault_code> chassis_info);

  void ChassisReqClockInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_req_clock> chassis_info);

  void ChassisBatteryStaInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_battery_sta> chassis_info);

  void ChassisSonarDataInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_sonar_data> chassis_info);

  void ChassisFaultWdtPushInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_fault_wdt_push> chassis_info);

//END 0402
  /**
   * @brief Chassis information callback in sdk
   * @param chassis_info Chassis information
   */
  void ChassisPoseInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_pose> chassis_info);

  /**
   * @brief Chassis speed control callback in ROS
   * @param vel Chassis speed control data
   */
  void ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel);

  /**
   * @brief Chassis speed and acceleration control callback in ROS
   * @param vel_acc Chassis speed and acceleration control data
   */
  void ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc);


  void ReqDataThreadHandle();
  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;
  //! sdk version client
  std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                      roborts_sdk::cmd_version_id>> verison_client_;

  //! sdk heartbeat thread
  std::thread heartbeat_thread_;
  std::thread req_data_thread_;
  //! sdk publisher for heartbeat
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;

  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_ack>> chassis_ack_pub_;
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_clock>> chassis_clock_pub_;
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_stop>> chassis_stop_pub_;
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_req>> chassis_req_pub_;
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_fault_wdt>> chassis_fault_wdt_pub_;
  //! sdk publisher for chassis speed control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_speed>> chassis_speed_pub_;
  //! sdk publisher for chassis speed and acceleration control
//  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_spd_acc>> chassis_spd_acc_pub_;

  //! ros node handler
  ros::NodeHandle ros_nh_;
  //! ros subscriber for speed control
  ros::Subscriber ros_sub_cmd_chassis_vel_;
  //! ros subscriber for chassis speed and acceleration control
  ros::Subscriber ros_sub_cmd_chassis_vel_acc_;
  //! ros publisher for odometry information
  ros::Publisher ros_odom_pub_;


  //! ros chassis odometry tf
  geometry_msgs::TransformStamped odom_tf_;
  //! ros chassis odometry tf broadcaster
  tf::TransformBroadcaster tf_broadcaster_;
  //! ros odometry message
  nav_msgs::Odometry odom_;
  //! ros uwb message
  geometry_msgs::PoseStamped uwb_data_;
};
}
#endif //ROBORTS_BASE_CHASSIS_H
