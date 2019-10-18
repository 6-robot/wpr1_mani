/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

static double robot_joints[6];
static bool bRobotJointValueRecv = false;
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //获取机器人手臂实体当前角度
    int nNumJoint = msg->position.size();
    robot_joints[0] = msg->position[1]; // "1_shoulder_roll";
    robot_joints[1] = msg->position[2]; // "2_shoulder_flex";
    robot_joints[2] = msg->position[3]; // "3_upperarm_roll";
    robot_joints[3] = msg->position[4]; // "4_elbow_flex";
    robot_joints[4] = msg->position[5]; // "5_forearm_roll";
    robot_joints[5] = msg->position[6]; // "6_wrist_flex";
    bRobotJointValueRecv = true;
}

static double fAngToDeg = 180/3.1415926;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpr1_arm_inverse_kinematics");
    ROS_WARN("wpr1_arm_inverse_kinematics start!");
    ros::NodeHandle nh;
    // 订阅机器人实体当前角度主题
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states",30,&JointStateCallback);

    //获取模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    //获取关节组，arm为规划组
    const robot_state::JointModelGroup *joint_model_group =kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    //获取机器人实体的当前角度值
    while(bRobotJointValueRecv == false)
    {
        ros::spinOnce();
    }
    std::vector<double> joint_values;
    joint_values.push_back(robot_joints[0]); //"1_shoulder_roll";
    joint_values.push_back(robot_joints[1]); //"2_shoulder_flex";
    joint_values.push_back(robot_joints[2]); //"3_upperarm_roll";
    joint_values.push_back(robot_joints[3]); //"4_elbow_flex";
    joint_values.push_back(robot_joints[4]); //"5_forearm_roll";
    joint_values.push_back(robot_joints[5]); //"6_wrist_flex";
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    printf("\n");
    ROS_WARN("***************** INPUT ***********************");
    /////////////////////////////////////////////////////////////////////////////
    // 输入选择一 以机械臂当前姿态作为输入
    //先获取正解作为反解的输入量，参数为机械臂手爪末端的link（ 查看TF Tree可以知道：wpr1_tip为机械臂手爪末端的link）
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("wpr1_tip");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
    //用指定的关节名称及其姿态作为反解输入，求反解
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state,"wpr1_tip", 10, 0.1);
    /////////////////////////////////////////////////////////////////////////////
    // 输入选择二 通过赋值指定一个目标姿态作为输入
    // geometry_msgs::Pose end_effector_pose;
    // end_effector_pose.position.x = 0.76;
    // end_effector_pose.position.y = 0;
    // end_effector_pose.position.z = 0.80;
    // tf::Quaternion quat;
    // 目标姿态朝向的欧拉角表示,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    // quat.setRPY(0.0, 0.0, 0.0);
    // 将欧拉角旋转量转换成四元数表达
    // tf::StampedTransform transform;
    // transform.setRotation(quat);
    // end_effector_pose.orientation.x = transform.getRotation().getX();
    // end_effector_pose.orientation.y = transform.getRotation().getY();
    // end_effector_pose.orientation.z = transform.getRotation().getZ();
    // end_effector_pose.orientation.w = transform.getRotation().getW();
    // ROS_INFO_STREAM("Position: \n" << end_effector_pose.position);
    // ROS_INFO_STREAM("Orientation: \n" << end_effector_pose.orientation);
    //用指定的关节名称及其姿态作为反解输入，求反解
    // bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose,"wpr1_tip", 10, 0.1);
    //////////////////////////////////////////////////////////////////////////////

    ROS_WARN("***************** OUTPUT **********************");
    if (found_ik)
    {
        //反解有解，定义一个数组 joint_values 用来获取反解的机械臂关节值
        std::vector<double> joint_values;
        joint_values.resize(6);
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            // 显示的时候将单位从弧度换算到度，这样便于观察比对
            ROS_INFO("Joint %s: %.2f Degree ", joint_names[i].c_str(), joint_values[i]*fAngToDeg);
        }
    }
    else
    {
        //反解无解
        ROS_WARN("[ERROR] Did not find IK solution");
    }

    // 获取反解得到的雅戈比逆矩阵
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position,
                                jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian);

    return 0;
}