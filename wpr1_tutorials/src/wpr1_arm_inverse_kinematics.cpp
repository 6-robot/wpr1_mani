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
    //先获取正解作为反解的输入量，参数为Moveit界面里的蓝色拖动球的link（ 查看urdf文件可以知道：wpm2_palm 为机械臂蓝色拖动球的link ）
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("wpm2_palm");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

    printf("\n");
    ROS_WARN("***************** OUTPUT **********************");
    //用当前机器人手臂末端(Moveit界面里的蓝色拖动球)的坐标作为反解输入，求反解
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
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
        ROS_INFO("Did not find IK solution");
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