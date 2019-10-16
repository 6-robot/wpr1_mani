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

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_WARN("************************");
    //显示关节的信息
    int nJointNum = msg->name.size();
    for(int i=0;i<nJointNum;i++)
    {
        if(i >=1 && i <= 6)
        {
            // 将手臂旋转关节换算成角度值，便于读数观察
            double degree = msg->position[i]*180/3.14;
            ROS_INFO("Joint[%d] name=%s  position=%.2f ", i , msg->name[i].c_str() , degree);
        }
        else
        {
            // 非手臂关节继续保持原数值
            double value = msg->position[i];
            ROS_INFO("Joint[%d] name=%s  position=%.2f ", i , msg->name[i].c_str() , value);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpr1_arm_joint_state");
    ROS_WARN("wpr1_arm_joint_state start!");
    ros::NodeHandle nh;

    // 订阅机器人实体信息主题
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states",30,&JointStateCallback);

    ros::spin();

    return 0;
}