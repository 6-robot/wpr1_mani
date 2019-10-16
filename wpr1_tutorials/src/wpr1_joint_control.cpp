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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpr1_manipulator_control");

    ros::NodeHandle n;
    ros::Publisher joint_ctrl_pub = n.advertise<sensor_msgs::JointState>("wpr1/joint_ctrl_degree", 30);

    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(8);
    ctrl_msg.position.resize(8);
    ctrl_msg.velocity.resize(8);
    ctrl_msg.name[0] = "0_torso_lift";
    ctrl_msg.name[1] = "1_shoulder_roll";
    ctrl_msg.name[2] = "2_shoulder_flex";
    ctrl_msg.name[3] = "3_upperarm_roll";
    ctrl_msg.name[4] = "4_elbow_flex";
    ctrl_msg.name[5] = "5_forearm_roll";
    ctrl_msg.name[6] = "6_wrist_flex";
    ctrl_msg.name[7] = "7_gripper";

    ctrl_msg.position[0] = 0;
    ctrl_msg.position[1] = -45;
    ctrl_msg.position[2] = -90;
    ctrl_msg.position[3] = 0;
    ctrl_msg.position[4] = -90;
    ctrl_msg.position[5] = -90;
    ctrl_msg.position[6] = -90;
    ctrl_msg.position[7] = 0.5;

    ctrl_msg.velocity[0] = 1500;
    ctrl_msg.velocity[1] = 1500;
    ctrl_msg.velocity[2] = 1500;
    ctrl_msg.velocity[3] = 1500;
    ctrl_msg.velocity[4] = 1500;
    ctrl_msg.velocity[5] = 1500;
    ctrl_msg.velocity[6] = 1500;
    ctrl_msg.velocity[7] = 1500;

    int nCount = 0;
    ros::Rate r(0.1);
    ros::spinOnce();
    r.sleep();
    
    while(ros::ok())
    {
        ROS_WARN("[wpr1_mani_ctrl_demo] nCount = %d", nCount);
        switch(nCount)
        {
        case 0: //home
            ctrl_msg.position[0] = 0.1;
            ctrl_msg.position[1] = -45;
            ctrl_msg.position[2] = -90;
            ctrl_msg.position[3] = 0;
            ctrl_msg.position[4] = -90;
            ctrl_msg.position[5] = -90;
            ctrl_msg.position[6] = -90;
            ctrl_msg.position[7] = 0.05;
            break;
        case 1:  //grab
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = 0;
            ctrl_msg.position[2] = -60;
            ctrl_msg.position[3] = 90;
            ctrl_msg.position[4] = -120;
            ctrl_msg.position[5] = -90;
            ctrl_msg.position[6] = 60;
            ctrl_msg.position[7] = 0.10;
            break;
        case 2: //hold
            ctrl_msg.position[0] = 0.1;
            ctrl_msg.position[1] = -45;
            ctrl_msg.position[2] = -90;
            ctrl_msg.position[3] = 0;
            ctrl_msg.position[4] = -90;
            ctrl_msg.position[5] = -90;
            ctrl_msg.position[6] = -45;
            ctrl_msg.position[7] = 0.05;
            break;
        case 3: //zero
            ctrl_msg.position[0] = 0;
            ctrl_msg.position[1] = 0;
            ctrl_msg.position[2] = 0;
            ctrl_msg.position[3] = 0;
            ctrl_msg.position[4] = 0;
            ctrl_msg.position[5] = 0;
            ctrl_msg.position[6] = 0;
            ctrl_msg.position[7] = 0.10;
            break;
        }
        joint_ctrl_pub.publish(ctrl_msg);
        ros::spinOnce();
        r.sleep();
        nCount ++;
        if(nCount >3)
        {
            nCount = 0;
        }
    }

    return 0;
}