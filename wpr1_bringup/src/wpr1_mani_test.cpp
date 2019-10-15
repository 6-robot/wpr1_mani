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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "Mani_driver.h"
#include <math.h>

static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double fJointAngle[8];
static int nJointSpeed[8];

static CMani_driver m_mani;


int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_mani_test");
    ROS_INFO("[wpr1_mani_test]");
    ros::NodeHandle n;
    for(int i=0;i<8;i++)
    {
        fJointAngle[i] = 0;
        nJointSpeed[i] = 1000;
    }

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/wpr1_mani");
    m_mani.Open(strSerialPort.c_str(),115200);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(10);
    std::vector<double> joint_pos(10);

    joint_name[0] = "0_torso_lift";
    joint_name[1] = "1_shoulder_roll";
    joint_name[2] = "2_shoulder_flex";
    joint_name[3] = "3_upperarm_roll";
    joint_name[4] = "4_elbow_flex";
    joint_name[5] = "5_forearm_roll";
    joint_name[6] = "6_wrist_flex";
    joint_name[7] = "7_gripper";
    joint_name[8] = "kinect_height";
    joint_name[9] = "kinect_pitch";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.0f;
    joint_pos[5] = 0.0f;
    joint_pos[6] = 0.0f;
    joint_pos[7] = 0.0f;
    joint_pos[8] = 1.03f;    //kinect_height
    joint_pos[9] = -0.22f;    //kinect_pitch

    int nCount = 100;
    int nFlag = 0;
    ros::Rate r(30);
    while(n.ok())
    {
        m_mani.ReadNewData();
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;
        msg.name = joint_name;
        msg.position = joint_pos;
        
        //////////////////////////
        double fTmp = 0;
        //脊柱升降
        fTmp = m_mani.nRecvJointPos[0];
        joint_pos[0] = fTmp*0.001;//脊柱升降

        //手臂关节
        for(int i=1;i<7;i++)
        {
            fTmp = m_mani.nRecvJointPos[i];
            fTmp *= 0.01;
            joint_pos[i] = fTmp*fDegToAng;
        }

        //手爪
        fTmp = m_mani.nRecvJointPos[7];
        if(fTmp < 5000) fTmp = 0;
        fTmp *= 0.5/38000;
        joint_pos[7] = fTmp;
        joint_state_pub.publish(msg);
      
        // 测试控制
        if(nCount > 300)
        {
            nCount = 0;
            if(nFlag == 0)
            {
                ROS_WARN("[wpr1_mani_test] 0 ");
                fJointAngle[0] = 0.1;
                fJointAngle[1] = -45;
                fJointAngle[2] = -90;
                fJointAngle[3] = 0;
                fJointAngle[4] = -90;
                fJointAngle[5] = -90;
                fJointAngle[6] = -90;
                fJointAngle[7] = 27000;
                nFlag ++;
            }
            else
            {
                ROS_WARN("[wpr1_mani_test] 1 ");
                fJointAngle[0] = 0;
                fJointAngle[1] = 0;
                fJointAngle[2] = -60;
                fJointAngle[3] = 90;
                fJointAngle[4] = -120;
                fJointAngle[5] = -90;
                fJointAngle[6] = 60;
                fJointAngle[7] = 47000;
                nFlag = 0;
            }
            m_mani.Set8Joints(fJointAngle, nJointSpeed);
        }
        nCount ++;

        ros::spinOnce();
        r.sleep();
    }
}