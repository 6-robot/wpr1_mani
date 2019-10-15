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
#include <std_msgs/Int32MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float position[7];
    int velocity[7];
}st_wpm_pose;

typedef struct
{
    float fGapSize;
    int nGripperPos;
    double fPosePerMM;
}stGripperPos;
static vector<stGripperPos> arGripperPos;

static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double fJointAngle[8];
static int nJointSpeed[8];
static double pos_send[7];
static int vel_send[7];
static st_wpm_pose tmpPose;
static vector<st_wpm_pose> arPose;
static int nExecIndex = 0;
static bool bExecPath = false;
static bool bExecToGoal = true;

static int nRecvJointPos[8];
static int nTargetJointPos[8];

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperServer;

void Set8Joints(double* inPosition, int* inSpeed)
{
    nTargetJointPos[0] = inPosition[0];  //升降
    //手臂关节
    for(int i=1;i<7;i++)
    {
        nTargetJointPos[i] = inPosition[i]*100;
    }
    nTargetJointPos[7] = inPosition[7];         //手爪 
}

void JointCtrlDegreeCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nNumJoint = msg->position.size();
    // for(int i=0;i<nNumJoint;i++)
    // {
    //     ROS_INFO("[wpr1_mani_cb] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    // }
    if(nNumJoint != 8)
    {
        ROS_WARN("[wpr1_manipulator] nNumJoint = %d .It should be 8!",nNumJoint);
        return;
    }
    //脊柱升降
    fJointAngle[0] = msg->position[0] * 1000;
    nJointSpeed[0] = msg->velocity[0];
    //手臂和手爪
    for(int i=1;i<8;i++)
    {
        fJointAngle[i] = msg->position[i];
        nJointSpeed[i] = msg->velocity[i];
        //nRecvJointPos[i] = fJointAngle[i] * 100;  //测试用，将下发值赋值到反馈值
    }

    Set8Joints(fJointAngle, nJointSpeed);
}

void JointCtrlAngleCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nNumJoint = msg->position.size();
    // for(int i=0;i<nNumJoint;i++)
    // {
    //     ROS_INFO("[wpr1_mani_cb] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    // }
    if(nNumJoint != 8)
    {
        ROS_WARN("[wpr1_manipulator] nNumJoint = %d .It should be 8!",nNumJoint);
        return;
    }
    //脊柱升降
    fJointAngle[0] = msg->position[0] * 1000;
    nJointSpeed[0] = msg->velocity[0];
    //手臂根关节
    for(int i=1;i<7;i++)
    {
        fJointAngle[i] = msg->position[i] * fAngToDeg;
        nJointSpeed[i] = msg->velocity[i];
    }
    //手爪
    fJointAngle[7] = msg->position[7];
    nJointSpeed[7] = msg->velocity[7];

    Set8Joints(fJointAngle, nJointSpeed);
    //ROS_INFO("[wpr1_mani_SetJoints] %.0f %.0f %.0f %.0f %.0f", fJointAngle[0],fJointAngle[1],fJointAngle[2],fJointAngle[3],fJointAngle[4]);
    //ROS_INFO("[wpr1_mani_SetVel] %d %d %d %d %d", nJointSpeed[0],nJointSpeed[1],nJointSpeed[2],nJointSpeed[3],nJointSpeed[4]);
}

bool poseArrived()
{
    bool bArrived = true;
    for(int i=0;i<6;i++)
    {
        double fDegDiff = fabs(arPose[nExecIndex].position[i] - (nRecvJointPos[i+1]*0.01));
        if(fDegDiff > 1)
        {
            //未运动到目标点
            //ROS_INFO("posArrived i=%d targe=%.2f recv=%.2f fDegDiff=%.2f",i,arPose[nExecIndex].position[i],nRecvJointPos[i+1]*0.01,fDegDiff);
            bArrived = false;
        }
    }
    return bArrived;
}

// 响应 Move_Group 的轨迹执行
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, TrajectoryServer* as)
{
    arPose.clear();
    int nrOfPoints = goal->trajectory.points.size();
    ROS_INFO("Trajectory with %d positions received \n", nrOfPoints);
    //是否快速执行
    if(bExecToGoal == false)
    {
        nExecIndex = 0;
    }
    else
    {
        nExecIndex = nrOfPoints - 1;
    }
    //解析关键帧列表
    for(int i=0; i<nrOfPoints; i++)
    {
        int nPos =  goal->trajectory.points[i].positions.size();
        //ROS_WARN("point[%d] hase %d positions\n",i, nPos);
        if(nPos > 7)
        nPos = 7;
        for(int j=0;j<nPos;j++)
        {
            tmpPose.position[j] = goal->trajectory.points[i].positions[j] * fAngToDeg;
            tmpPose.velocity[j] = 1000;
        }
        arPose.push_back(tmpPose);

        /////////////////////////////////////////////////
        // ROS_WARN("point[%d] pos %.1f %.1f %.1f %.1f %.1f %.1f",i,
        // goal->trajectory.points[i].positions[0],
        // goal->trajectory.points[i].positions[1],
        // goal->trajectory.points[i].positions[2],
        // goal->trajectory.points[i].positions[3],
        // goal->trajectory.points[i].positions[4],
        // goal->trajectory.points[i].positions[5]);
        //  ROS_WARN("point[%d] vel %.1f %.1f %.1f %.1f %.1f %.1f",i,
        // goal->trajectory.points[i].velocities[0],
        // goal->trajectory.points[i].velocities[1],
        // goal->trajectory.points[i].velocities[2],
        // goal->trajectory.points[i].velocities[3],
        // goal->trajectory.points[i].velocities[4],
        // goal->trajectory.points[i].velocities[5]);
        // ROS_WARN("point[%d] acc %.1f %.1f %.1f %.1f %.1f %.1f",i,
        // goal->trajectory.points[i].accelerations[0],
        // goal->trajectory.points[i].accelerations[1],
        // goal->trajectory.points[i].accelerations[2],
        // goal->trajectory.points[i].accelerations[3],
        // goal->trajectory.points[i].accelerations[4],
        // goal->trajectory.points[i].accelerations[5]);
        // ROS_WARN("time_from_start = %f",goal->trajectory.points[i].time_from_start.toSec());
        // ROS_WARN("--------------------------------------------------");
        /////////////////////////////////////////////////
    }
    
    bExecPath = true;
    
    ros::Rate r(30);
    while( bExecPath == true)
    {
        if(poseArrived() == true)
        {
            nExecIndex ++;
        }
        if(nExecIndex >= arPose.size())
        {
            // 执行完毕
            bExecPath = false;
            ROS_INFO("ExecPath done!");
        }
        else
        {
            //未执行完毕
            for(int i=0;i<6;i++)
            {
                pos_send[i] = arPose[nExecIndex].position[i];
                vel_send[i] = arPose[nExecIndex].velocity[i];
            }
            
            // 速度优化
            double fTmpPosDiff[6];
            for(int i=0;i<6;i++)
            {
                fTmpPosDiff[i] = fabs(pos_send[i] - nRecvJointPos[i+1]*0.01);
            }
            // 找出位置差最大的值,以它为最大速度
            double fDiffMax = 0;
            int nDiffMaxIndex = 0;
            for(int i=0;i<6;i++)
            {
                if(fTmpPosDiff[i] > fDiffMax)
                {
                    fDiffMax = fTmpPosDiff[i];
                    nDiffMaxIndex = i;
                }
            }
            // 计算运动速度
            int nMaxVelocity = 4000;
            if(fDiffMax > 0)
            {
                for(int i=0;i<6;i++)
                {
                    double tmpVel = fTmpPosDiff[i];
                    tmpVel /= fDiffMax;
                    tmpVel *= nMaxVelocity;
                    vel_send[i] = tmpVel;
                    if(vel_send[i] < 1000)
                        vel_send[i] = 1000;
                }
            }
            
            for(int i=0;i<6;i++)
            {
                fJointAngle[i+1] = pos_send[i];
                nJointSpeed[i+1] = vel_send[i];
                //ROS_INFO("[executeTrajectory] %d - s_pos=%.2f d_pos=%.2f spd=%d", i, pos_send[i],goal->trajectory.points[nExecIndex].positions[i] * fAngToDeg,vel_send[i]);  //显示规划路径的角度值
            }
            Set8Joints(fJointAngle, nJointSpeed);
            //ROS_WARN("point= %d   nExecIndex= %d",nrOfPoints, nExecIndex);
        }

        r.sleep();
    }

    as->setSucceeded();
}

void InitGripperPosVal()
{
    stGripperPos tmpGP;
    tmpGP.fGapSize = 0.081;
    tmpGP.nGripperPos = 38000;
    tmpGP.fPosePerMM = 0;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.074;
    tmpGP.nGripperPos = 35000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.061;
    tmpGP.nGripperPos = 30000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.048;
    tmpGP.nGripperPos = 25000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.034;
    tmpGP.nGripperPos = 20000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.020;
    tmpGP.nGripperPos = 15000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.006;
    tmpGP.nGripperPos = 10000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.000;
    tmpGP.nGripperPos = 5000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.000;
    tmpGP.nGripperPos = 0;
    arGripperPos.push_back(tmpGP);

    int nNumGP = arGripperPos.size();
    for(int i=1;i<nNumGP;i++)
    {
        double fDiffSize = fabs(arGripperPos[i].fGapSize - arGripperPos[i-1].fGapSize);
        int nDiffPos = fabs(arGripperPos[i].nGripperPos - arGripperPos[i-1].nGripperPos);
        arGripperPos[i].fPosePerMM = fDiffSize/nDiffPos;
        //ROS_WARN("i=%d fPosePerMM =%f",i,arGripperPos[i].fPosePerMM);
    }
}

// 手爪位置计算
int CalGripperPos(float inGapSize)
{
    int nNumGP = arGripperPos.size();
    int nRetGripperPos = 0;
    if(nNumGP > 0)
    {
        int nIndexGP = 0;
        if(inGapSize >= arGripperPos[nIndexGP].fGapSize)
        {
            nRetGripperPos = arGripperPos[nIndexGP].nGripperPos;
            return nRetGripperPos;
        }
        for(int i=1;i<nNumGP;i++)
        {
            if(inGapSize > arGripperPos[i].fGapSize)
            {
                nIndexGP = i;
                break;
            }
        }
        if(nIndexGP < nNumGP)
        {
            double fDiffGapSize = fabs(inGapSize - arGripperPos[nIndexGP].fGapSize);
            int nDiffGripperPos = (fDiffGapSize/arGripperPos[nIndexGP].fPosePerMM);
            nRetGripperPos = arGripperPos[nIndexGP].nGripperPos + nDiffGripperPos;
        }
    }
    return nRetGripperPos;
}

// 响应 GripperCommand 回调函数
void executeGripper(const control_msgs::GripperCommandGoalConstPtr & goal, GripperServer* as)
{
	float gapSize = goal->command.position;
    float maxEffort = goal->command.max_effort;

    int nGripperPos = CalGripperPos(gapSize);
    ROS_WARN("[executeGripper]gapSize = %f, gripprPos = %d", gapSize, nGripperPos);

    // 执行指令
    fJointAngle[7] = nGripperPos;
    Set8Joints(fJointAngle, nJointSpeed);

    // 监测执行目标是否完成
    while(ros::ok())
    {
        int nDiff = abs(nGripperPos - nRecvJointPos[7]);
        if(nDiff < 100)
        {
            break;
        }
    }

	as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_mani_sim");
    ROS_INFO("[wpr1_mani_sim]");
    ros::NodeHandle n;
    ros::Subscriber joint_ctrl_ang_sub = n.subscribe("/wpr1/joint_ctrl_angle",30,&JointCtrlAngleCallback);
    ros::Subscriber joint_ctrl_deg_sub = n.subscribe("/wpr1/joint_ctrl_degree",30,&JointCtrlDegreeCallback);

	TrajectoryServer tserver(n, "wpr1_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
    tserver.start();
    InitGripperPosVal();
    GripperServer gserver(n, "wpr1_gripper_controller/gripper_command", boost::bind(&executeGripper, _1, &gserver), false);
 	ROS_INFO("GripperActionServer: Starting");
    gserver.start();

    for(int i=0;i<8;i++)
    {
        fJointAngle[i] = 0;
        nJointSpeed[i] = 1500;
    }
    fJointAngle[0] = 0;//"0_torso_lift"
    fJointAngle[1] = -45;//"1_shoulder_roll"
    fJointAngle[2] = -90;//"2_shoulder_flex"
    fJointAngle[3] = 0;//"3_upperarm_roll"
    fJointAngle[4] = -90;//"4_elbow_flex"
    fJointAngle[5] = -90;//"5_forearm_roll"
    fJointAngle[6] = -90;//"6_wrist_flex"
    fJointAngle[7] = 25000;//"7_gripper"

    nRecvJointPos[0] = nTargetJointPos[0] = fJointAngle[0] *1000;
    for (int i = 1; i < 7; i++)
	{
		nRecvJointPos[i] = nTargetJointPos[i] = fJointAngle[i]*100;
	}
    nRecvJointPos[7] = nTargetJointPos[7] = fJointAngle[7];

    ros::NodeHandle n_param("~");
    n_param.param<bool>("exec_to_goal", bExecToGoal, true);

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
        //模拟渐变
        int nStep = 100;
        for(int i=0;i<8;i++)
        {
            if(nRecvJointPos[i] != nTargetJointPos[i])
            {
                if(nRecvJointPos[i] > nTargetJointPos[i])
                {
                    int nDiff = nRecvJointPos[i] - nTargetJointPos[i];
                    if(nDiff > nStep)
                        nRecvJointPos[i] -= nStep;
                    else
                        nRecvJointPos[i] -= nDiff;
                }
                else
                {
                    int nDiff = nTargetJointPos[i] - nRecvJointPos[i] ;
                    if(nDiff > nStep)
                        nRecvJointPos[i] += nStep;
                    else
                        nRecvJointPos[i] += nDiff;
                }
            }
        }

        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;
        msg.name = joint_name;
        msg.position = joint_pos;

        double fTmp = 0;
        //脊柱升降
        fTmp = nRecvJointPos[0];
        joint_pos[0] = fTmp*0.001;//脊柱升降

        //手臂关节
        for(int i=1;i<7;i++)
        {
            fTmp = nRecvJointPos[i];
            fTmp *= 0.01;
            joint_pos[i] = fTmp*fDegToAng;
        }

        //手爪
        fTmp = nRecvJointPos[7];
        if(fTmp < 5000) fTmp = 0;
        fTmp *= 0.5/38000;
        joint_pos[7] = fTmp;

        joint_state_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
         ////////////////////////////////////////////////////////////
        // printf("[关节电机位置] ");
		// for (int i = 0; i < 7; i++)
		// {
		// 	printf("关节%d = %.6d ", i + 1, nRecvJointPos[i]);
		// }
		// printf("\n");
        ///////////////////////////////////////////////////////////
        // 测试控制用
        // nCount ++;
        // if(nCount > 300)
        // {
        //     nCount = 0;
        //     if(nFlag == 0)
        //     {
        //         ROS_WARN("[wpr1_mani] 0 ");
        //         fJointAngle[0] = 50;    //5cm
        //         fJointAngle[3] = -90;
        //         fJointAngle[4] = -40;
        //         fJointAngle[5] = 0;
        //         fJointAngle[6] = 7000;
        //         nFlag ++;
        //     }
        //     else
        //     {
        //         ROS_WARN("[wpr1_mani] 1 ");
        //         fJointAngle[0] = 0;
        //         fJointAngle[3] = 0;
        //         fJointAngle[4] = 0;
        //         fJointAngle[5] = 0;
        //         fJointAngle[6] = 25000;
        //         nFlag = 0;
        //     }
        //     SetJoints(fJointAngle, nJointSpeed);
        // }
        ///////////////////////////////////////////////////////////
    }
}
