/***
 * 控制机械臂移动到目标点位，点动
 ***/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_control"); //初始化
	ros::AsyncSpinner spinner(1); //多线程
	spinner.start(); //开启新的线程

    moveit::planning_interface::MoveGroupInterface arm("arm"); //初始化需要使用move group控制的机械臂中的arm group
  
    std::string end_effector_link = arm.getEndEffectorLink(); //获取终端link的名称
    std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;
   
    std::string reference_frame = "base_link"; //设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true); //当运动规划失败后，允许重新规划
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalOrientationTolerance(0.01);   
    arm.setMaxAccelerationScalingFactor(0.2); //设置允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.2);

    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
    std::cout<<"now Robot position: [x,y,z]: ["<<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"<<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["<<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z
       <<","<<now_pose.orientation.w<<"]"<<std::endl;
  
    arm.setNamedTarget("init_pose");    // 控制机械臂先回到初始化位置
    arm.move();
    sleep(1);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose1;
    pose1.position.x = -0.017;
	pose1.position.y = 0.154;	
	pose1.position.z = 0.852;
	pose1.orientation.x = 0.0;
	pose1.orientation.y = 0.707;
	pose1.orientation.z = 0;
	pose1.orientation.w = 0.707;
	waypoints.push_back(pose1); 


	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.002;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
	    // 执行运动
	    arm.execute(plan);    
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    
	ros::shutdown(); 
	return 0;
}