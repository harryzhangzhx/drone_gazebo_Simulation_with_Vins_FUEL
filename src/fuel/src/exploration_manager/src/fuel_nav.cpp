/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

//#include <geometry_msgs/PoseStamped.h>

//#include <geometry_msgs/Twist.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include<mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/msg/odometry.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
visualization_msgs::msg::Marker trackpoint;
//ros::Publisher *pubMarkerPointer;

rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr pubMarkerPointer;

tf::StampedTransform ts;//用来发布无人机当前位置的坐标系坐标轴
tf::TransformBroadcaster *tfBroadcasterPointer;	//广播坐标轴
unsigned short velocity_mask = VELOCITY2D_CONTROL;
mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::msg::Odometry position_msg;
geometry_msgs::msg::PoseStamped target_pos;
mavros_msgs::State current_state;
float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw, targetpos_x, targetpos_y;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
bool receive = false, get_now_pos = false;//触发轨迹的条件判断
float pi = 3.14159265;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

//read vehicle odometry
void position_cb(const nav_msgs::msg::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_frame";
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer->sendTransform(ts);
	if (!get_now_pos) 
	{
		now_x = position_msg.pose.pose.position.x;
		now_y = position_msg.pose.pose.position.y;
		tf2::Quaternion quat;
		tf2::convert(msg->pose.pose.orientation, quat);
		now_yaw = yaw;
		get_now_pos = true;
	}
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
	current_yaw = yaw;

}

void target_cb(const geometry_msgs::msg::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
  target_pos = *msg;
  targetpos_x = target_pos.pose.position.x;
  targetpos_y = target_pos.pose.position.y;
}

//读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
quadrotor_msgs::PositionCommand ego;
void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
    receive = true;
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
	ego_vel_x = ego.velocity.x;
	ego_vel_y = ego.velocity.y;
	ego_vel_z = ego.velocity.z;
	ego_yaw = ego.yaw;
	ego_yaw_rate = ego.yaw_dot;
}

int main(int argc, char **argv)
{
	//ros::init(argc, argv, "cxr_egoctrl_v1");
	rclcpp::init(argc, argv);

	setlocale(LC_ALL,"");
	//ros::NodeHandle nh;
	auto node = rclcpp::Node::make_shared("cxr_egoctrl_v1");
	tf::TransformBroadcaster tfBroadcaster;//这个必须定义在init后边，所以用上了指针
	tfBroadcasterPointer = &tfBroadcaster;
	
	//ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	//("/mavros/state", 10, state_cb);//读取飞控状态的话题
	
	auto state_sub = node->create_subscription<mavros_msgs::State>("/mavros/state", 10, state_cb);
	
	
	//ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
	//("/mavros/setpoint_raw/local", 1); //这个话题很重要，可以控制无人机的位置速度加速度和yaw以及yaw-rate，里面有个掩码选择，需要注意
	
	auto local_pos_pub = node->create_publisher<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

	//ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker> ("/track_drone_point", 5);
    
	
	auto pubMarker = node->create_publisher<visualization_msgs::msg::Marker>("/track_drone_point", 5);
	pubMarkerPointer = &pubMarker;
	
	//ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	//("/mavros/cmd/arming");//控制无人机解锁的服务端，不需要用
	
	auto arming_client = node->create_service<mavros_msgs::CommandBool>("/mavros/cmd/arming");

	//ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>
	//("/mavros/cmd/command");
	
	auto command_client = node->create_service<mavros_msgs::CommandLong>("/mavros/cmd/command");

	//ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	//("/mavros/set_mode");//设置飞机飞行模式的服务端，也不需要用
	
	auto set_mode_client = node->create_service<mavros_msg::SetMode>("/mavros/set_mode");

	//ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
	//("/planning/pos_cmd", 10, twist_cb);//订阅egoplanner的规划指令话题的
    
	auto twist_sub = node->subscription<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, twist_cb);

	//ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>
	//("move_base_simple/goal", 10, target_cb);
	
	auto target_sub = node->subscription<geometery_msgs::msg::PoseStamped>("move_base_simple/goal", 10, target_cb);

	//ros::Subscriber position_sub=nh.subscribe<nav_msgs::Odometry>
    //("/vins_fusion/imu_propagate",10,position_cb);

	auto position_sub = node->create_subscription<nav_msgs::msg::Odometry>("/vins_fusion/imu_propagate", 10, position_cb);
    
	//ros::Rate rate(50.0); //控制频率尽可能高点，大于30hz
	rclcpp::Rate rate(50.0)

	while(rclcpp::ok() && !current_state.connected)
	{
		//ros::spinOnce();
		rclcpp::spin_all(node, 0s);
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode; //用不上
	offb_set_mode.request.custom_mode = "OFFBOARD";//用不上
	mavros_msgs::CommandBool arm_cmd;//用不上
	arm_cmd.request.value = true;//用不上
	//send a few setpoints before starting
	for(int i = 100; rclcpp::ok() && i > 0; --i)
	{
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		local_pos_pub->publish(current_goal);
		//ros::spinOnce();
		rclcpp::spin_all(node, 0s);
		rate.sleep();
	}
	while(rclcpp::ok())
	{
		//take off 1m
		if(!receive) //如果没有在rviz上打点，则offboard模式下会保持在1m的高度
		{
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			current_goal.header.stamp = node->now();
			current_goal.type_mask = velocity_mask;
			current_goal.velocity.x = (now_x - position_x)*1;;
			current_goal.velocity.y = (now_y - position_y)*1;;
			current_goal.velocity.z = (1 - position_z)*1;
			current_goal.yaw = now_yaw;
			RCLCPP_INFO(node->get_logger(), "请等待");
			local_pos_pub->publish(current_goal);
			//ros::spinOnce();
			rclcpp::spin_all(node, 0s);
			rate.sleep();
		}

		//if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
		if(receive)//触发后进行轨迹跟踪
		{
			float yaw_erro;
			yaw_erro = (ego_yaw - current_yaw);
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系，一定要local系
			current_goal.header.stamp = node->now();
			current_goal.type_mask = velocity_mask;//这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
			current_goal.velocity.x =  0.5 * ego_vel_x + (ego_pos_x - position_x)*1;
			current_goal.velocity.y =  0.5 * ego_vel_y + (ego_pos_y - position_y)*1;
			current_goal.velocity.z =  (ego_pos_z - position_z)*1;
			current_goal.yaw = ego_yaw;
			//ROS_INFO("EGO规划速度：vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2)+pow(current_goal.velocity.y, 2)));
			RCLCPP_INFO(node->get_logger(), "EGO规划速度：vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2)+pow(current_goal.velocity.y, 2)));
		}
		local_pos_pub->publish(current_goal);
		receive = false;
		//ros::spinOnce();
		rclcpp::spin_all(nde, 0s);
		rate.sleep();
	}

	return 0;
}