#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;


class MocapPX4Bridge : public rclcpp::Node
{
public:
	MocapPX4Bridge() : Node("mocap_px4_bridge") {

		/**
		 * @brief quality of service
		*/
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

		auto qos = rclcpp::QoS(
				   rclcpp::QoSInitialization(
				   qos_profile.history,
				   qos_profile.depth
				   ),qos_profile);

		/**
		 * @brief Create subscriber
		*/
		vehicle_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name,qos,std::bind(&MocapPX4Bridge::poseCallback,this,_1));
		
		/**
		 * @brief Create publisher
		*/
		vehicle_odometry_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",qos);

	}

private:

	std::string topic_name = "/Drone/pose";
	bool flag_print = 0;
	bool ground_pos_flag = 0;
	float x_ground = 0.0;
	float y_ground = 0.0;
	float z_ground = 0.0;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_pub;

	void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
};

void MocapPX4Bridge::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	
	px4_msgs::msg::VehicleOdometry vehicle_odometry_Msg;

	vehicle_odometry_Msg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	vehicle_odometry_Msg.timestamp_sample = vehicle_odometry_Msg.timestamp;

	vehicle_odometry_Msg.pose_frame = vehicle_odometry_Msg.POSE_FRAME_NED;

	if(ground_pos_flag == 0){
		x_ground = poseMsg->pose.position.x;
		y_ground = poseMsg->pose.position.y;
		z_ground = poseMsg->pose.position.z;
		ground_pos_flag = 1;
		printf("Initial position saved\n");
	}


	/**
	 * @brief Convert Mocap reference frame into PX4 reference frame
	*/
	vehicle_odometry_Msg.position[0] = - (poseMsg->pose.position.y - y_ground);
	vehicle_odometry_Msg.position[1] = - (poseMsg->pose.position.x - x_ground);
	vehicle_odometry_Msg.position[2] = - (poseMsg->pose.position.z - z_ground);

	vehicle_odometry_Msg.q[0] = poseMsg->pose.orientation.w;  // W = -W
	vehicle_odometry_Msg.q[1] = poseMsg->pose.orientation.x;  // X = W
	vehicle_odometry_Msg.q[2] = -poseMsg->pose.orientation.y; // Y = -Y
	vehicle_odometry_Msg.q[3] = -poseMsg->pose.orientation.z; // Z = -Z

	if (flag_print){
		std::cout << "X: " << vehicle_odometry_Msg.position[0] << '\t' << "Y: " << vehicle_odometry_Msg.position[1] << '\t' << "Z: " << vehicle_odometry_Msg.position[2] << "\n";
	}

	vehicle_odometry_pub -> publish(vehicle_odometry_Msg);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
