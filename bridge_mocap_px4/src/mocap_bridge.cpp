#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <vector>
#include <algorithm>
#include <cmath>

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
		vehicle_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/Car/pose",qos,std::bind(&MocapPX4Bridge::poseCallback,this,_1));
		
		/**
		 * @brief Create publisher
		*/
		vehicle_odometry_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_mocap_odometry",qos);

		// auto timer_callback = [this]() -> void {
		// 	std::cout << "ciao" << std::endl;

		// };

		// timer_ = this->create_wall_timer(100ms, timer_callback);		

	}

private:

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_pub;

	void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
};

void MocapPX4Bridge::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	
	px4_msgs::msg::VehicleOdometry vehicle_odometry_Msg;

	vehicle_odometry_Msg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	vehicle_odometry_Msg.timestamp_sample = vehicle_odometry_Msg.timestamp;

	vehicle_odometry_Msg.pose_frame = vehicle_odometry_Msg.POSE_FRAME_NED;

	/**
	 * @brief Convert Mocap reference frame into PX4 reference frame
	*/
	vehicle_odometry_Msg.position[0] = poseMsg->pose.position.x;
	vehicle_odometry_Msg.position[1] = -poseMsg->pose.position.y;
	vehicle_odometry_Msg.position[2] = -poseMsg->pose.position.z;

	vehicle_odometry_Msg.q[0] = -poseMsg->pose.orientation.w; // W = W
	vehicle_odometry_Msg.q[1] = poseMsg->pose.orientation.x;  // X = W
	vehicle_odometry_Msg.q[2] = -poseMsg->pose.orientation.y; // Y = -Y
	vehicle_odometry_Msg.q[3] = poseMsg->pose.orientation.z;  // Z = -Z

	std::cout << "X: " << vehicle_odometry_Msg.position[0] << '\t' <<
				 "Y: " << vehicle_odometry_Msg.position[1] << '\t' <<
				 "Z: " << vehicle_odometry_Msg.position[2] << "\n";

	vehicle_odometry_pub -> publish(vehicle_odometry_Msg);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
