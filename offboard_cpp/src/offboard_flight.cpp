//   ___            _           _      
//  |_ _|_ __   ___| |_   _  __| | ___ 
//   | || '_ \ / __| | | | |/ _` |/ _ \
//   | || | | | (__| | |_| | (_| |  __/
//  |___|_| |_|\___|_|\__,_|\__,_|\___|
                                    
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>

//   _   _                                                
//  | \ | | __ _ _ __ ___   ___  ___ _ __   __ _  ___ ___ 
//  |  \| |/ _` | '_ ` _ \ / _ \/ __| '_ \ / _` |/ __/ _ \
//  | |\  | (_| | | | | | |  __/\__ \ |_) | (_| | (_|  __/
//  |_| \_|\__,_|_| |_| |_|\___||___/ .__/ \__,_|\___\___|
//                                  |_|                   

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

//    ____ _                  ___   __  __ _                         _  ____            _             _ 
//   / ___| | __ _ ___ ___   / _ \ / _|/ _| |__   ___   __ _ _ __ __| |/ ___|___  _ __ | |_ _ __ ___ | |
//  | |   | |/ _` / __/ __| | | | | |_| |_| '_ \ / _ \ / _` | '__/ _` | |   / _ \| '_ \| __| '__/ _ \| |
//  | |___| | (_| \__ \__ \ | |_| |  _|  _| |_) | (_) | (_| | | | (_| | |__| (_) | | | | |_| | | (_) | |
//   \____|_|\__,_|___/___/  \___/|_| |_| |_.__/ \___/ \__,_|_|  \__,_|\____\___/|_| |_|\__|_|  \___/|_|


class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		/**
		 * @brief of service
		*/

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

		auto qos = rclcpp::QoS(
			rclcpp::QoSInitialization(
				qos_profile.history,
				qos_profile.depth
			),qos_profile);
		
		// Set the durability setting to volatile
		// auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

		// Set the durability setting to transient local
		// auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

		/**
		 * @brief Create subscriber
		*/
		vehicle_status_sub_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status",qos,std::bind(&OffboardControl::vehicle_status_clbk,this,_1));

		/**
		 * @brief Create publisher
		*/
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

		/**
		 * @brief Set default variable
		*/
		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};

		// Start publisher timer
		timer_ = this->create_wall_timer(100ms, timer_callback);

		theta = 0.0;
		radius = 10.0;
		omega = 0.5;
		time_period = 0.02;
	}

	void arm();
	void disarm();
	void vehicle_status_callback(const VehicleStatus & msg);

private:

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_command_callback();
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void vehicle_status_callback(const VehicleStatus & msg)
{

}

void publish_command_callback()
{
	TrajectorySetpoint msg{};
	msg.position[0] = this->radius * cos(this->theta);
	msg.position[1] = this->radius * sin(this->theta);
	msg.position[2] = -5.0;
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	this->theta = this->theta + this->omega * this->time_period;
}














//   __  __       _       
//  |  \/  | __ _(_)_ __  
//  | |\/| |/ _` | | '_ \ 
//  | |  | | (_| | | | | |
//  |_|  |_|\__,_|_|_| |_|

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}