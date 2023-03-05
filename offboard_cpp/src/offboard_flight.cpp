/**
 * @author Corradini Giacomo
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

/**
 * @brief OffboardControl class
*/
class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
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
		vehicle_status_sub_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status",qos,std::bind(&OffboardControl::vehicle_status_callback,this,_1));
		timesync_sub_ = this->create_subscription<TimesyncStatus>("fmu/timesync/out",qos,[this](const TimesyncStatus::UniquePtr msg){
			timestamp_.store(msg->timestamp);
		});

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

		/**
		 * @brief Timer callback
		*/
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ < 10) {
				// offboard_control_mode needs to be paired with trajectory_setpoint
				publish_offboard_control_mode();
				publish_trajectory_setpoint_take_off();
			}	
			
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			if (offboard_setpoint_counter_ > 30 && offboard_setpoint_counter_ < 200) {
				
				publish_offboard_control_mode();
				publish_trajectory_setpoint_circle();

			}

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ == 400) {
				
				// Land the vehicle
				this->land();

				// Disarm
				this->disarm();
			}
			
			if (offboard_setpoint_counter_ < 501) {
				offboard_setpoint_counter_++;
			}

		};

		// Start publisher timer
		timer_ = this->create_wall_timer(100ms, timer_callback);

		this->theta = 0.0;
		this->radius = 10.0;
		this->omega = 0.5;
		this->time_period = 0.1;
	}

	void arm();
	void disarm();
	void land();

private:

	/**
	 * @brief Important variable
	*/
	rclcpp::TimerBase::SharedPtr timer_;
	uint8_t nav_state;
	uint8_t arming_state;
	std::atomic<uint64_t> timestamp_;    //!< common synced timestamped
	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	double theta = 0.0;
	double radius = 10.0;
	double omega = 0.5;
	double time_period = 0.1;

	/**
	 * @brief Publisher
	*/
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	/**
	 * @brief Subscriber
	*/
	rclcpp::Subscription<TimesyncStatus>::SharedPtr timesync_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint_take_off();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
	void publish_trajectory_setpoint_circle();
	void vehicle_status_callback(const VehicleStatus & msg);
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
 * @brief Send a command to Land the vehicle
 */
void OffboardControl::land()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

	RCLCPP_INFO(this->get_logger(), "Land command send");
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
	msg.attitude = true;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief 
*/
void OffboardControl::vehicle_status_callback(const VehicleStatus & msg)
{
	this->arming_state = msg.arming_state;
	this->nav_state = msg.nav_state;
	//RCLCPP_INFO(this->get_logger(), "ArmingState: %i\nNavState: %i\n", this->arming_state, this->nav_state);
	RCLCPP_INFO(this->get_logger(), "Setpoint: %li\n", this->offboard_setpoint_counter_);

}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint_take_off()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle flight in circle
 */
void OffboardControl::publish_trajectory_setpoint_circle()
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

/**
 * @brief main
*/

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}