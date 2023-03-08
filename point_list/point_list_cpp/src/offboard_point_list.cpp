/**
 * @author Corradini Giacomo
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <vector>
#include <algorithm>

class offboard_point_list
{

public:
	float position_x; 
	float position_y; 
	float position_z; 
	float position_yaw;

	offboard_point_list(float position_x, float position_y, float position_z, float position_yaw);
	~offboard_point_list();
};

offboard_point_list::offboard_point_list(float position_x, float position_y, float position_z, float position_yaw)
{
	this->position_x   = position_x; 
	this->position_y   = position_y; 
	this->position_z   = position_z; 
	this->position_yaw = position_yaw;
}

offboard_point_list::~offboard_point_list()
{
}

float distance(float x1, float y1, float z1, float x2, float y2, float z2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2) * 1.0);
}

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
		 * @brief add points to point list
		*/
		this->point_list.push_back(this->point_1);
		this->point_list.push_back(this->point_2);

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
		vehicle_position_sub_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position",qos,std::bind(&OffboardControl::vehicle_position_callback,this,_1));

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

			if (offboard_setpoint_counter_ == 10) {
				// Change to publish command mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}
			
			if(point_list.size() > this->index){
				publish_offboard_control_mode();
				publish_trajectory(this->point_list);
				if(point_list.size() == this->index) this->flag_to_land = offboard_setpoint_counter_ + 50;
			} else if(offboard_setpoint_counter_ == this->flag_to_land && offboard_setpoint_counter_ > 0) {
				
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				
				this->land_to_launch();

				// Disarm
				this->disarm();

				this->flag_to_shutdown = offboard_setpoint_counter_ + 100;
			}
			

			if(offboard_setpoint_counter_ == this->flag_to_shutdown && offboard_setpoint_counter_ > 0) rclcpp::shutdown();

			offboard_setpoint_counter_++;
		};

		// Start publisher timer
		timer_ = this->create_wall_timer(100ms, timer_callback);		
	}

	void arm();
	void disarm();
	void land();
	void land_to_launch();

private:

	/**
	 * @brief Important variable
	*/
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;    //!< common synced timestamped
	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
	uint64_t flag_to_shutdown = 0;
	uint64_t flag_to_land = 0;
	size_t index = 0;
	float tolerance = 0.05;     // tolerance

	/**
	 * @brief messages
	*/
	VehicleLocalPosition message;

	/**
	 * @brief Publisher
	*/
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	/**
	 * @brief Subscriber
	*/
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_sub_;

	/**
	 * @brief Points list
	*/
	offboard_point_list* point_1 = new offboard_point_list(0.,0.,-1.,-M_PI);
	offboard_point_list* point_2 = new offboard_point_list(1.,0.,-1.,-M_PI);
	std::vector<offboard_point_list *> point_list;

	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
	void vehicle_status_callback(const VehicleStatus & msg);
	void vehicle_position_callback(const VehicleLocalPosition & msg);
	void publish_trajectory_setpoint(offboard_point_list* point);
	void publish_trajectory(std::vector<offboard_point_list *> point_list);
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
 * @brief Send a command to Land the vehicle in the current position
 */
void OffboardControl::land()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

	RCLCPP_INFO(this->get_logger(), "Land to current position command send");
}

/**
 * @brief Send a command to Land the vehicle in the current position
 */
void OffboardControl::land_to_launch()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);

	RCLCPP_INFO(this->get_logger(), "Land to launch position command send");
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
 * @param param3    Command parameter 3
 * @param param4    Command parameter 4
 * @param param5    Command parameter 5
 * @param param6    Command parameter 6
 * @param param7    Command parameter 7
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
 * @brief vehicle status callback
*/
void OffboardControl::vehicle_status_callback(const VehicleStatus & msg)
{
	RCLCPP_INFO(this->get_logger(), "\nArming state: %i, Navigation state: %i\n", msg.arming_state, msg.nav_state);
}

/**
 * @brief vehicle position callback
*/
void OffboardControl::vehicle_position_callback(const VehicleLocalPosition & msg)
{	
	this->message = msg;
	RCLCPP_INFO(this->get_logger(),"\nX coordinate: %f\nY coordinate %f\nZ coordinate %f", message.x, message.y, message.z);
}

/**
 * @brief Publish a setpoint
 */
void OffboardControl::publish_trajectory_setpoint(offboard_point_list* point)
{
	TrajectorySetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position[0] = point->position_x;
	msg.position[1] = point->position_y;
	msg.position[2] = point->position_z;
	msg.yaw = point->position_yaw;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Pulblish a trajectory set points
*/
void OffboardControl::publish_trajectory(std::vector<offboard_point_list *> point_list)
{
	float dist = distance(this->message.x, this->message.y, this->message.z, point_list[this->index]->position_x, point_list[this->index]->position_y, point_list[this->index]->position_z);
	
	if(dist > this->tolerance){
		this->publish_trajectory_setpoint(point_list[this->index]);
		RCLCPP_INFO(this->get_logger(), "Distance %f, index %ld\n", dist, this->index);
	} else if (dist <= this->tolerance){
		this->index++;
	}
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