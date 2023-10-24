#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_constraints.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <vector>
#include <algorithm>
#include <cmath>

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
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
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
		vehicle_position_sub_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry",qos,std::bind(&OffboardControl::vehicle_position_callback,this,_1));

		/**
		 * @brief Create publisher
		*/
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);
		vehicle_constraints_publisher_ = this->create_publisher<VehicleConstraints>("/fmu/in/vehicle_constraints", qos);

		/**
		 * @brief Set default variable
		*/
		offboard_setpoint_counter_ = 0;

		/**
		 * @brief Timer callback
		*/
		auto timer_callback = [this]() -> void {

			this->publish_velocity_constraints();

			if (offboard_setpoint_counter_ == 10) {
				// Arm the vehicle
				this->takeoff();
				this->arm();
			}

			check_takeoff();
	
			if(point_list.size() > this->index && this->flag_takeoff == 1){
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				publish_offboard_control_mode();
				publish_trajectory(this->point_list);
				if(point_list.size() == this->index) this->flag_to_land = offboard_setpoint_counter_ + 10;
			} else if(offboard_setpoint_counter_ == this->flag_to_land && offboard_setpoint_counter_ > 0) {
				// Disarm
				this->land();
				this->disarm();

				this->flag_to_shutdown = offboard_setpoint_counter_ + 80;
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
	void takeoff();
	void loiter_time(float time);


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
	float tolerance = 1.00;              // tolerance
	float takeoff_height = -2;		 // default height for the takeoff
	uint flag_takeoff = 0;
	uint8_t nav_state;

	/**
	 * @brief messages
	*/
	VehicleOdometry message;

	/**
	 * @brief Publisher
	*/
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<VehicleConstraints>::SharedPtr vehicle_constraints_publisher_;
	/**
	 * @brief Subscriber
	*/
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_position_sub_;

	/**
	 * @brief Points list
	*/
	offboard_point_list* point_1 = new offboard_point_list(0.,0.,-2.5,std::nanf(""));
	offboard_point_list* point_2 = new offboard_point_list(2.,0.,-2.5,std::nanf(""));
	std::vector<offboard_point_list *> point_list;

	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = std::nanf(""), float param2 = std::nanf(""), float param3 = std::nanf(""), float param4 = std::nanf(""), float param5 = std::nanf(""), float param6 = std::nanf(""), float param7 = std::nanf(""));
	void vehicle_status_callback(const VehicleStatus & msg);
	void vehicle_position_callback(const VehicleOdometry & msg);
	void publish_trajectory_setpoint(offboard_point_list* point);
	void publish_trajectory(std::vector<offboard_point_list *> point_list);
	void publish_velocity_constraints();
	void check_takeoff();
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
 * @brief Send a command to takeoff the vehicle
 */
void OffboardControl::takeoff()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,std::nanf(""),std::nanf(""),std::nanf(""),std::nanf(""),std::nanf(""),std::nanf(""),takeoff_height);

	RCLCPP_INFO(this->get_logger(), "Takeoff command send");
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
 * @brief Send a command to Land the vehicle in the launch position
 */
void OffboardControl::land_to_launch()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);

	RCLCPP_INFO(this->get_logger(), "Land to launch position command send");
}

void OffboardControl::loiter_time(float time)
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LOITER_TIME,time);

	RCLCPP_INFO(this->get_logger(), "Loiter for %f seconds", time);
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
	// msg.attitude = true;
	// msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the velocity constraints
 */
void OffboardControl::publish_velocity_constraints()
{
	VehicleConstraints msg{};
	msg.speed_up = 0.5;
	msg.speed_down = 0.5;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_constraints_publisher_->publish(msg);
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
	this->nav_state = msg.nav_state;
	fprintf(stdout, "---------------------------------------------------------------------------\n");
	RCLCPP_INFO(this->get_logger(), "\n\nArming state: %i, Navigation state: %i\n\n", msg.arming_state, msg.nav_state);
	fprintf(stdout, "\n---------------------------------------------------------------------------");
}

/**
 * @brief vehicle position callback
*/
void OffboardControl::vehicle_position_callback(const VehicleOdometry & msg)
{	
	this->message = msg;
	// RCLCPP_INFO(this->get_logger(),"\nX: %f\nY: %f\nZ: %f", message.position[0], message.position[1], message.position[2]);
	// RCLCPP_INFO(this->get_logger(),"\nVX: %f\nVY: %f\nVZ: %f", message.velocity[0], message.velocity[1], message.velocity[2]);
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
	float dist = distance(this->message.position[0], this->message.position[1], this->message.position[2], point_list[this->index]->position_x, point_list[this->index]->position_y, point_list[this->index]->position_z);
	
	if(dist > this->tolerance){
		this->publish_trajectory_setpoint(point_list[this->index]);
		RCLCPP_INFO(this->get_logger(), "Distance %f, index %ld\n", dist, this->index);
	} else if (dist <= this->tolerance){
		//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		//his->loiter_time(2.0);
		this->index++;
	}
}

/**
 * @brief check if takeoff is completed
 */
void OffboardControl::check_takeoff(){
	
	float dist = distance(this->message.position[0], this->message.position[1], this->message.position[2], 0, 0, this->takeoff_height);
	
	if(dist < this->tolerance && this->nav_state == 4 && this->flag_takeoff == 0){
		this->flag_takeoff = 1;
		RCLCPP_INFO(this->get_logger(), "\n****Takeoff done!!****\n");
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
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