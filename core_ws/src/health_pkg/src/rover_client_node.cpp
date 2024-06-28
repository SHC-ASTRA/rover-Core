// ROS rclcpp Library
#include "rclcpp/rclcpp.hpp"
// Service Server Interface
#include "std_srvs/srv/trigger.hpp"
// Topic Message Interface
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <cstdlib>
#include <chrono>
#include <stdint.h>
#include <thread>

// Enable use of s, min, & other time literals
using namespace std::chrono_literals;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	
	// Node Name
	std::shared_ptr<rclcpp::Node> health_node = rclcpp::Node::make_shared("rover_health_client");

	// Publisher on the topic that will stop the rover
	// All modules should subscribe to this, and perform a hard stop
	// if a message - specifics TBD - is sent on here
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hard_stop_publisher =
		health_node->create_publisher<std_msgs::msg::String>("/astra/hard_stop", 10);

	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client =
		// Create a service client. The server (basestation) will have to host the same "/astra/core/health" service
		health_node->create_client<std_srvs::srv::Trigger>("/astra/core/health");
	
	// Error checking / checking for interruption (Ctrl+C)
	while(rclcpp::ok()) {

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attemping to connect to service server node.");

		// wait_for_service checks if there is a service server ready for a
		// provided duration, returning true when one is found
		// Otherwise returning false.
		// This will loop for one second and send a hard-stop message to the core node
		// https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1client_1_1ClientBase.html#a2999c323a3ee4935cd74d12675e668a0
		if (!service_client->wait_for_service(10s))
		{
			// Error checking / checking for interruption (Ctrl+C)
			if(!rclcpp::ok()) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while awaiting. Exiting");
				return 0;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Could not find service server after waiting, sending hard stop and trying again.");

			// Sending the hard stop
			auto message = std_msgs::msg::String();
			message.data = "HARD STOP DATA";
			hard_stop_publisher->publish(message);
		} 
		
		// Create a test request
		auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

		// Default to an error state
		// The thread will not sleep if there was an error state
		// and will attempt to immediately send another service request
		bool error_flag = true;

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request.");
		auto rover_data = service_client->async_send_request(request);
		// Wait for the result, timeout after 10 seconds
		// https://docs.ros2.org/beta3/api/rclcpp/namespacerclcpp.html#a21901a4dab8822c03c88e743486d2318
		switch(rclcpp::spin_until_future_complete(health_node, rover_data, 10s)) {
			case rclcpp::FutureReturnCode::SUCCESS:
				// No issues, go ahead and read the data
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Data returned.");
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message from rover: %s", rover_data.get()->message.c_str());
				// Change the error state to false
				error_flag = false;
				break;
			case rclcpp::FutureReturnCode::TIMEOUT:
				// Spinning "timed out"
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Waiting for a result timed out! Go ahead and try again?");
				break;
			case rclcpp::FutureReturnCode::INTERRUPTED:
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "User interrupted action.");
				// User-interrupted
				break;
		}

		// After a request has run, if there was no error state,
		// make the thread sleep for some time
		// This will be the time between the requests when there are
		// no apparent connection issues
		if(!error_flag) {
			std::this_thread::sleep_for(1s);
		}
	}
	// Shut down after rclcpp::ok() has an issue
	/*
		rclcpp::ok()

		This may return false for a context which has been shutdown,
		or for a context that was shutdown due to SIGINT being received by the rclcpp signal handler.
	*/
	rclcpp::shutdown();
	return 0;
}