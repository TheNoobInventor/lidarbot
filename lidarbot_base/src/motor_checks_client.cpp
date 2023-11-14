// Client node requests motor checks to be carried out

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// C++ namespace for representing time durations
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    // Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // Create a shared pointer to a Node type and name it "motor_checks_client"
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motor_checks_client");

    // Create a client inside the node to call the "checks" server node
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = 
        node->create_client<std_srvs::srv::Trigger>("checks");

    // Create the request, which is empty
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Wait 2 seconds for the service to be activated
    while (!client->wait_for_service(2s)) {
        // if ROS is shutdown before the service is activated, show this error
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        // Print in the screen some information so the user knows what is happening
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // Client sends its asynchronous request
    auto result = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        // Get the response's success field to see if all checks passed
        if (result.get()->success) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The checks were successful!");
        } else {
            // TODO: Message below does get outputted. Conversion seems to be the culprit
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The checks were not successful: %s", result.get()->message.c_str());
        }
    } else {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service 'checks'");
    }

    // Shut down rclcpp and client node
    rclcpp::shutdown();
    return 0;
}
