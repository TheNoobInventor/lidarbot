// Server node runs motor tests to confirm that the motor is working correctly by moving both motors forward

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lidarbot_base/motor_encoder.h"

// Reset pulse counters
void reset_pulse_counters()
{
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
}

// Move a specified motor forward
bool move_motor(int motor_id)
{   
    // Motor_id
    // MOTORA - 0 (left motor)
    // MOTORB - 1 (right motor)
    reset_pulse_counters();
    sleep(2);
    
    // Move motor FORWARD for 2 seconds at 50% speed
    Motor_Run(motor_id, FORWARD, 50);
    sleep(2);
    Motor_Stop(motor_id);

    // Motor counts should be greater than 0 to confirm that the motor moved forward
    if (motor_id == MOTORA) {
        if (left_wheel_pulse_count> 0) {
            return true;
        } else return false;
    }
    if (motor_id == MOTORB) {
        if (right_wheel_pulse_count > 0) {
            return true;
        } return false;
    }
}

// DDS helps pass the request and response between client and server
void checkMotors(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // Prepare response
    response->success = true;
    response->message = "";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to check motors...");

    // Left motor check
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking left motor...");
    auto left_motor_passed = move_motor(MOTORA);
    if (!left_motor_passed) {
        response->success = false;
        response->message += "Left motor check failed, confirm motor wiring.";
    }

    // Right motor check
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking right motor...");
    auto right_motor_passed = move_motor(MOTORB);
    if (!right_motor_passed) {
        response->success = false;
        response->message += "Right motor check failed, confirm motor wiring.";
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response...");
}

int main(int argc, char **argv)
{
    // Initialize motor driver
    Motor_Init();

    // Initialize wiringPi using GPIO BCM pin numbers
    wiringPiSetupGpio();
    
    // Setup GPIO encoder interrupt and direction pins
    pinMode(LEFT_WHL_ENC_INT, INPUT);
    pinMode(RIGHT_WHL_ENC_INT, INPUT);
    pinMode(LEFT_WHL_ENC_DIR, INPUT);
    pinMode(RIGHT_WHL_ENC_DIR, INPUT);

    // Setup pull up resistors on encoder pins
    pullUpDnControl(LEFT_WHL_ENC_INT, PUD_UP);
    pullUpDnControl(RIGHT_WHL_ENC_INT, PUD_UP);

    // Initialize encoder interrupts for falling signal states
    wiringPiISR(LEFT_WHL_ENC_INT, INT_EDGE_FALLING, left_wheel_pulse);
    wiringPiISR(RIGHT_WHL_ENC_INT, INT_EDGE_FALLING, right_wheel_pulse);

    // Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // Create a shared pointer to a Node type and name it "motor_checks_server"
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motor_checks_server");

    // Create a "checks" service with a checkMotors callback
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = 
        node->create_service<std_srvs::srv::Trigger>("checks", &checkMotors);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to check motors");

    // Spin the node until it's terminated
    rclcpp::spin(node);
    rclcpp::shutdown();
}
