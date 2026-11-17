#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include <cstdint>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "std_msgs/msg/bool.hpp"
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/autonomous_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class autonomous_drive : public rclcpp::Node
{
public:

  autonomous_drive()
    : Node("autonomous_driving_node")
  {
    // Initialize sequence state
    sequenceStarted = false;
    sequenceFinished = false;

    mustSafetyStop = true;
    elapsed_before_stop = 0.0f;

    // Initialize motor commands
    leftRearPwmCmd = 50;
    rightRearPwmCmd = 50;
    steeringAngle = 0;

    // Publisher: motor commands
    publisher_can_ =this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

    // Publish acknowledgement of having done a movement
    publisher_ack_ =this->create_publisher<interfaces::msg::AutonomousCommand>("movement_ack", 10);

    // Subscriber: autonomous start/stop command
    subscription_start_ =this->create_subscription<interfaces::msg::AutonomousCommand>("autonomous_command", 10,std::bind(&autonomous_drive::moveTypeCallback, this, _1));

    // Suscriber : obstacle stop command
    subscription_stop_ =this->create_subscription<std_msgs::msg::Bool>("obstacle_stop", 10,std::bind(&autonomous_drive::obstacleStopCallback, this, _1));

    // Timer: update commands every 50 ms
    //timer_ = this->create_wall_timer(50ms,std::bind(&autonomous_drive::updateCmd, this));

    // Timer updates SimonMoves each 50ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&autonomous_drive::moveSimon, this));

    RCLCPP_INFO(
		this->get_logger(),
		"autonomous_driving_node READY - waiting for command");
  }

private:

  // ==========================================
  // START / STOP CALLBACK
  // ==========================================

  void moveTypeCallback(
			const interfaces::msg::AutonomousCommand::SharedPtr msg)
  {
    moveType = msg->movetype;

    if (moveType != 0 && !sequenceStarted)
      {
	sequenceStarted = true;
	sequenceFinished = false;

	elapsed_before_stop = 0.0f;

	// Record sequence starting time
	time_start = this->now();

	RCLCPP_INFO(this->get_logger(), "Sequence STARTED");
      }
    else if (moveType == 0)
      {
	sequenceStarted = false;
	sequenceFinished = false;

	arreter();

	RCLCPP_INFO(this->get_logger(), "Sequence STOPPED");
      }
  }

  // ==========================================
  // OBSTACLE STOP CALLBACK
  // ==========================================

  void obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool previousStopState = mustSafetyStop;
    mustSafetyStop = msg->data;
    if (mustSafetyStop) {
      if (!previousStopState) {
	elapsed_before_stop = elapsed_before_stop + (this->now() - time_start).seconds();
	RCLCPP_INFO(this->get_logger(), "Sequence stopped due to obstacle");
      }
    }
    else {
      if (previousStopState) {
	time_start = this->now();
      }
    }
  }

  // ==========================================
  // PERIODIC COMMAND UPDATE
  // ==========================================

  /*
  void updateCmd()
  {
    auto motorsOrder = interfaces::msg::MotorsOrder();

    if (sequenceStarted && !sequenceFinished)
      {
	double elapsed = elapsed_before_stop + (this->now() - time_start).seconds();

	if (mustSafetyStop){
	  arreter();
	}
	else if (elapsed < 4.0)
	  {
	    // 0-3 seconds: first half of move
	    switch(moveType) {
	    case FORWARD:
	      avancer(0.5);
	      break;
	    case BACKWARD:
	      reculer(0.5);
	      break;
	    case LEFT:
	      gauche(0.5, false);
	      break;
	    case RIGHT:
	      droite(0.5, false);
	      break;
	    default:
	      arreter();
	    }
	  }
	else if (elapsed < 8.0)
	  {
	    // 4-8 seconds: turn right while moving
	    switch(moveType) {
	    case FORWARD:
	      reculer(0.5);
	      break;
	    case BACKWARD:
	      avancer(0.5);
	      break;
	    case LEFT:
	      gauche(0.5, true);
	      break;
	    case RIGHT:
	      droite(0.5, true);
	      break;
	    default:
	      arreter();
	    }
	  }
	else
	  {
	    // After 8 seconds: stop
	    arreter();

	    sequenceStarted = false;
	    sequenceFinished = true;

	    RCLCPP_INFO(
			this->get_logger(),
			"Sequence FINISHED");
	  }
      }
    else
      {
	// Stay stopped while waiting for a new command
	arreter();
      }

    // Fill motor message
    motorsOrder.left_rear_pwm = leftRearPwmCmd;
    motorsOrder.right_rear_pwm = rightRearPwmCmd;
    motorsOrder.steering_angle = steeringAngle;

    // Publish motor command
    publisher_can_->publish(motorsOrder);
  }
  */


  void moveSimon(){
    auto motorsOrder = interfaces::msg::MotorsOrder();
    auto ack_msg = interfaces::msg::AutonomousCommand();
        
    if (sequenceStarted && !sequenceFinished){
	double elapsed = elapsed_before_stop + (this->now() - time_start).seconds();
	if (mustSafetyStop){
	  arreter();
	}else if (elapsed < 4.0){
	    // 0-3 seconds: Moving in wanted direction
	    switch(moveType) {
	    case FORWARD:
	      avancer(0.5);
	      break;
	    case BACKWARD:
	      reculer(0.5);
	      break;
	    case LEFT:
	      gauche(0.5, false);
	      break;
	    case RIGHT:
	      droite(0.5, false);
	      break;
	    default:
	      arreter();
	    }
	  }else if(elapsed < 8.0){
	  // 4-8 seconds: Moving in the inverse of wanted direction
	  switch(moveType) {
	  case FORWARD:
	    reculer(0.5);
	    break;
	  case BACKWARD:
	    avancer(0.5);
	    break;
	  case LEFT:
	    gauche(0.5, true);
	    break;
	  case RIGHT:
	    droite(0.5, true);
	    break;
	  default:
	    arreter();
	  }
	}else{
	  // After 8 seconds: stop
	  arreter(); //Steering angle is initialised at 0degs
	  //movement_done = true; //On a finit le movement, faut attendre à nouveau un nouvel movement
	  sequenceStarted = false;
	  sequenceFinished = true;
	  ack_msg.movetype = moveType;
	  ack_msg.movement_done = true;
	  publisher_ack_->publish(ack_msg); //On publish l'acknowledgement au CAN, qui dit au generateur du jeu de basculer au prochain mouvement
	  RCLCPP_INFO(this->get_logger(),"Sequence FINISHED");
	}
      }
    else{arreter();}
    // Fill motor message
    motorsOrder.left_rear_pwm = leftRearPwmCmd;
    motorsOrder.right_rear_pwm = rightRearPwmCmd;
    motorsOrder.steering_angle = steeringAngle;
    // Publish motor command
    publisher_can_->publish(motorsOrder);
  }


  // ==========================================
  // MOVEMENT FUNCTIONS
  // ==========================================

  void avancer(float throttle)
  {
    manualPropulsionCmd(throttle, false,leftRearPwmCmd, rightRearPwmCmd);
    steeringAngle = 0;
  }
  
  void reculer(float throttle){
    manualPropulsionCmd(throttle, true,leftRearPwmCmd, rightRearPwmCmd);
    steeringAngle = 0;
  }
  
  void gauche(float throttle, bool reverse){
    manualPropulsionCmd(throttle, reverse,leftRearPwmCmd, rightRearPwmCmd);
    steeringAngle = -50;
  }

  void droite(float throttle, bool reverse){
    manualPropulsionCmd(throttle, reverse,leftRearPwmCmd, rightRearPwmCmd);
steeringAngle = 50;
  }

  void arreter()
  {
    // Neutral motor PWM = 50
    leftRearPwmCmd = 50;
    rightRearPwmCmd = 50;
    // Center steering
    steeringAngle = 0;
  }


  // ==========================================
  // PROPULSION HELPER
  // ==========================================

  void manualPropulsionCmd(float requestedThrottle,bool reverse,int8_t & leftPwm,int8_t & rightPwm){
    // Limit throttle to [0,1]
    requestedThrottle =
      std::clamp(requestedThrottle, 0.0f, 1.0f);

    if (reverse)
      {
	leftPwm = static_cast<int8_t>(
				      50 - 50 * requestedThrottle);
      }
    else
      {
	leftPwm = static_cast<int8_t>(
				      50 + 50 * requestedThrottle);
      }

    rightPwm = leftPwm;
  }


  // ==========================================
  // VARIABLES
  // ==========================================

  bool sequenceStarted = false;
  bool sequenceFinished = false;

  bool mustSafetyStop = true;
  double elapsed_before_stop = 0.0f;

  rclcpp::Time time_start;

  int8_t leftRearPwmCmd = 50;
  int8_t rightRearPwmCmd = 50;
  int8_t steeringAngle = 0;
  int8_t moveType = STOP;


  // ==========================================
  // ROS 2 COMMUNICATION
  // ==========================================

  rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr
  publisher_can_;

  rclcpp::Publisher<interfaces::msg::AutonomousCommand>::SharedPtr
  publisher_ack_; // Movement acknowledgement

  rclcpp::Subscription<interfaces::msg::AutonomousCommand>::SharedPtr
  subscription_start_;
    
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
  subscription_stop_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    
  //const YAML::Node config = YAML::LoadFile("../steps.yaml");
  //const YAML::Node sequence = config["sequence-2"];
    
  rclcpp::init(argc, argv);

  auto node = std::make_shared<autonomous_drive>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
