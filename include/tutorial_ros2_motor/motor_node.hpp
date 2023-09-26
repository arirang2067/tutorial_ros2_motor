#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>
#include <fstream>

#define motor1_dir 19
#define motor1_pwm 26
#define motor1_encA 23
#define motor1_encB 24

#define motor2_dir 6
#define motor2_pwm 13
#define motor2_encA 27
#define motor2_encB 17

#define PI 3.141592

using namespace std::chrono_literals;

// LoadParameters
void LoadParameters(void);
int pwm_range;
int pwm_frequency;
int pwm_limit;
double control_cycle;
int acceleration_ratio;
double wheel_radius;
double robot_radius;
int encoder_resolution;
double wheel_round;
double robot_round;

// InitMotors
int InitMotors(void);
int pinum;
int current_pwm1;
int current_pwm2;
bool current_direction1;
bool current_direction2;
int acceleration;

// SetInterrupts
void Interrupt_Setiing(void);
volatile int encoder_count_1;
volatile int encoder_count_2;
volatile int encoder_count_1A;
volatile int encoder_count_1B;
volatile int encoder_count_2A;
volatile int encoder_count_2B;
volatile int speed_count_1;
volatile int speed_count2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int SumMotor1Encoder();
int SumMotor2Encoder();
void InitEncoders(void);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void);

// Controller
void MotorController(int motor_num, bool direction, int pwm);
void AccelController(int motor_num, bool direction, int desired_pwm);

// Example
bool switch_direction;
int theta_distance_flag;
void SwitchTurn(int pwm1, int pwm2);
void ThetaTurn(double theta, int pwm);
void DistanceGo(double distance, int pwm);
void ThetaTurnDistanceGo(double theta, int turn_pwm, double distance, int go_pwm);

// Utility
int LimitPwm(int pwm);
double rpm_value1;
double rpm_value2;
void CalculateRpm();
void InfoMotors();
class RosCommunicator : public rclcpp::Node
{
public:
  RosCommunicator();

private:
  void TimerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

#endif // MOTOR_NODE_H
