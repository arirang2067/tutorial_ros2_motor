/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <tutorial_ros2_motor/motor_node.hpp>

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/robot_ws/src/tutorial_ros2_motor/data/motor_input.txt");
  for (std::string line; std::getline(inFile, line);)
  {
    found = line.find("=");

    switch (i)
    {
    case 0:
      PWM_range = atof(line.substr(found + 2).c_str());
      break;
    case 1:
      PWM_frequency = atof(line.substr(found + 2).c_str());
      break;
    case 2:
      PWM_limit = atof(line.substr(found + 2).c_str());
      break;
    case 3:
      Control_cycle = atof(line.substr(found + 2).c_str());
      break;
    case 4:
      Acceleration_ratio = atof(line.substr(found + 2).c_str());
      break;
    case 5:
      Wheel_radius = atof(line.substr(found + 2).c_str());
      break;
    case 6:
      Robot_radius = atof(line.substr(found + 2).c_str());
      break;
    case 7:
      Encoder_resolution = atof(line.substr(found + 2).c_str());
      break;
      // case :  = atof(line.substr(found+2).c_str()); break;
    }
    i += 1;
  }
  inFile.close();
}
int Motor_Setup(void)
{
  pinum = pigpio_start(NULL, NULL);

  if (pinum < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup failed");
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  acceleration = PWM_limit / (Acceleration_ratio);

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup Fin");
  return 0;
}
void Interrupt_Setting(void)
{
  callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
  callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
  callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
  callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_DIR) == true)
    EncoderCounter1A++;
  else
    EncoderCounter1A--;
  EncoderSpeedCounter1++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_DIR) == true)
    EncoderCounter1B++;
  else
    EncoderCounter1B--;
  EncoderSpeedCounter1++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_DIR) == true)
    EncoderCounter2A--;
  else
    EncoderCounter2A++;
  EncoderSpeedCounter2++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_DIR) == true)
    EncoderCounter2B--;
  else
    EncoderCounter2B++;
  EncoderSpeedCounter2++;
}
int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}
void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2 * PI * Wheel_radius;
  Robot_round = 2 * PI * Robot_radius;

  switch_direction = true;
  Theta_Distance_Flag = 0;

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "PWM_range %d", PWM_range);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "PWM_frequency %d", PWM_frequency);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "PWM_limit %d", PWM_limit);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Control_cycle %f", Control_cycle);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Acceleration_ratio %d", Acceleration_ratio);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");

  printf("\033[2J");
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if (motor_num == 1)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = false;
    }
  }

  else if (motor_num == 2)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor2_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
      current_PWM2 = local_PWM;
      current_Direction2 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor2_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
      current_PWM2 = local_PWM;
      current_Direction2 = false;
    }
  }
}
void Accel_Controller(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_PWM;
  int local_current_PWM;

  if (motor_num == 1)
  {
    local_current_direction = current_Direction1;
    local_current_PWM = current_PWM1;
  }
  else if (motor_num == 2)
  {
    local_current_direction = current_Direction2;
    local_current_PWM = current_PWM2;
  }

  if (direction == local_current_direction)
  {
    if (desired_pwm > local_current_PWM)
    {
      local_PWM = local_current_PWM + acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else if (desired_pwm < local_current_PWM)
    {
      local_PWM = local_current_PWM - acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
  else
  {
    if (desired_pwm >= 0)
    {
      local_PWM = local_current_PWM - acceleration;
      if (local_PWM <= 0)
      {
        local_PWM = 0;
        Motor_Controller(motor_num, direction, local_PWM);
      }
      else
        Motor_Controller(motor_num, local_current_direction, local_PWM);
    }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
}

void Switch_Turn_Example(int PWM1, int PWM2)
{
  int local_PWM1 = Limit_Function(PWM1);
  int local_PWM2 = Limit_Function(PWM2);
  if (switch_direction == true)
  {
    Motor_Controller(1, switch_direction, local_PWM1);
    Motor_Controller(2, switch_direction, local_PWM2);
    switch_direction = false;
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "true");
  }
  else
  {
    Motor_Controller(1, switch_direction, local_PWM1);
    Motor_Controller(2, switch_direction, local_PWM2);
    switch_direction = true;
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "false");
  }
}
void Theta_Turn(double Theta, int PWM)
{
  double local_encoder;
  int local_PWM = Limit_Function(PWM);
  if (Theta_Distance_Flag == 1)
  {
    Init_Encoder();
    Theta_Distance_Flag = 2;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if (Theta > 0)
  {
    local_encoder = (Encoder_resolution * 4 / 360) * (Robot_round / Wheel_round) * Theta;
    Motor_Controller(1, false, local_PWM);
    Motor_Controller(2, false, local_PWM);
    // Accel_Controller(1, false, local_PWM);
    // Accel_Controller(2, false, local_PWM);
  }
  else
  {
    local_encoder = -(Encoder_resolution * 4 / 360) * (Robot_round / Wheel_round) * Theta;
    Motor_Controller(1, true, local_PWM);
    Motor_Controller(2, true, local_PWM);
    // Accel_Controller(1, true, local_PWM);
    // Accel_Controller(2, true, local_PWM);
  }

  if (EncoderCounter1 > local_encoder)
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    Theta_Distance_Flag = 3;
  }
}
void Distance_Go(double Distance, int PWM)
{
  double local_encoder = (Encoder_resolution * 4 * Distance) / Wheel_round;
  int local_PWM = Limit_Function(PWM);
  bool Direction = true;
  if (Distance < 0)
  {
    Direction = false;
    local_encoder = -local_encoder;
  }
  if (Theta_Distance_Flag == 3)
  {
    Init_Encoder();
    Theta_Distance_Flag = 4;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if (EncoderCounter1 < local_encoder)
  {
    if (Direction == true)
    {
      Motor_Controller(1, false, local_PWM);
      Motor_Controller(2, true, local_PWM);
      // Accel_Controller(1, false, local_PWM);
      // Accel_Controller(2, true, local_PWM);
    }
    else
    {
      Motor_Controller(1, true, local_PWM);
      Motor_Controller(2, false, local_PWM);
      // Accel_Controller(1, true, local_PWM);
      // Accel_Controller(2, false, local_PWM);
    }
  }
  else
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    // Accel_Controller(1, true, 0);
    // Accel_Controller(2, true, 0);
    Theta_Distance_Flag = 0;
  }
}
void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM)
{
  if (Theta_Distance_Flag == 0)
  {
    Theta_Distance_Flag = 1;
  }
  else if (Theta_Distance_Flag == 1 || Theta_Distance_Flag == 2)
  {
    Theta_Turn(Theta, Turn_PWM);
  }
  else if (Theta_Distance_Flag == 3 || Theta_Distance_Flag == 4)
  {
    Distance_Go(Distance, Go_PWM);
  }
}

int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit * 2)
  {
    output = PWM_limit;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "PWM too fast!!!");
  }
  else if (pwm > PWM_limit)
    output = PWM_limit;
  else if (pwm < 0)
  {
    output = 0;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
  }
  else
    output = pwm;
  return output;
}
void RPM_Calculator()
{
  RPM_Value1 = (EncoderSpeedCounter1 * (60 * Control_cycle)) / (Encoder_resolution * 4);
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2 * (60 * Control_cycle)) / (Encoder_resolution * 4);
  EncoderSpeedCounter2 = 0;
}
void Motor_View()
{
  RPM_Calculator();
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
  printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
  printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
  printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
  printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
  printf("Acc  :%10.0d\n", acceleration);
  printf("\n");
}

class RosCommunicator : public rclcpp::Node
{
public:
  RosCommunicator()
      : Node("tutorial_ros2_motor"), count_(0)
  {
    // publisher_ = this->create_publisher<std_msgs::msg::String>("/tutorial/topic", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&RosCommunicator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Motor_Controller(1, true, 100);
    // Motor_Controller(2, true, 100);
    // Accel_Controller(1, true, 100);
    // Accel_Controller(2, true, 100);
    // Switch_Turn_Example(100, 100);
    // Theta_Distance(180,100,30,110);
    Motor_View();
  }
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Initialize();
  rclcpp::spin(std::make_shared<RosCommunicator>());

  rclcpp::shutdown();
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  return 0;
}
