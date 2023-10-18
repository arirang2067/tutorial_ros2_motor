/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <tutorial_ros2_motor/motor_node.hpp>

void LoadParameters(void)
{
  std::ifstream inFile("/home/ubuntu/robot_ws/src/tutorial_ros2_motor/data/motor_input.txt");
  if (!inFile.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Unable to open the file");
    return;
  }

  int i = 0;
  std::size_t found;
  for (std::string line; std::getline(inFile, line);)
  {
    found = line.find("=");

    switch (i)
    {
    case 0:
      pwm_range = atof(line.substr(found + 2).c_str());
      break;
    case 1:
      pwm_frequency = atof(line.substr(found + 2).c_str());
      break;
    case 2:
      pwm_limit = atof(line.substr(found + 2).c_str());
      break;
    case 3:
      control_cycle = atof(line.substr(found + 2).c_str());
      break;
    case 4:
      acceleration_ratio = atof(line.substr(found + 2).c_str());
      break;
    case 5:
      wheel_radius = atof(line.substr(found + 2).c_str());
      break;
    case 6:
      robot_radius = atof(line.substr(found + 2).c_str());
      break;
    case 7:
      encoder_resolution = atof(line.substr(found + 2).c_str());
      break;
      // case :  = atof(line.substr(found+2).c_str()); break;
    }
    i += 1;
  }
  inFile.close();
}

int InitMotors(void)
{
  pinum = pigpio_start(NULL, NULL);

  if (pinum < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Setup failed");
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_dir, PI_OUTPUT);
  set_mode(pinum, motor2_dir, PI_OUTPUT);
  set_mode(pinum, motor1_pwm, PI_OUTPUT);
  set_mode(pinum, motor2_pwm, PI_OUTPUT);
  set_mode(pinum, motor1_encA, PI_INPUT);
  set_mode(pinum, motor1_encB, PI_INPUT);
  set_mode(pinum, motor2_encA, PI_INPUT);
  set_mode(pinum, motor2_encB, PI_INPUT);

  gpio_write(pinum, motor1_dir, PI_LOW);
  gpio_write(pinum, motor2_dir, PI_LOW);

  set_PWM_range(pinum, motor1_pwm, pwm_range);
  set_PWM_range(pinum, motor2_pwm, pwm_range);
  set_PWM_frequency(pinum, motor1_pwm, pwm_frequency);
  set_PWM_frequency(pinum, motor2_pwm, pwm_frequency);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);

  set_pull_up_down(pinum, motor1_encA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_encB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_encA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_encB, PI_PUD_DOWN);

  current_pwm1 = 0;
  current_pwm2 = 0;

  current_direction1 = true;
  current_direction2 = true;

  acceleration = pwm_limit / (acceleration_ratio);

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup Fin");
  return 0;
}

void SetInterrupts(void)
{
  callback(pinum, motor1_encA, EITHER_EDGE, Interrupt1A);
  callback(pinum, motor1_encB, EITHER_EDGE, Interrupt1B);
  callback(pinum, motor2_encA, EITHER_EDGE, Interrupt2A);
  callback(pinum, motor2_encB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1A--;
  else
    encoder_count_1A++;
  speed_count_1++;
}

void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1B--;
  else
    encoder_count_1B++;
  speed_count_1++;
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2A--;
  else
    encoder_count_2A++;
  speed_count2++;
}

void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2B--;
  else
    encoder_count_2B++;
  speed_count2++;
}

int SumMotor1Encoder()
{
  encoder_count_1 = encoder_count_1A + encoder_count_1B;
  return encoder_count_1;
}

int SumMotor2Encoder()
{
  encoder_count_2 = encoder_count_2A + encoder_count_2B;
  return encoder_count_2;
}

void InitEncoders(void)
{
  encoder_count_1 = 0;
  encoder_count_2 = 0;
  encoder_count_1A = 0;
  encoder_count_1B = 0;
  encoder_count_2A = 0;
  encoder_count_2B = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{
  LoadParameters();
  InitMotors();
  InitEncoders();
  SetInterrupts();

  wheel_round = 2 * PI * wheel_radius;
  robot_round = 2 * PI * robot_radius;

  switch_direction = true;
  theta_distance_flag = 0;

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", pwm_range);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", pwm_frequency);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", pwm_limit);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", control_cycle);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %d", acceleration_ratio);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");

  printf("\033[2J");
}

void MotorController(int motor_num, bool direction, int pwm)
{
  int local_pwm = LimitPwm(pwm);

  if (motor_num == 1)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor1_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor1_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = false;
    }
  }

  else if (motor_num == 2)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor2_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor2_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = false;
    }
  }
}
void AccelController(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_pwm;
  int local_current_pwm;

  if (motor_num == 1)
  {
    local_current_direction = current_direction1;
    local_current_pwm = current_pwm1;
  }
  else if (motor_num == 2)
  {
    local_current_direction = current_direction2;
    local_current_pwm = current_pwm2;
  }

  if (direction == local_current_direction)
  {
    if (desired_pwm > local_current_pwm)
    {
      local_pwm = local_current_pwm + acceleration;
      MotorController(motor_num, direction, local_pwm);
    }
    else if (desired_pwm < local_current_pwm)
    {
      local_pwm = local_current_pwm - acceleration;
      MotorController(motor_num, direction, local_pwm);
    }
    else
    {
      local_pwm = local_current_pwm;
      MotorController(motor_num, direction, local_pwm);
    }
  }
  else
  {
    if (desired_pwm >= 0)
    {
      local_pwm = local_current_pwm - acceleration;
      if (local_pwm <= 0)
      {
        local_pwm = 0;
        MotorController(motor_num, direction, local_pwm);
      }
      else
        MotorController(motor_num, local_current_direction, local_pwm);
    }
    else
    {
      local_pwm = local_current_pwm;
      MotorController(motor_num, direction, local_pwm);
    }
  }
}

void SwitchTurn(int pwm1, int pwm2)
{
  int local_pwm1 = LimitPwm(pwm1);
  int local_pwm2 = LimitPwm(pwm2);
  if (switch_direction == true)
  {
    MotorController(1, switch_direction, local_pwm1);
    MotorController(2, switch_direction, local_pwm2);
    switch_direction = false;
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "true");
  }
  else
  {
    MotorController(1, switch_direction, local_pwm1);
    MotorController(2, switch_direction, local_pwm2);
    switch_direction = true;
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "false");
  }
}

void ThetaTurn(double theta, int pwm)
{
  double local_encoder;
  int local_pwm = LimitPwm(pwm);
  if (theta_distance_flag == 1)
  {
    InitEncoders();
    theta_distance_flag = 2;
  }
  SumMotor1Encoder();
  SumMotor2Encoder();
  if (theta > 0)
  {
    local_encoder = (encoder_resolution * 4 / 360) * (robot_round / wheel_round) * theta;
    MotorController(1, true, local_pwm);
    MotorController(2, true, local_pwm);
    // AccelController(1, true, local_pwm);
    // AccelController(2, true, local_pwm);
  }
  else
  {
    local_encoder = -(encoder_resolution * 4 / 360) * (robot_round / wheel_round) * theta;
    MotorController(1, false, local_pwm);
    MotorController(2, false, local_pwm);
    // AccelController(1, false, local_pwm);
    // AccelController(2, false, local_pwm);
  }

  if (encoder_count_1 > local_encoder)
  {
    InitEncoders();
    MotorController(1, true, 0);
    MotorController(2, true, 0);
    theta_distance_flag = 3;
  }
}

void DistanceGo(double distance, int pwm)
{
  double local_encoder = (encoder_resolution * 4 * distance) / wheel_round;
  int local_pwm = LimitPwm(pwm);
  bool direction = true;
  if (distance < 0)
  {
    direction = false;
    local_encoder = -local_encoder;
  }
  if (theta_distance_flag == 3)
  {
    InitEncoders();
    theta_distance_flag = 4;
  }
  SumMotor1Encoder();
  SumMotor2Encoder();
  if (encoder_count_1 < local_encoder)
  {
    if (direction == true)
    {
      MotorController(1, true, local_pwm);
      MotorController(2, false, local_pwm);
      // AccelController(1, true, local_pwm);
      // AccelController(2, false, local_pwm);
    }
    else
    {
      MotorController(1, false, local_pwm);
      MotorController(2, true, local_pwm);
      // AccelController(1, false, local_pwm);
      // AccelController(2, true, local_pwm);
    }
  }
  else
  {
    InitEncoders();
    MotorController(1, true, 0);
    MotorController(2, true, 0);
    // AccelController(1, true, 0);
    // AccelController(2, true, 0);
    theta_distance_flag = 0;
  }
}

void ThetaTurnDistanceGo(double theta, int turn_pwm, double distance, int go_pwm)
{
  if (theta_distance_flag == 0)
  {
    theta_distance_flag = 1;
  }
  else if (theta_distance_flag == 1 || theta_distance_flag == 2)
  {
    ThetaTurn(theta, turn_pwm);
  }
  else if (theta_distance_flag == 3 || theta_distance_flag == 4)
  {
    DistanceGo(distance, go_pwm);
  }
}

int LimitPwm(int pwm)
{
  int output;
  if (pwm > pwm_limit * 2)
  {
    output = pwm_limit;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
  }
  else if (pwm > pwm_limit)
    output = pwm_limit;
  else if (pwm < 0)
  {
    output = 0;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
  }
  else
    output = pwm;
  return output;
}

void CalculateRpm()
{
  rpm_value1 = (speed_count_1 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count_1 = 0;
  rpm_value2 = (speed_count2 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count2 = 0;
}

void InfoMotors()
{
  CalculateRpm();
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
  printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
  printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);
  printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_pwm1, current_pwm2);
  printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
  printf("ACC  :%11.0d\n", acceleration);
  printf("\n");
}

RosCommunicator::RosCommunicator()
    : Node("tutorial_ros2_motor"), count_(0)
{
  timer_ = this->create_wall_timer(
      100ms, std::bind(&RosCommunicator::TimerCallback, this));
  subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/tutorial/teleop", 10, std::bind(&RosCommunicator::TeleopCallback, this, _1));
}

void RosCommunicator::TimerCallback()
{
  // MotorController(1, true, 100);
  // MotorController(2, true, 100);
  // AccelController(1, true, 100);
  // AccelController(2, true, 100);
  // SwitchTurn(100, 100);
  // ThetaTurnDistanceGo(180,100,30,110);
  InfoMotors();
}

void RosCommunicator::TeleopCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
  bool tmp_dir1, tmp_dir2;
  if (msg->data[0] == 0)
    tmp_dir1 = true;
  else
    tmp_dir1 = false;
  if (msg->data[1] == 0)
    tmp_dir2 = true;
  else
    tmp_dir2 = false;

  AccelController(1, tmp_dir1, msg->data[2]);
  AccelController(2, tmp_dir2, msg->data[3]);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Initialize();
  rclcpp::spin(std::make_shared<RosCommunicator>());

  rclcpp::shutdown();
  MotorController(1, true, 0);
  MotorController(2, true, 0);
  return 0;
}
