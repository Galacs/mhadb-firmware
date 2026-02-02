#include <Arduino.h>
#include <SimpleFOC.h>
#include "STM32_CAN.h"
#include <can_controller.h>
#include <can_messages.h>

#define INHA PA8
#define INHB PA9
#define INHC PA10
#define INLA PB13
#define INLB PB14
#define INLC PB15

#define SOA PA1
#define SOB PA2
#define SOC PA3

#define SCL PB6
#define SDA PB7

#define bldc_LEFT
// #define bldc_RIGHT

float motor_target = 0;

//HardwareSerial Serial3(PB11, PB10);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(INHA , INLA, INHB, INLB, INHC, INLC);
// BLDCDriver3PWM driver = BLDCDriver3PWM(INHA , INHB, INHC);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire Wire2(SDA, SCL);

// Real time FOC
HardwareTimer* timer = new HardwareTimer(TIM2);

void change_state(bldc_state_t new_state, bool broadcast = true);
bldc_state_t state = bldc_state_t::RESET;

class BldcCanHandler
{
  public:
  static MHADBCanController<BldcCanHandler>* controller; 
  
  static void setController(MHADBCanController<BldcCanHandler>* ctrl) {
    controller = ctrl;
  }

  static void handle_struct(t_bldc_set_speed data) {
    motor_id_t side;
    #ifdef bldc_RIGHT
    side = motor_id_t::RIGHT;
    #endif
    #ifdef bldc_LEFT
    side = motor_id_t::LEFT;
    #endif
    if (data.motor_id == side) {
      if (state == bldc_state_t::RUNNING) {
        motor_target = data.speed;
      }
    }
  }

  static void handle_struct(t_bldc_disable data) {
    timer->pause();
    change_state(bldc_state_t::OFF);
  }

  static void handle_struct(t_bldc_alignment_settings data) {
    motor_id_t side;
    #ifdef bldc_RIGHT
    side = motor_id_t::RIGHT;
    #endif
    #ifdef bldc_LEFT
    side = motor_id_t::LEFT;
    #endif
    if (data.motor_id == side && data.align_request == data.STORED && state != bldc_state_t::RUNNING) {
      // digitalWrite(PA15, HIGH);
      timer->pause();
      motor.zero_electric_angle  = data.zero_electric_angle;
      motor.sensor_direction = (Direction)data.sensor_direction;
      motor_target = 0;
      motor.initFOC();
      change_state(bldc_state_t::RUNNING);
      timer->resume();
    }
  }

  static void handle_struct(t_bldc_alignment_start data) {
    if (state == bldc_state_t::CALIBRATING)
      return;
    timer->pause();
    change_state(bldc_state_t::CALIBRATING);
    // Serial.println("aligning...");
    motor.zero_electric_angle  = NOT_SET;
    motor.sensor_direction = Direction::UNKNOWN; // CW or CCW
    motor_target = 0;
    motor.initFOC();
    change_state(bldc_state_t::RUNNING);

    t_bldc_alignment_settings settings_msg;
    settings_msg.align_request = settings_msg.CALIBRATED;
    motor_id_t side;
    #ifdef bldc_RIGHT
    side = motor_id_t::RIGHT;
    #endif
    #ifdef bldc_LEFT
    side = motor_id_t::LEFT;
    #endif
    settings_msg.motor_id = side;
    settings_msg.zero_electric_angle = motor.zero_electric_angle;
    settings_msg.sensor_direction = motor.sensor_direction;
    controller->send_struct(settings_msg);
    timer->resume();
  }

  static bool update_struct(t_bldc_current_pos* data) {
    data->shaft_angle = motor.shaft_angle;
    return true;
  }
  static bool update_struct(t_bldc_current_speed* data) {
    data->speed = motor.shaft_velocity;
    return true;
  }

  static bool update_struct(t_bldc_alignment_settings* data) {
    motor_id_t side;
    #ifdef bldc_RIGHT
    side = motor_id_t::RIGHT;
    #endif
    #ifdef bldc_LEFT
    side = motor_id_t::LEFT;
    #endif
    data->zero_electric_angle = motor.zero_electric_angle;
    data->sensor_direction = motor.sensor_direction;
    return true;
  }

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

MHADBCanController<BldcCanHandler>* BldcCanHandler::controller = nullptr;
MHADBCanController<BldcCanHandler> can = MHADBCanController<BldcCanHandler>();

void change_state(bldc_state_t new_state, bool broadcast) {
  state = new_state;
  t_bldc_state msg;
  #ifdef bldc_RIGHT
  msg.motor_id = motor_id_t::RIGHT;
  #endif
  #ifdef bldc_LEFT
  msg.motor_id = motor_id_t::LEFT;
  #endif
  can.send_struct(msg);
}

bool a = true;

struct __attribute__ ((packed)) t_motor_command   {
  uint8_t id;
  uint16_t value;
};

void setup() {
  can.init();
  
  change_state(bldc_state_t::RESET);

  sensor.init(&Wire2);

  driver.voltage_power_supply = 4.1*3;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  driver.pwm_frequency = 30000;

  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;

  motor.P_angle.P = 15;

  motor.LPF_velocity.Tf = 0.001;

  // initialize motor
  motor.init();
  motor.voltage_sensor_align = 10.0f;


  motor.target = 0; //initial target velocity 1 rad/s

  // Set timer frequency to 10kHz
  timer->setOverflow(1200, HERTZ_FORMAT); // MAX 1500
  // add the loopFOC and move to the timer
  timer->attachInterrupt([](){
    // call the loopFOC and move functions
    digitalWrite(PC13, a);
    a = !a;

    motor.loopFOC();
    motor.move(motor_target);
  });
  // start the timer
  // timer->resume();

  HardwareTimer* can_timer = new HardwareTimer(TIM3);
  can_timer->setOverflow(400, HERTZ_FORMAT);
  can_timer->attachInterrupt([](){
    can.handle_can();
  });
  // start the timer
  can_timer->resume();
  can.send_rtr(BLDC_ALIGNMENT_SETTINGS);
}

int last = 0;
void loop() {
  if (millis() > last + 1000 && state == bldc_state_t::RESET) {
    last = millis();
    can.send_rtr(BLDC_ALIGNMENT_SETTINGS);
  }
}