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

#define Serial3 Serial

float motor_target = 0;

//HardwareSerial Serial3(PB11, PB10);

BLDCMotor motor = BLDCMotor(7);
// BLDCDriver6PWM driver = BLDCDriver6PWM(INHA , INLA, INHB, INLB, INHC, INLC);
BLDCDriver3PWM driver = BLDCDriver3PWM(INHA , INHB, INHC);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire Wire2(PB11, PB10);

// Real time FOC
HardwareTimer* timer = new HardwareTimer(TIM2);

class BldcCanHandler
{
  public:
  static void handle_struct(t_line_sensor_data data) {
    Serial.printf("Line pos: %d\n", data.line_pos);
    // Serial.println(sizeof(t_bldc_current_pos));
    motor_target = map(data.line_pos, -4000, 4000, -50, 50);
  }

  static void handle_struct(t_bldc_alignment_start data) {
    timer->pause();
    Serial.println("aligning...");
    motor.zero_electric_angle  = NOT_SET;
    motor.sensor_direction = Direction::UNKNOWN; // CW or CCW
    motor.initFOC();
    Serial.println("aligned...");
    timer->resume();
  }

  static bool update_struct(t_bldc_current_pos* data) {
    data->motor_id = data->RIGHT;
    data->shaft_angle = motor.shaft_angle;
    // data->shaft_angle = 10.5;
    // Serial.printf("%f will be sent", data->shaft_angle);
    Serial.println(data->shaft_angle);
    Serial.println("received rtr");
    return true;
  }
  static bool update_struct(t_bldc_current_speed* data) {
    data->motor_id = data->RIGHT;
    data->speed = motor.shaft_velocity;
    return true;
  }

  static bool update_struct(t_bldc_alignment_results* data) {
    data->motor_id = data->RIGHT;
    data->zero_electric_angle = motor.zero_electric_angle;
    data->sensor_direction = motor.sensor_direction;
    return true;
  }

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

CanController<MHADBCanController<BldcCanHandler>> can;

// Commander command = Commander(Serial3);
// void doMotor(char* cmd) { command.motor(&motor, cmd); }

bool a = true;

struct __attribute__ ((packed)) t_motor_command   {
  uint8_t id;
  uint16_t value;
};

void setup() {
  pinMode(PC13, OUTPUT);

  pinMode(INLA, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INLC, OUTPUT);
  digitalWrite(INLA, HIGH);
  digitalWrite(INLB, HIGH);
  digitalWrite(INLC, HIGH);

  Serial3.setRx(PB7);
  Serial3.setTx(PB6);
  Serial3.begin(115200);

  can.init();

  // SimpleFOC setup
  // SimpleFOCDebug::enable(&Serial3);

  // command.verbose = VerboseMode::machine_readable;

  sensor.init(&Wire2);


  driver.voltage_power_supply = 3.6*3;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // default voltage_power_supply
  motor.voltage_limit = 3.6*3; // Volts

  driver.pwm_frequency = 30000;

  // comment out if not needed
  // motor.useMonitoring(Serial3);
  // motor.monitor_downsample = 0; // disable monitor at first - optional

  // controller configuration based on the control type 

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.0001;
  motor.PID_velocity.output_ramp = 1000;

  motor.P_angle.P = 15;

  motor.LPF_velocity.Tf = 0.01;

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.zero_electric_angle  = 4.36f; // rad
  motor.sensor_direction = Direction::CCW; // CW or CCW
  motor.voltage_sensor_align = 10.0f;
  motor.initFOC();

  // add target command M
  // command.add('M', doMotor, "motor");

  Serial3.println(F("Motor ready."));
  Serial3.println(F("Set the target velocity using Serial3 terminal:"));
  
  motor.target = 5; //initial target velocity 1 rad/s
  Serial3.println("Target velocity: 1 rad/s");
  Serial3.println("Voltage limit 2V");

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
  timer->resume();
}
int i = 0;
void loop() {
  t_line_sensor_raw_data a {.id=10, .value=(int)motor.shaft_velocity};

  // can.send_struct(a);
 
  can.handle_can();

  // command.run();
  // motor.monitor();
  t_can_frame frame;
  // if (can.receive_can(&frame)) {
  //   Serial.println("received");
  // }

  // read_can();
  //delay(200);
  // Serial.printf("running..., %d\n", i);
  i++;
  // delay(200);

}