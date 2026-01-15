#include <Arduino.h>
#include <SimpleFOC.h>
#include "STM32_CAN.h"
#include <can_controller.h>

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

class BldcCanController: public CanController
{
public:
  void handle_struct(t_line_sensor_raw_data data) {
    Serial.println("sdfsdsdf received");
  }

};

BldcCanController can;

//HardwareSerial Serial3(PB11, PB10);

BLDCMotor motor = BLDCMotor(7);
//BLDCDriver6PWM driver = BLDCDriver6PWM(INHA, INLA, INHB, INLB, INHC, INLC);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

Commander command = Commander(Serial3);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

struct __attribute__ ((packed)) t_motor_command   {
  uint8_t id;
  uint16_t value;
};

void setup() {
  Serial3.begin(115200);
  Serial3.println("salut");
  Serial3.println("salut");
  Serial.println("et salut");
  
  can.init();
  //Wire.setClock(400000);

  // SimpleFOC setup
  //SimpleFOCDebug::enable(&Serial3);

  command.verbose = VerboseMode::machine_readable;

  /*sensor.init();


  driver.voltage_power_supply = 12;
  //driver.pwm_frequency = 45000;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity_openloop;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // default voltage_power_supply
  motor.voltage_limit = 2; // Volts

  // comment out if not needed
  motor.useMonitoring(Serial3);
  motor.monitor_downsample = 0; // disable monitor at first - optional

  // controller configuration based on the control type 

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;

  motor.LPF_velocity.Tf = 0.01;

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command M
  command.add('M', doMotor, "motor");

  Serial3.println(F("Motor ready."));
  Serial3.println(F("Set the target velocity using Serial3 terminal:"));
  
  motor.target = 1; //initial target velocity 1 rad/s
  Serial3.println("Target velocity: 1 rad/s");
  Serial3.println("Voltage limit 2V");
  _delay(1000);
*/
}

// void read_can() {
//   if (Can.read(CAN_RX_msg)) {
//     Serial3.print("received id: ");
//     Serial3.println(CAN_RX_msg.id);
//     t_motor_command msg_content;
//     memcpy(&msg_content, CAN_RX_msg.buf, sizeof(msg_content));
//     Serial3.printf("On motor: %d, the value: %d", msg_content.id, msg_content.value);
//   }
// }
int i = 0;
void loop() {

  t_line_sensor_raw_data a {.id=10, .value=126};

  can.send_struct(a);
 
  //can.handle_can();

  /*motor.loopFOC();

  motor.move();

  command.run();
  motor.monitor();*/

  // read_can();
  delay(200);
  Serial.printf("running..., %d\n", i);
  i++;

}

