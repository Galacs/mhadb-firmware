#include <Arduino.h>
#include "can_messages.h"
#include <Preferences.h>
#include "Commander.h"
#include <QuickPID.h>
#include <Button2.h>

#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"
#include "WiFi.h"
#include "esp_wifi.h"

#define BUZZER_PIN 47
#define STARTER_PIN 33
#define ALERT_PIN 41
#define EMS_PIN 3
#define RGB1_PIN 10
#define RGB2_PIN 11
#define CAN_ST_PIN 21

float Setpoint, Input, Output, speed = 0, speed_Output, speed_Input = 0;

float Kp = 15, Ki = 10, Kd = 0;

float current_speed = 0, speed_setpoint;

QuickPID myPID(&Input, &Output, &Setpoint);
QuickPID speedPID(&current_speed, &speed_Output, &speed_setpoint);

uint16_t line_sensors_mapped[10] = {0};
uint16_t line_sensors_raw[10] = {0};

bool line_debug = false;
bool line_raw_debug = false;
bool line_pos_debug = false;
unsigned long last_debug_line_raw = 0;
unsigned long last_debug_line = 0;
unsigned long last_debug_line_pos = 0;

Button2 ems_btn;
Button2 starter_btn;

// ESP NOW
// Channel to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 5

#define ESPNOW_WIFI_MODE WIFI_STA  // WiFi Mode
#define ESPNOW_WIFI_IF WIFI_IF_STA // WiFi Interface

const MacAddress peer_mac({0x50, 0x78, 0x7D, 0x18, 0x74, 0x44});
ESP_NOW_Serial_Class NowSerial(peer_mac, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);
#define Serial NowSerial

enum bldc_main_t: uint8_t {
  RESET,
  INITIALIZING,
  ARMED,
  FOLLOWING,
  EMG,
};

bldc_main_t state = RESET;

struct music_score_t {
  uint16_t freq;
  uint16_t length;
};
static music_score_t hymn[] = {
  { 587, 236 },
  { 0, 14 }, { 659, 236 }, { 0, 14 }, { 698, 474 },
  { 0, 26 }, { 698, 474 }, { 0, 26 }, { 659, 157 },
  { 0, 9 }, { 659, 157 }, { 0, 9 }, { 698, 157 },
  { 0, 9 }, { 587, 474 }, { 0, 26 }, { 523, 236 },
  { 0, 14 }, { 587, 236 }, { 0, 14 }, { 587, 236 },
  { 0, 14 }, { 659, 236 }, { 0, 14 }, { 523, 157 },
  { 0, 9 }, { 784, 157 }, { 0, 9 }, { 698, 157 },
  { 0, 9 }, { 587, 236 }, { 0, 14 }, { 659, 236 },
  { 0, 14 }, { 698, 474 }, { 0, 26 }, { 698, 474 },
  { 0, 26 }, { 659, 157 }, { 0, 9 }, { 659, 157 },
  { 0, 9 }, { 698, 157 }, { 0, 9 }, { 587, 474 },
  { 0, 26 }, { 523, 236 }, { 0, 14 }, { 587, 236 },
  { 0, 14 }, { 587, 236 }, { 0, 14 }, { 659, 236 },
  { 0, 14 }, { 523, 157 }, { 0, 9 }, { 784, 157 },
  { 0, 9 }, { 698, 157 }, { 0, 9 }, { 587, 236 },
  { 0, 14 }, { 523, 236 }, { 0, 14 }, { 587, 474 },
  { 0, 26 }, { 659, 474 }, { 0, 26 }, { 440, 236 },
  { 0, 14 }, { 440, 236 }, { 0, 14 }, { 659, 157 },
  { 0, 9 }, { 659, 157 }, { 0, 9 }, { 698, 157 },
  { 0, 9 }, { 659, 157 }, { 0, 9 }, { 587, 157 },
  { 0, 9 }, { 784, 157 }, { 0, 9 }, { 494, 157 },
  { 0, 9 }, { 587, 157 }, { 0, 9 }, { 523, 157 },
  { 0, 9 }, { 698, 474 }, { 0, 26 }, { 587, 236 },
  { 0, 14 }, { 523, 236 }, { 0, 14 }, { 587, 474 },
  { 0, 26 }, { 659, 474 }, { 0, 26 }, { 440, 236 },
  { 0, 14 }, { 440, 236 }, { 0, 14 }, { 659, 157 },
  { 0, 9 }, { 659, 157 }, { 0, 9 }, { 698, 157 },
  { 0, 9 }, { 659, 157 }, { 0, 9 }, { 587, 157 },
  { 0, 9 }, { 784, 157 }, { 0, 9 }, { 494, 157 },
  { 0, 9 }, { 587, 157 }, { 0, 9 }, { 523, 157 },
  { 0, 9 }, { 698, 474 }
};

static music_score_t armed_waiting[] = {
  { 1500, 100 }, { 1000, 100 }
};

static music_score_t armed_error[] = {
  { 300, 100 }, { 200, 100 }
};

static music_score_t ems_triggered[] = {
  { 200, 700 }
};

static music_score_t follow_start[] = {
  { 900, 100 }, { 1100, 100 }, { 1300, 100 }
};

static music_score_t follow_end[] = {
  { 1300, 100 }, { 1100, 100 }, { 900, 100 }
};

struct music_t {
  music_score_t* notes;
  size_t notes_count;
  float speed;
};

enum buzzer_state_t {
  IDLE,
  PLAYING,
};

buzzer_state_t buzzer_state = buzzer_state_t::IDLE;

QueueHandle_t buzzer_queue;
void taskBuzzer(void *pvParams) {
  music_t music;
  for (;;) {
    int ret = xQueueReceive(buzzer_queue, &music, portMAX_DELAY);
    if (ret == pdPASS) {
      buzzer_state = buzzer_state_t::PLAYING;
      for (size_t i = 0; i < music.notes_count; i++) {
        if (music.notes[i].freq) {
          ledcWriteTone(BUZZER_PIN, music.notes[i].freq);
        }
        delay(music.notes[i].length / music.speed);
      }
      ledcWrite(BUZZER_PIN, 0);
      buzzer_state = buzzer_state_t::IDLE;
    } else if (ret == pdFALSE) {
      Serial.println("The `Buzzer task was unable to receive data from the Queue");
    }
  }
}


void print_line_values(uint16_t* values) {
     for (size_t i = 0; i < 10; i++) {
        Serial.print(values[i]);
        Serial.print('\t');
     }
     Serial.println("");
}

unsigned long last_armed_beep = 0;
Commander command = Commander(Serial);

// Temp settings
Preferences prefs;
float p_motor_left_zero;
int8_t p_motor_left_direction;
float p_motor_right_zero;
int8_t p_motor_right_direction;

int16_t line = 0;
line_pos_state_t line_state = line_pos_state_t::LOST;

void init_settings() {
  prefs.begin("mainPrefs");
  bool tpInit = prefs.isKey("nvsInit");
  if (tpInit == false) {
    prefs.putFloat("motorLZero", 0);
    prefs.putFloat("motorRZero", 0);

    prefs.putChar("motorLDir", 0);
    prefs.putChar("motorRDir", 0);

    prefs.putBool("nvsInit", true);
  }
}

void load_settings() {
  p_motor_left_zero = prefs.getFloat("motorLZero");
  p_motor_right_zero = prefs.getFloat("motorRZero");

  p_motor_left_direction = prefs.getChar("motorLDir");
  p_motor_right_direction = prefs.getChar("motorRDir");
}

class MainCanHandler
{
  public:
  static MHADBCanController<MainCanHandler>* controller; 
  
  static void setController(MHADBCanController<MainCanHandler>* ctrl) {
    controller = ctrl;
  }
  
  static void handle_struct(t_bldc_state data) {
    const static char motors[] = {'R', 'L'};
    const static String states[] = {"Reset", "Calibrating", "Running", "Off", "EMG"};
    Serial.printf("%c: %s\n", motors[data.motor_id], states[(char)data.sate]);
  }

  static void handle_struct(t_bldc_current_pos data) {
    // Serial.printf("Line pos: %d\n", data.shaft_angle);
    // Serial.printf("yay: %f", data.shaft_angle);
    Serial.print("oof: ");
    Serial.println(data.shaft_angle);
    // Serial.println(sizeof(t_bldc_current_pos));
  }
  static void handle_struct(t_bldc_current_speed data) {
    Serial.print("speed: ");
    Serial.println(data.speed);
  }

  static void handle_struct(t_bldc_alignment_settings data) {
    Serial.print("zero angle: ");
    Serial.println(data.zero_electric_angle);
    Serial.printf("direction: %d\n", data.sensor_direction);

    if (data.align_request != data.CALIBRATED)
      return;
    if (data.motor_id == motor_id_t::LEFT) {
      prefs.putFloat("motorLZero", data.zero_electric_angle);
      prefs.putChar("motorLDir", data.sensor_direction);
    } else {
      prefs.putFloat("motorRZero", data.zero_electric_angle);
      prefs.putChar("motorRDir", data.sensor_direction);
    }
    load_settings();
    Serial.printf("Got: %f, %d", data.zero_electric_angle, data.sensor_direction);
  }

  static bool update_struct(t_bldc_alignment_settings *data) {
    t_bldc_alignment_settings right;
    right.align_request = right.STORED;
    right.motor_id = motor_id_t::RIGHT;
    right.zero_electric_angle = p_motor_right_zero;
    right.sensor_direction = p_motor_right_direction;
    controller->send_struct(right);

    data->align_request = data->STORED;
    data->motor_id = motor_id_t::LEFT;
    data->zero_electric_angle = p_motor_left_zero;
    data->sensor_direction = p_motor_left_direction;
    Serial.println("sent saved settings");
    return true;
  }

  static void handle_struct(t_line_sensor_raw_data data) {
    if (data.sensor_id == 9 && line_debug && elapsed(&last_debug_line, 100)) {
      print_line_values(line_sensors_mapped);
    }
    if (data.sensor_id == 9 && line_raw_debug && elapsed(&last_debug_line_raw, 100)) {
      print_line_values(line_sensors_raw);
    }
    line_sensors_mapped[data.sensor_id] = data.mapped_value;
    line_sensors_raw[data.sensor_id] = data.raw_value;
  }
  static void handle_struct(t_line_sensor_data data) {
    if (line_pos_debug && elapsed(&last_debug_line_pos, 100)) {
      Serial.printf("Line pos: %d\n", data.line_pos);
    }
    line = data.line_pos;
    line_state = data.state;
    if (line_state == line_pos_state_t::T) Serial.println("T junction");
  }
  

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

MHADBCanController<MainCanHandler>* MainCanHandler::controller = nullptr;
CanController<MHADBCanController<MainCanHandler>> can;

void doStartAlign(char *cmd) {
  // command.scalar(&debugValue, cmd);
  // Serial.println(debugValue);
  t_bldc_alignment_start msg;
  msg.motor_id = motor_id_t::LEFT; // not used by bldc 
  can.send_struct(msg);
  Serial.println("Sent Start align");
  // can.send_rtr(BLDC_ALIGNMENT_SETTINGS);
}

float direction = 0;

void commetuveux(float speed, float direction){
  float coef=5;
  // Serial.printf("Speed: %f\n" ,speed);

  float puissance_rotation = direction * coef;
  t_bldc_set_speed data;
  data.motor_id = motor_id_t::LEFT;
  data.speed = speed + puissance_rotation;
  can.send_struct(data);
  data.motor_id = motor_id_t::RIGHT;
  data.speed = speed - puissance_rotation;
  can.send_struct(data);
}

void doSendAlign(char *cmd) {
  // command.scalar(&debugValue, cmd);
  // Serial.println(debugValue);
  t_bldc_alignment_settings right;
  right.align_request = right.STORED;
  right.motor_id = motor_id_t::RIGHT;
  right.zero_electric_angle = p_motor_right_zero;
  right.sensor_direction = p_motor_right_direction;
  can.send_struct(right);

  t_bldc_alignment_settings left;
  left.align_request = left.STORED;
  left.motor_id = motor_id_t::LEFT;
  left.zero_electric_angle = p_motor_left_zero;
  left.sensor_direction = p_motor_left_direction;
  can.send_struct(left);
  Serial.println("sent saved align settings");
  state = bldc_main_t::RESET;
}

void doFollow(char *cmd) {
  if (state == FOLLOWING) {
    Serial.println("now stop....");
    commetuveux(0, 0);
    music_t music;
    music.notes = follow_end;
    music.speed = 1;
    music.notes_count = sizeof(follow_end)/sizeof(music_score_t);
    xQueueSend(buzzer_queue, (void *)&music, 0);
    state = RESET;
  } else {
    Serial.println("now following....");
    doSendAlign(nullptr);
    myPID.Reset();
    speedPID.Reset();
    state = FOLLOWING;

    music_t music;
    music.notes = follow_start;
    music.speed = 1;
    music.notes_count = sizeof(follow_start)/sizeof(music_score_t);
    xQueueSend(buzzer_queue, (void *)&music, 0);
  }
}

void doDisableBLDC(char *cmd) {
  t_bldc_disable data;
  can.send_struct(data);
  Serial.println("disabled motors");
}

void doDebug(char *cmd) {
  switch (cmd[0]) {
  case 'M':
    line_debug = !line_debug;
    break;
  case 'R':
    line_raw_debug = !line_raw_debug;
  break;
  case 'L':
    line_pos_debug = !line_pos_debug;
    break;
  };
}

unsigned long ems_time = 0;
void handleEMSTap(Button2& b) {
  if (state != bldc_main_t::EMG) {
    ems_time = millis();
    state = bldc_main_t::EMG;
    Serial.println("going into emergency");
    music_t music;
    music.notes = ems_triggered;
    music.speed = 1;
    music.notes_count = sizeof(ems_triggered)/sizeof(music_score_t);
    xQueueSend(buzzer_queue, (void *)&music, 0);
    commetuveux(0, 0);
    for (size_t i = 0; i < 5; i++) {
      doDisableBLDC(NULL);
    }
  }
}

void handleEMSLongTap(Button2& b) {
  Serial.println("Long");
  if (elapsed(&ems_time, 1000)) {
    doSendAlign(NULL);
    if (!starter_btn.isPressed()) {
      music_t music;
      music.notes = armed_error;
      music.speed = 1;
      music.notes_count = sizeof(armed_error)/sizeof(music_score_t);
      xQueueSend(buzzer_queue, (void *)&music, 0);
      Serial.println("Armed");
      return;
    }
    state = bldc_main_t::ARMED;
  }
}

void handleStarter(Button2& b) {
  if (state == bldc_main_t::ARMED) {
    speed = 5;
    doFollow(NULL);
  }
}

// Temp test function
void forward(float speed) {
  t_bldc_set_speed data;
  data.motor_id = motor_id_t::LEFT;
  data.speed = speed;
  can.send_struct(data);
  data.motor_id = motor_id_t::RIGHT;
  can.send_struct(data);
}

void doSendForward(char *cmd) {
  float target = 0.0;
  command.scalar(&target, cmd);
  // Serial.printf("forward: %f\n", target);
  // delay(10);
  forward(target);
}


unsigned long last_pid_print = 0;
void following() {
  // if (state == FOLLOWING) {
  //   Serial.println(line);
  //   if (line > 50) {
  //     direction = 0.4;
  //   } else if (line < -50) {
  //     direction = -0.4;
  //   } else {
  //     direction = 0;
  //   }
  //   commetuveux(speed, direction);
  // }
 if (state == FOLLOWING) {
    if (elapsed(&last_pid_print, 200)) {
      //Serial.printf("line: %f, sortie: %f\n", Input*20, Output);
    }
    if (line_state == line_pos_state_t::NO_LINE) {
      myPID.Reset();
      speedPID.Reset();
      speed = 0;
      direction = 0;
      commetuveux(speed, direction);
    }
  //commetuveux(speed, -Output);
  //if (line_state == line_pos_state_t::LOST) Serial.print("ligne perdu\n");
  //if (abs(line) > 1400 && abs(line) < 2400) commetuveux(speed/5, -Output);
  //else if (abs(line) >= 2400) commetuveux(speed/10, -Output);
  //else commetuveux(speed, -Output);
  current_speed += speed_Output;
  if (line_state == line_pos_state_t::LOSTING) {
    // commetuveux(speed_Output/2, -Output);
    speed_setpoint = speed/2;
    // Serial.print("PID Speed slwoowow: ");
    // Serial.print(current_speed);
    // Serial.print("    ");
    // Serial.print(speed_setpoint);
    // Serial.print("    ");
    // Serial.println(Output);
    //Serial.print("ligne perdu\n");
    //Serial.printf("line: %f, sortie: %f\n", Input*20, Output);
    commetuveux(current_speed, -Output);
  } else {
    speed_setpoint = speed*(1-abs(line)/4200);
    commetuveux(current_speed, -Output);
    // Serial.print("PID Speed: ");
    // Serial.print(current_speed);
    // Serial.print("    ");
    // Serial.print(speed_setpoint);
    // Serial.print("    ");
    // Serial.println(Output);
  };
  // } else commetuveux(0, -Output);
  
  // commetuveux(current_speed, -Output);
  // else commetuveux(speed, -Output);
  //Serial.print("Speed ");
  //Serial.println(speed);
 }
}

void doSpeed(char *cmd) {
  command.scalar(&speed, cmd);
  commetuveux(speed, direction);
}

void doDirection(char *cmd) {
  command.scalar(&direction, cmd);
  commetuveux(speed, direction);
}

void doP(char *cmd) { command.scalar(&Kp, cmd); myPID.SetTunings(Kp, Ki, Kd); myPID.Reset(); }
void doI(char *cmd) { command.scalar(&Ki, cmd); myPID.SetTunings(Kp, Ki, Kd); myPID.Reset(); }
void doD(char *cmd) { command.scalar(&Kd, cmd); myPID.SetTunings(Kp, Ki, Kd); myPID.Reset(); }
void doR(char *cmd) { myPID.Reset(); }

void doMusic(char *cmd) {
  music_t music;
  music.notes = hymn;
  music.speed = 1.416666;
  music.notes_count = sizeof(hymn)/sizeof(music_score_t);
  xQueueSend(buzzer_queue, (void *)&music, 0);
}

void setup() {
  Serial.begin(115200);
  can.init((gpio_num_t)48, (gpio_num_t)34);
  // can.init((gpio_num_t)10, (gpio_num_t)9);
  

  WiFi.mode(ESPNOW_WIFI_MODE);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  while (!WiFi.STA.started()) {
    delay(100);
  }
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("ESP-NOW communication starting...");
  NowSerial.begin(115200);
  Serial.printf("ESP-NOW version: %d, max data length: %d\n",
                ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
  Serial.println(
      "You can now send data to the peer device using the Serial Monitor.\n");

  // Buttons config
  ems_btn.begin(EMS_PIN);
  starter_btn.begin(STARTER_PIN);

  ems_btn.setTapHandler(handleEMSTap);
  ems_btn.setLongClickHandler(handleEMSLongTap);
  ems_btn.setLongClickTime(1500);

  starter_btn.setReleasedHandler(handleStarter);

  // Buzzer
  ledcAttach(BUZZER_PIN, 4000, 13);
  buzzer_queue = xQueueCreate(4, sizeof(music_t));
  xTaskCreate(taskBuzzer, "Buzzer", 2048, NULL, 2, NULL);

  Setpoint = 0;
  
  // Commander
  command.verbose = VerboseMode::user_friendly;
  command.decimal_places = 5;
  command.add('A', doStartAlign);
  command.add('B', doSendAlign);
  command.add('F', doSendForward);
  command.add('S', doSpeed);
  command.add('O', doDirection);
  command.add('L', doFollow);
  command.add('X', doDisableBLDC);
  command.add('H', doDebug);

  command.add('P', doP);
  command.add('I', doI);
  command.add('D', doD);
  command.add('R', doR);

  command.add('M', doMusic);


  init_settings();
  load_settings();

  myPID.SetTunings(Kp, Ki, Kd);
  // speedPID.SetTunings(1.2, 1.2, 0.08);
  speedPID.SetTunings(0.1, 0, 0);
  enum class Control : uint8_t {manual, automatic, timer, toggle};  // controller mode
  myPID.SetMode((uint8_t) Control::automatic);
  speedPID.SetMode((uint8_t) Control::automatic);
  myPID.SetOutputLimits(-1, 1);
  speedPID.SetOutputLimits(-1, 1);

  myPID.SetSampleTimeUs(1000);
  speedPID.SetSampleTimeUs(1000);

  // Play boot sound
  //doMusic(NULL);
}

bool done = false;

int last_run = 0;

unsigned long speed_start = 0;
unsigned long speed_start_two = 0;

float speed_temp;

void loop() {
  if (state == bldc_main_t::ARMED && elapsed(&last_armed_beep, 1500)) {
    music_t music;
    music.notes = armed_waiting;
    music.speed = 1;
    music.notes_count = sizeof(armed_waiting)/sizeof(music_score_t);
    xQueueSend(buzzer_queue, (void *)&music, 0);
  }

  if (millis() > last_run + 50) {
    last_run = millis();
    following();
  }
  //Serial.println(state);
  Input = line;
  Input /= 100000;
  myPID.Compute();
  speedPID.Compute();
  
  // Retardage sortie PID speed
  if (millis() > speed_start + 10) {
    speed_temp = speed_Output;
  }
  if (millis() > speed_start + 20) {
    speed_start = millis();
    // if (speed_Input)
      // speed_Input = speed_temp;
    // Serial.print("input: ");
    // Serial.println(speed_temp);
  }

  // t_line_sensor_raw_data a {.id=10, .value=126};
  // can.send_struct(a);
  // can.send_rtr(CAN_ID::LINE_RAW_SENSOR_DATA);
  can.handle_can();
  // if (millis() > 10000 && !done) {
  //   done = true;
  //   forward(5.0);
  //   Serial.println("gooo");
  //   delay(5000);
  //   forward(0.0);
  // }
  command.run();
  // delay(20);
  ems_btn.loop();
  starter_btn.loop();
}
