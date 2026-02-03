#include <Arduino.h>
#include "can_messages.h"
#include <Preferences.h>
#include "Commander.h"
#include <QuickPID.h>

#define BUZZER_PIN 47
#define STARTER_PIN 33
#define ALERT_PIN 41
#define EMS_PIN 3
#define RGB1_PIN 10
#define RGB2_PIN 11
#define CAN_ST_PIN 21

float Setpoint, Input, Output;

float Kp = 15, Ki = 10, Kd = 0;

QuickPID myPID(&Input, &Output, &Setpoint);

uint16_t line_sensors_mapped[10] = {0};
uint16_t line_sensors_raw[10] = {0};

bool line_debug = false;
bool line_raw_debug = false;
bool line_pos_debug = false;
unsigned long last_debug_line_raw = 0;
unsigned long last_debug_line = 0;
unsigned long last_debug_line_pos = 0;

bool elapsed(unsigned long* last_run, unsigned long time) {
  if (millis() > *last_run + time) {
    *last_run = millis();
    return true;
  }
  return false;
}

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
          ledcWriteTone(0, music.notes[i].freq);
        }
        delay(music.notes[i].length / music.speed);
      }
      ledcWrite(0, 0);
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
    //if (line_state == line_pos_state_t::LOSTING) Serial.println("ligne perdue");
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

float speed = 0;
float direction = 0;

void commetuveux(float speed, float direction){
  float coef=5;

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
}

void doFollow(char *cmd) {
  if (state == FOLLOWING) {
    Serial.println("now stop....");
    commetuveux(0, 0);
    state = RESET;
  } else {
    Serial.println("now following....");
    doSendAlign(nullptr);
    state = FOLLOWING;
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
      speed = 0;
      direction = 0;
      commetuveux(speed, direction);
    }
  //commetuveux(speed, -Output);
  //if (line_state == line_pos_state_t::LOST) Serial.print("ligne perdu\n");
  //if (abs(line) > 1400 && abs(line) < 2400) commetuveux(speed/5, -Output);
  //else if (abs(line) >= 2400) commetuveux(speed/10, -Output);
  //else commetuveux(speed, -Output);
  if (line_state == line_pos_state_t::LOSTING) {
    commetuveux(speed/8, -Output);
    //Serial.print("ligne perdu\n");
    //Serial.printf("line: %f, sortie: %f\n", Input*20, Output);
  }
  else commetuveux(speed, -Output);
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
  music.speed = 170/120;
  music.notes_count = sizeof(hymn)/sizeof(music_score_t);
  xQueueSend(buzzer_queue, (void *)&music, 0);
}

void setup() {
  Serial.begin(115200);
  can.init((gpio_num_t)48, (gpio_num_t)34);
  // can.init((gpio_num_t)10, (gpio_num_t)9);

  // Buzzer
  ledcSetup(0, 4000, 13);
  ledcAttachPin(BUZZER_PIN, 0);
  xTaskCreate(taskBuzzer, "Buzzer", 2048, NULL, 2, NULL);
  buzzer_queue = xQueueCreate(1, sizeof(music_t));

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
  enum class Control : uint8_t {manual, automatic, timer, toggle};  // controller mode
  myPID.SetMode((uint8_t) Control::automatic);
  myPID.SetOutputLimits(-1, 1);
  myPID.SetSampleTimeUs(1000);

  // Play boot sound
  //doMusic(NULL);
}

bool done = false;

int last_run = 0;

void loop() {
  if (millis() > last_run + 50) {
    last_run = millis();
    following();
  }
  //Serial.println(state);
  Input = line;
  Input /= 100000;
  myPID.Compute();

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
}
