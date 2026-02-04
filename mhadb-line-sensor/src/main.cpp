#include <Arduino.h>
#include <FastLED.h>

#include "STM32_CAN.h"

#include <can_messages.h>


#define RGB1 PB12
#define RGB2 PB13

#define BTN1 PB7
#define BTN2 PB6

#define LS0 PA0
#define LS1 PA1
#define LS2 PA2
#define LS3 PA3
#define LS4 PA4
#define LS5 PA5
#define LS6 PA6
#define LS7 PA7
#define LS8 PB0
#define LS9 PB1


#define LINE_TRESH 800
#define LINE_MAX 950

line_led_state_t led_state = line_led_state_t::FOLLOWING;

class LineCanHandler {
public:
  static void handle_struct(t_line_led_state data) {
    led_state = data.state;
  }
  
  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

CanController<MHADBCanController<LineCanHandler>> can;

int sensors[] = {LS0, LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9 };

uint16_t raw_values[10] = {0};
uint16_t mapped_values[10] = {0};

line_pos_state_t line_state = line_pos_state_t::LOST;

CRGB leds_A[5];
CRGB leds_B[5];

uint16_t line_sensors_mapped[10];
uint16_t line_sensors_raw[10];

bool elapsed(unsigned long* last_run, unsigned long time) {
  if (millis() > *last_run + time) {
    *last_run = millis();
    return true;
  }
  return false;
}


void init_leds() {
  FastLED.addLeds<WS2812B, PB12, GRB>(leds_A, 5);
  FastLED.addLeds<WS2812B, PB13, GRB>(leds_B, 5);
}

DEFINE_GRADIENT_PALETTE( heatmap_gp ) {
  0,     0,   0,  0,   //black
128,   250,  15,  255,   //red
255,   15,  250,  255 };
CRGBPalette16 myPal = heatmap_gp;

bool ems_blink = 0;
unsigned long ems_time = 0;

void set_led_line() {
  for (size_t i = 0; i < 10; i++) {
    uint8_t brightness = map(mapped_values[i], 0, 100, 0, 250);
    switch (i)
    {
    case 0:
      leds_A[4] = ColorFromPalette(myPal, brightness);
      break;
    case 1:
      leds_A[3] = ColorFromPalette(myPal, brightness);
      break;
    case 2:
      leds_A[2] = ColorFromPalette(myPal, brightness);
      break;
    case 3:
      leds_A[1] = ColorFromPalette(myPal, brightness);
      break;
    case 4:
      leds_A[0] = ColorFromPalette(myPal, brightness);
      break;

    case 5:
      leds_B[0] = ColorFromPalette(myPal, brightness);
      break;
    case 6:
      leds_B[1] = ColorFromPalette(myPal, brightness);
      break;
    case 7:
      leds_B[2] = ColorFromPalette(myPal, brightness);
      break;
    case 8:
      leds_B[3] = ColorFromPalette(myPal, brightness);
      break;
    case 9:
      leds_B[4] = ColorFromPalette(myPal, brightness);
      break;
    }
  }
}
uint8_t gHue = 0;
unsigned long armed_time = 0;
void update_leds() {
  switch (led_state) {
  case line_led_state_t::FOLLOWING:
    set_led_line();
    break;
    case line_led_state_t::EMS:
    if (elapsed(&ems_time, 250))
      ems_blink = !ems_blink;
      for (size_t i = 0; i < 5; i++) {
        if (ems_blink) {
          leds_A[i] = CRGB::Red2;
          leds_B[i] = CRGB::Red2;
        } else {
          set_led_line();
        }
      }
    break;
    case line_led_state_t::ARMED:
    if (elapsed(&armed_time, 30)){
      fill_rainbow(leds_A, 5, gHue, 30);
      fill_rainbow(leds_B, 5, gHue, 30);
      gHue += 10;
    }
    break;
  }

  FastLED.show();
}

void update_line_sensors(int* sensors) {
  for (size_t i = 0; i < 10; i++) {
    // Devboard sensors
    //#if 0
    // int val = analogRead(sensors[i]);
    // if (val<500) val=500;
    // values[i] = map(val, 500, 1024, 0, 100);
    //#endif
    uint16_t a = analogRead(sensors[i]);
    raw_values[i] = raw_values[i] + a;
    a = constrain(a, LINE_TRESH, LINE_MAX);
    mapped_values[i] = mapped_values[i] + map(a, LINE_TRESH, LINE_MAX, 0, 100);
    // if (i == 3 /*|| i == 4*/ || i == 5) {
    //   values[i] = analogRead(sensors[i]);
    // } else {
    //   values[i] = 0;
    // }
  }
}

void print_line_values(uint16_t* values) {
     for (size_t i = 0; i < 10; i++) {
        Serial.print(values[i]);
        Serial.print('\t');
     }
     Serial.println("");
}

// int16_t pos_before_lost = 0;
// unsigned long lost_time = 0;
// unsigned long no_line_time = 0;
// int last_side = 1;
// bool is_full = false;
// unsigned long last_full = 0;
// unsigned long t_start = 0;
// unsigned long nine_start = 0;
// unsigned long last_detected = 0;
int last_side = 0;
unsigned long losing_start = 0;
unsigned long last_full = 0;
unsigned long t_start = 0;
unsigned long last_right = 0;
unsigned long last_left = 0;
unsigned long right_start = 0;
unsigned long last_right_turn = 0;

int16_t get_line_position(uint16_t* values) {
  const int8_t weights[] = {-44, -34, -24, -14, -4, 4, 14, 24, 34, 44};
  int total = 0, moy_pon = 0, sum = 0;

  // Calculate weighted average
  for (int i = 0; i < 10; i++) {
    total += weights[i] * (mapped_values[i] / 10);
    sum += (mapped_values[i] / 10);
  }

  // Avoid division by zero
  if (sum > 0) {
    moy_pon = (total * 100) / sum;
  } else {
    moy_pon = 0;
  }

  // Set last side based on edge sensors
  if (mapped_values[0] > 30) {
    last_side = -1;
    last_right = millis();
  } else if (mapped_values[9] > 30) {
    last_side = 1;
    last_left = millis();
  }

  // Stay in T for some time
  if (line_state == line_pos_state_t::T && millis() < t_start + 1000) {
    return 5000;
  } else if (line_state == line_pos_state_t::T) {
    t_start = 0;
    last_full = 0;
  }
  if (line_state == line_pos_state_t::RIGHT && millis() < right_start + 1000) {
    last_right_turn = millis();
    return -5000;
  } else if (line_state == line_pos_state_t::RIGHT) {
    right_start = 0;
  }

  // Stop losting if line
  if (sum > 20 && line_state == line_pos_state_t::LOSTING) {
    line_state = line_pos_state_t::DETECTED;
  }

  // No line
  if (sum < 10 && line_state != line_pos_state_t::LOST) {
    if (line_state != line_pos_state_t::LOSTING) {
      line_state = line_pos_state_t::LOSTING;
      losing_start = millis();
    }
  } else if (sum < 10 && line_state == line_pos_state_t::LOST) {
    return 0;
  } else if (sum > 80*9) {
    line_state = line_pos_state_t::FULL;
    last_full = millis();
    return moy_pon;
  }

  // Keep last edge value while line is losting
  if (line_state == line_pos_state_t::LOSTING && (millis() < losing_start + 1000) && millis() > last_full + 3000) {
    return 5000 * last_side;
  } else if (line_state == line_pos_state_t::LOSTING) {
    if (millis() < last_full + 3000 && millis() > losing_start + 1000) {
      line_state = line_pos_state_t::T;
      t_start = millis();
      return 5000;
    } else {
      // line_state = line_pos_state_t::LOST;
      return 0;
    }
  }

  // Y
  if ((mapped_values[0]/10 + mapped_values[1]/10 + mapped_values[2]/10 + mapped_values[3]/10 + mapped_values[4]/10 > 80 &&
      mapped_values[5]/10 + mapped_values[6]/10 + mapped_values[7]/10 + mapped_values[8]/10 + mapped_values[9]/10 > 80) &&
      mapped_values[4]/10 + mapped_values[5]/10 < 10) {
    line_state = line_pos_state_t::T;
    t_start = millis();
    if (millis() < last_right + 500) {
      if (millis() > last_right_turn + 5000) {
        right_start = millis();
        line_state = line_pos_state_t::RIGHT;
        return -5000;
      } else {
        line_state = line_pos_state_t::DETECTED;
        last_right_turn = millis();
        return 0;
      }
    }
    return 5000;
  }

  // 90s
  if (mapped_values[0]/10 + mapped_values[1]/10 + mapped_values[2]/10 + mapped_values[3]/10 + mapped_values[4]/10 > 4 * 80 ||
      mapped_values[5]/10 + mapped_values[6]/10 + mapped_values[7]/10 + mapped_values[8]/10 + mapped_values[9]/10 > 4 * 80) {
    if (moy_pon > 0) {
      moy_pon += 2000;
    } else {
      moy_pon -= 2000;
    }
  }

  line_state = line_pos_state_t::DETECTED;
  return moy_pon;
  
  // Line lost, keep last edge value
  

  // const int8_t weights[] = {-44, -34, -24, -14, -4, 4, 14, 24, 34, 44};
  // int total = 0, moy_pon = 0, sum = 0;

  // // Calculate weighted average
  // for (int i = 0; i < 10; i++) {
  //   total += weights[i] * (mapped_values[i] / 10);
  //   sum += (mapped_values[i] / 10);
  // }

  // // Avoid division by zero
  // if (sum > 0) {
  //   moy_pon = (total * 100) / sum;
  // } else {
  //   moy_pon = 0;
  // }

  // // Set last side based on edge sensors
  // if (mapped_values[0] > 30) {
  //   last_side = -1;
  // } else if (mapped_values[9] > 30) {
  //   last_side = 1;
  // }

  // // T junction detection
  // if (sum > 450) {
  //   if (abs(moy_pon) < 2500) {
  //     line_state = line_pos_state_t::FULL;
  //     is_full = true;
  //     last_full = millis();
  //   }
  // } 
  // // Left/right 90 degree turn detection
  // else if (mapped_values[0] + mapped_values[1] + mapped_values[2] + mapped_values[3] + mapped_values[4] > 5 * 50) {
  //   line_state = line_pos_state_t::TURN;
  //   nine_start = millis();
  // } else if (mapped_values[5] + mapped_values[6] + mapped_values[7] + mapped_values[8] + mapped_values[9] > 5 * 50) {
  //   line_state = line_pos_state_t::TURN;
  //   nine_start = millis();
  // }

  // // Handle T-junction state
  // if (line_state == line_pos_state_t::T && t_start && millis() < t_start + 800) {
  //   return 4400;
  // } else if (line_state == line_pos_state_t::T) {
  //   t_start = 0;
  // }

  // // Handle turn state
  // if (line_state == line_pos_state_t::TURN && nine_start && millis() < nine_start + 300) {
  //   return last_side * 4400;
  // } else if (line_state == line_pos_state_t::TURN) {
  //   nine_start = 0;
  // }

  // // Handle line lost or no line
  // if (sum < 5) {
  //   if (line_state == line_pos_state_t::LOST && millis() > 5000 + no_line_time) {
  //     no_line_time = 0;
  //     line_state = line_pos_state_t::NO_LINE;
  //     return 0;
  //   }
      
  //   if (is_full && (millis() > last_detected + 2000)) {
  //     line_state = line_pos_state_t::T;
  //     is_full = false;
  //     t_start = millis();
  //     return 4400;
  //   }
      
  //   if (lost_time == 0) {
  //     lost_time = millis();
  //     line_state = line_pos_state_t::LOSTING;
  //   }
      
  //   if (line_state == line_pos_state_t::LOSTING) {
  //     if ((lost_time + 2000) < millis()) {
  //       line_state = line_pos_state_t::LOST;
  //       pos_before_lost = 0;
  //       lost_time = 0;
  //       no_line_time = millis();
  //       return 0;
  //     }
          
  //     // Use last_side to determine turn direction when losing line
  //     if (last_side == 1) {
  //       return 4400;
  //     } else if (last_side == -1) {
  //         return -4400;
  //     }
  //   }
      
  //   line_state = line_pos_state_t::NO_LINE;
  //   return 0;
  // } 
  // // Handle strong line detection (turn adjustment)
  // else if (sum > 300) {
  //   // Optional: add turn adjustment here if needed
  //   // if (moy_pon > 0) moy_pon += 2000;
  //   // else moy_pon -= 2000;
  //   line_state = line_pos_state_t::DETECTED;
  //   lost_time = 0;
  //   last_full = 0;
  //   last_detected = millis();
  // } 
  // // Normal line following
  // else {
  //   line_state = line_pos_state_t::DETECTED;
  //   lost_time = 0;
  //   last_full = 0;
  //   last_detected = millis();
  // }

  // pos_before_lost = moy_pon;
  // return moy_pon;
  // const int8_t weights[] = {-44, -34, -24, -14, -4, 4, 14, 24, 34, 44};
  // int total = 0, moy_pon, sum=0;
  // for (int i = 0; i < 10; i++) {
  //   // values[i] = 100 - values[i];
  //   // if (values[i] < 900)
  //   //   values[i] = 0;
  //   total+=weights[i]*(mapped_values[i]/10);
  //   sum+=(mapped_values[i]/10);
  // }
  // moy_pon = (total*100)/sum;
  
  // // Set last side
  // if (mapped_values[0] > 30) {
  //   last_side = -1;
  // } else if (mapped_values[9] > 30) {
  //   last_side = 1;
  // }

  // int a = 0;
  // for (int i = 0; i < 10; i++) {
  //   a += mapped_values[i]/10;
  // }
  // // T junction detection
  // if (a > 450) {
  //   if (abs(moy_pon) < 2500 ) {
  //     line_state = line_pos_state_t::FULL;
  //     is_full = true;
  //     last_full = millis();
  //   }
  //   // Left/right 90
  // } else if (values[0] + values[1] + values[2] + values[3] + values[4] > 5*50) {
  //   // line_state = line_pos_state_t::TURN;
  //   // nine_start = millis();
  // } else if (values[5] + values[6] + values[7] + values[8] + values[9] > 5*50) {
  //   // line_state = line_pos_state_t::TURN;
  //   // nine_start = millis();
  // }
  // if (line_state == line_pos_state_t::T && millis() < t_start + 800) {
  //   return 4400;
  // } else {
  //   t_start = 0;
  // }
  // if (line_state == line_pos_state_t::TURN && millis() < nine_start + 300) {
  //   return last_side*4400;
  // } else {
  //   nine_start = 0;
  // }

  // if (a < 5) {
  //   if (line_state == line_pos_state_t::LOST && millis() > 5000 + no_line_time) {
  //     no_line_time = 0;
  //     line_state = line_pos_state_t::NO_LINE;
  //     return 0;
  //   }
  //   if (is_full && (millis() > last_full + 3000)) {
  //     line_state = line_pos_state_t::T;
  //     is_full = false;
  //     t_start = millis();
  //     return 4400;
  //   }
  //   if (!lost_time) {
  //     lost_time = millis();
  //     line_state = line_pos_state_t::LOSTING;
  //   }
  //   if (line_state == line_pos_state_t::LOSTING) {
  //     if ((lost_time + 2000) < millis()) {
  //       line_state = line_pos_state_t::LOST;
  //       pos_before_lost = 0;
  //       lost_time = 0;
  //       no_line_time = millis();
  //       return 0;
  //     }
  //     // if (pos_before_lost > 0) {
  //     //   return 4400; {}
  //     // } else {
  //     //   return -4400;
  //     // } 
  //     if (last_side == 1) {
  //       return 4400;
  //     } else if (last_side == -1) {
  //       return -4400;
  //     } 
  //   }
  //   line_state = line_pos_state_t::NO_LINE;
  // } else if (a > 300) {
  //   // Left/Right turn
  //   if (moy_pon > 0) {
  //     // moy_pon += 2000;
  //   } else {
  //     // moy_pon -= 2000;
  //   }
  // } else {
  //   line_state = line_pos_state_t::DETECTED;
  //   lost_time = 0;
  //   last_full = 0;
  // }


  // pos_before_lost = moy_pon;
  // return moy_pon; 
}

void update_can() {
  // for (uint8_t i = 0; i < 10; i++) {
  //   t_line_sensor_raw_data msg_content = {i, values[i]};
  //   Can.send_struct(msg_content);
  // }
  t_line_sensor_data msg_content { .line_pos=get_line_position(mapped_values), .state=line_state };
  // Serial.printf("can to be line pos: %d\n", msg_content.line_pos);
  can.send_struct(msg_content);
  for (size_t i = 0; i < 10; i++) {
    t_line_sensor_raw_data data {.sensor_id=i, .mapped_value=mapped_values[i]/10, .raw_value=raw_values[i]/10};
    can.send_struct(data);
  }
}

// void read_can() {


void setup() {
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);

  pinMode(PA15, OUTPUT);
  digitalWrite(PA15, LOW);

  can.init();
  // delay(1000);
  Serial.println("salut rhey");
  // can.send_rtr(CAN_ID::BLDC_CURRENT_POS);
  // can.send_rtr(CAN_ID::BLDC_CURRENT_SPEED);
  // t_bldc_alignment_start align_msg = {.motor_id=motor_id_t::LEFT};
  // can.send_struct(align_msg);
  
  init_leds();
  //digitalWrite(PA15, HIGH);
  leds_A[0] = CRGB::Red4;
  FastLED.setBrightness(30);
  FastLED.show();

  //digitalWrite(PA15, HIGH);
  // Pins used for serial on devboard
  // pinMode(BTN1, INPUT_PULLUP);
  // pinMode(BTN2, INPUT_PULLUP);
 
  // init_leds();

  HardwareTimer* can_timer = new HardwareTimer(TIM2);
  can_timer->setOverflow(500, HERTZ_FORMAT);
  can_timer->attachInterrupt([](){
    can.handle_can();
  });
  // start the timer
  can_timer->resume();

}

int oversampling = 0;

void loop() {
  update_line_sensors(sensors);
  if (oversampling == 10 ) {
    oversampling = 0;
    // print_line_values(line_sensors);
    update_can();
    update_leds();
    // can.handle_can();
    // can.send_rtr(CAN_ID::BLDC_ALIGNMENT_RESULTS);
    for (size_t i = 0; i < 10; i++) {
      raw_values[i] = 0;
      mapped_values[i] = 0;
    }
  }
  delay(5);
  oversampling++;
}