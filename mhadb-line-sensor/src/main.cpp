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

class LineCanHandler {
public:
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
    // Serial.print("zero angle: ");
    // Serial.println(data.zero_electric_angle);
    // Serial.printf("direction: %d\n", data.sensor_direction);
  }

  template<typename T>
  static void handle_struct(T data) {};
  template<typename T>
  static bool update_struct(T* data) {return false;};
};

CanController<MHADBCanController<LineCanHandler>> can;

int sensors[] = {LS0, LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9 };

CRGB leds_A[5];
CRGB leds_B[5];

uint16_t line_sensors[10];

void init_leds() {
  FastLED.addLeds<WS2812B, PB12, GRB>(leds_A, 5);
  FastLED.addLeds<WS2812B, PB13, GRB>(leds_B, 5);
}

DEFINE_GRADIENT_PALETTE( heatmap_gp ) {
  0,     0,   0,  0,   //black
128,   250,  15,  255,   //red
255,   15,  250,  255 };
CRGBPalette16 myPal = heatmap_gp;

void update_leds(uint16_t*  values) {
  for (size_t i = 0; i < 10; i++) {
    uint8_t brightness = map(values[i], 0, 100, 0, 250);
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

  FastLED.show();
}

void update_line_sensors(int* sensors, uint16_t*  values) {
  for (size_t i = 0; i < 10; i++) {
    // Devboard sensors
    //#if 0
    int val = analogRead(sensors[i]);
    if (val<500) val=500;
    values[i] = map(val, 500, 1024, 0, 100);
    //#endif
    //values[i]= analogRead(sensors[i]);
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

int16_t get_line_position(uint16_t* values) {
  // fonction 1 rémi
  #if 0
  // TODO: Moyenne réduite ou cubique ou mieux
  const uint8_t weights[] = {-44, -34, -24, -14, -4, 4, 14, 24, 34, 44};
  // Devboard
  // const uint8_t weights[] = {30, 30, 30, 30, 30};
  int32_t total_moy = 0;
  int total = 0;
  for (uint8_t i = 0; i < 10; i++) {
    // Serial.println(weights[i % 4] * values[i] * ((i < 4)? 1: -1));
    // TODO: les poids ne sont pas symmétriques
    total_moy += weights[i % 4] * values[i] * ((i < 4)? 1: -1);
    total += values[i];
  }
  // Serial.printf("Line pos: %d \t", total_moy);
  // Devboard
  // return total_moy/(total);
  return total_moy;
  //#endif
  // fonction 1 victor
  if 0
  double coef = 0.95;
  int max=0, k=0, sum=0; 
  int tab[10]={0}; 

  for (int i=0; i<10; i++) {
    if (values[i] > max) max=values[i];
  }
  for (int j=0;j<10;j++) {
    if (values[j] > max*coef){
      tab[k] = j;
      //sum += values[j];
      sum += j;
      k++;
    }
  }
  return (sum/k);
  #endif
  // fonction 2 victor
  #if 0
  int nb_val=10;
  int size_group = 2;
  

  int tab[nb_val]={0};
  int max=0, sum=0, calc, moy_val, moy_pos, count=0, max_ch=0, pos=0;

  if (nb_val>10) {
    for(int i=0; i<nb_val; i++){
      if (i%2==0){
        calc=values[i/2];
        tab[i]=calc;
        sum+=calc;
        if (calc > max) max=calc;
      }
      else {
        calc=int(values[((i-1)/2)]+values[((i+1)/2)]);
        tab[i]=calc;
        sum+=calc;
        if (calc > max) max=calc;
      }
    }
  }
  if (nb_val==10){
    for (int i=0;i<10;i++) {
      sum+=values[i];
      tab[i] = values[i];
    }
  }

  moy_val=int(sum/nb_val);
  for(int j=0; j<nb_val; j++){
    if (tab[j]<=moy_val) tab[j]=0;
  }

  if (size_group == 2) {
    for (int k=2;k<nb_val;k++){
      if((tab[k])&& ((tab[k+1])||(tab[k-1])) ) continue;
      else tab[k]=0;
    }
  }
  if (size_group == 3) {
    for (int k=2;k<nb_val;k++){
      if( (tab[k])&& ((tab[k+1]) && (tab[k+2])  ||  (tab[k-1] && tab[k-2]) || (tab[k-1] && tab[k+1])) ) continue;
      else tab[k]=0;
    }
  }

  count = 0;
  max_ch = 0;
  for(int i=1;i<nb_val;i++) {
    if (tab[i-1] && tab[i]) {
      count++;
      if ((tab[i-2]==0)&& i!=1) pos=i-1;
    }
    else {
      if (count>max) max_ch=count;
      count=0;
    }
  } // potentiellement pos=0 et max_ch=0

  sum=0;
  for(int i=pos;i<pos+count;i++){
    sum+=i;
  }
  return float(sum/count);
  #endif
  const int8_t weights[] = {-44, -34, -24, -14, -4, 4, 14, 24, 34, 44};
  int total = 0, moy_pon, sum=0;
  for (int i = 0; i < 10; i++) {
    if (values[i] < 50)
      values[i] = 0;
    total+=weights[i]*(values[i]);
    sum+=(values[i]);
  }
  moy_pon = int(total/sum*4);

  return moy_pon; 
}

void update_can(uint16_t* values) {
  // for (uint8_t i = 0; i < 10; i++) {
  //   t_line_sensor_raw_data msg_content = {i, values[i]};
  //   Can.send_struct(msg_content);
  // }
  t_line_sensor_data msg_content { .line_pos=get_line_position(values) };
  // Serial.printf("can to be line pos: %d\n", msg_content.line_pos);
  can.send_struct(msg_content);
  for (size_t i = 0; i < 10; i++) {
    t_line_sensor_raw_data data {.sensor_id=i, .value=values[i]};
    //can.send_struct(data);
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
  can.send_rtr(CAN_ID::BLDC_CURRENT_POS);
  can.send_rtr(CAN_ID::BLDC_CURRENT_SPEED);
  t_bldc_alignment_start align_msg = {.motor_id=motor_id_t::LEFT};
  can.send_struct(align_msg);
  
  init_leds();
  //digitalWrite(PA15, HIGH);
  leds_A[0] = CRGB::Red4;
  FastLED.setBrightness(20);
  FastLED.show();

  //digitalWrite(PA15, HIGH);
  // Pins used for serial on devboard
  // pinMode(BTN1, INPUT_PULLUP);
  // pinMode(BTN2, INPUT_PULLUP);
 
  // init_leds();

  HardwareTimer* can_timer = new HardwareTimer(TIM2);
  can_timer->setOverflow(120, HERTZ_FORMAT);
  can_timer->attachInterrupt([](){
    can.handle_can();
    update_can(line_sensors);
  });
  // start the timer
  can_timer->resume();

}

void loop() {
  update_line_sensors(sensors, line_sensors);
  print_line_values(line_sensors);
  update_leds(line_sensors);
  // can.handle_can();
  delay(100);
  // can.send_rtr(CAN_ID::BLDC_ALIGNMENT_RESULTS);
}