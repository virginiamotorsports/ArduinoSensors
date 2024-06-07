#include <stdio.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <Arduino_CAN.h>

// #define MAX_LEFT_SA_SENSOR 40
// #define MAX_LEFT_SA_WHEEL -23
// #define MAX_RIGHT_SA_SENSOR 120
// #define MAX_RIGHT_SA_WHEEL 23

// Signals definitions:
// These are random guesses until we do continuity checks
#define RR_PIN A0
#define RL_PIN A1
#define FR_PIN A3
#define FL_PIN A4
#define BP_PIN A5

//#define NUM_LEDS 17
//#define SA_PIN A5 
//#define FLWS_PIN A4
//#define RLWS_PIN A3
//#define IS_PIN 3
//#define LED_PIN 6
// #define CANRX 13 
//#define DRIVESIGNAL_PIN 12
// #define CANTX 10
//#define BUZZER_PIN 9
//#define BP_PIN A2

#define ADC_MAX_VAL ((1 << 14) - 1)

const float vref = 5.0;
const float bp_full_scale = 100.0;

const float rr_full_scale = 100.0;
const float rl_full_scale = 100.0;
const float fr_full_scale = 100.0;
const float fl_full_scale = 100.0;


const uint16_t battery_health_red[9] = {(255/11)*8, (255/11)*7, (255/11)*6, (255/11)*5, (255/11)*4, (255/11)*3, (255/11)*2, (255/11)*1, (255/11)*0};
const uint16_t battery_health_green[9] = {(255/11)*3, (255/11)*4, (255/11)*5, (255/11)*6, (255/11)*7, (255/11)*8, (255/11)*9, (255/11)*10, (255/11)*11};

//Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

typedef struct {
    bool AMS_Fault;
    bool BSPD_Fault;
    bool IMD_Fault;
    bool PUMP_Indicator;
    bool FAN_Indicator;
    bool Launch_Control;
    bool Brake_Status;
    bool Miscellaneous;
} Flags;

typedef struct {
    uint8_t AMS_Indicator_Index;
    uint8_t BSPD_Indicator_Index;
    uint8_t IMD_Indicator_Index;
    uint8_t PUMP_Indicator_Index;
    uint8_t FAN_Indicator_Index;
    uint8_t Launch_Control_Indicator_Index;
    uint8_t Brake_Status_Indicator_Index;
    uint8_t Miscellaneous_Indicator_Index;
} Flags_Indexes;

Flags Indicator_Flags = {true, 
                         true, 
                         true, 
                         true,
                         true,
                         false,
                         true,
                         true
                         };  // is_enabled = true, is_visible = false, is_editable = true

Flags_Indexes Indicator_Indexes = {14, //
                                   15,
                                   16,
                                   1,
                                   0,
                                   2,
                                   3,
                                   4
                                   };
                                   
                                   

/* 
void UpdateIndicators(void) {
  if (Indicator_Flags.AMS_Fault) {
    pixels.setPixelColor(Indicator_Indexes.AMS_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.AMS_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.BSPD_Fault) {
    pixels.setPixelColor(Indicator_Indexes.BSPD_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.BSPD_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.IMD_Fault) {
    pixels.setPixelColor(Indicator_Indexes.IMD_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.IMD_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.PUMP_Indicator) {
    pixels.setPixelColor(Indicator_Indexes.PUMP_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.PUMP_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.FAN_Indicator) {
    pixels.setPixelColor(Indicator_Indexes.FAN_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.FAN_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.Launch_Control) {
    pixels.setPixelColor(Indicator_Indexes.Launch_Control_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.Launch_Control_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.Brake_Status) {
    pixels.setPixelColor(Indicator_Indexes.Brake_Status_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.Brake_Status_Indicator_Index, pixels.Color(0, 30, 0));

  if (Indicator_Flags.Miscellaneous) {
    pixels.setPixelColor(Indicator_Indexes.Miscellaneous_Indicator_Index, pixels.Color(255, 0, 0));
  }
  else pixels.setPixelColor(Indicator_Indexes.Miscellaneous_Indicator_Index, pixels.Color(0, 30, 0));

  pixels.show();
}

void UpdateBatteryHealth(uint8_t bat) {

  if(bat > 100){
    bat = 0; //this is to ensure that bad data doesnt cause the battery to go higher
  }
  uint8_t scaled_percent = ceil(0.09 * bat);
  // Highlight the red ones:
  uint8_t actual_num = 13-scaled_percent;
  for (int i=5; i<=actual_num; i++) {
    pixels.setPixelColor(i, pixels.Color(100, 0, 0));
  }
  // highlight the green ones
  for (int i=13; i > actual_num; i--) {
    // Serial.print(i);
    pixels.setPixelColor(i, pixels.Color(battery_health_red[scaled_percent], battery_health_green[scaled_percent], 0));
  }
  pixels.show();
  // delay(2000);
} */

float process_brake_pressure(uint32_t adc) {
  
  float bp_voltage;
  float bp;

  bp_voltage = (adc*vref)/ADC_MAX_VAL ;
  bp = ((bp_voltage - 0.5) / (vref-1)) * bp_full_scale;

   Serial.print("BP Voltage: ");
   Serial.println(bp_voltage);
  // Serial.print("BP ADC Val: ");
  // Serial.println(analogRead(BP_PIN));

  return bp;
}

float process_RR_suspension(uint32_t adc) {
  float rr_voltage;
  float rr;
  rr_voltage = (adc*vref)/ADC_MAX_VAL;
  rr = ((rr_voltage - 0.5) / (vref-1)) * rr_full_scale;

  Serial.print("RR voltage: ");
  Serial.print(rr_voltage);
  // Serial.print("RR ADC Val: ");
  // Serial.println(analogRead(RR_PIN));
}

float process_RL_suspension(uint32_t adc) {
  float rl_voltage;
  float rl;
  rl_voltage = (adc*vref)/ADC_MAX_VAL;
  rl = ((rl_voltage - 0.5) / (vref-1)) * rl_full_scale;

  Serial.print("RL voltage: ");
  Serial.print(rl_voltage);
  // Serial.print("RL ADC Val: ");
  // Serial.println(analogRead(RL_PIN));
  return rl;
}

float process_FR_suspension(uint32_t adc) {
  float fr_voltage;
  float fr;
  fr_voltage = (adc*vref)/ADC_MAX_VAL;
  fr = ((fr_voltage - 0.5) / (vref-1)) * fr_full_scale;

  Serial.print("FR voltage: ");
  Serial.print(fr_voltage);
  // Serial.print("FR ADC Val: ");
  // Serial.println(analogRead(FR_PIN));
  return fr;
}

float process_FL_suspension(uint32_t adc) {
  float fl_voltage;
  float fl;
  fl_voltage = (adc*vref)/ADC_MAX_VAL;
  fl = ((fl_voltage - 0.5) / (vref-1)) * fl_full_scale;

  Serial.print("FL voltage: ");
  Serial.print(fl_voltage);
  // Serial.print("FL ADC Val: ");
  // Serial.println(analogRead(FL_PIN));
  return fl;
}


/* uint32_t process_wheel_speed(uint32_t adc) {

  uint32_t ws_voltage;
  uint32_t ws_frequency;

  ws_voltage = (vref/ADC_MAX_VAL)*adc;

  // VOUT = VCC × f × C1 × R1
  // C1 = 10 nf
  // R1 = LINPOT VALUE

  ws_frequency = ws_voltage / (.00000001 * 5 * 100);

  return ws_frequency;
}

uint32_t process_steering_angle(uint32_t adc) {

  uint32_t sa_voltage;

  sa_voltage = (vref/ADC_MAX_VAL)*adc;

  return sa_voltage;
}

void PlayRTDBuzzer(uint8_t pin_num) {
    digitalWrite(pin_num, HIGH);

    delay(1000); // Delay for 1000 milliseconds (1 second)

    digitalWrite(pin_num, LOW);
    delay(1000);
} */