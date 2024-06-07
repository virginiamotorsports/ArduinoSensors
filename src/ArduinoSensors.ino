#include "can_helpers.hpp"
#include <Arduino_CAN.h>
#include <experimental/filesystem>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <helper_functions.hpp>

#define BAUDRATE 1000000


//define initial states 
uint32_t steering_angle;
uint32_t rearleftws;
uint32_t frontleftws;
uint32_t inertia;
bool drive;
bool last_drive;
uint8_t battery_soc_percent = 25;
int8_t max_temp = 0;
int8_t min_temp = 0;
float_t actual_pack_voltage = 0;

float_t brake_pressure;
uint32_t coolant_temp;

uint8_t battery_health = 3;

uint16_t count = 0;

// Note frequencies (in Hz) for the "ray ray, UVA" part
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_E5 659

// Note durations (in milliseconds)
#define QUARTER_NOTE 375
#define HALF_NOTE 750

int melody[] = {NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_E5};
int noteDurations[] = {HALF_NOTE, HALF_NOTE, QUARTER_NOTE, QUARTER_NOTE, QUARTER_NOTE};


uint8_t is_bytes[8] {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, inertia ? 0x01 : 0x00};
// char rtd_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, drive ? 0xFF : 0x00};
// char flws_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 
//               ((frontleftws >> 24) & 0xFF), ((frontleftws >> 16) & 0xFF), 
//               ((frontleftws >> 8) & 0xFF), (frontleftws & 0xFF)};
// char sa_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 
//               ((sa_wheel_angle >> 24) & 0xFF), ((sa_wheel_angle >> 16) & 0xFF), 
//               ((sa_wheel_angle >> 8) & 0xFF), (sa_wheel_angle & 0xFF)};
// char rlws_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 
//               ((rearleftws >> 24) & 0xFF), ((rearleftws >> 16) & 0xFF), 
//               ((rearleftws >> 8) & 0xFF), (rearleftws & 0xFF)};

void setup() {

  analogReadResolution(14);
  pinMode(SA_PIN, INPUT_PULLUP);
  // pinMode(FLWS_PIN, INPUT);
  // pinMode(RLWS_PIN, INPUT);
  pinMode(IS_PIN, INPUT);
  // pinMode(LED_PIN, INPUT);
  // pinMode(CANRX, INPUT);
  pinMode(DRIVESIGNAL_PIN, INPUT);
  // pinMode(CANTX, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(BAUDRATE, SERIAL_8N1);
  Serial1.setTimeout(1000);

  pixels.clear(); // Set all pixel colors to 'off'
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.print("Hello this the the Dashboard Code\r\n");
}


void loop() {

  brake_pressure = process_brake_pressure(analogRead(BP_PIN));
  // Serial.print("Brake Pressure: ");
  // Serial.println(brake_pressure);

  rearleftws = process_wheel_speed(analogRead(RLWS_PIN));
  frontleftws = process_wheel_speed(analogRead(FLWS_PIN));

  inertia = (digitalRead(IS_PIN) == HIGH ? true : false);
  drive = (digitalRead(DRIVESIGNAL_PIN) == HIGH ? true : false);

  steering_angle = process_steering_angle(analogRead(SA_PIN));

  if(drive == true && last_drive == false){
    PlayRTDBuzzer(BUZZER_PIN);
  }
  last_drive = drive;

  UpdateBatteryHealth(battery_soc_percent);

  UpdateIndicators();

  send_can_data();

  while (CAN.available()) {
    CanMsg msg = CAN.read();
    handleCanMessage(msg);
  }
  // delay(10);
}

void send_can_data(void){

  CanMsg is_message = CanMsg(CanExtendedId(0x1806E5F4), 8, is_bytes);

  if(count > 1){
    int ret = CAN.write(is_message);
    Serial.println(is_message);
    if(!(ret == 0 || ret == 1)){
        Serial.print("CAN Error: ");
        Serial.println(ret);
        CAN.clearError();
    }
    count = 0;
  }

}

void handleCanMessage(const CanMsg &msg) {
  // Serial.print("CAN ID: ");
  // Serial.println(msg.id);
  switch (msg.id) {
    case ACC_STATUS:
      handleAmsStatus(msg);
      break;
    case VCU_INPUTS_ID:
      handleBPmsg(msg);
    default:
      // Unknown message ID, ignore or handle as needed
      break;
  }
}

void handleAmsToDash(const CanMsg &msg) {
  // Parse the status of the IMD fault, AMS fault
}

void handleAmsFault(const CanMsg &msg) {
  // Parse any other fault conditions
}

void handleAmsStatus(const CanMsg &msg) {
  battery_soc_percent = msg.data[1];
  Indicator_Flags.AMS_Fault = (msg.data[0] >> 1) & 0x1;
  Indicator_Flags.IMD_Fault = (msg.data[0]) & 0x1;
  max_temp = msg.data[4];
  min_temp = msg.data[5];
  uint16_t scaled_pack_voltage = ((msg.data[2] << 8) | msg.data[3]);
  actual_pack_voltage = scaled_pack_voltage / 10.0;


}

void handleBPmsg(const CanMsg &msg) {
  Serial.print("Received: ");
  float bp = msg.data[BP_MSG_INDEX];
  Serial.println(bp);
}

void PlayRTDBuzzer(int pin_num){
  // Play the melody
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, melody[i], noteDurations[i]);
    delay(noteDurations[i] * 1.30);  // Add a small delay between notes
    noTone(BUZZER_PIN);              // Stop the tone
  }
}