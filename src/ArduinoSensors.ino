#include "can_helpers.hpp"
#include <Arduino_CAN.h>
//#include <experimental/filesystem>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <helper_functions.hpp>

#define BAUDRATE 1000000


//define initial states 

uint32_t brake_pressure;
uint32_t rearright_suspension;
uint32_t rearleft_suspension;
uint32_t frontright_suspension;
uint32_t frontleft_suspension;

// uint32_t steering_angle;
// uint32_t rearleftws;
// uint32_t frontleftws;
// uint32_t inertia;
//bool drive;
//bool last_drive;
// uint8_t battery_soc_percent = 25;
// int8_t max_temp = 0;
// int8_t min_temp = 0;
// float_t actual_pack_voltage = 0;

// float_t brake_pressure;
// uint32_t coolant_temp;

// uint8_t battery_health = 3;

 uint16_t count = 0;

// Note frequencies (in Hz) for the "ray ray, UVA" part
//#define NOTE_G4 392
//#define NOTE_A4 440
//#define NOTE_E5 659

// Note durations (in milliseconds)
//#define QUARTER_NOTE 375
//#define HALF_NOTE 750

//int melody[] = {NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_E5};
//int noteDurations[] = {HALF_NOTE, HALF_NOTE, QUARTER_NOTE, QUARTER_NOTE, QUARTER_NOTE};


uint8_t brake_pressure_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}; //Cole

//uint8_t is_bytes[8] {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // inertia
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

  pinMode(RR_PIN, INPUT);
  pinMode(RL_PIN, INPUT);
  pinMode(FR_PIN, INPUT);
  pinMode(FL_PIN, INPUT);
  pinMode(BP_PIN, INPUT);


  //pinMode(SA_PIN, INPUT_PULLUP);
  // pinMode(FLWS_PIN, INPUT);
  //pinMode(RLWS_PIN, INPUT);
  //pinMode(IS_PIN, INPUT);
  // pinMode(LED_PIN, INPUT);
  // pinMode(CANRX, INPUT);
  //pinMode(DRIVESIGNAL_PIN, INPUT);
  // pinMode(CANTX, INPUT);
  //pinMode(BUZZER_PIN, OUTPUT);
  //pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(BAUDRATE, SERIAL_8N1);
  Serial1.setTimeout(1000);

  //pixels.clear(); // Set all pixel colors to 'off'
  //pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.print("Hello this the the sensor code\r\n");
}


void loop() {

  brake_pressure = process_brake_pressure(analogRead(BP_PIN));
  Serial.print("Brake Pressure: ");
  Serial.println(brake_pressure);
  
  rearright_suspension = process_RR_suspension(analogRead(RR_PIN));
  Serial.print("Rear right suspension: ");
  Serial.println(rearright_suspension);

  rearleft_suspension = process_RL_suspension(analogRead(RL_PIN));
  Serial.print("Rear left suspension: ");
  Serial.println(rearleft_suspension);

  frontright_suspension = process_FR_suspension(analogRead(FR_PIN));
  Serial.print("Front right suspension: ");
  Serial.println(frontright_suspension);

  frontleft_suspension = process_FL_suspension(analogRead(FL_PIN));
  Serial.print("Front left suspension: ");
  Serial.println(frontleft_suspension);




  //rearleftws = process_wheel_speed(analogRead(RLWS_PIN));
  //frontleftws = process_wheel_speed(analogRead(FLWS_PIN));

  //inertia = (digitalRead(IS_PIN) == HIGH ? true : false);
  //drive = (digitalRead(DRIVESIGNAL_PIN) == HIGH ? true : false);

  //steering_angle = process_steering_angle(analogRead(SA_PIN));


  // if(drive == true && last_drive == false){
  //   PlayRTDBuzzer(BUZZER_PIN);
  // }
  // last_drive = drive;

  //UpdateBatteryHealth(battery_soc_percent);

  // UpdateIndicators();

  send_can_data();

  while (CAN.available()) {
    CanMsg msg = CAN.read();
    handleCanMessage(msg);
  }
  // delay(10);
}

void send_can_data(void){

  uint8_t brake_pressure_bytes[8] {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
  brake_pressure_bytes[4] = (brake_pressure >> 24) & 0xFF;
  brake_pressure_bytes[5] = (brake_pressure >> 16) & 0xFF;
  brake_pressure_bytes[6] = (brake_pressure >> 8) & 0xFF;
  brake_pressure_bytes[7] = brake_pressure & 0xFF;

  CanMsg brake_pressure_message = CanMsg(CanExtendedId(0x1806E5F5), 8, brake_pressure_bytes);
  
  if(count > 1){
    int ret_brake = CAN.write(brake_pressure_message);
    Serial.print("Brake Pressure CAN Message: ");
    Serial.println(brake_pressure_message);
    if(!(ret_brake == 0 || ret_brake == 1)){
        Serial.print("CAN Error: ");
        Serial.println(ret_brake);
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
      //handleAmsStatus(msg);
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

/* 
void handleAmsStatus(const CanMsg &msg) {
  battery_soc_percent = msg.data[1];
  Indicator_Flags.AMS_Fault = (msg.data[0] >> 1) & 0x1;
  Indicator_Flags.IMD_Fault = (msg.data[0]) & 0x1;
  max_temp = msg.data[4];
  min_temp = msg.data[5];
  uint16_t scaled_pack_voltage = ((msg.data[2] << 8) | msg.data[3]);
  actual_pack_voltage = scaled_pack_voltage / 10.0;


} */

void handleBPmsg(const CanMsg &msg) {
  Serial.print("Received: ");
  float bp = msg.data[BP_MSG_INDEX];
  Serial.println(bp);
}

/* void PlayRTDBuzzer(int pin_num){
  // Play the melody
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, melody[i], noteDurations[i]);
    delay(noteDurations[i] * 1.30);  // Add a small delay between notes
    noTone(BUZZER_PIN);              // Stop the tone
  }
} */