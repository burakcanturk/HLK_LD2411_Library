#include "HLK_LD2411.h"
#include <SoftwareSerial.h>

#define ld2411_rx 14
#define ld2411_tx 27

//EspSoftwareSerial::UART ld2411_ser(ld2411_rx, ld2411_tx);

HLK_LD2411 ld2411(ld2411_rx, ld2411_tx, &Serial1);
//HLK_LD2411 ld2411(&ld2411_ser);
//HLK_LD2411 ld2411(&Serial2);

uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  ld2411.begin(115200);
  delay(1000);
  Serial.println(ld2411.setBaudrate(115200));
  Serial.println(ld2411.setParameters(30, 717, 30, 425, 20));
  //ld2411.reboot();
  //ld2411.enableConfiguration();
  //ld2411.closeBluetooth();
}

void loop() {

  Serial.println(ld2411.getFirmwareVersion());
  delay(1000);

  /*ld2411.read();

  Serial.print("Campaign Target: ");
  Serial.println(ld2411.getCampaignTarget());
  Serial.print("Micromotion Target: ");
  Serial.println(ld2411.getMicromotionTarget());
  Serial.println("---------------------------------------------------------");

  delay(100);*/

  //Serial.println(ld2411.setBaudrate(256000));
  //Serial.println(ld2411.setBaudrate(115200));
  //delay(1000);

  /*if (Serial.available() > 0) {
    
    char data = Serial.read();

    if (data == '+') {
      Serial.println(ld2411.setParameters(50, 500, 60, 300, 100));
    }

    else if (data == '-') {
      Serial.println(ld2411.factoryReset());
    }
  }

  ld2411.readParameters();

  Serial.print("Motion Range Min: ");
  Serial.println(ld2411.getMotionRangeMin()); // 30
  Serial.print("Motion Range Max: ");
  Serial.println(ld2411.getMotionRangeMax()); // 717
  Serial.print("Micromotion Range Min: ");
  Serial.println(ld2411.getMicromotionRangeMin()); // 30
  Serial.print("Micromotion Range Max: ");
  Serial.println(ld2411.getMicromotionRangeMax()); // 425
  Serial.print("No One Duration: ");
  Serial.println(ld2411.getNoOneDuration()); // 20
  Serial.println("---------------------------------------------------------");*/

  //Serial.println(ld2411.factoryReset());
  //delay(1000);

  /*if (Serial.available() > 0) {
    
    char data = Serial.read();

    if (data == '+') {
      Serial.println(ld2411.enableConfiguration());
    }

    else if (data == '-') {
      Serial.println(ld2411.reboot());
    }
  }

  if (Serial1.available() > 0)
    Serial.println(Serial1.read());*/

  /*Serial.println(ld2411.reboot());
  delay(1000);*/

  //Serial.println(ld2411.getMacAddress());

  //Serial.println(ld2411.openBluetooth());
  //delay(5000);
  //Serial.println(ld2411.closeBluetooth());
  //delay(5000);

  //Serial.println(ld2411.getFirmwareVersion());

  //Serial.println(ld2411.setParameters(50, 500, 60, 300, 100));

  //ld2411.setParameters(50, 250, 60, 400, 100);
  /*ld2411.setParameters(30, 717, 30, 425, 20);

  ld2411.readParameters();

  Serial.print("Motion Range Min: ");
  Serial.println(ld2411.getMotionRangeMin()); // 30
  Serial.print("Motion Range Max: ");
  Serial.println(ld2411.getMotionRangeMax()); // 717
  Serial.print("Micromotion Range Min: ");
  Serial.println(ld2411.getMicromotionRangeMin()); // 30
  Serial.print("Micromotion Range Max: ");
  Serial.println(ld2411.getMicromotionRangeMax()); // 425
  Serial.print("No One Duration: ");
  Serial.println(ld2411.getNoOneDuration()); // 20
  Serial.println("---------------------------------------------------------");*/

  //Serial.println(ld2411.enableConfiguration());

  /*if (Serial.available() > 0) {
    
    char data = Serial.read();

    if (data == '+') {
      Serial.println(ld2411.enableConfiguration());
    }

    else if (data == '-') {
      Serial.println(ld2411.endConfiguration());
    }
  }

  if (Serial1.available() > 0)
    Serial.println(Serial1.read());

  counter++;*/

  //ld2411.enableConfiguration();

  //Serial.println(ld2411.endConfiguration());
  //delay(1000);
  
  /*ld2411.read();

  Serial.print("Campaign Target: ");
  Serial.println(ld2411.getCampaignTarget());
  Serial.print("Micromotion Target: ");
  Serial.println(ld2411.getMicromotionTarget());
  Serial.println("---------------------------------------------------------");*/
}
