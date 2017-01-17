#include <Arduino.h>
#include "include/hexaLeg.h"

HexaLeg::HexaLeg(){}
HexaLeg::~HexaLeg(){}


void HexaLeg::setPins(uint8_t pinCoxa, uint8_t pinFemur, uint8_t pinTibia, uint16_t calibData[][2] ){

    Serial.print("leg min:");
    Serial.print(calibData[1][0]);
    Serial.println("");

    coxa.attach(pinCoxa);
    femur.attach(pinFemur);
    tibia.attach(pinTibia);

    coxa.attach(pinCoxa, calibData[0][0], calibData[0][1]);
    femur.attach(pinFemur, calibData[1][0], calibData[1][1]);
    tibia.attach(pinTibia, calibData[2][0], calibData[2][1]);

    easeTime = EASE_TIME;
    servoFrameMillis = 20;

    //coxaEaser.begin( coxa, servoFrameMillis );
    //femurEaser.begin( femur, servoFrameMillis );
    //tibiaEaser.begin( tibia, servoFrameMillis );
}

void HexaLeg::setAngles(uint8_t angleCoxa, uint8_t angleFemur, uint8_t angleTibia){

    coxa.write(angleCoxa);
    femur.write(angleFemur);
    tibia.write(angleTibia);
    //coxaEaser.easeTo(angleCoxa, easeTime);
    //femurEaser.easeTo(angleFemur, easeTime);
    //tibiaEaser.easeTo(angleTibia, easeTime);
}

