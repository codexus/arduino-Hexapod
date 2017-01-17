#ifndef HEXALEG_H
#define HEXALEG_H

#include <Servo.h>
#include "lib/ServoEaser.h"

#define EASE_TIME 200;

//#include "lib/Vector3.h" //eats memory
// therefore used a basic structure
struct BasicVector3{

    BasicVector3() : x(0.0f), y(0.0f), z(0.0f){};
    BasicVector3(float inX,float inY,float inZ){
        x = inX;
        y = inY;
        z = inZ;
    };
    float x, y, z;
};

class HexaLeg
{
    private:

        int lastMircos;
        int easeTime;
        int servoFrameMillis; // minimum time between servo updates

        Servo coxa, femur, tibia;
        //ServoEaser coxaEaser, femurEaser, tibiaEaser; // propably won't be used
        uint8_t _pinCoxa, _pinFemur, _pinTibia;


    public:

        //int initCoxaAngle;
        uint8_t legNum;
        // offset of the body center
        BasicVector3 offsetPos;
        // offset of the leg frame
        float offsetAngleY;


        //ctor
        HexaLeg();
        virtual ~HexaLeg();
        // setPins using only 8bits for setting apropriate pin to servo
        void setPins(uint8_t pinCoxa, uint8_t pinFemur, uint8_t pinTibia, uint16_t calibData[][2] );
        void setAngles(uint8_t posCoxa, uint8_t posFemur, uint8_t posTibia);

};

#endif // HEXALEG_H
