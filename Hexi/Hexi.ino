#include <Arduino.h>
#include "include/hexaLeg.h"
#include "src/fastFunc.ino"
#include <Servo.h>

#define START_IND 22
#define NUMLEGS 6
#define NUM_SERV_BYLEG 3


// lenght of each leg segment
#define COXA_LENGTH 4
#define FEMUR_LENGTH 5
#define TIBIA_LENGTH 7

// speed conversion between radians and angles
#define Deg2Rad 0.01745329
#define Rad2Deg 57.29578


// TODO: Add fast functions like lookup tables for sin and cos, as the double pi

uint16_t microSecAngle, val;

long previousMillis = 0;
long legUpdateTime = 15;



// init a array of legs
HexaLeg legs[NUMLEGS];
//legs calibration data:
//calibration is stored in convention: [leg][servo max min]
uint16_t calibration[NUMLEGS][NUM_SERV_BYLEG][2] = {
    { {780,2500}, {900,2800}, {790, 2600} }, // leg1 : min: 780, 780, 790 max:  2500, 2800, 2600
    { {780,2500}, {550,2300}, {650, 2400} }, // leg2 : min: 780, 500, 650 max:  2500, 2300, 2300
    { {730,2450}, {820,2400}, {640, 2300} }, // leg3 : min: 730, 820, 640 max:  2450, 2550, 2300
    { {580,2600}, {500,2500}, {550, 2400} }, // leg4 : min: 580, 540, 550 max:  2600, 2400, 2400
    { {750,2500}, {620,2300}, {620, 2300} }, // leg5 : min: 750, 620, 620 max:  2500, 2300, 2300
    { {800,2550}, {750,2500}, {750, 2450} }  // leg6 : min: 800, 750, ?750 max:  2550, 2450, ?2450 // REPLACE TEH SERVO
};



// vars needed for inputs
String readString, posX, posY, posZ;

int px, py, pz;

// Vars needed to walk
bool isMoving = false;

unsigned long old_time;
unsigned long new_time;
int currentCycleTime = 0;
float cycleTime = 0.6; // time in miliseconds. Need to change in arduino to milisec and read the mils insted of delta time
int state = 0;


unsigned long old_time_input;
unsigned long new_time_input;
int current_input_time = 0;
float input_interval = 0.4;


//walk : towards = 0, backwords = 1, set for every states
// alghorithm for the other side is to traverse the array from the end
uint8_t tripodWalk[NUMLEGS] = { 0,1,1,0,0,1 };



// Needed for rotation matrix
float sinRotX;
float sinRotY;
float sinRotZ;

float cosRotX;
float cosRotY;
float cosRotZ;


float BodyIKX = 0;
float BodyIKY = 2;
float BodyIKZ = 0;

float BodyROTX = 0;
float BodyROTY = 0;
float BodyROTZ = 0;



void setup()
{
    // open bluetooth and avrage pc serial
	Serial.begin(9600);
	Serial2.begin(9600);

    int i;

    for(i = 0 ; i < NUMLEGS ; i ++ ){

        Serial.print(i);

        int pinIndex = i * 4 + START_IND;
        legs[i].setPins( pinIndex, pinIndex + 1, pinIndex + 2 , calibration[i]);
        legs[i].offsetPos.y = 5.0f;
        legs[i].legNum = i;
        delay(20);
    }

    // we have 2 sec to lift the robot up
    delay(2000);

    // dirty non dynamic way setting the offset of distance of the body center
    legs[0].offsetPos.x = 6.0f;
    legs[0].offsetPos.z = 3.5f;
    legs[0].offsetAngleY = 60.0f;

    legs[1].offsetPos.x = 6.0f;
    legs[1].offsetPos.z = -3.5f;
    legs[1].offsetAngleY = 120.0f;

    legs[2].offsetPos.x = 0.0f;
    legs[2].offsetPos.z = 6.5f;
    legs[2].offsetAngleY = 0.0f;

    legs[3].offsetPos.x = 0.0f;
    legs[3].offsetPos.z = -6.5f;
    legs[3].offsetAngleY = 180.0f;

    legs[4].offsetPos.x = -6.0f;
    legs[4].offsetPos.z = 3.5f;
    legs[4].offsetAngleY = 300.0f; //-64

    legs[5].offsetPos.x = -6.0f;
    legs[5].offsetPos.z = -3.5f;
    legs[5].offsetAngleY = 240.0f; //-117

}

//*
// Leg Inverse Kinematics
//======================================================================================
// Calculates the angles of the coxa, femur and tibia for the given position of the feet
// ikFeetPosX            - Input position of the Feet X
// ikFeetPosY            - Input position of the Feet Y
// ikFeetPosZ            - Input Position of the Feet Z
// femurAngle            - Output Angle of Femur in degrees
// tibiaAngle            - Output Angle of Tibia in degrees
// coxaAngle             - Output Angle of Coxa in degrees
//======================================================================================

	void LegIK ( float ikFeetPosX, float ikFeetPosY, float ikFeetPosZ, HexaLeg leg){

		float L;                       		// Length between Femur and Tibia
		float A1;           // Angle of the line Femur and Tibia with respect to the ground in radians
		float A2;           // Angle of the line Femur and Tibia with respect to the femur in radians
		float B;            // Angle of tibia in radians
		float distance;     // Distance between the Coxa and Ground Contact

		int coxaAngle;
		int femurAngle;
		int tibiaAngle;

		distance = sqrt( ikFeetPosX * ikFeetPosX + ikFeetPosZ * ikFeetPosZ );

		L = sqrt( (distance - COXA_LENGTH ) * (distance - COXA_LENGTH ) + ikFeetPosY * ikFeetPosY );

		A1 = facos( ikFeetPosY / L);
		A2 = facos( (L * L + FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) / ( 2 * L * FEMUR_LENGTH ) ) ;

        float femurDebug = ( A1 + A2 ) * Rad2Deg  ;
        float tibiaDebug = ( facos ( ( FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - L * L ) / ( 2 * FEMUR_LENGTH * TIBIA_LENGTH ) ) ) * Rad2Deg ;

        femurAngle = ( A1 + A2 ) * Rad2Deg  ;

		tibiaAngle = ( facos ( ( FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - L * L ) / ( 2 * FEMUR_LENGTH * TIBIA_LENGTH ) ) ) * Rad2Deg ;

		coxaAngle = 90 + atan2(ikFeetPosX, ikFeetPosZ  ) * Rad2Deg;

        coxaAngle -= leg.offsetAngleY;
        if(coxaAngle < 0 ) coxaAngle = 360 + coxaAngle;



        /*/DEBUG
        Serial.println("=====>"); Serial.print("leg num: "); Serial.println(leg.legNum);
        Serial.print("leg num: ");
        Serial.println(leg.legNum);
        Serial.print("coxaAngle int: ");
		Serial.println(coxaAngle);


        Serial.print("posx: ");
        Serial.println(ikFeetPosX);  //print ot serial monitor to see results
        Serial.print("posy: ");
        Serial.println(ikFeetPosY);
        Serial.print("posz: ");
        Serial.println(ikFeetPosZ);


        Serial.print("A1: ");
        Serial.println(A1);  //print ot serial monitor to see results
        Serial.print("A2: ");
        Serial.println(A2);
        Serial.print("L: ");
        Serial.println(L);
        Serial.print("tibiaDebug: ");
        Serial.println(tibiaDebug);
        Serial.print("femurDebug: ");
        Serial.println(femurDebug);

        Serial.print("coxaAngle int: ");
		Serial.println(coxaAngle);
		Serial.print("femurAngle int: ");
        Serial.println(femurAngle);
        Serial.print("tibiaAngle int: ");
        Serial.println(tibiaAngle);
        //*/

        leg.setAngles(coxaAngle, 180 - femurAngle, tibiaAngle);
	}

	//Rotation matrix of every axis combined into one
	BasicVector3 calculateRotations( BasicVector3 pos ){

		float cosA = fcos( BodyROTX ) ;
		float cosB = fcos( BodyROTY ) ;
		float cosC = fcos( BodyROTZ ) ;

		float sinA = fsin( BodyROTX ) ;
		float sinB = fsin( BodyROTY ) ;
		float sinC = fsin( BodyROTZ ) ;

		float posX = pos.x * cosA * cosB - pos.z * cosB * sinC + pos.y * sinB;
		float posY = pos.x * ( sinA * sinB * cosC + cosA * sinC ) + pos.y * ( sinA * sinB * sinC + cosA * cosC) - pos.z * sinA * cosB;
		float posZ = pos.x * ( sinA * sinC - cosA * sinB * cosC) + pos.y * ( cosA * sinB * sinC + sinA * cosC) + pos.z * cosA * cosB;

        BasicVector3 result;
        result.x = posX;
        result.y = posY;
        result.z = posZ;

		return result;
	}


void walkTrippleGait(){

        new_time = millis();

        currentCycleTime += new_time - old_time;

        float currentCycleTimeSec = (float)currentCycleTime / 1000;


		// TODO: make a legs reset function
		// TODO: move the vars outside the function
        BasicVector3 inputPos;
        inputPos.x = inputPos.y = inputPos.z = 0;

        // TODO: Chaneg line below to gain accuracy // DONE
		float lifLeg = 6 * currentCycleTimeSec;//currentCycleTime > cycleTime/2 ? -3 : 0;

		//Serial.println("lifLeg: "); Serial.print(lifLeg);

		float walkUnitsX = BodyIKX * 2 / cycleTime;
		float walkUnitsY = BodyIKY / cycleTime;
		float walkUnitsZ = BodyIKZ * 2 / cycleTime;

        BasicVector3 towardMotion, backMotion;

        towardMotion.x = -BodyIKX + ( walkUnitsX * currentCycleTimeSec );
        towardMotion.y = walkUnitsY - lifLeg;
        towardMotion.z = -BodyIKZ + ( walkUnitsZ * currentCycleTimeSec);

        backMotion.x =  BodyIKX + ( - walkUnitsX * currentCycleTimeSec );
        backMotion.y = BodyIKY;
        backMotion.z = BodyIKZ + ( -walkUnitsZ * currentCycleTimeSec );


        if(state == 0){

            for(int i = 0 ; i < NUMLEGS ; i ++ ){
                    //Serial.print("numLeg normal: "); Serial.println(i);


                if( tripodWalk[i] == 0){
                    BasicVector3 bodyPos;

                    bodyPos.x = towardMotion.x + legs[i].offsetPos.x;
                    bodyPos.y = towardMotion.y + legs[i].offsetPos.y;
                    bodyPos.z = towardMotion.z  + legs[i].offsetPos.z;

                    BasicVector3 inputPos  = calculateRotations(bodyPos);
                    LegIK(inputPos.x, inputPos.y, inputPos.z, legs[i]);
                }

                if( tripodWalk[i] == 1){
                    BasicVector3 bodyPos;

                    bodyPos.x = backMotion.x + legs[i].offsetPos.x;
                    bodyPos.y = backMotion.y + legs[i].offsetPos.y;
                    bodyPos.z = backMotion.z  + legs[i].offsetPos.z;

                    BasicVector3 inputPos  = calculateRotations(bodyPos);
                    LegIK(inputPos.x, inputPos.y, inputPos.z, legs[i]);
                }

            }
        }
        else if(state == 1){

            uint8_t indexGait = NUMLEGS -1;



            for(int i = 0 ; i < NUMLEGS ; i ++ ){

                //Serial.print("indexLegs: "); Serial.println(indexLegs -i);

                if( tripodWalk[indexGait - i] == 0){
                    BasicVector3 bodyPos;

                    bodyPos.x = towardMotion.x + legs[i].offsetPos.x;
                    bodyPos.y = towardMotion.y + legs[i].offsetPos.y;
                    bodyPos.z = towardMotion.z  + legs[i].offsetPos.z;

                    BasicVector3 inputPos  = calculateRotations(bodyPos);
                    LegIK(inputPos.x, inputPos.y, inputPos.z, legs[i]);
                }

                if( tripodWalk[indexGait - i] == 1){
                    BasicVector3 bodyPos;

                    bodyPos.x = backMotion.x + legs[i].offsetPos.x;
                    bodyPos.y = backMotion.y + legs[i].offsetPos.y;
                    bodyPos.z = backMotion.z  + legs[i].offsetPos.z;

                    BasicVector3 inputPos  = calculateRotations(bodyPos);
                    LegIK(inputPos.x, inputPos.y, inputPos.z, legs[i]);
                }

            }
        }



        if( currentCycleTimeSec > cycleTime ){

            //Serial.print("state: "); Serial.println(state);
            //Serial.println("-----");

			currentCycleTime = 0;
			//state = state == 0 ? 1 : 0;
            if(state == 0) state = 1;
            else state = 0;
		}

		isMoving = true;


		old_time = new_time;

}






void loop()
{


    new_time_input = millis();
    current_input_time += new_time_input - old_time_input;

    float current_input_time_sec = (float)current_input_time / 1000;

    if(current_input_time_sec > 2){
        BodyIKX = 0;
        BodyIKZ = 0;
    }

    while (Serial2.available()) {
        delay(10);

        if (Serial2.available() >0) {
            char c = Serial2.read();  //gets one byte from serial buffer

            current_input_time = 0;

             Serial.print("I received: ");
             Serial.println(c);


             // 1 - up, 2 - down, 3 - left, 4 - right, 5 - x, 7 - sqare, 6 - circle, 8 - triangle, 9 - SELECT, A = START

            //BodyIKX = 0;
            //BodyIKZ = 0;


            if(c == '1'){ //up
                BodyIKX = 2;
            }
            if(c == '2'){ // down
                BodyIKX = -2;
            }
            if(c == '3'){
                BodyIKZ = 2;
            }
            if(c == '4'){
                BodyIKZ = -2;
            }

            // special treatment for Y
            if(c == '9'){ //SELECT
                BodyIKY += 1;
            }
            if(c == 'A'){ // START
                BodyIKY -= 1;
            }
            // end y

            if(c == '7'){ // Square
                BodyROTX += 5;
            }
            if(c == '6'){ // Circle
                BodyROTX -= 5;
            }
            if(c == '8'){ // Triangle
                BodyROTY += 5;
            }
            if(c == '5'){ // X
                BodyROTY -= 5;
            }
        }
    }

    isMoving = false;

    if( BodyIKX >= 1 || BodyIKZ >= 1 || BodyIKX <= -1 || BodyIKZ <= -1 ){

        walkTrippleGait();
    }

    if(isMoving) return;

    // Normal update
    for(int i = 0 ; i < NUMLEGS ; i ++ ){

        BasicVector3 bodyPos;
        bodyPos.x = BodyIKX + legs[i].offsetPos.x;
        bodyPos.y = BodyIKY + legs[i].offsetPos.y;
        bodyPos.z = BodyIKZ + legs[i].offsetPos.z;

        BasicVector3 pos = calculateRotations(bodyPos);
        //
        LegIK(pos.x, pos.y, pos.z, legs[i]);

    }

    old_time_input = new_time_input;
}



