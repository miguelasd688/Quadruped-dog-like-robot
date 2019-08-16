#include <Servo.h>


///VARIABLES SERVOMOTORES
#define MAX_SERVOS 12
Servo Servos[MAX_SERVOS];

//VARIABLES PARA RECIVIR EL COMANDO
const byte numChars = 32;
char receivedChars[numChars];
int spaceCounter = 0;

boolean newData = false;

int pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulseFR, pulseFL, pulseBR, pulseBL;

//VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMillis = 0;
const long interval = 5;
unsigned long loopTime;
unsigned long previousLooptime;



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{ 
  Serial.begin(115200);
  IMUSetup();

  
  ////////////////////////////////////////////////////
  ///////////////__CALIBRACION SERVOS__///////////////
  ////////////////////////////////////////////////////
  //pines (8,9,10,11,12,13,30,32,34,36,38,40)
  Servos[0].attach(32);//1 
  Servos[1].attach(34);//2
  Servos[2].attach(38);//3
  Servos[3].attach(40);//4
  Servos[4].attach(12);//5
  Servos[5].attach(11);//6
  Servos[6].attach(9);//7
  Servos[7].attach(10);//8
  Servos[8].attach(36);//FR
  Servos[9].attach(30);//FL
  Servos[10].attach(8);//BR
  Servos[11].attach(13);//BL

  Servos[0].writeMicroseconds(1750);
  Servos[1].writeMicroseconds(1860);
  Servos[2].writeMicroseconds(830);
  Servos[3].writeMicroseconds(870);
  Servos[4].writeMicroseconds(1925);
  Servos[5].writeMicroseconds(1920);
  Servos[6].writeMicroseconds(955);
  Servos[7].writeMicroseconds(770);
  Servos[8].writeMicroseconds(975);
  Servos[9].writeMicroseconds(1686);
  Servos[10].writeMicroseconds(940);
  Servos[11].writeMicroseconds(1855);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{ 
  unsigned long currentMillis = millis();
//    
  if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
    previousMillis = currentMillis;
    
/////////cuenta el tiempo que tarda el bucle en ejecutarse
    loopTime = currentMillis - previousLooptime;
//    Serial.print("Time: ");
    Serial.print("<");
    Serial.print(loopTime);
    Serial.print("#");
//    Serial.print("\t");
    previousLooptime=currentMillis;


    readAngles();


    //////////////////////////////RECIVE PULSES AND MOVE SERVOS
    recvWithStartEndMarkers();
//    showNewData();
    moveallServos();
    
 
    }

}
///////////////////////////////////////////////FUNCTIONS


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char spaceMarker = '#';
    
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker && rc != spaceMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else if (rc == spaceMarker ){
              receivedChars[ndx] = '\0';
              if (spaceCounter==0){
                //Serial.println(receivedChars);
                pulse1 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==1){
                //Serial.println(receivedChars);
                pulse2 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==2){
                //Serial.println(receivedChars);
                pulse3 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==3){
                //Serial.println(receivedChars);
                pulse4 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==4){
                //Serial.println(receivedChars);
                pulse5 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==5){
                //Serial.println(receivedChars);
                pulse6 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==6){
                //Serial.println(receivedChars);
                pulse7 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==7){
                //Serial.println(receivedChars);
                pulse8 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==8){
                //Serial.println(receivedChars);
                pulseFR = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==9){
                //Serial.println(receivedChars);
                pulseFL = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==10){
                //Serial.println(receivedChars);
                pulseBR = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==11){
                //Serial.println(receivedChars);
                pulseBL = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                //Serial.println(receivedChars);
                pulseBL = atoi(receivedChars);
                recvInProgress = false;
                ndx = 0;
                spaceCounter=0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("Pulsos: ");
        Serial.print(pulse1);Serial.print(" ");
        Serial.print(pulse2);Serial.print(" ");
        Serial.print(pulse3);Serial.print(" ");
        Serial.print(pulse4);Serial.print(" ");
        Serial.print(pulse5);Serial.print(" ");
        Serial.print(pulse6);Serial.print(" ");
        Serial.print(pulse7);Serial.print(" ");
        Serial.print(pulse8);Serial.print(" ");
        Serial.print(pulseFR);Serial.print(" ");
        Serial.print(pulseFL);Serial.print(" ");
        Serial.print(pulseBR);Serial.print(" ");
        Serial.println(pulseBL);

    }
}

void moveallServos(){
  newData = false;
  Servos[0].writeMicroseconds(pulse1);
  Servos[1].writeMicroseconds(pulse2);
  Servos[2].writeMicroseconds(pulse3);
  Servos[3].writeMicroseconds(pulse4);
  Servos[4].writeMicroseconds(pulse5);
  Servos[5].writeMicroseconds(pulse6);
  Servos[6].writeMicroseconds(pulse7);
  Servos[7].writeMicroseconds(pulse8);
  Servos[8].writeMicroseconds(pulseFR);
  Servos[9].writeMicroseconds(pulseFL);
  Servos[10].writeMicroseconds(pulseBR);
  Servos[11].writeMicroseconds(pulseBL);
//
//  Servos[0].write(ang);
//  Servos[1].write(ang);
//  Servos[2].write(ang);
//  Servos[3].write(ang);
//  Servos[4].write(ang);
//  Servos[5].write(ang);
//  Servos[6].write(ang);
//  Servos[7].write(ang);
//  Servos[8].write(ang);
//  Servos[9].write(ang);
//  Servos[10].write(ang);
//  Servos[11].write(ang);
//  ang++;

}
