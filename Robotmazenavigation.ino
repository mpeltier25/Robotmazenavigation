
#include <math.h>
#include <Ping.h>
#include "Arduino.h"
#include <DCMotor.h>
#include <Servo.h> 
// 4 1, 4 2
#define LIGHT_MULT .4
#define WALL_MULT 10
#define PING_RANGE 40
#define CRITICAL_RANGE 15
#define ROT_DELAY 170
#define MAX_LIGHT 2.0
#define GOAL_LIGHT 985

int rot = 30;

DCMotor motor_R(M0_EN, M0_D0, M0_D1);
DCMotor motor_L(M1_EN, M1_D0, M1_D1);

// Vector lookup:
float vecx[13];
float vecy[13];

char state = 88;
int buttonState = 0;

PingSensor ping(A0);

Servo myservo;  // create servo object to control a servo 

int pos = 180;    // variable to store the servo position 
int inc = 0;
float srange = 0;
int timestep = 0;
short timer1 = 0;
boolean rotate = true;

int target_x;
int target_y;
int ambient_l = 0;

void setup()
{
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  
  pinMode(SPEAKER, OUTPUT);
  myservo.attach(A5);  // attaches the servo on pin 9 to the servo object 
  
  vecx[0] = 0;
  vecy[0] = 1;
  vecx[1] = -.5;
  vecy[1] = 0.866025404;
  vecx[2] = -0.866025404;
  vecy[2] = .5;
  vecx[3] = -1;
  vecy[3] = 0;

  vecx[4] = -0.866025404;
  vecy[4] = -.5;
  vecx[5] = -.5;
  vecy[5] = -0.866025404;
  vecx[6] = 0;
  vecy[6] = -1;

  vecx[7] = .5;
  vecy[7] = -0.866025404;
  vecx[8] = 0.866025404;
  vecy[8] = -.5;
  vecx[9] = 1;
  vecy[9] = 0;
  
  vecx[10] = 0.866025404;
  vecy[10] = .5;
  vecx[11] = .5;
  vecy[11] = 0.866025404;
  vecx[12] = 1;
  vecy[12] = 0;
}

int readLights()
{
  int leftLight, rightLight;
  boolean found = false;
  
  // Check the light sensors, figure out where the light is
  // relative to robot's front
  leftLight = analogRead(A1);
  rightLight = analogRead(A2);

  if (leftLight > ambient_l) {
   target_x += -0.707106781 * (leftLight-ambient_l) * LIGHT_MULT;
   target_y += 0.707106781 * (leftLight-ambient_l) * LIGHT_MULT;
   found = true;
  }
  if (rightLight > ambient_l) {
   target_x += 0.707106781* (rightLight-ambient_l) * LIGHT_MULT;
   target_y += 0.707106781 * (rightLight-ambient_l) * LIGHT_MULT;
   found = true;
  }
  // go forward if no lights found
  if (rightLight > GOAL_LIGHT || leftLight > GOAL_LIGHT) {
    motor_L.brake();
    motor_R.brake();
    state=10;
    while(true);
    target_x = 0;
    target_y = 1;
  }
  // Crude linear estimation
  return leftLight + rightLight; 
}

int Turning(float vx, float vy, boolean post_move){
   float result = atan2(vy, vx) - 1.579;
   if (result < 0)
   	result=6.282+result;
   if (result > 6.282)
      result-=6.282;
   
   if (result > 3.141) {
     result = (6.262-result) * (1700.00/6.282);
     motor_R.setSpeed(-50);
     motor_L.setSpeed(-50); 
   } else {
     result = result * (1700.00/6.282);
     motor_R.setSpeed(50);
     motor_L.setSpeed(50);
   }     
   if (result > 1700)
     result = random(1700);
   if (result > 0)
     delay((int)result);
   motor_R.brake();
   motor_L.brake();
   if (post_move) {
     motor_R.setSpeed(25);
     motor_L.setSpeed(-25);
   }
}

int checkWalls()
{
    // pfield directly backward if too close

    int temp = ping.measureCM();
    if(temp<=PING_RANGE){
  
      // GENERATE PFIELD RELATIVE TO RANGE AT POS + 180
     target_x += vecx[pos/30] * (PING_RANGE - temp) * WALL_MULT;
     target_y += vecy[pos/30] * (PING_RANGE - temp) * WALL_MULT;
      
    }
    if (temp<=CRITICAL_RANGE && pos > 90 && pos < 270) {
       // Strong direct 180 repulsion, move backwards until the next update
       motor_L.brake();
       motor_R.brake(); 
       motor_R.setSpeed(-25);
       motor_L.setSpeed(25);
    }

}

void loop()
{
  int i, j, k; // Iterators
    
  switch (state) {
    case 88:
      state = 0;
    break;
    case 0:
    
    // State 0 calibrates the light sensors
    for(i = 0; i < 25; i++) {
       delay(100);
       ambient_l = (ambient_l + (analogRead(A1)+analogRead(A2))*.5)*.5;
    }
    ambient_l*=.50; // Extra margin of error
    state = 1; 
    target_x = 0;
    target_y = 0;
    break;
    case 1:
      
      srange = pos/(2.5-(pos*.0022));
      myservo.write(srange);
      delay(ROT_DELAY);

      checkWalls();
      readLights();

      pos += rot; 
       if (pos >= 270) {
         rot = -rot;
         state = 99;
         Turning(target_x, target_y, true);
         target_x = 0;
         target_y = 0;

       }
       if (pos <= 90) {
         rot = -rot;
         state = 99;
         Turning(target_x, target_y, true);
         target_x = 0;
         target_y = 0;
       }     
    break;
    case 99:
     state = 1;
    break;

  }
}
