#include <AccelStepper.h>

// Sensor input pins
#define SENSOR_FL       A0
#define SENSOR_L        A1
#define SENSOR_M        A2
#define SENSOR_R        A3
#define SENSOR_FR       A4

#define SENSOR_THRESHOLD 600
#define JUNCTION_THRESHOLD 10
#define ACCELERATION 600.0
#define UNIT 194.5
#define TURN_90_UNIT 85.0
#define DISTANCE_TO_JUNCTION 120.0

// stepper motors
AccelStepper stepperLeft (AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 6, 7, 8, 9);

// functions
void sensors();
int getSensorStatus();

int increaseDirection(int dir);
int decreaseDirection(int dir);

// sensors
boolean sensorFarLeft  = false;
boolean sensorMiddle   = false;
boolean sensorLeft     = false;
boolean sensorRight    = false;
boolean sensorFarRight = false;

// maze
// first four elements of array are the sides of junction and last one is the type of junction
// side => 0 -> unknown, 1 -> road, 2 -> wall 
// type => 0 -> junction(unknown), 1-> start, 2 -> finish
short int maze[10][10][5];
 
// car
int mode      = 0; // 0 -> standby, 1 -> explore, 2 -> shortest path
int state     = 0; // 1 -> forward (direction correction), 2 -> turning
int direction = 0; // 0 -> north, 1 -> east, 2 -> south, 3 -> west
int dirToGo   = 0; // 0 -> nowhere, 1 -> left, 2 -> forward, 3 -> right, 4 -> backward
int xPos      = 0; // current x position
int yPos      = 0; // current y position

// 0 -> left, 1 -> middle, 2 -> right 
int nextJunctionData[3] = { 0 };

// direction correction
int correctionSpeed[2]     = { 200, 200 }; // {left, right}
int correctionSpeeds[9][2] = {
  { correctionSpeed[0], correctionSpeed[1] - 20 }
  { correctionSpeed[0], correctionSpeed[1] - 15 },
  { correctionSpeed[0], correctionSpeed[1] - 10 },
  { correctionSpeed[0], correctionSpeed[1] - 5  },
  { correctionSpeed[0], correctionSpeed[1] },
  { correctionSpeed[0] - 5,  correctionSpeed[1] },
  { correctionSpeed[0] - 10, correctionSpeed[1] },
  { correctionSpeed[0] - 15, correctionSpeed[1] },
  { correctionSpeed[0] - 20, correctionSpeed[1] },
};

void setup() {

  // initialize maze as unknown
  for(int i=0; i<10; i++){
    for(int j=0; j<10; j++){
      for(int k=0; k<5; k++){
        maze[i][j][k] = 0;
      }
    }
  }

  // initilize maze[0][0]
  maze[xPos][yPos][0] = 1; // road
  maze[xPos][yPos][1] = 2; // wall
  maze[xPos][yPos][2] = 2; // wall
  maze[xPos][yPos][3] = 2; // wall
  maze[xPos][yPos][4] = 1; // type -> start

  
  stepperLeft.setMaxSpeed(correctionSpeed[0]);
  stepperLeft.setAcceleration(ACCELERATION);
  stepperLeft.moveTo(UNIT);

  stepperRight.setMaxSpeed(correctionSpeed[1]);
  stepperRight.setAcceleration(ACCELERATION);
  stepperRight.moveTo(-UNIT);

  Serial.begin(9600);

  delay(1000);
}

bool turning = true;

void loop() {

  // update sensor values
  sensors();
  
  int sensorStatus = getSensorStatus();
  
//  Serial.print("sensorStatus:");
//  Serial.println(sensorStatus);

  if(mode == 0){
    // stand-by

    Serial.println("Preparing to move...");
    delay(3000);
    mode = 1;
    
  }else if(mode == 1){
    // explore
    
    if(state == 0){
      // forward

      // direction correction
      stepperLeft.setMaxSpeed (sensorStatus < 9 ? correctionSpeeds[sensorStatus][0] : correctionSpeed[0]);
      stepperRight.setMaxSpeed(sensorStatus < 9 ? correctionSpeeds[sensorStatus][1] : correctionSpeed[1]);

      if(stepperLeft.distanceToGo()  < DISTANCE_TO_JUNCTION && stepperLeft.distanceToGo()  > 0 &&
         stepperRight.distanceToGo() < DISTANCE_TO_JUNCTION && stepperRight.distanceToGo() > 0){
        // start counting total sensor data until middle of next junction
      
        if(sensorFarLeft)
          nextJunctionData[0]++;
          
        if(sensorMiddle)
          nextJunctionData[1]++;
          
        if(sensorFarRight)
          nextJunctionData[2]++;

        Serial.print("left: ");
        Serial.print(nextJunctionData[0]);
        Serial.print(" - middle: ");
        Serial.print(nextJunctionData[1]);
        Serial.print(" - right: ");
        Serial.println(nextJunctionData[2]);
        
      }else if(stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0){
        // until middle of next junction

        updatePosition(); // update xPos, yPos according to direction

        // update current junction on maze
        maze[xPos][yPos][decreaseDirection(direction)] = nextJunctionData[0] > JUNCTION_THRESHOLD   ? 1 : 2; // left
        maze[xPos][yPos][direction]                    = nextJunctionData[1] > 3*JUNCTION_THRESHOLD ? 1 : 2; // forward
        maze[xPos][yPos][increaseDirection(direction)] = nextJunctionData[2] > JUNCTION_THRESHOLD   ? 1 : 2; // right
        maze[xPos][yPos][decreaseDirection(decreaseDirection(direction))] = 1;                               // backward (must be road)
        
        // find where to turn
        if(maze[xPos][yPos][decreaseDirection(direction)] == 1){
          dirToGo = 1;
        }else if(maze[xPos][yPos][direction] == 1){
          dirToGo = 2;
        }else if(maze[xPos][yPos][increaseDirection(direction)] == 1){
          dirToGo = 3;
        }else{ // if(maze[xPos][yPos][increaseDirection(increaseDirection(direction))] == 1) -> since backward is road
          dirToGo = 4;
        }

        // TODO: finish case
        
        state = 1;
      }
      
    }else if(state == 1){
      // turning

      if(dirToGo == 1){
        // turn left

        direction = decreaseDirection(direction);
        
        stepperLeft.move (-1 * TURN_90_UNIT);
        stepperRight.move(-1 * TURN_90_UNIT);
        
      }else if(dirToGo == 2){
        // go forward - nothing to do since its directed to forward
      }else if(dirToGo == 3){
        // turn right

        direction = increaseDirection(direction);
        
        stepperLeft.move (TURN_90_UNIT);
        stepperRight.move(TURN_90_UNIT);
        
      }else if(dirToGo == 4){
        // turn backward

        direction = increaseDirection(increaseDirection(direction)); // 
        
        stepperLeft.move (2 * TURN_90_UNIT);
        stepperRight.move(2 * TURN_90_UNIT);
        
      }

      dirToGo = 0;
      
      if(!stepperLeft.isRunning() && !stepperRight.isRunning()){
        // after turn move to the next junction
        
        nextJunctionData = { 0, 0, 0 }; // there might be an error
        state = 0;
        
        stepperLeft.move (UNIT);
        stepperRight.move(-UNIT);
      }
      
    }
    
  }else if(mode == 2){
    // solve
    
  }

  stepperLeft.run();
  stepperRight.run();
}



void sensors() {

  // print out the value you read:
  sensorFarRight = analogRead(SENSOR_FR) < SENSOR_THRESHOLD;
  sensorRight    = analogRead(SENSOR_R)  < SENSOR_THRESHOLD;
  sensorMiddle   = analogRead(SENSOR_M)  < SENSOR_THRESHOLD;
  sensorLeft     = analogRead(SENSOR_L)  < SENSOR_THRESHOLD;
  sensorFarLeft  = analogRead(SENSOR_FL) < SENSOR_THRESHOLD;

//  Serial.print("FL:");
//  Serial.print(sensorFarLeft);
//  Serial.print(" - L:");
//  Serial.print(sensorLeft);
//  Serial.print(" - M:");
//  Serial.print(sensorMiddle);
//  Serial.print(" - R:");
//  Serial.print(sensorRight);
//  Serial.print(" - FR:");
//  Serial.println(sensorFarRight);

}

int getSensorStatus() {
  if(sensorFarLeft && !sensorLeft && !sensorMiddle && !sensorRight && !sensorFarRight){
    return 0;
  }else if(sensorFarLeft && sensorLeft && !sensorMiddle && !sensorRight && !sensorFarRight){
    return 1;
  }else if(!sensorFarLeft && sensorLeft && !sensorMiddle && !sensorRight && !sensorFarRight){
    return 2;
  }else if(!sensorFarLeft && sensorLeft && sensorMiddle && !sensorRight && !sensorFarRight){
    return 3;
  }else if(!sensorFarLeft && !sensorLeft && sensorMiddle && !sensorRight && !sensorFarRight){
    return 4;
  }else if(!sensorFarLeft && !sensorLeft && sensorMiddle && sensorRight && !sensorFarRight){
    return 5;
  }else if(!sensorFarLeft && !sensorLeft && !sensorMiddle && sensorRight && !sensorFarRight){
    return 6;
  }else if(!sensorFarLeft && !sensorLeft && !sensorMiddle && sensorRight && sensorFarRight){
    return 7;
  }else if(!sensorFarLeft && !sensorLeft && !sensorMiddle && !sensorRight && sensorFarRight){
    return 8;
  }else if(!sensorFarLeft && !sensorLeft && !sensorMiddle && !sensorRight && !sensorFarRight){
    return 9;
  }else if(sensorFarLeft && sensorMiddle && sensorFarRight){
    return 10;
  }else if(sensorFarLeft && sensorMiddle){
    return 11;
  }else if(sensorMiddle && sensorFarRight){
    return 12;
  }else{
    return 13;
  }
}

void updatePosition(){
  if(direction == 0){
    yPos++;
  }else if(direction == 1){
    xPos++;
  }else if(direction == 2){
    yPos--;
  }else if(direction == 3){
    xPos--;
  }
}

int increaseDirection(int dir){
  int tDir = dir + 1;
  if(tDir == 4) tDir = 0;
  return tDir;
}

int decreaseDirection(int dir){
  int tDir = dir - 1;
  if(tDir == -1) tDir = 3;
  return tDir;
}
