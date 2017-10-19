#include <AccelStepper.h>
#include <StackArray.h>

// Sensor input pins
#define SENSOR_FL       A0
#define SENSOR_L        A1
#define SENSOR_M        A2
#define SENSOR_R        A3
#define SENSOR_FR       A4

#define SENSOR_THRESHOLD    750
#define JUNCTION_THRESHOLD  75
#define FINISH_THRESHOLD    500
#define ACCELERATION        450.0
#define UNIT                195.0
#define INITIAL_UNIT        250.0
#define TURN_90_UNIT        85.0
#define DISTANCE_FOR_WHEELS 80.0

// stepper motors
AccelStepper stepperLeft (AccelStepper::FULL4WIRE, 48, 46, 44, 42);
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 40, 38, 36, 34);

// functions
void sensors();
short int getSensorStatus();
void updatePosition();
short int increaseDirection(short int dir);
short int decreaseDirection(short int dir);

// sensors
boolean lastSensorValues[5][10] = { false };
boolean sensorFarLeft  = false;
boolean sensorMiddle   = false;
boolean sensorLeft     = false;
boolean sensorRight    = false;
boolean sensorFarRight = false;

// maze
// first four elements of array are the sides of junction and last one is the type of junction
// side => 0 -> road (unknown), 1 -> road (visited), 2 -> road (deadend), 3 -> wall 
// type => 0 -> junction(unknown), 1-> start, 2 -> finish
short int maze[10][10][4];
 
// car
short int mode      = 0; // 0 -> standby, 1 -> explore, 2 -> shortest path
short int state     = 0; // 1 -> forward (direction correction), 2 -> turning
short int direction = 0; // 0 -> north, 1 -> east, 2 -> south, 3 -> west
short int dirToGo   = 0; // 0 -> nowhere, 1 -> left, 2 -> forward, 3 -> right, 4 -> backward
short int xPos      = 0; // current x position
short int yPos      = 0; // current y position
short int xPosFinish = -1;
short int yPosFinish = -1;


// 0 -> left, 1 -> middle, 2 -> right 
short int junctionPassed      = 0;
short int nextJunctionData[3] = { 0 };

// direction correction
int correctionSpeed[2]     = { 200, 200 }; // {left, right}
int correctionSpeeds[9][2] = {
  { correctionSpeed[0] - 60, correctionSpeed[1] + 60 },
  { correctionSpeed[0] - 45, correctionSpeed[1] + 45 },
  { correctionSpeed[0] - 30, correctionSpeed[1] + 30 },
  { correctionSpeed[0] - 15, correctionSpeed[1] + 15 },
  { correctionSpeed[0], correctionSpeed[1] },
  { correctionSpeed[0] + 15, correctionSpeed[1] - 15 },
  { correctionSpeed[0] + 30, correctionSpeed[1] - 30 },
  { correctionSpeed[0] + 45, correctionSpeed[1] - 45 },
  { correctionSpeed[0] + 60, correctionSpeed[1] - 60 }
};

// required variables for dijktra's algorithm
struct JunctionStackItem{
  int xPos;
  int yPos;
  int lastDir;
};
StackArray <JunctionStackItem> stack;
short int mazeDistances[10][10];
short int distance = 0;
String reverseSolution = "";
String solution = "";
short int nextXPos = 0;
short int nextYPos = 0;

void setup() {
  
  // initialize maze as unknown
  for(int i=0; i<10; i++){
    for(int j=0; j<10; j++){
      for(int k=0; k<4; k++){
        maze[i][j][k] = 0;
      }
    }
  }

  // initilize maze[0][0]
  maze[xPos][yPos][0] = 1; // road
  maze[xPos][yPos][1] = 3; // wall
  maze[xPos][yPos][2] = 3; // wall
  maze[xPos][yPos][3] = 3; // wall

//  Serial.begin(9600);
  delay(1000);
  
  stepperLeft.setMaxSpeed(correctionSpeed[0]);
  stepperLeft.setAcceleration(ACCELERATION);
  stepperLeft.moveTo(INITIAL_UNIT);

  stepperRight.setMaxSpeed(correctionSpeed[1]);
  stepperRight.setAcceleration(ACCELERATION);
  stepperRight.moveTo(-INITIAL_UNIT);
  
}

void loop() {

  // update sensor values
  sensors();
  
  int sensorStatus = getSensorStatus();
  
  if(mode == 0){
    // stand-by for explorer mode

    delay(3000);
//    Serial.println("=== stand-by ===");
//    Serial.println(sensorStatus);
    mode = 1;
    
  }else if(mode == 1){
    // explore
    
    if(state == 0){
      // forward

      // direction correction
      stepperLeft.setMaxSpeed (sensorStatus < 9 ? correctionSpeeds[sensorStatus][0] : correctionSpeed[0]);
      stepperRight.setMaxSpeed(sensorStatus < 9 ? correctionSpeeds[sensorStatus][1] : correctionSpeed[1]);

      // collecting junction data
      if(sensorFarLeft)
        nextJunctionData[0]++;
        
      if(sensorMiddle)
        nextJunctionData[1]++;
        
      if(sensorFarRight)
        nextJunctionData[2]++;

      // Calibration - no calibration for straight forward junction
      if(sensorStatus > 9 && nextJunctionData[1] > JUNCTION_THRESHOLD && junctionPassed != 2){

//        if(junctionPassed == 0)
//          Serial.println(sensorStatus);
        
        junctionPassed = 1;

        if(nextJunctionData[0] > FINISH_THRESHOLD && nextJunctionData[2] > FINISH_THRESHOLD){
          // finish case
          
          stepperLeft.setSpeed (0); // stop engines
          stepperRight.setSpeed(0);
          
          updatePosition(); // update xPos, yPos according to direction

          if(xPos == 0 && yPos == 0){ // start

            for(int i=0; i<10; i++){
              for(int j=0; j<10; j++){
                mazeDistances[i][j] = 999;
              }
            }
            direction = 0;

            state = 2; // start solving maze

          }else{ // finish
            
            // initialize finish junction
            maze[xPos][yPos][decreaseDirection(direction)] = 3;                    // wall
            maze[xPos][yPos][direction] = 3;                                       // wall
            maze[xPos][yPos][increaseDirection(direction)] = 3;                    // wall
            maze[xPos][yPos][increaseDirection(increaseDirection(direction))] = 1; // road
  
            // turn back and return discovering the maze
            xPosFinish = xPos;
            yPosFinish = yPos;
            dirToGo = 4;
            state = 1;
            delay(3000); // competition rules!
          }
        }
      }

      if(((junctionPassed < 2 && sensorStatus == 9) || (junctionPassed == 1 && sensorStatus < 9)) && nextJunctionData[1] > JUNCTION_THRESHOLD){

//        String debug = "\n" + String(nextJunctionData[0]) + " - " + String(nextJunctionData[1]) + " - " + String(nextJunctionData[2]) + " s:" + String(sensorStatus);
//        Serial.println(debug);
        
        junctionPassed = 2;
        nextJunctionData[1] = 0;

        stepperLeft.move ( DISTANCE_FOR_WHEELS);
        stepperRight.move(-DISTANCE_FOR_WHEELS);
        
      }

      // Preparation of turn state
      if(stepperLeft.distanceToGo() == 0 || -stepperRight.distanceToGo() == 0){
        // when middle of next junction
//        Serial.println(F("on junction"));
//        String debug = "\n" + String(nextJunctionData[0]) + " - " + String(nextJunctionData[1]) + " - " + String(nextJunctionData[2]) + " s:" + String(sensorStatus);
//        Serial.println(debug);


        updatePosition(); // update xPos, yPos according to direction

        // update current junction on maze
        if(nextJunctionData[0] < JUNCTION_THRESHOLD)
          maze[xPos][yPos][decreaseDirection(direction)] = 3;

        if(nextJunctionData[1] < JUNCTION_THRESHOLD)
          maze[xPos][yPos][direction] = 3;

        if(nextJunctionData[2] < JUNCTION_THRESHOLD)
          maze[xPos][yPos][increaseDirection(direction)] = 3;    

        // find where to turn
        // first look for 0's if there aren't any then 1's
        if(maze[xPos][yPos][decreaseDirection(direction)] == 0){
          dirToGo = 1;
        }else if(maze[xPos][yPos][direction] == 0){
          dirToGo = 2;
        }else if(maze[xPos][yPos][increaseDirection(direction)] == 0){
          dirToGo = 3;
        }else if(maze[xPos][yPos][increaseDirection(increaseDirection(direction))] == 0){ // -> since backward is road
          dirToGo = 4;
        }else if(maze[xPos][yPos][decreaseDirection(direction)] == 1){
          dirToGo = 1;
        }else if(maze[xPos][yPos][direction] == 1){
          dirToGo = 2;
        }else if(maze[xPos][yPos][increaseDirection(direction)] == 1){
          dirToGo = 3;
        }else if(maze[xPos][yPos][increaseDirection(increaseDirection(direction))] == 1){ // -> since backward is visited road
          dirToGo = 4;
        }
        
        maze[xPos][yPos][increaseDirection(increaseDirection(direction))]++;
        state = 1;
        delay(100);
      }
      
    }else if(state == 1){
      // turning

      stepperLeft.setMaxSpeed (correctionSpeed[0]);
      stepperRight.setMaxSpeed(correctionSpeed[1]);
      
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

        direction = increaseDirection(increaseDirection(direction));
        
        stepperLeft.move (2 * TURN_90_UNIT);
        stepperRight.move(2 * TURN_90_UNIT);
        
      }

      dirToGo = 0;
      
      if(!stepperLeft.isRunning() && !stepperRight.isRunning()){
        // after turn move to the next junction
        
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        state = 0;
        maze[xPos][yPos][direction]++;
        junctionPassed = 0;
        delay(100);
        
        stepperLeft.move (UNIT);
        stepperRight.move(-UNIT);
      }
      
    }
    
  }else if(mode == 2){
    // stand-by and find shortest path
    
    // stop engines
    stepperLeft.stop();
    stepperRight.stop();
    
    while(true){
      
      if(distance < mazeDistances[xPos][yPos]){
        // junction should be improved
        
        mazeDistances[xPos][yPos] = distance;
  
        // find number of possible roads
        short int possibleRoads = 0;
        for(int i=3; i > 0; i--)
          if(maze[xPos][yPos][i] != 3 && i != increaseDirection(increaseDirection(direction)))
            possibleRoads++;
  
        if(possibleRoads == 0){
          // pop stack - if stack has no item then break loop and complete algorithm
          if(stack.isEmpty())
            break;
          
          JunctionStackItem item = stack.pop();
          
          // update coords.
          xPos = item.xPos;
          yPos = item.yPos;
  
          // update direction and increment 1
          direction = item.lastDir + 1;

          // update distance
          distance = mazeDistances[xPos][yPos];
          
          // find number of remained possible roads
          short int remainedPossibleRoads = 0;
          for(int i=direction+1; i < 4; i++)
            if(maze[xPos][yPos][i] != 3)
              remainedPossibleRoads++;
  
          if(remainedPossibleRoads > 0){
            // push stack with updated direction.

            item.lastDir = direction;
            stack.push(item);
          }
          
        }else{
          
          // findDirToGo
          for(int i=3; i > 0; i--)
            if(maze[xPos][yPos][i] != 3 && i != increaseDirection(increaseDirection(direction)))
              direction = i;
    
          if(possibleRoads > 1){
            // push stack

            JunctionStackItem item;

            item.xPos = xPos;
            item.yPos = yPos;
            item.lastDir = direction;

            stack.push(item);
          }
          
        }
        
      }else{
        // pop stack - if stack has no item then break loop and complete algorithm
        if(stack.isEmpty())
          break;
        
        JunctionStackItem item = stack.pop();
        
        // update coords.
        xPos = item.xPos;
        yPos = item.yPos;

        // update direction and increment 1
        direction = item.lastDir + 1;

        // update distance
        distance = mazeDistances[xPos][yPos];
        
        // find number of remained possible roads
        short int remainedPossibleRoads = 0;
        for(int i=direction+1; i < 4; i++)
          if(maze[xPos][yPos][i] != 3)
            remainedPossibleRoads++;
  
        if(remainedPossibleRoads > 0){
          // push stack with updated direction.

          item.lastDir = direction;
          stack.push(item);
        }
      }
  
      distance++;
  
      // move    
      updatePosition(); // update xPos, yPos according to direction
  
    }

    // maze to solution string

    xPos = xPosFinish;
    yPos = yPosFinish;
    nextXPos = xPosFinish;
    nextYPos = yPosFinish;

    distance = mazeDistances[xPos][yPos];
    direction = 0;

    for(int i=0; i<4; i++)
      if(maze[xPos][yPos][i] != 3)
        direction = i;

    reverseSolution = "F";

    updatePosition();

    
    while(xPos != 0 && yPos != 0){

      if(maze[xPos][yPos][decreaseDirection(direction)] != 3){

        updateNextPosition(decreaseDirection(direction));
        
        if(mazeDistances[nextXPos][nextYPos] < distance){
          direction = decreaseDirection(direction);
          reverseSolution += "L";
        }
      }else if(maze[xPos][yPos][direction] != 3){
        
        updateNextPosition(direction);
        
        if(mazeDistances[nextXPos][nextYPos] < distance){
//          direction = direction;
          reverseSolution += "F";
        }
      }else if(maze[xPos][yPos][increaseDirection(direction)] != 3){
        
        updateNextPosition(increaseDirection(direction));
        
        if(mazeDistances[nextXPos][nextYPos] < distance){
          direction = increaseDirection(direction);
          reverseSolution += "R";
        }
      }

      updatePosition();
      
    }

    for(int i=1; i <= reverseSolution.length(); i++){

      char s;

      if(reverseSolution.charAt(reverseSolution.length()-i) == 'R')
        s = 'L';
      else if(reverseSolution.charAt(reverseSolution.length()-i) == 'L')
        s = 'R';
        
      solution += s;
    }

    distance = 0;
    mode = 3;
    
  }else if(mode == 3){
    // solve

    if(state == 0){
      // forward

      // direction correction
      stepperLeft.setMaxSpeed (sensorStatus < 9 ? correctionSpeeds[sensorStatus][0] : correctionSpeed[0]);
      stepperRight.setMaxSpeed(sensorStatus < 9 ? correctionSpeeds[sensorStatus][1] : correctionSpeed[1]);

      // collecting junction data
      if(sensorFarLeft)
        nextJunctionData[0]++;
        
      if(sensorMiddle)
        nextJunctionData[1]++;
        
      if(sensorFarRight)
        nextJunctionData[2]++;

      // Calibration - no calibration for straight forward junction
      if(sensorStatus > 9){
        junctionPassed = 1;

        if(nextJunctionData[0] > FINISH_THRESHOLD && nextJunctionData[2] > FINISH_THRESHOLD){
          // finish case
          
          stepperLeft.setSpeed (0); // stop engines
          stepperRight.setSpeed(0);

          delay(3000);
        }
      }

      if((junctionPassed < 2 && sensorStatus == 9) || (junctionPassed == 1 && sensorStatus < 9)){

        junctionPassed = 2;
        nextJunctionData[1] = 0;

        stepperLeft.move ( DISTANCE_FOR_WHEELS);
        stepperRight.move(-DISTANCE_FOR_WHEELS);
        
      }

      // Preparation of turn state
      if(stepperLeft.distanceToGo() == 0 || -stepperRight.distanceToGo() == 0){
        // when middle of next junction

        // next move
        if(solution.charAt(distance) == 'L'){
          dirToGo = 1;
        }else if(solution.charAt(distance) == 'F'){
          dirToGo = 2;
        }else if(solution.charAt(distance) == 'R'){
          dirToGo = 3;
        }
        
        state = 1;
        delay(100);
      }
      
    }else if(state == 1){
      // turning

      stepperLeft.setMaxSpeed (correctionSpeed[0]);
      stepperRight.setMaxSpeed(correctionSpeed[1]);
      
      if(dirToGo == 1){
        // turn left

        stepperLeft.move (-1 * TURN_90_UNIT);
        stepperRight.move(-1 * TURN_90_UNIT);
        
      }else if(dirToGo == 2){
        
        // go forward - nothing to do since its directed to forward
        
      }else if(dirToGo == 3){
        // turn right

        stepperLeft.move (TURN_90_UNIT);
        stepperRight.move(TURN_90_UNIT);
        
      }

      dirToGo = 0;
      
      if(!stepperLeft.isRunning() && !stepperRight.isRunning()){
        // after turn move to the next junction
        
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        state = 0;
        distance++;
        junctionPassed = 0;
        delay(100);
        
        stepperLeft.move (UNIT);
        stepperRight.move(-UNIT);
      }
      
    }

  }

  stepperLeft.run();
  stepperRight.run();
}



void sensors() {

  for(int i=0; i < 5; i++){
    for(int j=9; j > 0; j--){
      lastSensorValues[i][j] = lastSensorValues[i][j-1];
    }
  }

  lastSensorValues[0][0] = analogRead(SENSOR_FR) < SENSOR_THRESHOLD;
  lastSensorValues[1][0] = analogRead(SENSOR_R)  < SENSOR_THRESHOLD;
  lastSensorValues[2][0] = analogRead(SENSOR_M)  < SENSOR_THRESHOLD;
  lastSensorValues[3][0] = analogRead(SENSOR_L)  < SENSOR_THRESHOLD;
  lastSensorValues[4][0] = analogRead(SENSOR_FL) < SENSOR_THRESHOLD;

  short int average = 0;
  for(int i=0; i < 10; i++)
    average += lastSensorValues[0][i] ? 1 : 0;
  sensorFarRight = average > 5;

  average = 0;
  for(int i=0; i < 10; i++)
    average += lastSensorValues[1][i] ? 1 : 0;
  sensorRight = average > 5;
  
  average = 0;
  for(int i=0; i < 10; i++)
    average += lastSensorValues[2][i] ? 1 : 0;
  sensorMiddle = average > 5;
  
  average = 0;
  for(int i=0; i < 10; i++)
    average += lastSensorValues[3][i] ? 1 : 0;
  sensorLeft = average > 5;
  
  average = 0;
  for(int i=0; i < 10; i++)
    average += lastSensorValues[4][i] ? 1 : 0;
  sensorFarLeft = average > 5;

  
//  Serial.print(F("FL:"));
//  Serial.print(analogRead(SENSOR_FR) < SENSOR_THRESHOLD);
//  Serial.print(F(" - L:"));
//  Serial.print(sensorLeft);
//  Serial.print(F(" - M:"));
//  Serial.print(sensorMiddle);
//  Serial.print(F(" - R:"));
//  Serial.print(sensorRight);
//  Serial.print(F(" - FR:"));
//  Serial.println(sensorFarRight);
}

short int getSensorStatus() {
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
  }else if(sensorFarLeft && sensorMiddle && !sensorFarRight){
    return 11;
  }else if(!sensorFarLeft && sensorMiddle && sensorFarRight){
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
void updateNextPosition(short int dir){
  
  nextXPos = xPos;
  nextYPos = yPos;
  
  if(dir == 0){
    nextYPos++;
  }else if(dir == 1){
    nextXPos++;
  }else if(dir == 2){
    nextYPos--;
  }else if(dir == 3){
    nextXPos--;
  }
}

short int increaseDirection(short int dir){
  short int tDir = dir + 1;
  if(tDir == 4) tDir = 0;
  return tDir;
}

short int decreaseDirection(short int dir){
  short int tDir = dir - 1;
  if(tDir == -1) tDir = 3;
  return tDir;
}
