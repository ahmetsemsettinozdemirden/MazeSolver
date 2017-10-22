#include <AccelStepper.h>
#include <StackArray.h>

// Sensor input pins
#define SENSOR_FL       A0
#define SENSOR_L        A1
#define SENSOR_M        A2
#define SENSOR_R        A3
#define SENSOR_FR       A4

#define SENSOR_THRESHOLD    650
#define JUNCTION_THRESHOLD  75
#define FINISH_THRESHOLD    500
#define ACCELERATION        450.0
#define UNIT                195.0
#define INITIAL_UNIT        400.0
#define TURN_90_UNIT        84.0
#define TURN_180_UNIT       162.0
#define DISTANCE_FOR_WHEELS 80.0
#define SENSOR_ARRAY_LENGHT 10
#define BUTTON_PIN 52

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
boolean lastSensorValues[5][SENSOR_ARRAY_LENGHT] = { false };
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
//short int maze[10][10][4] = {
//  {{2,3,3,3}, {3,2,2,3}, {3,2,3,3}, {3,2,3,3}, {2,3,3,3}, {2,2,2,3}, {3,3,2,3}, {2,2,3,3}, {3,3,2,3}, {3,2,3,3}},
//  {{3,2,3,3}, {2,3,3,2}, {2,2,2,2}, {3,3,2,2}, {2,3,3,3}, {2,2,2,2}, {3,3,2,3}, {2,2,3,2}, {3,3,2,3}, {3,2,3,2}},
//  {{2,3,3,2}, {2,2,2,3}, {2,3,2,2}, {2,3,2,3}, {2,2,2,3}, {2,3,2,2}, {2,3,2,3}, {2,2,2,2}, {2,3,2,3}, {3,2,2,2}},
//  {{3,2,3,3}, {2,3,3,2}, {2,2,2,3}, {3,2,2,3}, {2,2,3,2}, {2,2,2,2}, {2,3,2,3}, {3,3,2,2}, {2,2,3,3}, {3,3,2,2}},
//  {{2,3,3,2}, {2,3,2,3}, {3,3,2,2}, {2,2,3,2}, {3,2,2,2}, {2,3,3,2}, {2,3,2,3}, {2,2,2,3}, {2,3,2,2}, {3,2,2,3}},
//  {{2,2,3,3}, {2,3,2,3}, {2,2,2,3}, {3,3,2,2}, {2,2,3,2}, {2,3,2,3}, {3,3,2,3}, {2,2,3,2}, {2,2,2,3}, {3,3,2,2}},
//  {{2,3,3,2}, {3,3,2,3}, {3,2,3,2}, {2,3,3,3}, {3,3,2,2}, {2,2,3,3}, {2,2,2,3}, {3,3,2,2}, {2,2,3,2}, {3,2,2,3}},
//  {{2,2,3,3}, {2,3,2,3}, {3,2,2,2}, {2,2,3,3}, {2,2,2,3}, {3,3,2,2}, {3,3,3,2}, {2,2,3,3}, {2,3,2,2}, {3,2,2,2}},
//  {{3,3,3,2}, {2,3,3,3}, {2,2,2,2}, {3,2,2,2}, {2,2,3,2}, {2,3,2,3}, {2,2,2,3}, {2,3,2,2}, {2,3,2,3}, {3,2,2,2}},
//  {{2,3,3,3}, {2,3,2,3}, {3,3,2,2}, {2,3,3,2}, {3,3,2,2}, {2,3,3,3}, {2,3,2,2}, {3,3,2,3}, {2,3,3,3}, {3,3,2,2}}
//};

// car
short int mode       = 0; // 0 -> standby, 1 -> explore, 2 -> shortest path
short int state      = -1;// -1 initial,   0 -> forward (direction correction), 1 -> turning
short int direction  = 0; // 0 -> north,   1 -> east, 2 -> south,   3 -> west
short int dirToGo    = 0; // 0 -> nowhere, 1 -> left, 2 -> forward, 3 -> right, 4 -> backward
short int xPos       = 0; // current x position
short int yPos       = 0; // current y position
short int xPosFinish = 0;
short int yPosFinish = 0;

// 0 -> left, 1 -> middle, 2 -> right 
short int junctionPassed      = 0;
short int nextJunctionData[3] = { 0 };

// direction correction
int correctionSpeed[2]     = { 200, 200 }; // {left, right}
int correctionSpeeds[9][2] = {
  { correctionSpeed[0] - 60, correctionSpeed[1] + 60 },
  { correctionSpeed[0] - 45, correctionSpeed[1] + 45 },
  { correctionSpeed[0] - 30, correctionSpeed[1] + 30 },
  { correctionSpeed[0] - 18, correctionSpeed[1] + 18 },
  { correctionSpeed[0], correctionSpeed[1] },
  { correctionSpeed[0] + 18, correctionSpeed[1] - 18 },
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
  
  for(int i=0; i<10; i++){
    for(int j=0; j<10; j++){
      mazeDistances[i][j] = 999;
    }
  }

//  Serial.begin(9600);
  delay(1000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  stepperLeft.setMaxSpeed(correctionSpeed[0]);
  stepperLeft.setAcceleration(ACCELERATION);
  stepperLeft.move(INITIAL_UNIT);

  stepperRight.setMaxSpeed(correctionSpeed[1]);
  stepperRight.setAcceleration(ACCELERATION);
  stepperRight.move(-INITIAL_UNIT);
}

void loop() {

  // update sensor values
  sensors();
    
  int sensorStatus = getSensorStatus();
  
  if(mode == 0){
    // stand-by for explorer mode

    delay(3000);
    mode = 1;
    
  }else if(mode == 1){
    // explore
    
    if(state == -1){

      if(sensorStatus > 2 && sensorStatus < 6 ){

        for(int i=0; i < 3; i++){
          stepperLeft.run();
          stepperRight.run();
        }
        state = 0;
      }
      
    }else if(state == 0){
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
      if(sensorStatus > 9 && nextJunctionData[1] > 4*JUNCTION_THRESHOLD && junctionPassed != 2){

        if(junctionPassed == 0)
          Serial.println(sensorStatus);
        
        junctionPassed = 1;

        if(nextJunctionData[0] > FINISH_THRESHOLD && nextJunctionData[2] > FINISH_THRESHOLD){
          // finish case
          
          stepperLeft.setSpeed (0); // stop engines
          stepperRight.setSpeed(0);
          
          updatePosition(); // update xPos, yPos according to direction

          if(xPos == 0 && yPos == 0){ // start

            maze[xPos][yPos][increaseDirection(increaseDirection(direction))]++;
            direction = 0;

            state = 3; // wait for button push

          }else{ // finish
            
            // initialize finish junction
            maze[xPos][yPos][decreaseDirection(direction)] = 3;                    // wall
            maze[xPos][yPos][direction] = 3;                                       // wall
            maze[xPos][yPos][increaseDirection(direction)] = 3;                    // wall
            maze[xPos][yPos][increaseDirection(increaseDirection(direction))] = 1; // road

            stepperLeft.setMaxSpeed (correctionSpeed[0]);
            stepperRight.setMaxSpeed(correctionSpeed[1]);
            stepperLeft.move (DISTANCE_FOR_WHEELS/2);
            stepperRight.move(-DISTANCE_FOR_WHEELS/2);
        
            // turn back and return discovering the maze
            xPosFinish = xPos;
            yPosFinish = yPos;
            dirToGo = 4;
            state = 2;
            delay(3000);
          }
        }
      }

      if(((junctionPassed < 2 && sensorStatus == 9) || (junctionPassed == 1 && sensorStatus < 9)) && nextJunctionData[1] > 4*JUNCTION_THRESHOLD){
        
        junctionPassed = 2;
        nextJunctionData[1] = 0;
        sensorStatus = 4;

        stepperLeft.move ( DISTANCE_FOR_WHEELS);
        stepperRight.move(-DISTANCE_FOR_WHEELS);
        
        String debug = String(nextJunctionData[0]) + "-" +String(nextJunctionData[1]) + "-" + String(nextJunctionData[2]);
        Serial.println(debug);
        
        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
      }

      // Preparation of turn state
      if(stepperLeft.distanceToGo() == 0 || -stepperRight.distanceToGo() == 0){
        // when middle of next junction
        
        String debug = String(nextJunctionData[0]) + "-" +String(nextJunctionData[1]) + "-" + String(nextJunctionData[2]);
        Serial.println(debug);
        
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
        
        stepperLeft.move (TURN_180_UNIT);
        stepperRight.move(TURN_180_UNIT);
        
      }

      dirToGo = 0;
      
      if(!stepperLeft.isRunning() && !stepperRight.isRunning()){
        // after turn move to the next junction
        
        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        state = 0;
        maze[xPos][yPos][direction]++;
        junctionPassed = 0;
        delay(100);
        
        stepperLeft.move (UNIT);
        stepperRight.move(-UNIT);
      }
      
    }else if(state == 2){

      if(stepperLeft.distanceToGo() == 0 || -stepperRight.distanceToGo() == 0){
        delay(100);
        state = 1;
      }
      
    }else if(state == 3){

      if(digitalRead(BUTTON_PIN) == LOW){

        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        junctionPassed = 0;
        dirToGo = 0;
        state = -1;
        mode = 2;
        delay(1000);
        
      }
      
    }
    
  }else if(mode == 2){
    // stand-by and find shortest path
    
    // stop engines
    stepperLeft.stop();
    stepperRight.stop();

    xPos = 0;
    yPos = 0;
    
    Serial.print('\n');

    for(int j=9; j >= 0; j--){
      for(int i=0; i < 10; i++){
        Serial.print(mazeDistances[i][j]);
        Serial.print('\t');
      }
      Serial.print('\n');
    }
    
    while(true){
        
      if(distance < mazeDistances[xPos][yPos]){
        // junction should be improved
        
        mazeDistances[xPos][yPos] = distance;
  
        // find number of possible roads
        short int possibleRoads = 0;
        short int lastDir = direction;
        for(int i=3; i >= 0; i--)
          if(maze[xPos][yPos][i] != 3 && i != increaseDirection(increaseDirection(lastDir)))
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
          short int lastDir = direction;
          for(int i=3; i >= 0; i--)
            if(maze[xPos][yPos][i] != 3 && i != increaseDirection(increaseDirection(lastDir)))
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

    Serial.println(F("numbers inserted"));
    Serial.println(F("\n"));
    
    for(int j=9; j >= 0; j--){
      for(int i=0; i < 10; i++){
        Serial.print(mazeDistances[i][j]);
        Serial.print('\t');
      }
      Serial.print('\n');
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

    reverseSolution = "";

    updatePosition();
    
    while(xPos != 0 || yPos != 0){
      
      if(maze[xPos][yPos][decreaseDirection(direction)] != 3){

        updateNextPosition(decreaseDirection(direction));
        
        if(mazeDistances[nextXPos][nextYPos] < distance){
          direction = decreaseDirection(direction);
          reverseSolution += "L";
          distance = mazeDistances[nextXPos][nextYPos];
          updatePosition();
          continue;
        }
      }
      
      if(maze[xPos][yPos][direction] != 3){
        
        updateNextPosition(direction);
        
        if(mazeDistances[nextXPos][nextYPos] < distance){
//          direction = direction;
          reverseSolution += "F";
          distance = mazeDistances[nextXPos][nextYPos];
          updatePosition();
          continue;
        }
      }
      
      if(maze[xPos][yPos][increaseDirection(direction)] != 3){
        
        updateNextPosition(increaseDirection(direction));
        
        if(mazeDistances[nextXPos][nextYPos] < distance){
          direction = increaseDirection(direction);
          reverseSolution += "R";
          distance = mazeDistances[nextXPos][nextYPos];
          updatePosition();
          continue;
        }
      }
      
    }

    Serial.print(F("reverseSolution:"));
    Serial.println(reverseSolution);

    solution = "";
    
    for(int i=1; i <= reverseSolution.length(); i++){

      char s = 'F';

      if(reverseSolution.charAt(reverseSolution.length()-i) == 'R')
        s = 'L';
      else if(reverseSolution.charAt(reverseSolution.length()-i) == 'L')
        s = 'R';
        
      solution += s;
    }

    Serial.print(F("solution:"));
    Serial.println(solution);

    distance = 0;
    state = -1;
    mode = 3;
    
  }else if(mode == 3){
    // solve

    if(state == -1){

      stepperLeft.setMaxSpeed(correctionSpeed[0]);
      stepperRight.setMaxSpeed(correctionSpeed[1]);
      
      stepperLeft.move(INITIAL_UNIT);
      stepperRight.move(-INITIAL_UNIT);
        
      if(sensorStatus >= 2 && sensorStatus <= 6){
        
        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        state = 0;
        junctionPassed = 0;
        
      }
      
    }else if(state == 0){
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
      if(sensorStatus > 9 && nextJunctionData[1] > 5*JUNCTION_THRESHOLD && junctionPassed != 2){

        if(junctionPassed == 0)
          Serial.println(sensorStatus);
        
        junctionPassed = 1;

        if(nextJunctionData[0] > FINISH_THRESHOLD && nextJunctionData[2] > FINISH_THRESHOLD){
          // finish case
          
          stepperLeft.setSpeed (0); // stop engines
          stepperRight.setSpeed(0);

          Serial.println(nextJunctionData[0]);
          Serial.println(nextJunctionData[2]);
          Serial.println("FINISH!");
          
          mode = 4;
          state = 0;
          distance = 0;
          solution = reverseSolution;

          delay(3000);
        }
      }

      if(((junctionPassed < 2 && sensorStatus == 9) || (junctionPassed == 1 && sensorStatus < 9)) && nextJunctionData[1] > 5*JUNCTION_THRESHOLD){

        Serial.println("--");
        Serial.println(sensorStatus);
        Serial.println(nextJunctionData[0]);
        Serial.println(nextJunctionData[1]);
        Serial.println(nextJunctionData[2]);
        Serial.println("");
        
        junctionPassed = 2;
        nextJunctionData[1] = 0;
        sensorStatus = 4;

        stepperLeft.move ( DISTANCE_FOR_WHEELS);
        stepperRight.move(-1*DISTANCE_FOR_WHEELS);
        
        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
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

        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        state = 0;
        distance++;
        junctionPassed = 0;
        delay(100);
        
        stepperLeft.move (UNIT);
        stepperRight.move(-1*UNIT);
        
      }
      
    }

  }else if(mode == 4){

    if(state == 0){
      
      stepperLeft.setMaxSpeed(correctionSpeed[0]);
      stepperRight.setMaxSpeed(correctionSpeed[1]);
      stepperLeft.move(TURN_180_UNIT);
      stepperRight.move(TURN_180_UNIT);

      state = 1;
    }
    
    if(stepperLeft.distanceToGo() == 0 && -stepperRight.distanceToGo() == 0){

      if(state == 1){
        
        stepperLeft.setMaxSpeed(correctionSpeed[0]);
        stepperRight.setMaxSpeed(correctionSpeed[1]);
        stepperLeft.move(-DISTANCE_FOR_WHEELS/2);
        stepperRight.move(DISTANCE_FOR_WHEELS/2);

        state = 2;
        
      }else if(state == 2){
        for(int i=0; i < 5; i++)
          for(int j=0; j < SENSOR_ARRAY_LENGHT; j++)
            lastSensorValues[i][j] = false;
        for(int i=0; i<3; i++)
          nextJunctionData[i] = 0;
        solution = reverseSolution;
        mode = 3;
        state = -1;
        distance = 0;
        junctionPassed = 0;
        delay(100);
        
        stepperLeft.move (UNIT);
        stepperRight.move(-1*UNIT);
      }
    }
  }

  stepperLeft.run();
  stepperRight.run();
}



void sensors() {

  for(int i=0; i < 5; i++)
    for(int j=SENSOR_ARRAY_LENGHT-1; j > 0; j--)
      lastSensorValues[i][j] = lastSensorValues[i][j-1];

  lastSensorValues[0][0] = analogRead(SENSOR_FR) < SENSOR_THRESHOLD;
  lastSensorValues[1][0] = analogRead(SENSOR_R)  < SENSOR_THRESHOLD;
  lastSensorValues[2][0] = analogRead(SENSOR_M)  < SENSOR_THRESHOLD;
  lastSensorValues[3][0] = analogRead(SENSOR_L)  < SENSOR_THRESHOLD;
  lastSensorValues[4][0] = analogRead(SENSOR_FL) < SENSOR_THRESHOLD;

  short averageFarRight = 0;
  for(int i=0; i < SENSOR_ARRAY_LENGHT; i++)
    averageFarRight += lastSensorValues[0][i] ? 1 : 0;
  sensorFarRight = averageFarRight > SENSOR_ARRAY_LENGHT/2;

  short averageRight = 0;
  for(int i=0; i < SENSOR_ARRAY_LENGHT; i++)
    averageRight += lastSensorValues[1][i] ? 1 : 0;
  sensorRight = averageRight > SENSOR_ARRAY_LENGHT/2;
  
  short averageMiddle = 0;
  for(int i=0; i < SENSOR_ARRAY_LENGHT; i++)
    averageMiddle += lastSensorValues[2][i] ? 1 : 0;
  sensorMiddle = averageMiddle > SENSOR_ARRAY_LENGHT/2;
  
  short averageLeft = 0;
  for(int i=0; i < SENSOR_ARRAY_LENGHT; i++)
    averageLeft += lastSensorValues[3][i] ? 1 : 0;
  sensorLeft = averageLeft > SENSOR_ARRAY_LENGHT/2;
  
  short averageFarLeft = 0;
  for(int i=0; i < SENSOR_ARRAY_LENGHT; i++)
    averageFarLeft += lastSensorValues[4][i] ? 1 : 0;
  sensorFarLeft = averageFarLeft > SENSOR_ARRAY_LENGHT/2;

//  sensorFarRight = analogRead(SENSOR_FR) < SENSOR_THRESHOLD;
//  sensorRight    = analogRead(SENSOR_R)  < SENSOR_THRESHOLD;
//  sensorMiddle   = analogRead(SENSOR_M)  < SENSOR_THRESHOLD;
//  sensorLeft     = analogRead(SENSOR_L)  < SENSOR_THRESHOLD;
//  sensorFarLeft  = analogRead(SENSOR_FL) < SENSOR_THRESHOLD;

//  String debug = String(averageFarLeft) + "-" +String(averageLeft) + "-" + String(averageMiddle) + "-" + String(averageRight) + "-" + String(averageFarRight);
//  Serial.println(debug);
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
