/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"
#include <signal.h>

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction

int maxSensorIter=30;           // Max number of iterations we allow because of faulty colour sensor so we never end up in an infinite loop
int dataLength=10;              // Length of the past data that is stored 
int pastColour[10][3];  // Stores the past colours from the colour sensor RGB
int T_STEP=2;                   // Represents how many seconds are in a tick
int previousSensorReading[3] = {-1, -1, -1};
int calibratedColourValues[6][3]; // We have 6 different Colours
                                  //   BLACKCOLOR   = 1,
                                  //   BLUECOLOR    = 2,
                                  //   GREENCOLOR   = 3,
                                  //   YELLOWCOLOR  = 4,
                                  //   REDCOLOR     = 5,
                                  //   WHITECOLOR   = 6

// Driving motors
int left_motor_drive = -33;//-33;
int right_motor_drive = -37;//-40;
int right_motor_right_turn = 15;
int left_motor_right_turn = -15;
int right_motor_left_turn = -15;
int left_motor_left_turn = 15;

// Sensor motor
int LR_power = 100;
int mid_power = 30;
double LR_time_increment = 1.5;
double mid_time_increment = 3;

static void catchFunction(int signo) {
  printf("Caught Ctrl + C\n");
  sleep(1);
  BT_all_stop(1);
  BT_close();
  exit(1);
}


int main(int argc, char *argv[])
{ 
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }

 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/
 // Safely assume calibration file exists, read and set calibration values
 FILE *fp = fopen("calibration.txt", "r");
 if (fp == NULL) {
  perror("cannot open calibration file for reading");
  exit(1);
 }
 char line[256];
 int i = 0;
 while (fgets(line, sizeof(line), fp)) {
  int j=0;
  char *token = strtok(line, ",");
  while (token != NULL) {
    calibratedColourValues[i][j] = atoi(token);
    token = strtok(NULL, ",");
    j++;
  }
  i++;
 }
 fclose(fp);
 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }

 signal(SIGINT, &catchFunction);

//  // Open a socket to the EV3 for remote controlling the bot.
//  if (BT_open(HEXKEY)!=0)
//  {
//   fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
//   fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
//   free(map_image);
//   exit(1);
//  }

 fprintf(stderr,"All set, ready to go!\n");
 
/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.
BT_all_stop(0);
int dir = 5;
// Test code
// while(1) {
//   BT_turn(MOTOR_A, left_motor_drive, MOTOR_D, right_motor_drive);
//   sleep(0.25);
//   BT_all_stop(1);
// }

int test;
scan_intersection(&test, &test, &test, &test);

//drive_along_street();

// turn_at_intersection(1);

// while(1){
//   printf("detected colour: %d\n", scanColour());
//   // BT_drive(MOTOR_A, MOTOR_D,-60);
//   // BT_drive(MOTOR_C, MOTOR_C, dir);
//   // dir = dir*-1;
//   // sleep(3);
// }


//  int rgb[3] = {-1,-1,-1};

//  while(true){
//   BT_read_colour_sensor_RGB(0, rgb);
//   for (int i=0; i<3; i++){
//     printf("%d ", rgb[i]);
//   }
//   printf("\n");
//  }
 
 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
 BT_close();
 free(map_image);
 exit(0);
}

int *averageSensorReading(int *sensorReadings, int numSamples)
{
  /*
    Possible colours are
    RGB 
    white: 255,255,255
    green: 0  ,255,0
    blue:  
  */
  int *avgColours = (int *) calloc(3,sizeof(int));
  if (!avgColours){
    fprintf(stderr, "Malloc error\n");
    return NULL;
  }
  for (int i=0; i<numSamples; i++){
    for (int rgbIndex=0; rgbIndex < 3; rgbIndex++)
    {
      *(avgColours+rgbIndex) += *(sensorReadings+3*i+rgbIndex);
    }
  }
  for (int i=0; i<3; i++){
    *(avgColours+i) = *(avgColours+i) / numSamples;
  }
  return avgColours; 
}

int getColourFromReading(int sensorReading[3])
{
  // BECAREFUL SINCE ARRAY INDEXING STARTS AT 0
//  typedef   enum
// {
//   BLACKCOLOR   = 1,
//   BLUECOLOR    = 2,
//   GREENCOLOR   = 3,
//   YELLOWCOLOR  = 4,
//   REDCOLOR     = 5,
//   WHITECOLOR   = 6
// }

  int minSquaredError = INT_MAX;
  int minSquaredErrorIndex = -1;
  for (int i=0; i<6; i++){
    int squaredError = 0;
    for (int rgbIndex = 0; rgbIndex < 3; rgbIndex++){
      squaredError += pow(calibratedColourValues[i][rgbIndex] - sensorReading[rgbIndex], 2);
    }
    printf("squaredError: %d\n", squaredError);
    if (squaredError < minSquaredError){
      minSquaredError = squaredError;
      minSquaredErrorIndex = i;
    }
  }
  // shift it by one since indexing starts at 0, but blackcolour starts at 1
  return minSquaredErrorIndex+1;
}

int isReadingValid(int sensorReading[3])
{
  /*
    Sometimes the results from API returned doesn't make sense in which case we want to discard the result
  */
  int difReading = 0;
  for (int i=0; i<3; i++)
  {
    int reading = sensorReading[i];
    if (reading < 0 || reading > 1020) {return 0;}
    if (reading != previousSensorReading[i]) {difReading=1;}
  }
  for (int i=0; i<3; i++)
  {
    // Update previous sensor reading
    previousSensorReading[i] = sensorReading[i];
  }
  return difReading;
}

int *getColourReading(int numSamples)
{
  int validReadings=0;
  int totalReadings=0;
  int *sampledColours = (int *) calloc(numSamples * 3, sizeof(int));
  if (!sampledColours)
  {
    fprintf(stderr,"Malloc failed\n");
    return NULL;
  }
  // maxSnesorIter is constant but not numSamples, maybe do maxSensorIter + numSamples
  // printf("start sampling\n");
  while(validReadings < numSamples && totalReadings < maxSensorIter)
  {
    BT_read_colour_sensor_RGB(PORT_1, sampledColours+3*validReadings);
    // printf("sampling %d r: %d g: %d b: %d\n", validReadings, *(sampledColours+3*validReadings),*(sampledColours+3*validReadings+1),*(sampledColours+3*validReadings+2));
    if (isReadingValid(sampledColours+3*validReadings)) { validReadings++; };
    totalReadings++;
    // TODO shouldn't have T_STEP here at all
    // sleep(T_STEP);
  }
  if (maxSensorIter<totalReadings) { fprintf(stderr,"Error: reached max sensor iteration!\n"); }
  int *sensorReading = averageSensorReading(sampledColours, numSamples);
  free(sampledColours);
  return sensorReading;
}

int scanColour()
{
  /*
   * This function is used to get the colour from the colour sensor.
   */
  int *colourReading = getColourReading(10);
  printf("R: %d G: %d B: %d\n", *colourReading, *(colourReading+1), *(colourReading+2));
  int colourValue = getColourFromReading(colourReading);
  free(colourReading);
  return colourValue;
}

int find_street(void)   
{
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */   

  // TODO Questions: will the center of rotation be placed on the line of will the sensor be placed in the center
  // NEED to do: figure out how to rotate on the spot
  int curColour = -1;
  scanColour();
  while (curColour != BLACKCOLOR){
    
    sleep(T_STEP);
  }

  return(0);
}


int turnTowardsStreet(void){
  BT_all_stop(1);
  int curColour = 0;
  for (int ticks = 1; ticks < 20; ticks+=2) {
    // ticks = 1;
    // Turn right to find the road;
    BT_timed_motor_port_start(MOTOR_A, -15, 50, 50*ticks, 100);
    BT_timed_motor_port_start(MOTOR_D, 15, 50, 50*ticks, 100);
    
    sleep(1.5);
    curColour = scanColour();
    if(curColour == 1 || curColour == 4) {return 1;}


    // Turn left to find the road;
    BT_timed_motor_port_start(MOTOR_A, 15, 50, 50*(ticks+1), 100);
    BT_timed_motor_port_start(MOTOR_D, -15, 50, 50*(ticks+1), 100);
    
    sleep(1.5);
    curColour = scanColour();
    if(curColour == 1 || curColour == 4) {return 1;}


    // // Undo the right turn
    // BT_timed_motor_port_start(MOTOR_A, 20, 50, 80*ticks, 200);
    // BT_timed_motor_port_start(MOTOR_D, -20, 50, 80*ticks, 200);
    // sleep(1);

    // // Turn left to find the road
    // BT_timed_motor_port_start(MOTOR_A, 15, 50, 50*ticks, 200);
    // BT_timed_motor_port_start(MOTOR_D, -15, 50, 50*ticks, 200);
    // sleep(1);
    // curColour = scanColour();
    // if(curColour == 1 || curColour == 4) {return 1;}

    // // Undo the left turn
    // BT_timed_motor_port_start(MOTOR_A, -20, 50, 80*ticks, 200);
    // BT_timed_motor_port_start(MOTOR_D, 20, 50, 80*ticks, 200);
    // sleep(1);
    // BT_all_stop(1);
  }
  return 0;
}



int drive_along_street(void)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   
  

  // Keep going straight until we get a reading that is not black
  bool onBlack = 1;
  while (onBlack) {
    int curColour = scanColour();

    // if (curColour == 4) {
    //   // assume we are straight with the road
    //   return 1;
    // }

    if (curColour == 1 || curColour == 4){
      fprintf(stderr, "On Black\n");
      BT_turn(MOTOR_A, left_motor_drive, MOTOR_D, right_motor_drive);
    } else {
      fprintf(stderr, "Away from Black\n");
      turnTowardsStreet();
      // BT_turn(MOTOR_A, left_motor_right_turn, MOTOR_D, right_motor_right_turn);
      // BT_timed_motor_port_start(char port_id, char power, int ramp_up_time, int run_time, int ramp_down_time);
      // BT_timed_motor_port_start(MOTOR_A, -15, 50, 50, 100);
      // sleep(1);
      // BT_all_stop(1);
      // onBlack = 0;
    }
  }




  // // not aligned on the street
  // while (1) {
  //   int *colour_triplet = scanTriplet();
  //   if (colour_triplet[1] == 4) {
  //     fprintf(stderr, "Intersection detected");
  //     return 1;
  //   } else if (colour_triplet[1] != 1) {
  //     return -1; // deviate
  //   } else  {
  //     BT_turn(MOTOR_A, left_motor_drive,  MOTOR_D, right_motor_drive);
  //     sleep(1);
  //     BT_all_stop(1);
  //   }
  //   free(colour_triplet);
  // }
  // return 0;
}

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
 *(tl)=-1;
 *(tr)=-1;
 *(br)=-1;
 *(bl)=-1;

 // Move sensor to far right position
 BT_timed_motor_port_start_v2(MOTOR_C, LR_power, mid_time_increment * 1000);

 // Check if the robot is lined up at the intersection
 if (scanColour() != 1) {
  printf("Robot is either not aligned with street at intersection or too far away from centre of intersection\n");
  return 0;
 }

 // Drive forwards until the sensor detects a colour other than black
 BT_turn(MOTOR_A, left_motor_drive,  MOTOR_D, right_motor_drive);
 while (true) {
  if (scanColour() != 1) {
    // Slowly drive forwards a little to ensure the sensor is in the middle of the building
    BT_timed_motor_port_start(MOTOR_A, 0, 100, 0, 0);
    BT_timed_motor_port_start(MOTOR_D, 0, 100, 0, 0);

    // Move sensor to middle position
    BT_timed_motor_port_start_v2(MOTOR_C, -mid_power, mid_time_increment * 1000);
    break;
  }
 }
 
 // Perform a colour scan and store left and right colours scanned
 int *colour_triplet = scanTriplet();
 *tl = colour_triplet[0];
 *tr = colour_triplet[2];

 fprintf(stderr, "Top left: %d, Top right: %d\n", colour_triplet[0], colour_triplet[2]);

 // Realign robot with the street
 
 // Move sensor to far right position
 BT_timed_motor_port_start_v2(MOTOR_C, LR_power, mid_time_increment * 1000);

 // Drive backwards until the sensor detects black, then drive backwards until a colour other than black is detected
 BT_turn(MOTOR_A, -left_motor_drive,  MOTOR_D, -right_motor_drive);
 while (true) {
  if (scanColour() == 1) {
    while (true) {
      if (scanColour() != 1) {
      // Slowly drive backwards a little to ensure the sensor is in the middle of the building
      BT_timed_motor_port_start(MOTOR_A, 0, 100, 0, 0);
      BT_timed_motor_port_start(MOTOR_D, 0, 100, 0, 0);

      // Move sensor to middle position
      BT_timed_motor_port_start_v2(MOTOR_C, -mid_power, mid_time_increment * 1000);
      break;
      }
    }
    break;
  }
 }
 // Perform a colour scan and store left and right colours scanned
  colour_triplet = scanTriplet();
  *bl = colour_triplet[0];
  *br = colour_triplet[2];

  fprintf(stderr, "Bottom left: %d, Bottom right: %d\n", colour_triplet[0], colour_triplet[2]);

  free(colour_triplet);
  return(1);
 
}

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */

  // Assume intersection is under sensor, assume building have been scanned, based on turn direction, you have a triplet of integers
  // which is the desired reading for the color sensor. (for now, just look for black in the middle)
  // Turn small amounts, and scan. Repeat until sensor detects desired triplet.
  // keep track of total time turned, if above threshold, robot is stuck. return 0. go to find street
  // return 1 if successful
  // Assumes A is left wheel, D is right wheel, C is color sensor
  ////////////////////////////////////////////////////////////////////////////////////////////
  int total_time = 0;
  int total_time_threshold = 10;
  double time_increment = 1;
  int desired_reading[] = {0, 1, 0}; // update with scanned buildings once scanning is possible
  int keep_turning = 1;
  ////////////////////////////////////////////////////////////////////////////////////////////
  while(keep_turning && total_time < total_time_threshold){
    if (turn_direction){
      // turn left, implies right motor should be given power
      BT_turn(MOTOR_A, left_motor_left_turn, MOTOR_D, right_motor_left_turn);
    } else {
      BT_turn(MOTOR_A, left_motor_right_turn, MOTOR_D, right_motor_right_turn);
    }
    sleep(time_increment);
    BT_all_stop(1);
    total_time += time_increment;
    int *colour_triplet = scanTriplet();
    // change this once we know all 3 desired values
    keep_turning = !(colour_triplet[1] == 1);
    free(colour_triplet);
  }

  if (total_time >= total_time_threshold){
    return 0;
  }
  return 1;
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return an invalid location/direction and notify that localization was unsuccessful (you will delete this and replace it
 // with your code).
 *(robot_x)=-1;
 *(robot_y)=-1;
 *(direction)=-1;
 return(0);
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  return(0);  
}

void calibrate_sensor(void)
{
 /*
  * This function is called when the program is started with -1  -1 for the target location. 
  *
  * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
  * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
  * level.
  * 
  * The principle is - Your code should allow you to sample the different colours in the map, and store representative
  * values that will help you figure out what colours the sensor is reading given the current conditions.
  * 
  * Inputs - None
  * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
  * 
  * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
  */   

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr,"Calibration function called!\n");
  // reset file
  fclose(fopen("calibration.txt", "w"));
  // open in append mode
  FILE *fp = fopen("calibration.txt", "a");
  if (fp == NULL){
       perror("cannot open calibration file for writing");
       exit(1);
  }
  const char *colours[6] = {"Black", "Blue", "Green", "Yellow", "Red", "White"};
  for (int i=0; i<6; i++){
    printf("Place sensor over %s, then press ENTER", colours[i]);
    getchar();
    int *rgb = getColourReading(30);
    if (!rgb) {
      perror("error in allocating memory for color readings");
      exit(1);
    }
    fprintf(fp, "%d,%d,%d\n", *(rgb), *(rgb+1), *(rgb+2));
    free(rgb);
  }
  fclose(fp);
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}

int* scanTriplet() {
  int *colour_triplet = (int*)calloc(sizeof(int), 3);

  BT_all_stop(1);
  // Scan x3
  // Assume sensor is in middle
  colour_triplet[1] = scanColour();
  // Move sensor motor to right (from middle)
  BT_timed_motor_port_start_v2(MOTOR_C, LR_power, LR_time_increment * 1000);
  colour_triplet[2] = scanColour();
  // Move sensor to the left (more power/time to cross the barrier)
  BT_timed_motor_port_start_v2(MOTOR_C, -LR_power, LR_time_increment * 1000);
  colour_triplet[0] = scanColour();
  // Set sensor back to the middle
  // Get through the barrier
  BT_timed_motor_port_start_v2(MOTOR_C, LR_power, LR_time_increment * 1000);
  // Come back to the middle (move left from the right side)
  BT_timed_motor_port_start_v2(MOTOR_C, -mid_power, mid_time_increment * 1000);

  return colour_triplet;
}