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

int map[400][4];                                                               // This holds the representation of the map, up to 20x20
                                                                               // intersections, raster ordered, 4 building colours per
                                                                               // intersection.
int sx, sy;                                                                    // Size of the map (number of intersections along x and y)
double beliefs[400][4];                                                        // Beliefs for each location and motion direction
int calibrated_colours[6][3];                                                  // stores calibrated values of colours
                                                                               // We have 6 different Colours
                                                                               //   BLACKCOLOR   = 1,
                                                                               //   BLUECOLOR    = 2,
                                                                               //   GREENCOLOR   = 3,
                                                                               //   YELLOWCOLOR  = 4,
                                                                               //   REDCOLOR     = 5,
                                                                               //   WHITECOLOR   = 6
const char *colours[6] = {"Black", "Blue", "Green", "Yellow", "Red", "White"}; // integer to colour array
int pastColour = 0;                                                            // Stores the past colours from the colour sensor RGB
double currentMinSquareError = -1;

// Driving motors
int left_motor_drive = -33;  //-33;
int right_motor_drive = -35; //-40;
int right_motor_right_turn = 15;
int left_motor_right_turn = -15;
int right_motor_left_turn = -15;
int left_motor_left_turn = 15;
int last_turn = -1;
#define CALIBRATION_COUNT 30

// CATCH CTRL+C AND STOP ROBOT AND EXIT PROGRAM WHEN CALLED
static void catchFunction(int signo)
{
  printf("Caught Ctrl + C\n");
  sleep(1); // check if we need this
  BT_all_stop(1);
  BT_close();
  exit(1);
}

int main(int argc, char *argv[])
{
  char mapname[1024];
  int dest_x, dest_y, rx, ry;
  unsigned char *map_image;

  memset(&map[0][0], 0, 400 * 4 * sizeof(int));
  sx = 0;
  sy = 0;

  if (argc < 4)
  {
    fprintf(stderr, "Usage: EV3_Localization map_name dest_x dest_y\n");
    fprintf(stderr, "    map_name - should correspond to a properly formatted .ppm map image\n");
    fprintf(stderr, "    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
    exit(1);
  }
  strcpy(&mapname[0], argv[1]);
  dest_x = atoi(argv[2]);
  dest_y = atoi(argv[3]);

  // Open a socket to the EV3 for remote controlling the bot.
  if (BT_open(HEXKEY) != 0)
  {
    fprintf(stderr, "Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
    fprintf(stderr, " hex key for the EV3 matches the one in EV3_Localization.h\n");
    free(map_image);
    exit(1);
  }

  if (dest_x == -1 && dest_y == -1)
  {
    calibrate_sensor();
    exit(1);
  }

  /******************************************************************************************************************
   * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
   *   read your calibration data for use in your localization code. Skip this if you are not using calibration
   * ****************************************************************************************************************/
  FILE *fp = fopen("calibration.txt", "r");
  if (fp == NULL)
  {
    perror("cannor open calibration file for reading");
    exit(1);
  }
  char line[256];
  int i = 0;
  while (fgets(line, sizeof(line), fp))
  {
    int j = 0;
    char *token = strtok(line, ",");
    while (token)
    {
      calibrated_colours[i][j] = atoi(token); // store colour readings to global array
      token = strtok(NULL, ",");
      j++;
    }
    i++;
  }
  fclose(fp);

  // Your code for reading any calibration information should not go below this line //

  map_image = readPPMimage(&mapname[0], &rx, &ry);
  if (map_image == NULL)
  {
    fprintf(stderr, "Unable to open specified map image\n");
    exit(1);
  }

  if (parse_map(map_image, rx, ry) == 0)
  {
    fprintf(stderr, "Unable to parse input image map. Make sure the image is properly formatted\n");
    free(map_image);
    exit(1);
  }

  if (dest_x < 0 || dest_x >= sx || dest_y < 0 || dest_y >= sy)
  {
    fprintf(stderr, "Destination location is outside of the map\n");
    free(map_image);
    exit(1);
  }

  // Initialize beliefs - uniform probability for each location and direction
  for (int j = 0; j < sy; j++)
  {
    for (int i = 0; i < sx; i++)
    {
      beliefs[i + (j * sx)][0] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][1] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][2] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][3] = 1.0 / (double)(sx * sy * 4);
    }
  }

  signal(SIGINT, &catchFunction);

  fprintf(stderr, "All set, ready to go!\n");

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
  robot_localization(0, 0, 0);

  // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
  BT_close();
  free(map_image);
  exit(0);
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
  return (0);
}

void courseCorrect()
{
  // Move back a bit
  printf("going back a bit\n");
  BT_turn(MOTOR_A, 10, MOTOR_D, 10); // 12 before
  sleep(1);
  BT_all_stop(1);
  // Called during deviations in drive street, once course corrected, return
  // You havent deviated that far off, move the sensor to the, you can
  // reasonably expect the left sensor position to be on the right side of the street
  // regardless of the direction of deviation.
  BT_timed_motor_port_start_v2(MOTOR_C, -100, 1500);
  // pivot on right wheel, move slowly till sensor detects black/yellow
  // 3 times in a row
  int Ticks = 0;
  int TickThreshold = 1;
  while (Ticks < TickThreshold)
  {
    int colour = scanColour(3);
    if (colour == 1 || colour == 4 || colour == 5)
    {
      Ticks++;
    }
    else
    {
      Ticks = 0;
    }
    BT_motor_port_stop(MOTOR_D, 1);
    BT_motor_port_start(MOTOR_A, -20);
    fprintf(stderr, "pivoting right\n");
  }
  printf("threshold reached, stopping\n");
  BT_all_stop(1);
  // centre the sensor again
  BT_timed_motor_port_start_v2(MOTOR_C, 100, 1500);
  BT_timed_motor_port_start_v2(MOTOR_C, -40, 500);
  // pivot on right wheel, move slowly till sensor detects black 3 times in a row
  Ticks = 0;
  while (Ticks < TickThreshold)
  {
    int colour = scanColour(1);
    if (colour == 1 || colour == 4 | colour == 5)
    {
      Ticks++;
    }
    else
    {
      Ticks = 0;
    }
    BT_motor_port_stop(MOTOR_D, 1);
    BT_motor_port_start(MOTOR_A, 20);
    fprintf(stderr, "pivoting left\n");
  }
  printf("course correction complete\n");
  BT_all_stop(1);
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
  while (scanColour(1) == 1)
  {
    // drive forward
    printf("driving on street\n");
    BT_turn(MOTOR_A, -18, MOTOR_D, -20);
  }
  // you stopped detecting black, stop moving
  BT_all_stop(1);
  int colour = scanColour(10);
  if (colour == 1)
  {
    // false alarm
    printf("false alarm\n");
    return drive_along_street();
  }
  if (colour == 4)
  {
    // intersection detected
    printf("Intersection detected\n");
    return 1; // success
  }
  if (colour == 5)
  {
    // map edge detected
    printf("Map Edge detected, figure out what to do in robot localization\n");
    return 0; // edge
  }
  // If you reach this, you have deviated, course correct
  printf("need to course correct\n");
  courseCorrect();
  // drive along street again
  return drive_along_street();
}

void getToSensor()
{
  BT_turn(MOTOR_A, -14, MOTOR_D, -16);
  sleep(2);
  BT_all_stop(1);
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
  *(tl) = -1;
  *(tr) = -1;
  *(br) = -1;
  *(bl) = -1;

  // stop moving
  BT_all_stop(1);
  printf("SCANNING INTERSECTION\n");
  // Move sensor to far right
  BT_timed_motor_port_start_v2(MOTOR_C, 60, 1000);

  // Check if the robot is aligned
  if (scanColour(5) != 1)
  {
    printf("Robot is misaligned\n");
    BT_timed_motor_port_start_v2(MOTOR_C, -40, 1000);
    sleep(1);
    getToSensor();
    while (scanColour(1) != 1)
    {
      if (last_turn)
      {
        BT_turn(MOTOR_A, 12, MOTOR_D, -12);
      }
      else
      {
        BT_turn(MOTOR_A, -12, MOTOR_D, 12);
      }
    }
    // go back to intersection
    while (scanColour(3) != 4)
    {
      BT_turn(MOTOR_A, 18, MOTOR_D, 20);
    }
    BT_all_stop(1);
  }
  // Move sensor to far right
  BT_timed_motor_port_start_v2(MOTOR_C, 60, 1000);
  sleep(1);

  // Drive backwards till sensor detects non black
  BT_turn(MOTOR_A, 18, MOTOR_D, 20);
  while (1)
  {
    if (scanColour(5) != 1)
    {
      // Stop slowly
      BT_timed_motor_port_start(MOTOR_A, 0, 200, 0, 0);
      BT_timed_motor_port_start(MOTOR_D, 0, 200, 0, 0);
      break;
    }
  }
  *br = scanColour(10);
  // Move sensor to left
  BT_timed_motor_port_start_v2(MOTOR_C, -80, 1500);
  *bl = scanColour(10);
  printf("The sensor detects:\n BOTTOM LEFT: %s, BOTTOM RIGHT: %s\n", colours[*bl - 1], colours[*br - 1]);
  while (1)
  {
    // Drive forwards until the sensor detects black
    BT_turn(MOTOR_A, -18, MOTOR_D, -20);
    if (scanColour(5) == 1)
    {
      break;
    }
  }
  while (1)
  {
    // drive forward till you cross black
    BT_turn(MOTOR_A, -18, MOTOR_D, -20);
    if (scanColour(5) != 1)
    {
      // Slow stop
      BT_timed_motor_port_start(MOTOR_A, 0, 200, 0, 0);
      BT_timed_motor_port_start(MOTOR_D, 0, 200, 0, 0);
      break;
    }
  }
  *tl = scanColour(10);
  BT_timed_motor_port_start_v2(MOTOR_C, 100, 1000); // go to right
  *tr = scanColour(10);
  BT_timed_motor_port_start_v2(MOTOR_C, -40, 2000); // come to middle from right
  printf("The sensor detects:\n TOP LEFT: %s, TOP RIGHT: %s\n", colours[*tl - 1], colours[*tr - 1]);

  return (1); // success
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

  // Keep going forward if you are detecting black
  BT_turn(MOTOR_A, left_motor_drive + 13, MOTOR_D, right_motor_drive + 13);
  while (1)
  {
    int colour_detected = scanColour(5);
    // slowly move forward on road
    if (colour_detected == 1)
    {
      BT_turn(MOTOR_A, left_motor_drive + 13, MOTOR_D, right_motor_drive + 13);
    }
    // if deviated
    else if (!(colour_detected == 4 || colour_detected == 5))
    {
      BT_all_stop(1);
      courseCorrect();
    }
    // we hit the next intersection or the edge of the map
    else
    {
      // Brake
      // BT_timed_motor_port_start(MOTOR_A, left_motor_drive+13, 0, 0, 200);
      // BT_timed_motor_port_start(MOTOR_D, right_motor_drive+13, 0, 0, 200);
      // sleep(1);
      BT_all_stop(0);
      // move back slowly till you detect black
      // while (true) {
      //   if (scanColour(5) == 1) {
      //     BT_all_stop(1);
      //     break;
      //   }
      //   BT_turn(MOTOR_A, -left_motor_drive-15,  MOTOR_D, -right_motor_drive-15);
      // }

      // BT_timed_motor_port_start(MOTOR_A, -left_motor_drive-15, 50, 400, 200);
      // BT_timed_motor_port_start(MOTOR_D, -right_motor_drive-15, 50, 400, 200);
      // sleep(1);
      break;
    }
  }
  if (turn_direction)
  {
    // BT_timed_motor_port_start(MOTOR_A, 22, 50, 500, 2000);
    // BT_timed_motor_port_start(MOTOR_D, -22, 50, 500, 2000);
    BT_motor_port_stop(MOTOR_D, 1);
    BT_motor_port_start(MOTOR_A, 22);
    fprintf(stderr, "Turning left\n");
  }
  else
  {
    // BT_timed_motor_port_start(MOTOR_A, -22, 50, 500, 2000);
    // BT_timed_motor_port_start(MOTOR_D, 22, 50, 500, 2000);
    BT_motor_port_stop(MOTOR_A, 1);
    BT_motor_port_start(MOTOR_D, 20);
    fprintf(stderr, "Turning right\n");
  }
  sleep(1);

  int Ticks = 0;
  int TickThreshold = 3;
  printf("pivoting for turn at intersection\n");
  while (Ticks < TickThreshold)
  {
    int colour_detected = scanColour(1);
    if (colour_detected == 1)
    {
      Ticks++;
    }
    else
    {
      Ticks = 0;
    }
  }
  sleep(0.2);
  BT_all_stop(1);
  fprintf(stderr, "Finished Turn\n");
  last_turn = turn_direction;
  return (0);
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

  // 0 = FOWARD, 1 = RIGHT, 2 = BACKWARDS, 3 = LEFT
  int moveDir = 0;
  int netDir = moveDir;
  int detectedRed = drive_along_street();

  // Initial call: get onto an intesection without scanning, then begin localization process
  // Drive along street and if it hits red, update the beliefs to account for turning right
  while (detectedRed == 0)
  {
    turn_at_intersection(0);
    detectedRed = drive_along_street();
  }

  // After every iteration, assume we are on an intersection
  while (1)
  {
    moveDir = rand() % 4;
    while (moveDir == 2)
    {
      moveDir = rand() % 4;
    }

    netDir = moveDir;
    // Action Model
    // Turn at intersection if needed
    if (moveDir == 1)
    {
      turn_at_intersection(0);
    }
    else if (moveDir == 3)
    {
      turn_at_intersection(1);
    }

    // Drive along street and if it hits red, update the beliefs to account for turning right
    detectedRed = drive_along_street();
    while (detectedRed == 0)
    {
      turn_at_intersection(0);

      // TODO: Update belief to turn right
      if (netDir == 3)
      {
        netDir = 0;
      }
      else
      {
        netDir++;
      }

      detectedRed = drive_along_street();
    }

    // Scan intersection
    int tl;
    int tr;
    int br;
    int bl;
    scan_intersection(&tl, &tr, &br, &bl);

    // Update beliefs
  }

  return (0);
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
  return (0);
}

int scanColour(int n)
{
  //   BLACKCOLOR   = 1,
  //   BLUECOLOR    = 2,
  //   GREENCOLOR   = 3,
  //   YELLOWCOLOR  = 4,
  //   REDCOLOR     = 5,
  //   WHITECOLOR   = 6
  int colour = -1;
  double *avg_reading = (double *)calloc(3, sizeof(double));
  getColourReading(n, avg_reading);
  // Find which colour we are closest to
  int minSquaredError = INT_MAX;
  int minSquaredErrorIndex = -1;
  for (int i = 0; i < 6; i++)
  {
    int squaredError = 0;
    for (int rgbIndex = 0; rgbIndex < 3; rgbIndex++)
    {
      squaredError += pow(calibrated_colours[i][rgbIndex] - avg_reading[rgbIndex], 2);
    }
    if (squaredError < minSquaredError)
    {
      minSquaredError = squaredError;
      minSquaredErrorIndex = i;
    }
  }
  // if (minSquaredError > 2000){
  //   return pastColour;
  // }
  // shift it by one since indexing starts at 0, but blackcolour starts at 1
  colour = minSquaredErrorIndex + 1;
  currentMinSquareError = minSquaredError;

  // TODO: If we detect green, get turn_at_intersection to keep turning at a slower rate until it hits black OR
  // if we use the pivoting on 1 wheel method for turning, just make sure that black is detected for a certain

  // TODO: test every edge combination with black (especially the edge between green and black since we need
  // to account for the case where we detect green while turning at a green square)
  free(avg_reading);
  printf("The sensor is detecting: %s\n", colours[colour - 1]);
  pastColour = colour;
  return colour;
}

// Scan n times, average the readings, and place in array whose pointer is provided
void getColourReading(int n, double *avg_reading)
{
  int *total_readings = (int *)calloc(3, sizeof(int));
  int counter = 0;
  int *current_reading = (int *)calloc(3, sizeof(int));
  while (counter < n)
  {
    // get and store a scan
    BT_read_colour_sensor_RGB(PORT_1, current_reading);
    // check validity of scan
    int valid = 1;
    for (int i = 0; i < 3; i++)
    {
      if (current_reading[i] < 0 || current_reading[i] > 1020)
      {
        // one of the r, g, b values are incorrect
        valid = 0;
      }
    }
    if (valid)
    {
      // add colour
      for (int i = 0; i < 3; i++)
      {
        total_readings[i] += current_reading[i];
      }
      counter++;
    }
  }
  // store reading in given pointer
  for (int i = 0; i < 3; i++)
  {
    avg_reading[i] = total_readings[i] / ((double)n);
  }
  free(current_reading);
  free(total_readings);
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
  fprintf(stderr, "Calibration function called!\n");
  // reset file if it exists
  fclose(fopen("calibration.txt", "w"));
  // open file in append mode
  FILE *fp = fopen("calibration.txt", "a");
  if (fp == NULL)
  {
    perror("cannot open calibration file for reading");
    exit(1);
  }
  for (int i = 0; i < 6; i++)
  {
    printf("Place sensor over %s, then press ENTER", colours[i]); // Get specific colour
    getchar();
    double *avg_reading = (double *)calloc(3, sizeof(double));
    getColourReading(CALIBRATION_COUNT, avg_reading);
    // store reading in file
    fprintf(fp, "%f,%f,%f\n", avg_reading[0], avg_reading[1], avg_reading[2]);
    free(avg_reading);
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

     Colour values for map locations are defined as follows (this agrees with what the
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
  int x, y;
  unsigned char R, G, B;
  int ix, iy;
  int bx, by, dx, dy, wx, wy; // Intersection geometry parameters
  int tgl;
  int idx;

  ix = iy = 0; // Index to identify the current intersection

  // Determine the spacing and size of intersections in the map
  tgl = 0;
  for (int i = 0; i < rx; i++)
  {
    for (int j = 0; j < ry; j++)
    {
      R = *(map_img + ((i + (j * rx)) * 3));
      G = *(map_img + ((i + (j * rx)) * 3) + 1);
      B = *(map_img + ((i + (j * rx)) * 3) + 2);
      if (R == 255 && G == 255 && B == 0)
      {
        // First intersection, top-left pixel. Scan right to find width and spacing
        bx = i; // Anchor for intersection locations
        by = j;
        for (int k = i; k < rx; k++) // Find width and horizontal distance to next intersection
        {
          R = *(map_img + ((k + (by * rx)) * 3));
          G = *(map_img + ((k + (by * rx)) * 3) + 1);
          B = *(map_img + ((k + (by * rx)) * 3) + 2);
          if (tgl == 0 && (R != 255 || G != 255 || B != 0))
          {
            tgl = 1;
            wx = k - i;
          }
          if (tgl == 1 && R == 255 && G == 255 && B == 0)
          {
            tgl = 2;
            dx = k - i;
          }
        }
        for (int k = j; k < ry; k++) // Find height and vertical distance to next intersection
        {
          R = *(map_img + ((bx + (k * rx)) * 3));
          G = *(map_img + ((bx + (k * rx)) * 3) + 1);
          B = *(map_img + ((bx + (k * rx)) * 3) + 2);
          if (tgl == 2 && (R != 255 || G != 255 || B != 0))
          {
            tgl = 3;
            wy = k - j;
          }
          if (tgl == 3 && R == 255 && G == 255 && B == 0)
          {
            tgl = 4;
            dy = k - j;
          }
        }

        if (tgl != 4)
        {
          fprintf(stderr, "Unable to determine intersection geometry!\n");
          return (0);
        }
        else
          break;
      }
    }
    if (tgl == 4)
      break;
  }
  fprintf(stderr, "Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n", bx, by, wx, wy, dx, dy);

  sx = 0;
  for (int i = bx + (wx / 2); i < rx; i += dx)
  {
    R = *(map_img + ((i + (by * rx)) * 3));
    G = *(map_img + ((i + (by * rx)) * 3) + 1);
    B = *(map_img + ((i + (by * rx)) * 3) + 2);
    if (R == 255 && G == 255 && B == 0)
      sx++;
  }

  sy = 0;
  for (int j = by + (wy / 2); j < ry; j += dy)
  {
    R = *(map_img + ((bx + (j * rx)) * 3));
    G = *(map_img + ((bx + (j * rx)) * 3) + 1);
    B = *(map_img + ((bx + (j * rx)) * 3) + 2);
    if (R == 255 && G == 255 && B == 0)
      sy++;
  }

  fprintf(stderr, "Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n", sx, sy);

  // Scan for building colours around each intersection
  idx = 0;
  for (int j = 0; j < sy; j++)
    for (int i = 0; i < sx; i++)
    {
      x = bx + (i * dx) + (wx / 2);
      y = by + (j * dy) + (wy / 2);

      fprintf(stderr, "Intersection location: %d, %d\n", x, y);
      // Top-left
      x -= wx;
      y -= wy;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][0] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][0] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][0] = 6;
      else
        fprintf(stderr, "Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n", i, j, R, G, B);

      // Top-right
      x += 2 * wx;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][1] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][1] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][1] = 6;
      else
        fprintf(stderr, "Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n", i, j, R, G, B);

      // Bottom-right
      y += 2 * wy;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][2] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][2] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][2] = 6;
      else
        fprintf(stderr, "Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n", i, j, R, G, B);

      // Bottom-left
      x -= 2 * wx;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][3] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][3] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][3] = 6;
      else
        fprintf(stderr, "Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n", i, j, R, G, B);

      fprintf(stderr, "Colours for this intersection: %d, %d, %d, %d\n", map[idx][0], map[idx][1], map[idx][2], map[idx][3]);

      idx++;
    }

  return (1);
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

  im = NULL;
  f = fopen(filename, "rb+");
  if (f == NULL)
  {
    fprintf(stderr, "Unable to open file %s for reading, please check name and path\n", filename);
    return (NULL);
  }
  fgets(&line[0], 1000, f);
  if (strcmp(&line[0], "P6\n") != 0)
  {
    fprintf(stderr, "Wrong file format, not a .ppm file or header end-of-line characters missing\n");
    fclose(f);
    return (NULL);
  }
  fprintf(stderr, "%s\n", line);
  // Skip over comments
  fgets(&line[0], 511, f);
  while (line[0] == '#')
  {
    fprintf(stderr, "%s", line);
    fgets(&line[0], 511, f);
  }
  sscanf(&line[0], "%d %d\n", rx, ry); // Read image size
  fprintf(stderr, "nx=%d, ny=%d\n\n", *rx, *ry);

  fgets(&line[0], 9, f); // Read the remaining header line
  fprintf(stderr, "%s\n", line);
  im = (unsigned char *)calloc((*rx) * (*ry) * 3, sizeof(unsigned char));
  if (im == NULL)
  {
    fprintf(stderr, "Out of memory allocating space for image\n");
    fclose(f);
    return (NULL);
  }
  fread(im, (*rx) * (*ry) * 3 * sizeof(unsigned char), 1, f);
  fclose(f);

  return (im);
}