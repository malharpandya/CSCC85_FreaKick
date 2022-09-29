/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/

#include <math.h>
#include <queue>
#include <iostream>

#include "Lander_Control.h"
#include <cstdio>


using namespace std;

// TICK COUNTER
double COUNTER = 0;
// BOOLEAN FLAGS
bool POS_X_OK = 1;
bool POS_Y_OK = 1;
bool VEL_X_OK = 1;
bool VEL_Y_OK = 1;
bool ANGLE_OK = 1;
bool SONAR_OK = 1;

// SENSOR RANGE OF RELIABILITY *JACKSON*
double POS_X_Range = 70; // 100 300+ once it fails
double POS_Y_Range = 70; // 100 300+ once it fails 
double VEL_X_Range = 2;
double VEL_Y_Range = 2;
double ANGLE_Range = 10; // 50+ once it fails
double SONAR_Range = 1000;

// SENSOR DATA
double POS_X;
double POS_Y;
double VEL_X;
double VEL_Y;
double ANGLE;
double SONAR[36];

// CONTROL DATA
// NOTE: since angle and thruster commands are mutually exclusive, fill in -1 for thruster data when changing angle and vice versa
double MT_COMMAND; // [0 ,1]
double LT_COMMAND; // [0 ,1]
double RT_COMMAND; // [0 ,1]
double ROTATE_COMMAND; // 0 if not rotating, or howewer much is left to rotate (in degrees) in which case all thrusters are 0
int stage = 1;

// CONSISTENCY AND NOISE REFINEMENT
int SENSOR_COUNT = 100000;
double VARIANCE_THRESHOLD = 0.05;
double T = 0.025;

double TestT = 0.025;
// 02385
// HELPERS


// SIMULATE CURRENT VALUES USING PAST TICK VALUE AND COMMAND GIVEN
void Simulate(double* address) {
  // DON'T CALL Simulate ON FIRST ITERATION as we have no previous data
  double PPX = POS_X;
  double PPY = POS_Y;
  double PVX = VEL_X;
  double PVY = VEL_Y;
  double PA = ANGLE;
  double AX = MT_ACCEL*MT_COMMAND*sin(PA) + (LT_ACCEL*LT_COMMAND - RT_ACCEL*RT_COMMAND)*cos(PA); // 0 if in rotation
  double AY = (G_ACCEL - MT_ACCEL*MT_COMMAND*cos(PA)) + (LT_ACCEL*LT_COMMAND - RT_ACCEL*RT_COMMAND)*sin(PA); // G if in rotation
  double DX = PVX*T + 0.5*AX*TestT*TestT;
  double DY = PVY*T + 0.5*AY*TestT*TestT;
  double CPX = PPX + DX;
  double CPY = PPY - DY;
  double CVX = PVX + AX*TestT;
  double CVY = PVY - AY*TestT;
  double CA = PA + ROTATE_COMMAND; // Need to account for max rotate rate
  double max_rotate_degrees = MAX_ROT_RATE*180/PI;
  int sign = (ROTATE_COMMAND > 0) - (ROTATE_COMMAND < 0); // 1 if positive, -1 if negative, 0 if 0
  if (abs(ROTATE_COMMAND) > max_rotate_degrees) {
    CA = PA + sign*max_rotate_degrees;
  }
  address[0] = CPX;
  address[1] = CPY;
  address[2] = CVX;
  address[3] = CVY;
  address[4] = CA;
}

// SENSOR CONSISTENCY AND DENOISING
// Detect sensor failure and put denoised value into global variable if not faulty
bool Sensor_Update(bool *SENSOR_STATUS, double (*Sensor_Call)(void), double *SENSOR) {
  if (SENSOR_STATUS == 0) {return 0;}
  
  // Call sensor multiple times and store it in an array
  double sensor_readings[SENSOR_COUNT];
  double sensor_reading_total = 0;
  double max = -1;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensor_readings[i] = Sensor_Call();
    sensor_reading_total += sensor_readings[i];
    if (abs(sensor_readings[i]) > max) {
      max = abs(sensor_readings[i]);
    }
  }
  // Calculate the mean of the sensor
  double mean = sensor_reading_total / SENSOR_COUNT;
  // Calculate the variance (normalize the readings first)
  double variance = 0;
  for (int i=0; i < SENSOR_COUNT; i++) {
    variance += pow((sensor_readings[i] - mean)/max, 2);
  }
  variance = variance / SENSOR_COUNT;
  // cout << "Variance: " << variance << "\n";
  // Check if sensor is faulty
  if (variance >= VARIANCE_THRESHOLD) { // TODO make sure if 0 status don't just it a chance to be made correct
    *SENSOR_STATUS = 0;
    //cout << "fail\n";
    return 0; // sensor fail
  }
  *SENSOR = mean;
  return 1; // denoised value updated to global variable
}
// Detect angle sensor failure and return denoised value if not faulty
bool Angle_Update(void)
{
  if (!ANGLE_OK)
  {
    //cout << "*************************************************************";
    return 0;
  }

  double angle_readings[SENSOR_COUNT];
  double angle_reading_total = 0;
  double max = -500;
  double min = 500;
  double ANGLE_CURR;
  bool HAS_EDGE = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    ANGLE_CURR = Angle();
    angle_readings[i] = ANGLE_CURR;

    if (!HAS_EDGE && (ANGLE_CURR >= 350 || ANGLE_CURR < 0))
    {
      HAS_EDGE = 1;
    }

    //cout << "Angle before conversion: " << angle_readings[i] << "\n";
  }
  
  // CODE THAT MODIFIES THE ANGLES
  if (HAS_EDGE)
  {
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      angle_readings[i] = fmod(angle_readings[i] + 90, 360);
      //cout << "Angle after conversion: " << angle_readings[i] << "\n";
    }
  }
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    angle_reading_total += angle_readings[i];
    if (angle_readings[i] > max) {
      max = angle_readings[i];
    }
    if (angle_readings[i] < min)
    {
      min = angle_readings[i];
    }
    
  }

  // Calculate the mean of all angles
  double mean = angle_reading_total/SENSOR_COUNT;

  // Check if sensor is faulty
  if (max - min > 3.5)
  {
    ANGLE_OK = 0;
    return 0;
  }

  mean -= HAS_EDGE * 90;
  if (mean < 0)
  {
    mean += 360;
  }
  //cout << "Difference: " << max - min << "\n";
  //cout << "Angle returned: " << mean << "\n";
  ANGLE = mean;
  return 1;
}

deque<double> OLD_POS_X_DATA;

// DATA UPDATING
void Update(void) {
  // Generate simulation first before you update global variable with current denoised sensor reading
  // DONT RUN FOR FIRST TICK AS THERE IS NO PREVIOUS DATA/CONTROL COMMANDS GIVEN
  double simulated_values[6];
  if (COUNTER > 0) {
    Simulate(simulated_values);
  }

  double PXP = POS_X;
  double PYP = POS_Y;
  double VXP = VEL_X;
  double VYP = VEL_Y;
  double AP = ANGLE;
  // WE ASSUME FIRST BUNCH OF TICKS DO NOT CAUSE ANY SENSOR FAILURES
  // cout << "Calling Sensor Updates\n";

  if (!Sensor_Update(&POS_X_OK, &Position_X, &POS_X)) {
    //cout << "Position X sensor faulty\n";
    //cout << "actual position x:" << POS_X << " estimated position X: " << PXP +  T * VXP << " Difference: " << abs((PXP +  T * VXP)-POS_X)<<"\n";
    if (VEL_X_OK) {
      //cout << "update\n";
      POS_X = PXP +  T * VXP;
    } else {
      POS_X = simulated_values[0];
    }
  }
  
  OLD_POS_X_DATA.push_front(POS_X);
  if (OLD_POS_X_DATA.size() == 3 + 1){ OLD_POS_X_DATA.pop_back(); }


  if (!Sensor_Update(&POS_Y_OK, &Position_Y, &POS_Y)) {
    //cout << "Position Y sensor faulty\n";
    if (VEL_Y_OK) {
      POS_Y -=  T * VYP;
    } else {
      POS_Y = simulated_values[1];
    }
  }
  if (!Sensor_Update(&VEL_X_OK, &Velocity_X, &VEL_X)) {
    // cout << "Velocity X sensor faulty\n";
    // cout << POS_X << "\n";
    // cout << POS_X << " ; " << PXP << "\n";
    // cout << "actual Velocity_X: " << VEL_X << " estimated Vel X: " << (POS_X - OLD_POS_X_DATA.back())<< " Difference: " << (VEL_X)/(POS_X - OLD_POS_X_DATA.back()) << "\n";
    // cout << "actual Velocity_X: " << VEL_X << " estimated Vel X: " << (POS_X - PXP)/T<< "\n";
    if (POS_X_OK) {
      // cout << "sanity check\n";
      VEL_X =  (POS_X - PXP)/T;
    } else {
      VEL_X = simulated_values[2];
    }
  }
  if (!Sensor_Update(&VEL_Y_OK, &Velocity_Y, &VEL_Y)) {
    //cout << "Velocity Y sensor faulty\n";
    if (POS_Y_OK) {
      VEL_Y =  -(POS_Y - PYP)/T;
    } else {
      VEL_Y = simulated_values[3];
    }
  }
  if (!Angle_Update()) {
    ANGLE = simulated_values[4];
  }

}

// Flight Control
int Flight_Mode(void)
{
  // All thrusters nominal
  if (MT_OK && RT_OK && LT_OK)
  {
    return 0;
  }
  
  // 2 thrusters disabled
  // Main thruster nominal
  if (!LT_OK && !RT_OK)
  {
    return 1;
  }

  // Left thruster nominal
  if (!MT_OK && !RT_OK)
  {
    return 2;
  }

  // Right thruster nominal
  if (!MT_OK && !LT_OK)
  {
    return 3;
  }

  // 1 thruster disabled, 1 sensor disabled
  // Angle sensor works
  if (ANGLE_OK)
  {
    // Use same flight mode as 2 thrusters disabled case, and choose new main thruster accordingly (prefer main thruster)
    if (MT_OK)
    {
      return 1;
    }

    if (LT_OK)
    {
      return 2;
    }

    if (RT_OK)
    {
      return 3;
    }
  }
  
  // Angle sensor doesn't work
  // Choose new main thruster
  if (MT_OK)
  {
    return 1;
  }

  if (LT_OK)
  {
    return 2;
  }

  if (RT_OK)
  {
    return 3;
  }

  return -1;
}

void Turn_Burn(int THRUSTER, int DIR)
{
  // THRUSTER: 1 = MAIN, 2 = LEFT, 3 = RIGHT
  // DIR: 1 = DOWN, 2 = LEFT, 3 = RIGHT, 4 = UP

  // Determine the offset so the when the new main thruster is pointed down, the current angle is 0
  double ANGLE_OFFSET = 0;
  double POWER = 0.5;
  bool SIDETHRUSTER = 1;

  if (stage < 4)
  {
    POWER = 1;
  }
  
  if (THRUSTER == 2)
  {
    ANGLE_OFFSET = 90;
  }
  else if (THRUSTER == 3)
  {
    ANGLE_OFFSET = -90;
  }
  else
  {
    SIDETHRUSTER = 0;
  }
  double CURRENT_ANGLE = fmod(ANGLE + ANGLE_OFFSET, 360);
  if (CURRENT_ANGLE < 0)
  {
    CURRENT_ANGLE += 360;
  }

  // Determine target angle based on desired direction
  double TARGET_ANGLE = 0;
  
  if (DIR == 2)
  {
    if (SIDETHRUSTER)
    {
      TARGET_ANGLE = 315.2;
    }
    else
    {
      TARGET_ANGLE = 300.45;
    }
  }
  else if (DIR == 3)
  {
    if (SIDETHRUSTER)
    {
      TARGET_ANGLE = 44.8;
    }
    else
    {
      TARGET_ANGLE = 59.55;
    }
  }

  // If the current angle is reasonably close to the target angle and the direction we want to go is not DOWN
  if (CURRENT_ANGLE <= TARGET_ANGLE + 0.5 && CURRENT_ANGLE >= TARGET_ANGLE - 0.5 && DIR != 1)
  {
    // Thrust since angle is good (don't rotate this tick)
    if (THRUSTER == 1)
    {
      Main_Thruster(POWER);
    }
    else if (THRUSTER == 2)
    {
      Left_Thruster(POWER); 
    }
    else
    {
      Right_Thruster(POWER);
    }
    cout << "Thrust\n";
  }
  else
  {
    // Stop thrusters
    Main_Thruster(0);
    Left_Thruster(0);
    Right_Thruster(0);

    // Rotate to target angle (don't use thrust this tick)    
    double ROT_DEG_SIGNED = TARGET_ANGLE - CURRENT_ANGLE;
    if (ROT_DEG_SIGNED > 180)
    {
      ROT_DEG_SIGNED -= 360;
    }
    else if (ROT_DEG_SIGNED < -180)
    {
      ROT_DEG_SIGNED += 360;
    }
    
    cout << "ROT_DEG_SIGNED: " << ROT_DEG_SIGNED << "\n";
    cout << "TARGET_ANGLE: " << TARGET_ANGLE << "\n";
    Rotate(ROT_DEG_SIGNED);
    ROTATE_COMMAND = ROT_DEG_SIGNED;
  }
} 

void Flight_Control(double Desired_Vel_X, double Desired_Vel_Y, bool Upright)
{
  cout << "Desired_Vel_X: " << Desired_Vel_X << " Desired_Vel_Y: " << Desired_Vel_Y << "\n";
  cout << "VEL_X: " << VEL_X << " VEL__Y: " << VEL_Y << "\n";
  if (Upright)
  {
    // Stop thrusters
    Main_Thruster(0);
    Left_Thruster(0);
    Right_Thruster(0);

    double ROT_DEG_SIGNED = 0 - ANGLE;
    if (ROT_DEG_SIGNED > 180)
    {
      ROT_DEG_SIGNED -= 360;
    }
    else if (ROT_DEG_SIGNED < -180)
    {
      ROT_DEG_SIGNED += 360;
    }
    cout << "Uprighting \n";
    Rotate(ROT_DEG_SIGNED);
    ROTATE_COMMAND = ROT_DEG_SIGNED;
    return;
  }
  
  int FLIGHT_MODE = Flight_Mode();
  int DIR;
  int THRUSTER = FLIGHT_MODE; // NOTE: for now, THRUSTER = FLIGHT_MODE because flight mode numbering corresponds to THRUSTER numbering in Turn_Burn()
  if (FLIGHT_MODE == 0)
  {
    return;
  }
  else if (FLIGHT_MODE == 3)
  {
    double Vel_Y_Diff = Desired_Vel_Y - VEL_Y;
    double Vel_X_Diff = Desired_Vel_X - VEL_X;
    if (Vel_X_Diff >= 1)
    {
      // Desired direction is right
      DIR = 3;
    }
    else if (Vel_X_Diff <= -0.5)
    {
      // Desired direction is left
      DIR = 2;
    }
    else if (Vel_Y_Diff >= 0.5)
    {
      // Desired direction is up
      DIR = 4;
    }
    else if (Vel_Y_Diff < 0)
    {
      // Desired direction is down
      DIR = 1;
    }
  }

  //cout << "FLIGHT MODE: " << FLIGHT_MODE << "\n";
  
  if (FLIGHT_MODE != 0)
  {
    cout << "THRUSTER: " << THRUSTER << " DIR: " << DIR << "\n";
    Turn_Burn(THRUSTER, DIR);
  }
  
}

void Lander_Control(void) {
  // call the sensor status and update sensors
  // update the data based on robust calls
  // use global data arrays[-1] to decide where to go
  // store comands given to global (keep all thruster commands between [0.1, 0.9] and keep exclusive from rotation in which case thuster power should be 0)
  // add 1 to Tick counter at the very end of the code

 double VXlim;
 double VYlim;

 //cout << POS_Y << "\n";

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
  
 // Call Sensor_Status() then Update_Data_Lists() here
 Update();
 COUNTER ++;
  
 if (fabs(POS_X-PLAT_X)>200) VXlim=25;
 else if (fabs(POS_X-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-POS_Y>200) VYlim=-5;
 else if (PLAT_Y-POS_Y>150) VYlim=-3;  // These are negative because they
 else if (PLAT_Y-POS_Y>50) VYlim=-2;
 else VYlim=-0.1;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-POS_X)/fabs(VEL_X)>1.25*fabs(PLAT_Y-POS_Y)/fabs(VEL_Y)) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Rotate first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Rotate() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.

 double distToDesiredDestination;

 if (stage == 1) {
  cout << "STAGE: " << stage << "\n";
  if (Angle()>1&&Angle()<359)
  {
    // Call Flight_Control to upright lander
    Flight_Control(0.0, 0.0, true);
    // if (Angle()>=180) Rotate(360-Angle());
    // else Rotate(-Angle());
    // return;
  }else{
    stage++;
  }
 }

 if (stage == 2) {
  cout << "STAGE: " << stage << "\n";
  // Ceilling of 55 will clear everything
  // accend
  //   POS_Y
  
  if (POS_Y > 55) {
    Flight_Control(0.0, 10.0, false);
  } else {
    stage++;
  }
 }

 if (stage == 3) {
    cout << "STAGE: " << stage << "\n";
    if (VEL_Y > 0.1) {
        Flight_Control(0.0, -5, false);
    } else {
        stage++;
    }
 }

//platform landing buffer of 5

 if (stage == 4) {
    cout << "STAGE: " << stage << "\n";
    double acceleration = 8.77;
    double thruster_turning_angle = 45.2 * PI / 180; // 30.45 for main thruster
    double alpha = ANGLE_OK ? 1 : 2;
    double turning_buffer =  thruster_turning_angle / 0.075 * VEL_X * alpha;
    double critical_distance = pow(VEL_X,2.0) / (2 * acceleration) + turning_buffer;

    if (abs(POS_X - PLAT_X) > critical_distance) {
        if ((POS_X - PLAT_X) > 20)
        {
          Flight_Control(-20.0, 0.0, false);
        }
        else if ((POS_X - PLAT_X) < -20)
        {
          Flight_Control(20.0, 0.0, false);
        }
        else
        {
          Flight_Control(0.0, 0.0, false);
        }
        
        //Flight_Control(((POS_X - PLAT_X) < 0) ? 20 : -20, 0.0, false);
    } else {
        Flight_Control(0.0,0.0, false);
    }
    if (abs(POS_X - PLAT_X) < 20 && VEL_X < 3 && VEL_X > -3) { // Over platform
        cout << "LAST PART OF STAGE 4: " << "\n";
        Flight_Control(0.0, VYlim, false);
    }
    cout << abs(POS_Y - PLAT_Y) << "\n";
    if (abs(POS_Y - PLAT_Y) <= 40)
    {
      stage++;
    }
 }

 if (stage == 5) {
    cout << "STAGE: " << stage << "\n";
    Flight_Control(0.0,-2, true);
 }

/*
 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (POS_X>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (VEL_X>(-VXlim)) Right_Thruster((VXlim+fmin(0,VEL_X))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   Left_Thruster(fabs(VXlim-VEL_X));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (VEL_X<VXlim) Left_Thruster((VXlim-fmax(0,VEL_X))/VXlim);
  else
  {
   Left_Thruster(0);
   Right_Thruster(fabs(VXlim-VEL_X));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (VEL_Y<VYlim) Main_Thruster(1.0);
 else Main_Thruster(0);*/
}

void Safety_Override(void) {
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/

 double DistLimit;
 double Vmag;
 double dmin;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=VEL_X*VEL_X;
 Vmag+=VEL_Y*VEL_Y;

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-POS_X)<150&&fabs(PLAT_Y-POS_Y)<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 /*
 dmin=1000000;
 if (VEL_X>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 if (dmin<DistLimit*fmax(.25,fmin(fabs(VEL_X)/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }

  if (VEL_X>0){
   Right_Thruster(1.0);
   Left_Thruster(0.0);
  }
  else
  {
   Left_Thruster(1.0);
   Right_Thruster(0.0);
  }
 }

 // Vertical direction
 dmin=1000000;
 if (VEL_Y>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
  if (VEL_Y>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   Main_Thruster(1.0);
  }
 }*/
}
