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
double POS_X_Range = -1;
double POS_Y_Range = -1;
double VEL_X_Range = 2;
double VEL_Y_Range = -1;
double ANGLE_Range = -1;
double SONAR_Range = -1;

// SENSOR DATA
int DATA_SIZE = 10;
deque<double> POS_X_DATA;
deque<double> POS_Y_DATA;
deque<double> VEL_X_DATA;
deque<double> VEL_Y_DATA;
deque<double> ANGLE_DATA;
deque<double> SONAR_DATA[36];

// CONTROL DATA
// NOTE: since angle and thruster commands are mutually exclusive, fill in -1 for thruster data when changing angle and vice versa
double MT_DATA; // [0 ,1]
double LT_DATA; // [0 ,1]
double RT_DATA; // [0 ,1]
double ROTATE_DATA; // 0 if not rotating, or howewer much is left to rotate (in degrees) in which case all thrusters are 0

// DENOISING VECTORS
double WEIGHT_ARR [10];
double SD = 1;

// HELPERS


// SIMULATE CURRENT VALUES USING PAST TICK VALUE AND COMMAND GIVEN
double* Simulate(double* address) {
  // DON'T CALL Simulate ON FIRST ITERATION as we have no previous data
  double PPX = POS_X_DATA[0];
  double PPY = POS_Y_DATA[0];
  double PVX = VEL_X_DATA[0];
  double PVY = VEL_Y_DATA[0];
  double PA = ANGLE_DATA[0];
  double AX = MT_ACCEL*MT_DATA*sin(PA) + (LT_ACCEL*LT_DATA - RT_ACCEL*RT_DATA)*cos(PA); // 0 if in rotation
  double AY = (G_ACCEL - MT_ACCEL*MT_DATA*cos(PA)) + (LT_ACCEL*LT_DATA - RT_ACCEL*RT_DATA)*sin(PA); // G if in rotation
  double DX = PVX*T_STEP + 0.5*AX*T_STEP*T_STEP;
  double DY = PVY*T_STEP + 0.5*AY*T_STEP*T_STEP;
  double CPX = PPX + DX;
  double CPY = PPY + DY;
  double CVX = PVX + AX*T_STEP;
  double CVY = PVY + AY*T_STEP;
  double CA = PA + ROTATE_DATA; // Need to account for max rotate rate
  double max_rotate_degrees = MAX_ROT_RATE*180/PI;
  int sign = (ROTATE_DATA > 0) - (ROTATE_DATA < 0); // 1 if positive, -1 if negative, 0 if 0
  if (abs(ROTATE_DATA) > max_rotate_degrees) {
    CA = PA + sign*max_rotate_degrees;
  }
  address[0] = CPX;
  address[1] = CPY;
  address[2] = CVX;
  address[3] = CVY;
  address[4] = CA;
  return address;
}

// GET CURRENT READING EXCEPT SONAR
double* Get_Current_Readings(double* address) {
  /*
      Given an array (pointer), fill it with sensor readings
      WARNING: initialize array "address" with the right size (6)
  */
  address[0] = Position_X();
  address[1] = Position_Y();
  address[2] = Velocity_X();
  address[3] = Velocity_Y();
  address[4] = Angle();
  return address;
}

// WEIGHTED ARRAY CREATION
double normalCDF(double value)
{
   return 0.5 * erfc(-value * sqrt(0.5));
}

void createWeightedArr(void)
{
  for (int i = 0; i < 10; i++)
  {
    WEIGHT_ARR[i] = normalCDF(-i/SD);
    cout << WEIGHT_ARR[i] << "\n";
  }
  
}

// FAULT DETECTION
void Update_Sensor_Status(void) {
  /*
      Update global flags of sensors statuses based on current reading and previous reading (prior Tick) stored in the global array
  */
  double Current_Readings[6];
  double* Current = Get_Current_Readings(Current_Readings);
  if (POS_X_OK)
  {
    if (abs(Current[0] - POS_X_DATA[0]) > POS_X_Range) 
    {
      POS_X_OK = 0;
    }
  }
  if (POS_Y_OK)
  {
    if (abs(Current[1] - POS_Y_DATA[0]) > POS_Y_Range) 
    {
      POS_Y_OK = 0;
    }
  }
  if (VEL_X_OK)
  {
    if (abs(Current[2] - VEL_X_DATA[0]) > VEL_X_Range) 
    {
      VEL_X_OK = 0;
    }
  }
  if (VEL_Y_OK)
  {
    if (abs(Current[3] - VEL_Y_DATA[0]) > VEL_Y_Range) 
    {
      VEL_Y_OK = 0;
    }
  }
  if (ANGLE_OK)
  {
    if (abs(Current[4] - ANGLE_DATA[0]) > ANGLE_Range) 
    {
      ANGLE_OK = 0;
    }
  }
  // FIGURE SONAR STATUS
  return;
}

// DENOISING
double Denoise(deque <double> DATA) {
  int DATA_LIST_SIZE = DATA.size();
  double AVERAGE = 0;
  for (int i = 0; i < DATA_LIST_SIZE; i++)
  {
    AVERAGE += DATA.at(i) * WEIGHT_ARR[i];
  }
  
  return AVERAGE;
}

// ROBUST READINGS
double Robust_Position_X(double* simulation) {
  if (POS_X_OK) {
    return Denoise(POS_X_DATA);
  }
  if (VEL_X_OK) {
    return POS_X_DATA[0] + (T_STEP * VEL_X_DATA[0]);
  }
  return simulation[0];
}

double Robust_Position_Y(double* simulation) {
  if (POS_Y_OK) {
    return Denoise(POS_Y_DATA);
  }
  if (VEL_Y_OK) {
    return POS_Y_DATA[0] + (T_STEP * VEL_Y_DATA[0]);
  }
  return simulation[1];
}

double Robust_Velocity_X(double* simulation) {
  if (VEL_X_OK) {
    return Denoise(VEL_X_DATA);
  }
  if (POS_X_OK) {
    return (Position_X() - POS_X_DATA[0])/T_STEP;
  }
  return simulation[2];
}

double Robust_Velocity_Y(double* simulation) {
  if (VEL_Y_OK) {
    return Denoise(VEL_Y_DATA);
  }
  if (POS_Y_OK) {
    return (Position_Y() - POS_Y_DATA[0])/T_STEP;
  }
  return simulation[3];
}

double Robust_Angle(double* simulation) {
  if (ANGLE_OK) {
    return Denoise(ANGLE_DATA);
  }
  return simulation[4];
}
// DATA UPDATING
void Update(void) {
  // Generate Simulation
  
  if (COUNTER == 0)
  {
    // Create weight array for Gaussian averaging
    createWeightedArr();

    // No need for simulation or sensor status update, all sensors are reliable
    POS_X_DATA.push_front(Position_X());
    POS_Y_DATA.push_front(Position_Y());
    VEL_X_DATA.push_front(Velocity_X());
    VEL_Y_DATA.push_front(Velocity_Y());
    ANGLE_DATA.push_front(Angle());
    return;
  }
  Update_Sensor_Status();
  double simulate[6];
  double* simulation = Simulate(simulate);
  POS_X_DATA.push_front(Robust_Position_X(simulate));
  if (POS_X_DATA.size() == DATA_SIZE + 1){
    POS_X_DATA.pop_back();
  }
  POS_Y_DATA.push_front(Robust_Position_Y(simulate));
  if (POS_Y_DATA.size() == DATA_SIZE + 1){
    POS_Y_DATA.pop_back();
  }
  VEL_X_DATA.push_front(Robust_Velocity_X(simulate));
  if (VEL_X_DATA.size() == DATA_SIZE + 1){
    VEL_X_DATA.pop_back();
  }
  VEL_Y_DATA.push_front(Robust_Velocity_Y(simulate));
  if (VEL_Y_DATA.size() == DATA_SIZE + 1){
    VEL_Y_DATA.pop_back();
  }
  ANGLE_DATA.push_front(Robust_Angle(simulate));
  if (ANGLE_DATA.size() == DATA_SIZE + 1){
    ANGLE_DATA.pop_back();
  }
  /* Wait for robust sonar data to be finished
  SONAR_DATA.push_front(SONAR_DIST[]);
  if (SONAR_DATA.size() == DATA_SIZE + 1){
    SONAR_DATA.pop_back();
  }
  */
}

void Lander_Control(void) {
  // call the sensor status and update sensors
  // update the data based on robust calls
  // use global data arrays[-1] to decide where to go
  // store comands given to global (keep all thruster commands between [0.1, 0.9] and keep exclusive from rotation in which case thuster power should be 0)
  // add 1 to Tick counter at the very end of the code
  Update();
  COUNTER++;
 double VXlim;
 double VYlim;

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
  
 // Call Sensor_Status() then Update_Data_Lists() here
  
 if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
 else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-Position_Y()>200) VYlim=-20;
 else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) VYlim=0;

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

 if (Angle()>1&&Angle()<359)
 {
  if (Angle()>=180) Rotate(360-Angle());
  else Rotate(-Angle());
  return;
 }

 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (Position_X()>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (Velocity_X()>(-VXlim)) Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   Left_Thruster(fabs(VXlim-Velocity_X()));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (Velocity_X()<VXlim) Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
  else
  {
   Left_Thruster(0);
   Right_Thruster(fabs(VXlim-Velocity_X()));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (Velocity_Y()<VYlim) Main_Thruster(1.0);
 else Main_Thruster(0);
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
 Vmag=Velocity_X()*Velocity_X();
 Vmag+=Velocity_Y()*Velocity_Y();

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 if (Velocity_X()>0)
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
 if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X())/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }

  if (Velocity_X()>0){
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
 if (Velocity_Y()>5)      // Mind this! there is a reason for it...
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
  if (Velocity_Y()>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   Main_Thruster(1.0);
  }
 }
}
