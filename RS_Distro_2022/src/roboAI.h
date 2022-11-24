/***************************************************
 CSC C85 - UTSC RoboSoccer AI core
 
 This file contains the definition of the AI data
 structure which holds the state of your bot's AI.

 You must become familiar with this structure and
 its contents. 

 You will need to modify this file to add headers
 for any functions you added to implemet the 
 soccer playing functionality of your bot.

 Be sure to document everything you do thoroughly.

 AI scaffold: Parker-Lee-Estrada, Summer 2013
 Updated by F. Estrada, Jul. 2022

***************************************************/

#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"
#include "API/btcomm.h"
#include <stdio.h>
#include <stdlib.h>

// Change this to match the ports your bots motors are connected to
#define LEFT_MOTOR MOTOR_D
#define RIGHT_MOTOR MOTOR_A

#define AI_SOCCER 0 	// Play soccer!
#define AI_PENALTY 1    // Go score some goals!
#define AI_CHASE 2 	    // Kick the ball around and chase it!

#define NOISE_VAR 5.0                   // Minimum amount of displacement considered NOT noise (in pixels).

struct AI_data{
	// This data structure is used to hold all data relevant to the state of the AI.
	// This includes, of course, the current state, as well as the status of
	// our own bot, the opponent (if present), and the ball (if present).
	// For each agent in the game we keep a pointer to the blob that corresponds
	// to the agent (see the blob data structure in imageCapture.h), and data
	// about its old position, as well as current velocity and heading vectors.
	//
	// MIND THE NOISE.

	// Robot's playfield side id (w.r.t. the viepoint of the camera).
	int side;		// side=0 implies the robot's own side is the left side
                    // side=1 implies the robot's own side is the right side
                    // This is set based on the robot's initial position
                    // on the field
    int botCol;		// Own bot's colour. 0 - green, 1 - red

	int state;		// Current AI state

	// Object ID status for self, opponent, and ball. Just boolean 
        // values indicating whether blobs have been found for each of these
	// entities.
	int selfID;
	int oppID;
	int ballID;

	// Blob track data. Ball likely needs to be detected at each frame
	// separately. So we keep old location to estimate v
	struct blob *ball;		       // Current ball blob *NULL* if ball is not visible/found
	double old_bcx, old_bcy;	   // Previous ball cx,cy
	double bvx,bvy;			       // Ball velocity vector
	double bmx,bmy;			       // Ball motion vector
	double bdx,bdy;                // Ball heading direction (from blob shape)

	// Self track data. Done separately each frame
    struct blob *self;		       // Current self blob *NULL* if not visible/found
	double old_scx, old_scy;	   // Previous self (cx,cy)
	double svx,svy;			       // Current self [vx vy]
	double smx,smy;			       // Self motion vector
	double sdx,sdy;                // Self heading direction (from blob shape)

	// Opponent track data. Done separately each frame
    struct blob *opp;		       // Current opponent blob *NULL* if not visible/found
	double old_ocx, old_ocy;	   // Previous opponent (cx,cy)
	double ovx,ovy;			       // Current opponent [vx vy]
	double omx,omy;			       // Opponent motion vector
	double odx,ody;                // Opponent heading direction (from blob shape)
};

struct RoboAI {
	// Main AI data container. It allows us to specify which function
	// will handle the AI, and sets up a data structure to store the
	// AI's data (see above).
	void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
	void (* calibrate)(struct RoboAI *ai, struct blob *);
	struct AI_data st;
    struct displayList *DPhead;
};


/**
 * \brief Set up an AI structure for playing roboSoccer
 *
 * Set up an AI structure for playing roboSoccer. Must be
 * called before using the AI structure during gameplay.
 * \param[in] mode The operational mode for the AI
 * \param[out] ai A structure containing data necessary for
 * 		AI algorithms
 * \pre ai is uninitialized
 * \post ai is set up for use, and must be cleaned up using
 * 		cleanupAI
 */
int setupAI(int mode, int own_col, struct RoboAI *ai);

/**
 * \brief Top-level AI loop.
 * 
 * Decides based on current state and blob configuration what
 * the bot should do next, and calls the appropriate behaviour
 * function.
 *
 * \param[in] ai, pointer to the data structure for the running AI
 * \param[in] blobs, pointer to the current list of tracked blobs
 * \param[out] void, but the state description in the AI structure may have changed
 * \pre ai is not NULL, blobs is not NULL
 * \post ai is not NULL, blobs is not NULL
 */
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state);

// Calibration stub
void AI_calibrate(struct RoboAI *ai, struct blob *blobs);

/* PaCode - just the function headers - see the functions for descriptions */
void id_bot(struct RoboAI *ai, struct blob *blobs);
struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col);
void track_agents(struct RoboAI *ai, struct blob *blobs);

// Display List functions
// the AI data structure provides a way for you to add graphical markers on-screen,
// the functions below add points or lines at a specified location and with the
// colour you want. Items you add will remain there until cleared. Do not mess
// with the list directly, use the functions below!
// Colours are specified as floating point values in [0,255], black is [0,0,0]
// white is [255,255,255].
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B);
struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B);
struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B);
struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B);
struct displayList *clearDP(struct displayList *head);

/****************************************************************************
 TO DO:
   Add headers for your own functions implementing the bot's soccer
   playing functionality below.
*****************************************************************************/

// Constants
#define NUM_EVENTS 20
#define PID_TIME 10

// #define Kp 0.0005
#define Kp 0.0001
#define Kd 0.000005
#define Ki 0.0000057


#define BALL_MOVEMENT_THRESHOLD 10

// #define Kd 0.0000
// #define Ki 0.0000001

#define CHASE_BALL_THRESHOLD 150

#define STOP_ROTATING_THRESHOLD 0.2 // this is in radians

// Motion Controls

#define DRIVE_FORWARD BT_turn(MOTOR_A, 97, MOTOR_D, 100);
#define TURN_ON_STOP_CW BT_turn(MOTOR_A, -50, MOTOR_D, 50);
#define TURN_ON_STOP_CCW BT_turn(MOTOR_A, 50, MOTOR_D, -50);

//Events 
#define SUCCESS 0 
#define FAIL 1
#define RESET 2
#define NOT_ALIGNED_WITH_BALL 3
#define POS_ANGLE 4
#define NEG_ANGLE 5
#define BALL_MOVED 6

void update_cleaned_mx_my(struct RoboAI *ai);

void move_forward(struct RoboAI *ai, struct blob *blobs); 
void calculate_heading_direction_forward(struct RoboAI *ai, struct blob *blobs);
void move_backward(struct RoboAI *ai, struct blob *blobs);
void calculate_heading_direction_backward(struct RoboAI *ai, struct blob *blobs);
void rotate_towards_ball(struct RoboAI *ai, struct blob *blobs);

void shift_to_rotate_mode_ccw(struct RoboAI *ai, struct blob *blobs);
void shift_to_rotate_mode_cw(struct RoboAI *ai, struct blob *blobs);
void rotate(struct RoboAI *ai, struct blob *blobs);
void rotate_180_towards_ball(struct RoboAI *ai, struct blob *blobs);
void move_towards_ball(struct RoboAI *ai, struct blob *blobs);



void move_forward_no_motion_direction(struct RoboAI *ai, struct blob *blobs);
void initiate_rotate_towards_ball(struct RoboAI *ai, struct blob *blobs);
void rotate_towards_ball_ccw(struct RoboAI *ai, struct blob *blobs);
void rotate_towards_ball_cw(struct RoboAI *ai, struct blob *blobs);
void initiate_drive_to_ball_pid(struct RoboAI *ai, struct blob *blobs);
void drive_to_ball_pid(struct RoboAI *ai, struct blob *blobs);
void arrived_at_chase_ball(struct RoboAI *ai, struct blob *blobs);



void initialize_penalty_kick(struct RoboAI *ai, struct blob *blobs);
void grab(struct RoboAI *ai, struct blob *blobs);
double find_angle_to_goal(struct RoboAI *ai);
void initiate_rotate_towards_goal(struct RoboAI *ai, struct blob *blobs);
void rotate_towards_goal_cw(struct RoboAI *ai, struct blob *blobs);
void rotate_towards_goal_ccw(struct RoboAI *ai, struct blob *blobs);
void kick(struct RoboAI *ai, struct blob *blobs);
#endif
