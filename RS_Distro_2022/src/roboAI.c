/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 2.0 - Updated Jul. 2022 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!
extern int sx;              // Get access to the image size from the imageCapture module
extern int sy;
int laggy=0;

int playRoboSoccer = 1;
int defend = 1;

//SANITIZED HEADING VECTOR
double sanitized_dx, sanitized_dy;
// BOUNDARY BUFFER
double top_buffer = 100;
double bottom_buffer = 100;
double left_buffer = 100;
double right_buffer = 100;
double ball_buffer = 100;
double kick_distance = 300;
// GLOBAL BALL POSITION TO CHECK IF BALL MOVED
double ball_x, ball_y;
// GLOBAL BALL POSITION TO CHECK IF BALL MOVED
double self_x, self_y;
// GLOBAL ENEMY GOAL LOCATION
double enemy_goal_x, enemy_goal_y;
// GLOBAL TARGET VARIABLE
double target_x, target_y;
// void (*state_functions[300]) (struct RoboAI *ai, struct blob *blobs);
// int T[300][NUM_EVENTS];

// global variables used for pid
// u = e + de/dt + integral e dt


  // 0 = (y2 - y1)x - (x2 - x1)y + c
  // double line_x1, line_y1;
  // double line_x2, line_y2;
  // double line_c;

// double prev_self_cx, prev_self_cy;

// double left_motor_speed;
// double right_motor_speed;

// double pastError[PID_TIME];

// double enemy_goal_x;
// double enemy_goal_y;
// double own_goal_x;
// double own_goal_y;

/**************************************************************
 * Display List Management
 * 
 * The display list head is kept as a pointer inside the A.I. 
 * data structure. Initially NULL (of course). It works like
 * any other linked list - anytime you add a graphical marker
 * it's added to the list, the imageCapture code loops over
 * the list and draws any items in there.
 * 
 * The list IS NOT CLEARED between frames (so you can display
 * things like motion paths that go over mutiple frames).
 * Your code will need to call clearDP() when you want this
 * list cleared.
 * 
 * ***********************************************************/
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type=0;
  newNode->x1=x;
  newNode->y1=y;
  newNode->x2=-1;
  newNode->y2=-1;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  
  newNode->next=head;
  return(newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x2;
  newNode->y2=y2;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);  
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;
  
  l=sqrt((dx*dx)+(dy*dy));
  dx=dx/l;
  dy=dy/l;
  
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addVector(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x1+(length*dx);
  newNode->y2=y1+(length*dy);
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x-length;
  newNode->y1=y;
  newNode->x2=x+length;
  newNode->y2=y;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  head=newNode;

  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x;
  newNode->y1=y-length;
  newNode->x2=x;
  newNode->y2=y+length;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while(head)
  {
      q=head->next;
      free(head);
      head=q;
  }
  return(NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Blue bot
 //                   1 -> Red bot
 //                   2 -> Yellow ball
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 static double Mh[4]={-1,-1,-1,-1};
 static double mx0,my0,mx1,my1,mx2,my2;
 FILE *f;
 
 // Import calibration data from file - this will contain the colour values selected by
 // the user in the U.I.
 if (Mh[0]==-1)
 {
  f=fopen("colours.dat","r");
  if (f!=NULL)
  {
   fread(&Mh[0],4*sizeof(double),1,f);
   fclose(f);
   mx0=cos(Mh[0]);
   my0=sin(Mh[0]);
   mx1=cos(Mh[1]);
   my1=sin(Mh[1]);
   mx2=cos(Mh[2]);
   my2=sin(Mh[2]);
  }
 }

 if (Mh[0]==-1)
 {
     fprintf(stderr,"roboAI.c :: id_coloured_blob2(): No colour calibration data, can not ID blobs. Please capture colour calibration data on the U.I. first\n");
     return NULL;
 }
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.90;                                              // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // The reference colours here are in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : blue bot,  1 : red bot, 2 : yellow ball
  if (col==0) {vr_x=mx0; vr_y=my0;}                                                    
  else if (col==1) {vr_x=mx1; vr_y=my1;}
  else if (col==2) {vr_x=mx2; vr_y=my2;}

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Motion direction vector. Not valid
 //   while rotating - possibly valid while turning
 // - Heading direction - vector obtained from the blob shape, it is
 //   correct up to a factor of (-1) (i.e. it may point backward w.r.t.
 //   the direction your bot is facing). This vector remains valid
 //   under rotation.
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 
 // Reset ID flags and agent blob pointers
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;
 
 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;
  ai->st.bdx=p->dx;
  ai->st.bdy=p->dy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute motion direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  else
  {
    ai->st.bmx=0;
    ai->st.bmy=0;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
 p=id_coloured_blob2(ai,blobs,ai->st.botCol);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob
  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;
  ai->st.sdx=p->dx;
  ai->st.sdy=p->dy;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
//  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf], old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf, %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }
  else
  {
   ai->st.smx=0;
   ai->st.smy=0;
  }
  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent - whatever colour is not botCol
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	
  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;
  ai->st.odx=p->dx;
  ai->st.ody=p->dy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  else
  {
   ai->st.omx=0;
   ai->st.omy=0;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 static double oldX,oldY;
 double frame_inc=1.0/5.0;
 double dist;
 
 track_agents(ai,blobs);		// Call the tracking function to find each agent

 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30);			// Start forward motion to establish heading
                                                // Will move for a few frames.
  
 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI 
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
  fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.sdx=0;
 ai->st.sdy=0;
 ai->st.odx=0;
 ai->st.ody=0;
 ai->st.bdx=0;
 ai->st.bdy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 ai->DPhead=NULL;

////////////////////////////////////////////
// CHASE STATES AND TRANSITIONS
////////////////////////////////////////////
state_functions[201] = set_global;
T[201][SUCCESS] = 202; // should never fail
T[201][FAIL] = 201; // but in case ...
// you now have a sanitized heading vector stored globally
// and all boundary + kick distance
state_functions[202] = select_target; // big function
T[202][SUCCESS] = 203; // should never fail
T[202][FAIL] = 202; // but in case ...
// you now have a target, use a PID with ingenious obstacle avoidance
state_functions[203] = get_to_target;
T[203][SUCCESS] = 204; // reached target
T[203][FAIL] = 203; // ball stationary but still failed
T[203][BALL_MOVED] = 202;
state_functions[204] = face_ball;
T[204][SUCCESS] = 205;
T[204][FAIL] = 204;
T[204][BALL_MOVED] = 204;
state_functions[205] = chase_finish;






//  state_functions[201] = move_forward_no_motion_direction;
//  state_functions[202] = initiate_rotate_towards_ball;
//  state_functions[203] = rotate_towards_ball_ccw;
//  state_functions[204] = rotate_towards_ball_cw;
//  state_functions[205] = initiate_drive_to_ball_pid;
//  state_functions[206] = drive_to_ball_pid;
//  state_functions[207] = arrived_at_chase_ball;

// //  T[201][SUCCESS] = 202; // found heading direction
// //  T[202][SUCCESS] = 205; // arrived close enough to ball
// //  T[203][]
// //  T[203][NOT_ALIGNED_WITH_BALL] = 201;

//  T[201][SUCCESS] = 202;
//  T[202][SUCCESS] = 205;
//  T[202][POS_ANGLE] = 203;
//  T[202][NEG_ANGLE] = 204;
//  T[203][SUCCESS] = 205;
//  T[204][SUCCESS] = 205;
//  T[205][SUCCESS] = 206;
//  T[205][BALL_MOVED] = 201;
//  T[206][SUCCESS] = 207;
//  T[206][BALL_MOVED] = 201;
//  T[207][BALL_MOVED] = 201;

//  state_functions[101] = move_forward_no_motion_direction;
//  state_functions[102] = initialize_penalty_kick;
//  state_functions[103] = initiate_rotate_towards_ball;
//  state_functions[104] = rotate_towards_ball_ccw;
//  state_functions[105] = rotate_towards_ball_cw;
//  state_functions[106] = initiate_drive_to_ball_pid;
//  state_functions[107] = drive_to_ball_pid;
//  state_functions[108] = arrived_at_ball;
//  state_functions[109] = grab;
//  state_functions[110] = initiate_rotate_towards_goal;
//  state_functions[111] = rotate_towards_goal_ccw;
//  state_functions[112] = rotate_towards_goal_cw;
//  state_functions[113] = kick;

//  T[101][SUCCESS] = 102;
//  T[102][SUCCESS] = 103;
//  T[103][SUCCESS] = 106;
//  T[103][POS_ANGLE] = 104;
//  T[103][NEG_ANGLE] = 105;
//  T[104][SUCCESS] = 106;
//  T[105][SUCCESS] = 106;
//  T[106][SUCCESS] = 107;
//  T[106][BALL_MOVED] = 103;
//  T[107][SUCCESS] = 108;
//  T[107][BALL_MOVED] = 103;
//  T[108][SUCCESS] = 109;
//  T[108][BALL_MOVED] = 103;
//  T[109][SUCCESS] = 110;
//  T[109][FAIL] = 103;
//  T[110][SUCCESS] = 113;
//  T[110][POS_ANGLE] = 111;
//  T[110][NEG_ANGLE] = 112;
//  T[111][SUCCESS] = 113;
//  T[112][SUCCESS] = 113;

//  state_functions[1] = initialize_soccer;
//  state_functions[2] = strategy_check; // "Parent FSM"
//  state_functions[3] = initialize_defense; // Defense mode start
//  state_functions[4] = move_to_intercept;
//  state_functions[5] = slow_advance;
//  state_functions[6] = clear_shot_check; // Offense mode start
//  state_functions[7] = kick;
//  state_functions[8] = initiate_dribble;
//  state_functions[9] = dribble_kick;
//  state_functions[10] = initiate_rotate_towards_ball; // Chase start
//  state_functions[11] = rotate_towards_ball_ccw;
//  state_functions[12] = rotate_towards_ball_cw;
//  state_functions[13] = initiate_drive_to_ball_pid;
//  state_functions[14] = drive_to_ball_pid;
//  state_functions[15] = arrived_at_chase_ball;
//  state_functions[19] = move_forward_no_motion_direction; // Chase end
//  state_functions[16] = initialize_tackle; // Tackle start
//  state_functions[17] = grab;
//  state_functions[18] = collision_detection;

//  T[1][SUCCESS] = 2; // Choose strategy

//  T[2][DEFENSE] = 3; // Defense start
//  T[3][SUCCESS] = 4;
//  T[4][SUCCESS] = 5;
//  T[5][FAIL] = 4;
//  T[5][SUCCESS] = 2; // Defense end

//  T[2][TACKLE] = 16; // Tackle start
//  T[10][SUCCESS] = 13; // Chase start
//  T[10][POS_ANGLE] = 11;
//  T[10][NEG_ANGLE] = 12;
//  T[11][SUCCESS] = 13;
//  T[12][SUCCESS] = 13;
//  T[13][SUCCESS] = 14;
//  T[13][BALL_MOVED] = 19;
//  T[14][SUCCESS] = 15;
//  T[14][BALL_MOVED] = 19;
//  T[15][BALL_MOVED] = 19; // Chase end
//  T[15][SUCCESS] = 17; 
//  T[17][FAIL] = 2; // Tackle end

//  T[17][SUCCESS] = 6; // Offense start
//  T[6][SUCCESS] = 7;
//  T[7][SUCCESS] = 2;
//  T[6][FAIL] = 8;
//  T[8][SUCCESS] = 9;
//  T[9][SUCCESS] = 13; // Chase start
//  T[10][POS_ANGLE] = 11;
//  T[10][NEG_ANGLE] = 12;
//  T[11][SUCCESS] = 13;
//  T[12][SUCCESS] = 13;
//  T[13][SUCCESS] = 14;
//  T[13][BALL_MOVED] = 19;
//  T[14][SUCCESS] = 15;
//  T[14][BALL_MOVED] = 19;
//  T[15][BALL_MOVED] = 19; // Chase end
//  T[15][SUCCESS] = 6; // Offense end

 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 // The code here just makes sure the image processing loop is constantly
 // tracking the bots while they're placed in the locations required
 // to do the calibration (i.e. you DON'T need to add anything more
 // in this function).
 track_agents(ai,blobs);
}


/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/
  static double ux,uy,len,mmx,mmy,tx,ty,x1,y1,x2,y2;
  double angDif;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	  // The id_bot() routine will change the AI state to initial state + 1
  {				                 // if robot identification is successful.
      
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;         // This sets the side the bot thinks as its own side 0->left, 1->right
   BT_all_stop(0);
   
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
   
   if (ai->st.self!=NULL)
   {
       // This checks that the motion vector and the blob direction vector
       // are pointing in the same direction. If they are not (the dot product
       // is less than 0) it inverts the blob direction vector so it points
       // in the same direction as the motion vector.
       if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
       {
           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
           ai->st.sdx*=-1;
           ai->st.sdy*=-1;
       }
       old_dx=ai->st.sdx;
       old_dy=ai->st.sdy;
   }
  
   if (ai->st.opp!=NULL)
   {
       // Checks motion vector and blob direction for opponent. See above.
       if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
           ai->st.odx*=-1;
           ai->st.ody*=-1;
       }       
   }

         
  }
  
  // Initialize BotInfo structures
   
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
  fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
  track_agents(ai,blobs);		// Currently, does nothing but endlessly track
  (*state_functions[ai->st.state]) (ai, blobs);
  update_global(ai);
  // ai->DPhead = addVector(ai->DPhead, ai->st.self->cx, ai->st.self->cy, cleaned_mx, cleaned_my, 40, 255,0,0);
//   if (ai -> st.state == 1) {

//   } else if (ai -> st.state == 101) {
    
//   } else if (ai -> st.state == 201) {
//     fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
//     track_agents(ai,blobs);		// Currently, does nothing but endlessly track
  
//     // Have some transition table? 
//     (*state_functions[ai->st.state]) (ai, blobs);
//   }
 }

}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/
///////////////////////////////////////
// COMMON LOGIC
///////////////////////////////////////
double find_angle(double x1, double y1, double x2, double y2)
{
  return atan2(x1*y2 - x2*y1, x1*x2 + y1*y2);
}

int is_ball_moving()
{
  double bx, by;
  if (ai->st.ball != NULL)
  {
    bx = ai->st.ball.cx;
    by = ai->st.ball->cy;
  } else {
    bx = ai->st.old_bcx;
    by = ai->st.old_bcy;
  }
  return (abs(ball_x - bx) >= 10 || abs(ball_y - by) >= 10)
}

void set_global(struct RoboAI *ai, struct blob *blob)
{
  // move forward
  BT_turn(MOTOR_A, -20, MOTOR_B, -20);
  sleep(0.5); //TUNE
  // now we have a motion direction to compare with
  double mx, my, dx, dy;
  if (ai->st.self != NULL)
  {
    mx = ai->st.self->mx;
    my = ai->st.self->my;
    dx = ai->st.self->dx;
    dy = ai->st.self->dy;
    self_x = ai->st.self->cx;
    self_y = ai->st.self->cy;
  } else {
    mx = ai->st.smx;
    my = ai->st.smy;
    dx = ai->st.sdx;
    dy = ai->st.sdy;
    self_x = ai->st.old_scx;
    self_y = ai->st.old_scy;
  }

  if (ai->st.ball != NULL)
  {
    ball_x = ai->st.ball->cx;
    ball_y = ai->st.ball->cy;
  } else {
    ball_x = ai->st.old_bcx;
    ball_y = ai->st.old_bcy;
  }
  enemy_goal_x = ai->st.side*sx;
  enemy_goal_y = sy/2;
  double angle = find_angle(mx, my, dx, dy);
  sanitized_dx = (abs(angle) <= M_PI/2) ? dx : -dx;
  sanitized_dy = (abs(angle) <= M_PI/2) ? dy : -dy;
  ai->st.state = T[ai->st.state][SUCCESS];
}

double norm(double x, double y)
{
  return sqrt(pow(x, 2)+pow(y, 2));
}
void update_global(struct RoboAI *ai)
{
  // ned to update sanitized_dx, sanitized_dy, ball_x, ball_y
  double mx, my, dx, dy;
  if (ai->st.self != NULL)
  {
    mx = ai->st.self->mx;
    my = ai->st.self->my;
    dx = ai->st.self->dx;
    dy = ai->st.self->dy;
    self_x = ai->st.self->cx;
    self_y = ai->st.self->cy;
  } 
  else {
    mx = ai->st.smx;
    my = ai->st.smy;
    dx = ai->st.sdx;
    dy = ai->st.sdy;
    self_x = ai->st.old_scx;
    self_y = ai->st.old_scy;
  }
  if (ai->st.ball != NULL)
  {
    ball_x = ai->st.ball->cx;
    ball_y = ai->st.ball->cy;
  } 
  else {
    ball_x = ai->st.old_bcx;
    ball_y = ai->st.old_bcy;
  }
  // check dx dy value
  if (abs(find_angle(sanitized_dx, sanitized_dy, dx, dy)) < 1.5)
  {
    sanitized_dx = dx;
    sanitized_dy = dy;
  }
  else
  {
    sanitized_dx = -dx;
    sanitized_dy = -dy;
  }
}

// left  = 0, right  = 1
void turn(int direction)
{
  if (direction)
  {
    BT_turn(MOTOR_A, -20, MOTOR_D, 20);
  }
  else
  {
    BT_turn(MOTOR_A, 20, MOTOR_D, -20);
  }
}
///////////////////////////////////////
// CHASE LOGIC
///////////////////////////////////////
void select_target(struct RoboAI *ai, struct blob *blob)
{
  // check if ball in a corner
  
  if ((ball_x < left_buffer || ball_x > (sx-right_buffer)) && (ball_y < top_buffer || ball_y > (sy - bottom_buffer)))
  {
    // set target as ball
    target_x = ball_x;
    target_y = ball_y;
    ai->st.state = T[ai->st.state][SUCCESS];
  } else {
    double delta_x = ball_x - enemy_goal_x;
    double delta_y = ball_y - enemy_goal_y;
    double length = norm(delta_x, delta_y);
    target_x = ball_x + delta_x*kick_distance/length;
    target_y = ball_y + delta_y*kick_distance/length;
    if (ball_x < left_buffer || ball_x > (sx-right_buffer) || ball_y < top_buffer || ball_y > (sy - bottom_buffer))
    {
      target_x = ball_x;
      target_y = ball_y;
    }
    ai->st.state = T[ai->st.state][SUCCESS];
  }
}

double updateInt(struct RoboAI *ai, double curr_err)
{
  double sum = 0;
  for (int k = PID_TIME-1; k>0; k--)
  {
    pastError[k] = pastError[k-1];
    sum += pastError[k];
  }
  pastError[0] = curr_err;
  return sum;
}

void get_to_target(struct RoboAI *ai, struct blob *blob)
{
  // Use PID to drive to target_x, target_y
  // Every frame, check if ball moved, and if so, transition to select_target

    // There are global flags for where the target is 
  if (!(ai->st.selfID))
  {
    // Not really sure what we should do? maybe BT_all_stop();
    return;
  }
  if (is_ball_moving())
  {
    BT_all_stop(1);
    ai->st.state = T[ai->st.state][BALL_MOVED];
  }
  else if (has_reached_target(ai))
  {
    BT_all_stop(1);
    ai->st.state = T[ai->st.state][SUCCESS];
  }
  else
  {

    // for now assume we arent going to be set in a way that we collide with the ball
    // on our way to the target
    // but if we have time, change this so that it checks whether ball is within kick_distance
    // and change vector to target to be tangential to the direction to ball so you
    // drive along the circumference until you are clear to move forward
    double vect2tgt_x = target_x - ai->st.self->cx;
    double vect2tgt_y = target_y - ai->st.self->cy;

    vect2tgt_x /= norm(vect2tgt_x, vect2tgt_y);
    vect2tgt_y /= norm(vect2tgt_x, vect2tgt_y); 
  
    double err = find_angle(sanitized_dx, sanitized_dy, vect2tgt_x, vect2tgt_y);
    // if error too big, stop moving and align yourself so that error is manageable
    if (abs(err) >= 0.4)
    {
      // err < 0 means we need to turn right
      turn((err<0));
    }
    else
    {
      // PID here
      // also only use the P, we dont need the I or D
      continue;
    }
  }
  
}

void face_ball(struct RoboAI *ai, struct blob *blob)
{
  if (is_ball_moving())
  {
    ai->st.state = T[ai->st.state][BALL_MOVED]
  } else {
    double angle = find_angle(sanitized_dx, sanitized_dy, ball_x-self_x, ball_y-self_y);
    if (abs(find_angle()) < 0.1)
    {
    ai->st.state = T[ai->st.state][SUCCESS]
    }
    else
    {
      // need to turn in place according to angle, stay in the same state
      turn((angle<0));
    }
  }
}

// double eval_implicit_line(double x, double y){
//   // drawLine(dp-x1,dp->y1,dp->x2-dp->x1,dp->y2-dp->y1,1,dp->R,dp->G,dp->B,blobIm);
//   return (line_y2-line_y1)*x - (line_x2-line_x1)*y + line_c;
// }


// FINDS ANGLE BETWEEN MOTION DIRECTION (mode = 0) OR
// HEADING DIRECTION (mode = 1) OR cleaned HEADING DIRECTION (mode = 2) AND THE DIRECTION FROM THE ROBOT TO THE BALL
// returns an angle in radians between (-pi, pi], negative angle means we need to move the robot
// clockwise, positive angle means we ned to move the robot counterclockwise


// void get_to_target_pid(struct RoboAI *ai, struct blob *blob)
// {
//   // There are global flags for where the target is 
//   if (!(ai->st.selfID))
//   {
//     // Not really sure what we should do? maybe BT_all_stop();
//     return;
//   }
//   if (ball_moved(ai))
//   {
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][BALL_MOVED];
//   }
//   if (reached_dst(ai))
//   {
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][SUCCESS];
//   }
  
//   double vect2dst = 

// }

// void drive_to_ball_pid(struct RoboAI *ai, struct blob *blob)
// {
//   /*
//     Use angle 
//   */
//  if (!(ai->st.selfID && ai->st.ballID))
//  {
//   // Not really sure what we should do? maybe BT_all_stop();
//   return;
//  }
//  if (ball_moved(ai))
//  {
//   ai->st.state = T[ai->st.state ]
//  }
// }

// void move_forward(struct RoboAI *ai, struct blob *blobs)
// {
//   // Set back wheels to the middle (shouldn't be an issue)
//   // Drive forward for 1 second
//   BT_timed_motor_port_start_v2(MOTOR_A, -100, 1000);
//   // Update State
//   ai->st.state = 202; //Need a transition table
// }

// void calculate_heading_direction_forward(struct RoboAI *ai, struct blob *blobs)
// {
//   // Did we find ourselves and the ball
//   // if (selfID == 0 || ballID == 0) {
//   //   ai->st.state = 200 // Need a transition table T[ai->st.state][event]
//   // }
//   // Find change in bot position
//   ai->st.self->cx - ai->st.old_scx;
//   ai->st.self->cy - ai->st.old_scy;

//   // check magnitude of vector is bigger than a threshold

//   //

//   // If can't find change we set to move_backward state
//   // Else calculate and set the heading direction
//   // Update State
// }

// void move_backward(struct RoboAI *ai, struct blob *blobs)
// {
//   // Set backwheels to the middle (shouldn't be an issue)
//   // Drive backward for 1 second 
//   // Update State
// }

// void calculate_heading_direction_backward(struct RoboAI *ai, struct blob *blobs)
// {
//   // Find change in bot position
//   // If can't find change we set to initial state
//   // Else calculate and set the heading direction
//   // Update State
// }

// // returns 1 if the robot is moving in the direction it is facing
// // return 0 otherwise, assumes the robot is moving
// int is_parallel(struct RoboAI *ai, double threshold)
// {
//   double mx, my, dx, dy;
//   if (ai->st.self == NULL)
//   {
//     mx = ai->st.smx;
//     my = ai->st.smy;
//     dx = ai->st.sdx;
//     dy = ai->st.sdy;
//   } else
//   {
//     mx = ai->st.self->mx;
//     my = ai->st.self->my;
//     dx = ai->st.self->dx;
//     dy = ai->st.self->dy;
//   }
//   // find angle between [mx, my] and [dx, dy]
//   double norm_motion = sqrt(pow(abs(mx), 2)+pow(abs(my), 2));
//   double norm_direction = sqrt(pow(abs(dx), 2)+pow(abs(dy), 2));
//   double dot = mx*dx+my*dy;
//   double angle = acos(dot/(norm_direction*norm_motion));
//   angle = angle < (M_PI-angle) ? angle : (M_PI-angle);
//   return (angle < threshold);
// }

// void update_cleaned_mx_my(struct RoboAI *ai)
// {
//   if (!(ai->st.selfID)) {return;}
//   double dot = ai->st.self->dx * cleaned_mx + ai->st.self->dy * cleaned_my;
//   double det = ai->st.self->dx*cleaned_my - cleaned_mx*ai->st.self->dy;
//   double angle = atan2(det, dot);

//   if (fabs(dot)<UPDATE_MX_MY_THRESHOLD) {return;}
//   if (dot > 0)
//   {
//     cleaned_mx = ai->st.self->dx;
//     cleaned_my = ai->st.self->dy;
//   } else {
//     cleaned_mx = -ai->st.self->dx;
//     cleaned_my = -ai->st.self->dy;
//   }
//   fprintf(stderr,"cleaned mx %f my %f\n", cleaned_mx, cleaned_my);
// }

// void move_forward_no_motion_direction(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d move_forward_no_motion_direction\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {
//     BT_all_stop(1);
//     return;
//   }
//   // Drive forward
//   DRIVE_FORWARD

//   if (ai->st.self->mx != 0 && ai->st.self->my != 0)
//   {
//     // has a heading direction
//     fprintf(stderr,"Found heading direction %f %f\n", ai->st.self->mx,ai->st.self->my);
//     cleaned_mx = ai->st.self->mx;
//     cleaned_my = ai->st.self->my;
//     ai->st.state = T[ai->st.state][SUCCESS];
//     BT_all_stop(1);
//     return;
//   }
// }

// void initiate_rotate_towards_ball(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d initiate_rotate_towards_ball\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {return;}
//   line_x2 = ai->st.ball->cx;
//   line_y2 = ai->st.ball->cy;
//   // Assuming we are fully stopped already 
//   // rotate towards the ball
//   double angleToBall = find_angle(ai,2);
//   if (fabs(angleToBall) < STOP_ROTATING_THRESHOLD && angleToBall != 0.0)
//   {
//     // oritentated towards facing the ball
//     // update the state
//     ai->st.state = T[ai->st.state][SUCCESS];
//   } else if (angleToBall < 0)
//   {
//     // Keep turning ccw
//     ai->st.state = T[ai->st.state][POS_ANGLE];
//   } else 
//   {
//     // angleToBall < 0
//     // Keep turing cw
//     ai->st.state = T[ai->st.state][NEG_ANGLE];
//   }
// }

// int vertical_angle(double theta)
// {
//   // return (fabs(theta) < STOP_ROTATING_THRESHOLD || PI - fabs(theta) < STOP_ROTATING_THRESHOLD)  ? 1 : 0;
//   return (fabs(theta) < STOP_ROTATING_THRESHOLD)  ? 1 : 0;
// }

// void rotate_towards_ball_cw(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d rotate_towards_ball_cw\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {
//     BT_all_stop(1);
//     return;
//   }
//   update_cleaned_mx_my(ai);
//   if (vertical_angle(find_angle(ai,2)))
//   {
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][SUCCESS];
//     return;
//   }
//   // Keep rotating CW 
//   TURN_ON_STOP_CW
// }

// void rotate_towards_ball_ccw(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d rotate_towards_ball_ccw\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {
//     BT_all_stop(1);
//     return;
//   }
//   update_cleaned_mx_my(ai);
//   if (vertical_angle(find_angle(ai,2)))
//   {
//     // aligned with ball
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][SUCCESS];
//     return;
//   }
//   // Keep rotating CCW
//   TURN_ON_STOP_CCW
// }

// int ball_moved(struct RoboAI *ai)
// {
//   return (fabs(ai->st.ball->cx - line_x2) > BALL_MOVEMENT_THRESHOLD || fabs(ai->st.ball->cy - line_y2)> BALL_MOVEMENT_THRESHOLD);
// }

// void initiate_drive_to_ball_pid(struct RoboAI *ai, struct blob *blob)
// {
//   // set the destination
//   // set the path equation (implicit equation of a line in this case)

//   // implicit equation is ax + by + c = 0
//   // given end points (x1, y1) and (x2, y2) -> 0 =(y2 - y1)x - (x2 - x1)y + c
//   // c = x2y1 - y2x1
//   if (!(ai->st.selfID && ai->st.ballID)) {
//     BT_all_stop(1);
//     return;
//   }
//   fprintf(stderr,"state %d initiate_drive_to_ball_pid\n", ai->st.state);
//   if (ball_moved(ai)) {
//     ai->st.state = T[ai->st.state][BALL_MOVED];
//     return;
//   }
//   line_x1 = ai->st.self->cx;
//   line_y1 = ai->st.self->cy;

//   line_c = line_x2*line_y1 - line_y2*line_x1;
//   left_motor_speed = 30;
//   right_motor_speed = 30;
//   prev_self_cx = ai->st.self->cx;
//   prev_self_cy = ai->st.self->cy;
//   ai->DPhead = addLine(ai->DPhead, line_x1, line_y1, line_x2, line_y2, 255, 255, 255);
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// double updateInt(struct RoboAI *ai, double curr_err)
// {
//   double sum = 0;
//   for (int k = PID_TIME-1; k>0; k--)
//   {
//     pastError[k] = pastError[k-1];
//     sum += pastError[k];
//   }
//   pastError[0] = curr_err;
//   return sum;
// }

// void drive_to_ball_pid(struct RoboAI *ai, struct blob *blob)
// {
//   /*
//   we should probably keep driving forward during this step
//   */
//   fprintf(stderr,"state %d drive_to_ball_pid\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {
//     BT_all_stop(1);
//     return;
//   }

//   // Confirm we are going the right direction 
//   fprintf(stderr, "help %f\n",fabs(find_angle(ai,1)));
//   if (ball_moved(ai) || fabs(find_angle(ai,0)) > PI/2) {
//     ai->st.state = T[ai->st.state][BALL_MOVED];
//     return;
//   }


//   // ai->DPhead = addLine(ai->DPhead, line_x1, line_y1, line_x2, line_y2, 0, 0, 0);
//  // if close enough to ball, transition to next successful state
//   if (sqrt(pow(line_x2 - ai->st.self->cx,2)+pow(line_y2 - ai->st.self->cy,2)) < CHASE_BALL_THRESHOLD)
//   {
//     ai->st.state = T[ai->st.state][SUCCESS];
//     BT_all_stop(1);
//     return;
//   }

//   if (playRoboSoccer && sqrt(pow(ai->st.opp->cx - ai->st.self->cx,2)+pow(ai->st.opp->cy - ai->st.self->cy,2)) < 500)
//   {
//     return;
//   }



//   // calculate u = e + de + integral e
//   double curr_err = eval_implicit_line(ai->st.self->cx, ai->st.self->cy);
//   double delta_err = curr_err - eval_implicit_line(prev_self_cx, prev_self_cy);
  
//   fprintf(stderr, "here:%f %f %f %f\n", ai->st.self->cx, ai->st.self->cy,prev_self_cx, prev_self_cy);
  
//   prev_self_cx = ai->st.self->cx;
//   prev_self_cy = ai->st.self->cy;
//   double int_err = updateInt(ai, curr_err);

//   fprintf(stderr, "e: %f de: %f, integral e: %f\n", curr_err, delta_err, int_err);
//   double u = Kp * curr_err + Kd * delta_err + Ki * int_err;
//   fprintf(stderr, "pid result u: %f", u);
//   fprintf(stderr, "sx: %d sy: %d\n", sx, sy);

//   // u = min(5, max(-5, u));
//   left_motor_speed = 75;
//   right_motor_speed = 70;
//   double temp = u > -30 ? u : -30;
//   u = 30 > temp ? temp : 30; 
//   left_motor_speed += u;
//   right_motor_speed -= u;
//   fprintf(stderr, "left_motor_speed: %f right_motor_speed: %f\n", left_motor_speed, right_motor_speed);
//   BT_turn(MOTOR_D, left_motor_speed, MOTOR_A, right_motor_speed);
// }


// //probably turn this into helper function, since this is too general for it to be a state
// void shift_to_rotate_mode_ccw(struct RoboAI *ai, struct blob *blobs)
// {
//   BT_motor_port_start(MOTOR_D, -55);
//   sleep(1);
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// void shift_to_rotate_mode_cw(struct RoboAI *ai, struct blob *blobs)
// {
//   BT_motor_port_start(MOTOR_D, 55);
//   sleep(1);
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// void arrived_at_chase_ball(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d arrived_at_chase_ball\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {return;}
//   if (sqrt(pow(ai->st.ball->cx - ai->st.self->cx,2)+pow(ai->st.ball->cy - ai->st.self->cy,2)) >= CHASE_BALL_THRESHOLD)
//   {
//     ai->st.state = T[ai->st.state][BALL_MOVED];
//   }
// }

// void arrived_at_ball(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d arrived_at_ball\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {return;}

//   if (sqrt(pow(ai->st.ball->cx - ai->st.self->cx,2)+pow(ai->st.ball->cy - ai->st.self->cy,2)) >= CHASE_BALL_THRESHOLD)
//   {
//     ai->st.state = T[ai->st.state][BALL_MOVED];
//     return;
//   }
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// void rotate(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr, "rotating\n");
//   fprintf(stderr, "ball %d self %d\n", ai->st.ballID, ai->st.selfID);
//   fprintf(stderr, "ah ball x:%f \n",ai->st.ball->cx);
//   fprintf(stderr, "ah ball y:%f \n",ai->st.ball->cy);
  
//   fprintf(stderr, "eh bot x:%f \n",ai->st.self->cx);
//   fprintf(stderr, "eh bot x:%f \n",ai->st.self->cy);

//   double x = ai->st.ball->cx - ai->st.self->cx;
//   double y = (ai->st.ball->cy - ai->st.self->cy);
//   double length = sqrt(pow(x,2) + pow(y,2));
//   x = x/length;
//   y = y/length;
//   fprintf(stderr, "x: %f\n", x);
//   fprintf(stderr, "y: %f\n", y);
//   fprintf(stderr, "dx:%f\n", (ai->st.self->dx));
//   fprintf(stderr, "dx:%f\n", (ai->st.self->dy));
//   fprintf(stderr, "top:%f\n", (x*ai->st.self->dx+y*ai->st.self->dy));
//   fprintf(stderr, "bot:%f\n", ((sqrt(pow(x,2)+pow(y,2)))*(sqrt(pow(ai->st.self->dx,2) + pow(ai->st.self->dy,2)))));
//   double theta = acos((x*ai->st.self->dx+y*ai->st.self->dy) / ((sqrt(pow(x,2)+pow(y,2)))*(sqrt(pow(ai->st.self->dx,2) + pow(ai->st.self->dy,2)))));
//   fprintf(stderr, "theta %f\n\n\n", theta);
//   if ((theta > -0.5 && theta < 0.5) || theta > 3)
//   {
//     fprintf(stderr,"aligned\n");
//     BT_all_stop(1);

//     BT_motor_port_start(MOTOR_D, 55);
//     while (true) {
//       if (BT_read_touch_sensor(PORT_1)) {
//         fprintf(stderr,"centered\n");
//         BT_all_stop(1);
//         break;
//       }
//     }

//     ai->st.state = T[ai->st.state][SUCCESS];
//     return;
//   }
//   // BT_timed_motor_port_start_v2(MOTOR_A, -100, 3000);
//   BT_motor_port_start(MOTOR_A, -100);

//   // BT_all_stop(1);
//   // sleep(0.2);
//   // BT_all_stop(1);
//   // sleep(1);
//   // BT_motor_port_start(MOTOR_A, -100);
// }

// void rotate_180_towards_ball(struct RoboAI *ai, struct blob *blobs)
// {
//   // start rotating 
//   // sleep
//   // same steps are rotate_towards_ball, except rotation direction is alread decided
//   // double theta = acos(dot(v,w) / ((sqrt(vpow(x,2)+vpow(y,2)))*(sqrt(wpow(x,2) + wpow(y,2)))));
//   // fprintf(stderr,"theta angle towards ball %f", theta);
//   // if (theta < 0.3)
//   // {
    
//   // }
// }

// void move_towards_ball(struct RoboAI *ai, struct blob *blobs)
// {
//   double x = ai->st.ball->cx - ai->st.self->cx;
//   double y = (ai->st.ball->cy - ai->st.self->cy);
//   double length = sqrt(pow(x,2) + pow(y,2));
//   if (length < 10) {
//     BT_all_stop(1);
//     return;
//   }
//   x = x/length;
//   y = y/length;
//   // If close enough to ball -> stay at same state and return 
//   // Check the angle with the ball is still below threshold
//   // If not update state to rotate_towards_ball
//   // Drive foward 
//   double theta = acos((x*ai->st.self->dx+y*ai->st.self->dy) / ((sqrt(pow(x,2)+pow(y,2)))*(sqrt(pow(ai->st.self->dx,2) + pow(ai->st.self->dy,2)))));
//   fprintf(stderr, "theta %f\n\n\n", theta);
//   if (!((theta > -0.5 && theta < 0.5) || theta > 3))
//   {
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][NOT_ALIGNED_WITH_BALL];
//     return;
//   }
//   BT_timed_motor_port_start_v2(MOTOR_A, -100, 3000);
// }

// ////////////////////////////////////////////////////////////////////////////////////
// // PENALTY LOGIC
// ////////////////////////////////////////////////////////////////////////////////////

// void initialize_penalty_kick(struct RoboAI *ai, struct blob *blobs){
//   // Populate enemy goal x and y based on inputted own goal flag
//   enemy_goal_x = (ai -> st.side) * sx;
//   if (enemy_goal_x == 0)
//   {
//     enemy_goal_x = sx;
//   } else {
//     enemy_goal_x = 0;
//   }
  
//   enemy_goal_y = (double) sy / 2;
//   BT_motor_port_start(MOTOR_B, 100);
//   sleep(2);
//   BT_all_stop(1);
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// void grab(struct RoboAI *ai, struct blob *blobs){
//   // Assume arms are already open and just grab (NOTE: MAKE SURE ARMS ARE OPEN ALWAYS UNLESS GRAB IS CALLED)
//   fprintf(stderr,"state %d grab\n\n\n\n\n\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {return;}
//   BT_motor_port_start(MOTOR_B, -100);
//   sleep(2);
//   BT_all_stop(1);

//   // Initialize direction vector and norm from bot to ball
//   double robotBallDiffX = ai -> st.ball->cx - ai -> st.self->cx;
//   double robotBallDiffY = ai -> st.ball->cy - ai -> st.self->cy;
//   double robotBallDiffNorm = sqrt(pow(robotBallDiffX, 2) + pow(robotBallDiffY, 2));

//   // Use find_angle() in mode 2 to check if the ball in in front of the robot and check if the distance (norm) is less than a threshold
//   fprintf(stderr, "angle: %f robotBallDiffNorm %f\n", fabs(find_angle(ai, 2)), robotBallDiffNorm);
//   // if (fabs(find_angle(ai, 2)) < 0.174533 && robotBallDiffNorm < 25) { // 38 is a magic number from dividing 1280 pixels by 115 cm and angle is 10 deg in radians
//     ai->st.state = T[ai->st.state][SUCCESS];
//   // } else {
//   //   ai->st.state = T[ai->st.state][FAIL];
//   // }
// }

// double find_angle_to_goal(struct RoboAI *ai)
// {
//   double cx, cy;
//   if (ai->st.self == NULL)
//   {
//     cx = ai->st.old_scx;
//     cy = ai->st.old_scy;
//   }
//   else 
//   {
//     cx = ai->st.self->cx;
//     cy = ai->st.self->cy;
//   }
//   double vec2goal_x = enemy_goal_x - cx;
//   double vec2goal_y = enemy_goal_y - cy;
//   double dot = cleaned_mx*vec2goal_x + cleaned_my*vec2goal_y;
//   double det = cleaned_mx*vec2goal_y - cleaned_my*vec2goal_x;
//   double angle = atan2(det, dot);
//   return angle;
// }

// void initiate_rotate_towards_goal(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d initiate_rotate_towards_goal\n", ai->st.state);
//   // Assuming we are fully stopped already 
//   // rotate towards the ball
//   double angleToGoal = find_angle_to_goal(ai);
//   if (fabs(angleToGoal) < STOP_ROTATING_THRESHOLD)
//   {
//     // oritentated towards facing the ball
//     // update the state
//     ai->st.state = T[ai->st.state][SUCCESS];
//   } else if (angleToGoal < 0)
//   {
//     // Keep turning ccw
//     ai->st.state = T[ai->st.state][POS_ANGLE];
//   } else 
//   {
//     // angleToBall < 0
//     // Keep turing cw
//     ai->st.state = T[ai->st.state][NEG_ANGLE];
//   }
// }

// void rotate_towards_goal_cw(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d rotate_towards_goal_cw\n", ai->st.state);
//   update_cleaned_mx_my(ai);
//   if (vertical_angle(find_angle_to_goal(ai)))
//   {
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][SUCCESS];
//     return;
//   }
//   // Keep rotating CW 
//   TURN_ON_STOP_CW
// }

// void rotate_towards_goal_ccw(struct RoboAI *ai, struct blob *blobs)
// {
//   fprintf(stderr,"state %d rotate_towards_goal_ccw\n", ai->st.state);
//   update_cleaned_mx_my(ai);
//   if (vertical_angle(find_angle_to_goal(ai)))
//   {
//     // aligned with ball
//     BT_all_stop(1);
//     ai->st.state = T[ai->st.state][SUCCESS];
//     return;
//   }
//   // Keep rotating CCW
//   TURN_ON_STOP_CCW
// }

// void kick(struct RoboAI *ai, struct blob *blobs){
//   fprintf(stderr,"state %d kick\n", ai->st.state);
//   // Assume ball is grabbed and robot is aligned
//   // drive forward for some time
//   BT_turn(MOTOR_D, 100, MOTOR_A, 100);
//   sleep(1);
  
//   // BT_timed_motor_port_start(MOTOR_D, 100,0,2000,0);
//   // BT_timed_motor_port_start(MOTOR_A, 80,0,2000,0);

//   // release the ball ASAP
//   BT_motor_port_start(MOTOR_B, 100);
//   sleep(0.5);
//   BT_motor_port_stop(MOTOR_A, 0);
//   sleep(1);

//   // stop moving
//   BT_all_stop(1);
// }

// ////////////////////////////////////////////////////////////////////////////////////
// // PLAY SOCCER LOGIC
// ////////////////////////////////////////////////////////////////////////////////////

// void initialize_soccer(struct RoboAI *ai, struct blob *blobs)
// {
//   // Set enemy and own goal coords, robot facing dir, etc.
//   enemy_goal_x = (ai -> st.side) * sx;
//   enemy_goal_y = (double) sy / 2;
//   BT_motor_port_start(MOTOR_B, 100);
//   sleep(2);
//   BT_all_stop(1);

//   fprintf(stderr,"state %d move_forward_no_motion_direction\n", ai->st.state);
//   if (!(ai->st.selfID && ai->st.ballID)) {
//     return;
//   }
//   // Drive forward
//   DRIVE_FORWARD

//   if (ai->st.self->mx != 0 && ai->st.self->my != 0)
//   {
//     // has a heading direction
//     fprintf(stderr,"Found heading direction %f %f\n", ai->st.self->mx,ai->st.self->my);
//     cleaned_mx = ai->st.self->mx;
//     cleaned_my = ai->st.self->my;
//     ai->st.state = T[ai->st.state][SUCCESS];
//     BT_all_stop(1);
//     return;
//   }
// }

// void strategy_check(struct RoboAI *ai, struct blob *blobs)
// {
//   // If ball is closer to enemy than us, defend, else tackle
//   double robotBallDiffX = ai -> st.ball->cx - ai -> st.self->cx;
//   double robotBallDiffY = ai -> st.ball->cy - ai -> st.self->cy;
//   double robotBallDiffNorm = sqrt(pow(robotBallDiffX, 2) + pow(robotBallDiffY, 2));

//   double oppBallDiffX = ai -> st.ball->cx - ai -> st.opp->cx;
//   double oppBallDiffY = ai -> st.ball->cy - ai -> st.opp->cy;
//   double oppBallDiffNorm = sqrt(pow(oppBallDiffX, 2) + pow(oppBallDiffY, 2));

//   // if (robotBallDiffNorm < oppBallDiffNorm) {
//   //   ai->st.state = T[ai->st.state][OFFENSE];
//   // } else {
//   //   ai->st.state = T[ai->st.state][TACKLE];
//   // }
// }

// void initialize_defense(struct RoboAI *ai, struct blob *blobs)
// {
  
// }

// void move_to_intercept(struct RoboAI *ai, struct blob *blobs)
// {
  
// }

// void slow_advance(struct RoboAI *ai, struct blob *blobs)
// {
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// void clear_shot_check(struct RoboAI *ai, struct blob *blobs)
// {
//   ai->st.state = T[ai->st.state][SUCCESS];
// }

// void initiate_dribble(struct RoboAI *ai, struct blob *blobs)
// {
  
// }

// void dribble_kick(struct RoboAI *ai, struct blob *blobs)
// {
  
// }

// void initialize_tackle(struct RoboAI *ai, struct blob *blobs)
// {

// }

// void collision_detection(struct RoboAi *ai, struct blob *blobs) {

// }