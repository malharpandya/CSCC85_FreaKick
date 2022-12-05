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

#include "roboAI.h" // <--- Look at this header file!
extern int sx;      // Get access to the image size from the imageCapture module
extern int sy;
int laggy = 0;
void (*state_functions[300])(struct RoboAI *ai);
int T[300][20];

// when calling a helper function, set this flag to 0 at the start, and change to 1
// if there is any sort of error, handle the error itself in the state_function
int ERROR_FLAG = 0;

// sanitized blob variables, once set, we update these and only use these
// never access the blob directly anymore
double self_dx = 0;
double self_dy = 0;
double self_x, self_y;
double ball_x, ball_y;
double enemy_x, enemy_y;

// target for robot
double target_x, target_y;
// Goal positions
double enemy_goal_x = -1;
// motor powers for penalty get to target
int penalty_drive_left = -20, penalty_drive_right = -20;
// penalty thresholds
double penalty_offset_threshold = 200;     // TUNE run up length
double penalty_proximity_threshold = 40;   // TUNE distance to target
double penalty_alignment_threshold = 0.25; // TUNE (in radians)
int penalty_turn_power = 20;               // TUNE
int penalty_shoot_threshold = 7;           // TUNE number of frames the penalty kick drives forward for
// static counter
static int penalty_shoot_counter = 0;
// Previous values
double prev_err = 0;
double prev_self_dx = 0;
double prev_self_dy = 0;
double prev_self_x = 0;
double prev_self_y = 0;
double prev_ball_x = 0;
double prev_ball_y = 0;
// PENALTY PID stuff
double pastError[PID_TIME];
double penalty_kp = 15;
double penalty_kd = 5;
double penalty_ki = 0.09;

// RoboSoccer Kickoff thresholds
double kickoff_offset_threshold = 200;
double kickoff_center_proximity_threshold = 50;
double kickoff_kick_counter_threshold = 8;
double kickoff_kick_counter = 0;

// motor powers for Kickoff
// int kickoff_drive_left = -80, kickoff_drive_right = -80;
int kickoff_drive_left = -40, kickoff_drive_right = -40;
int kickoff_shoot_drive_left = -90, kickoff_shoot_drive_right = -90;

// RoboSoccer KICKOFF PID stuff
// double kickoff_kp = 35;
// double kickoff_kd = 10;
// double kickoff_ki = 0.09;
// double kickoff_shoot_kp = 40;
// double kickoff_shoot_kd = 5;
// double kickoff_shoot_ki = 0.09;
double kickoff_kp = 35;
double kickoff_kd = 5;
double kickoff_ki = 0.09;
double kickoff_shoot_kp = 30;
double kickoff_shoot_kd = 5;
double kickoff_shoot_ki = 0.09;

// RoboSoccer thresholds
double vs_proximity_threshold = 150;
double kick_distance = 200; // TUNE
double flick_distance = 100; // TUNE
double side_buffer = 50; // TUNE feasibility check
double ball_moved_threshold = 20;
double vs_alignment_threshold = 0.2;
double vs_turn_power = 20;
double attack_counter = 0;
double attack_counter_threshold = 10;
double stuck_counter = 0;
double stuck_counter_threshold = 20;
double reverse_counter = 0;
double reverse_counter_threshold = 5;

// RoboSoccer PID stuff
double vs_kp = 15;
double vs_kd = 5;
double vs_ki = 0.09;

// Previous State
int prev_state;

// motor powers for Kickoff
int vs_drive_left = -40, vs_drive_right = -40;

// Flick
int flick_counter = 0;
int flick_counter_threshold = 5;

// DRAWING
struct displayList *tempDPhead;
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
  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL)
  {
    fprintf(stderr, "addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type = 0;
  newNode->x1 = x;
  newNode->y1 = y;
  newNode->x2 = -1;
  newNode->y2 = -1;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;

  newNode->next = head;
  return (newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL)
  {
    fprintf(stderr, "addLine(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x1;
  newNode->y1 = y1;
  newNode->x2 = x2;
  newNode->y2 = y2;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  return (newNode);
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;

  l = sqrt((dx * dx) + (dy * dy));
  dx = dx / l;
  dy = dy / l;

  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL)
  {
    fprintf(stderr, "addVector(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x1;
  newNode->y1 = y1;
  newNode->x2 = x1 + (length * dx);
  newNode->y2 = y1 + (length * dy);
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  return (newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL)
  {
    fprintf(stderr, "addLine(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x - length;
  newNode->y1 = y;
  newNode->x2 = x + length;
  newNode->y2 = y;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  head = newNode;

  newNode = (struct displayList *)calloc(1, sizeof(struct displayList));
  if (newNode == NULL)
  {
    fprintf(stderr, "addLine(): Out of memory!\n");
    return head;
  }
  newNode->type = 1;
  newNode->x1 = x;
  newNode->y1 = y - length;
  newNode->x2 = x;
  newNode->y2 = y + length;
  newNode->R = R;
  newNode->G = G;
  newNode->B = B;
  newNode->next = head;
  return (newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while (head)
  {
    q = head->next;
    free(head);
    head = q;
  }
  return (NULL);
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
  double vr_x, vr_y, maxfit, mincos, dp;
  double vb_x, vb_y, fit;
  double maxsize = 0;
  double maxgray;
  int grayness;
  int i;
  static double Mh[4] = {-1, -1, -1, -1};
  static double mx0, my0, mx1, my1, mx2, my2;
  FILE *f;

  // Import calibration data from file - this will contain the colour values selected by
  // the user in the U.I.
  if (Mh[0] == -1)
  {
    f = fopen("colours.dat", "r");
    if (f != NULL)
    {
      fread(&Mh[0], 4 * sizeof(double), 1, f);
      fclose(f);
      mx0 = cos(Mh[0]);
      my0 = sin(Mh[0]);
      mx1 = cos(Mh[1]);
      my1 = sin(Mh[1]);
      mx2 = cos(Mh[2]);
      my2 = sin(Mh[2]);
    }
  }

  if (Mh[0] == -1)
  {
    fprintf(stderr, "roboAI.c :: id_coloured_blob2(): No colour calibration data, can not ID blobs. Please capture colour calibration data on the U.I. first\n");
    return NULL;
  }

  maxfit = .025; // Minimum fitness threshold
  mincos = .90;  // Threshold on colour angle similarity
  maxgray = .25; // Maximum allowed difference in colour
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
  if (col == 0)
  {
    vr_x = mx0;
    vr_y = my0;
  }
  else if (col == 1)
  {
    vr_x = mx1;
    vr_y = my1;
  }
  else if (col == 2)
  {
    vr_x = mx2;
    vr_y = my2;
  }

  // In what follows, colours are represented by a unit-length vector in the direction of the
  // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
  // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
  // If the dot product is 1 the colours are identical (their vectors perfectly aligned),
  // from there, the dot product decreases as the colour vectors start to point in different
  // directions. Two colours that are opposite will result in a dot product of -1.

  p = blobs;
  while (p != NULL)
  {
    if (p->size > maxsize)
      maxsize = p->size;
    p = p->next;
  }

  p = blobs;
  fnd = NULL;
  while (p != NULL)
  {
    // Normalization and range extension
    vb_x = cos(p->H);
    vb_y = sin(p->H);

    dp = (vb_x * vr_x) + (vb_y * vr_y); // Dot product between the reference color vector, and the
                                        // blob's color vector.

    fit = dp * p->S * p->S * (p->size / maxsize); // <<< --- This is the critical matching criterion.
                                                  // * THe dot product with the reference direction,
                                                  // * Saturation squared
                                                  // * And blob size (in pixels, not from bounding box)
                                                  // You can try to fine tune this if you feel you can
                                                  // improve tracking stability by changing this fitness
                                                  // computation

    // Check for a gray-ish blob - they tend to give trouble
    grayness = 0;
    if (fabs(p->R - p->G) / p->R < maxgray && fabs(p->R - p->G) / p->G < maxgray && fabs(p->R - p->B) / p->R < maxgray && fabs(p->R - p->B) / p->B < maxgray &&
        fabs(p->G - p->B) / p->G < maxgray && fabs(p->G - p->B) / p->B < maxgray)
      grayness = 1;

    if (fit > maxfit && dp > mincos && grayness == 0)
    {
      fnd = p;
      maxfit = fit;
    }

    p = p->next;
  }

  return (fnd);
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
  double mg, vx, vy, pink, doff, dmin, dmax, adj;

  // Reset ID flags and agent blob pointers
  ai->st.ballID = 0;
  ai->st.selfID = 0;
  ai->st.oppID = 0;
  ai->st.ball = NULL; // Be sure you check these are not NULL before
  ai->st.self = NULL; // trying to access data for the ball/self/opponent!
  ai->st.opp = NULL;

  // Find the ball
  p = id_coloured_blob2(ai, blobs, 2);
  if (p)
  {
    ai->st.ball = p;                     // New pointer to ball
    ai->st.ballID = 1;                   // Set ID flag for ball (we found it!)
    ai->st.bvx = p->cx - ai->st.old_bcx; // Update ball velocity in ai structure and blob structure
    ai->st.bvy = p->cy - ai->st.old_bcy;
    ai->st.ball->vx = ai->st.bvx;
    ai->st.ball->vy = ai->st.bvy;
    ai->st.bdx = p->dx;
    ai->st.bdy = p->dy;

    ai->st.old_bcx = p->cx; // Update old position for next frame's computation
    ai->st.old_bcy = p->cy;
    ai->st.ball->idtype = 3;

    vx = ai->st.bvx; // Compute motion direction (normalized motion vector)
    vy = ai->st.bvy;
    mg = sqrt((vx * vx) + (vy * vy));
    if (mg > NOISE_VAR) // Update heading vector if meaningful motion detected
    {
      vx /= mg;
      vy /= mg;
      ai->st.bmx = vx;
      ai->st.bmy = vy;
    }
    else
    {
      ai->st.bmx = 0;
      ai->st.bmy = 0;
    }
    ai->st.ball->mx = ai->st.bmx;
    ai->st.ball->my = ai->st.bmy;
  }
  else
  {
    ai->st.ball = NULL;
  }

  // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
  p = id_coloured_blob2(ai, blobs, ai->st.botCol);
  if (p != NULL && p != ai->st.ball)
  {
    ai->st.self = p; // Update pointer to self-blob
    ai->st.selfID = 1;
    ai->st.svx = p->cx - ai->st.old_scx;
    ai->st.svy = p->cy - ai->st.old_scy;
    ai->st.self->vx = ai->st.svx;
    ai->st.self->vy = ai->st.svy;
    ai->st.sdx = p->dx;
    ai->st.sdy = p->dy;

    vx = ai->st.svx;
    vy = ai->st.svy;
    mg = sqrt((vx * vx) + (vy * vy));
    //  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf], old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf, %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
    if (mg > NOISE_VAR)
    {
      vx /= mg;
      vy /= mg;
      ai->st.smx = vx;
      ai->st.smy = vy;
    }
    else
    {
      ai->st.smx = 0;
      ai->st.smy = 0;
    }
    ai->st.self->mx = ai->st.smx;
    ai->st.self->my = ai->st.smy;
    ai->st.old_scx = p->cx;
    ai->st.old_scy = p->cy;
    ai->st.self->idtype = 1;
  }
  else
    ai->st.self = NULL;

  // ID our opponent - whatever colour is not botCol
  if (ai->st.botCol == 0)
    p = id_coloured_blob2(ai, blobs, 1);
  else
    p = id_coloured_blob2(ai, blobs, 0);
  if (p != NULL && p != ai->st.ball && p != ai->st.self)
  {
    ai->st.opp = p;
    ai->st.oppID = 1;
    ai->st.ovx = p->cx - ai->st.old_ocx;
    ai->st.ovy = p->cy - ai->st.old_ocy;
    ai->st.opp->vx = ai->st.ovx;
    ai->st.opp->vy = ai->st.ovy;
    ai->st.odx = p->dx;
    ai->st.ody = p->dy;

    ai->st.old_ocx = p->cx;
    ai->st.old_ocy = p->cy;
    ai->st.opp->idtype = 2;

    vx = ai->st.ovx;
    vy = ai->st.ovy;
    mg = sqrt((vx * vx) + (vy * vy));
    if (mg > NOISE_VAR)
    {
      vx /= mg;
      vy /= mg;
      ai->st.omx = vx;
      ai->st.omy = vy;
    }
    else
    {
      ai->st.omx = 0;
      ai->st.omy = 0;
    }
    ai->st.opp->mx = ai->st.omx;
    ai->st.opp->my = ai->st.omy;
  }
  else
    ai->st.opp = NULL;
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
  static double stepID = 0;
  static double oldX, oldY;
  double frame_inc = 1.0 / 5.0;
  double dist;

  track_agents(ai, blobs); // Call the tracking function to find each agent

  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -100); // Start forward motion to establish heading
                                          // Will move for a few frames.

  if (ai->st.selfID == 1 && ai->st.self != NULL)
    fprintf(stderr, "Successfully identified self blob at (%f,%f)\n", ai->st.self->cx, ai->st.self->cy);
  if (ai->st.oppID == 1 && ai->st.opp != NULL)
    fprintf(stderr, "Successfully identified opponent blob at (%f,%f)\n", ai->st.opp->cx, ai->st.opp->cy);
  if (ai->st.ballID == 1 && ai->st.ball != NULL)
    fprintf(stderr, "Successfully identified ball blob at (%f,%f)\n", ai->st.ball->cx, ai->st.ball->cy);

  stepID += frame_inc;
  if (stepID >= 1 && ai->st.selfID == 1) // Stop after a suitable number of frames.
  {
    ai->st.state += 1;
    stepID = 0;
    BT_all_stop(0);
  }
  else if (stepID >= 1)
    stepID = 0;

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

  switch (mode)
  {
  case AI_SOCCER:
    fprintf(stderr, "Standard Robo-Soccer mode requested\n");
    ai->st.state = 0; // <-- Set AI initial state to 0
    break;
  case AI_PENALTY:
    // 	fprintf(stderr,"Penalty mode! let's kick it!\n");
    ai->st.state = 100; // <-- Set AI initial state to 100
    break;
  case AI_CHASE:
    fprintf(stderr, "Chasing the ball...\n");
    ai->st.state = 200; // <-- Set AI initial state to 200
    break;
  default:
    fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
    ai->st.state = 0;
  }

  BT_all_stop(0);      // Stop bot,
  ai->runAI = AI_main; // and initialize all remaining AI data
  ai->calibrate = AI_calibrate;
  ai->st.ball = NULL;
  ai->st.self = NULL;
  ai->st.opp = NULL;
  ai->st.side = 0;
  ai->st.botCol = own_col;
  ai->st.old_bcx = 0;
  ai->st.old_bcy = 0;
  ai->st.old_scx = 0;
  ai->st.old_scy = 0;
  ai->st.old_ocx = 0;
  ai->st.old_ocy = 0;
  ai->st.bvx = 0;
  ai->st.bvy = 0;
  ai->st.svx = 0;
  ai->st.svy = 0;
  ai->st.ovx = 0;
  ai->st.ovy = 0;
  ai->st.sdx = 0;
  ai->st.sdy = 0;
  ai->st.odx = 0;
  ai->st.ody = 0;
  ai->st.bdx = 0;
  ai->st.bdy = 0;
  ai->st.selfID = 0;
  ai->st.oppID = 0;
  ai->st.ballID = 0;
  ai->DPhead = NULL;

  // PENALTY
  state_functions[101] = penalty_select_target;
  state_functions[102] = penalty_get_to_target;
  state_functions[103] = penalty_align_to_goal;
  state_functions[104] = penalty_shoot;
  state_functions[105] = penalty_finish;

  // Test tree
  T[101][PENALTY_TARGET_INFEASIBLE] = 101;
  T[101][PENALTY_TARGET_SELECTED] = 102;
  T[102][PENALTY_TARGET_REACHED] = 103;
  T[103][PENALTY_ALIGNED] = 104;
  T[104][PENALTY_SHOOT_COMPLETE] = 105;
  // we exit the program in state 105 after the ball is kicked;

  state_functions[1] = kickoff_setup;
  state_functions[2] = kickoff_head_to_center;
  state_functions[3] = kickoff_kick;
  state_functions[4] = robosoccer_select_tactic;
  state_functions[5] = attack;
  state_functions[9] = align_to_goal;
  state_functions[10] = shoot;
  state_functions[6] = defend;
  state_functions[7] = flick;
  state_functions[8] = tackle;
  // state_functions[11] = unstuck;

  T[1][SUCCESS] = 4;
  T[2][SUCCESS] = 3;
  T[3][SUCCESS] = 4;
  T[4][ATTACK] = 5;
  T[4][DEFEND] = 6;
  T[4][TACKLE] = 8;
  T[7][SUCCESS] = 4;
  T[5][SUCCESS] = 9;
  T[6][SUCCESS] = 7;
  T[8][SUCCESS] = 4;
  T[9][SUCCESS] = 10;
  T[10][SUCCESS] = 4;

  T[5][SELECT_TACTIC] = 4;
  T[6][SELECT_TACTIC] = 4;
  T[5][FLICK] = 7;

  fprintf(stderr, "Initialized!\n");

  return (1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
  // Basic colour blob tracking loop for calibration of vertical offset
  // See the handout for the sequence of steps needed to achieve calibration.
  // The code here just makes sure the image processing loop is constantly
  // tracking the bots while they're placed in the locations required
  // to do the calibration (i.e. you DON'T need to add anything more
  // in this function).
  track_agents(ai, blobs);
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

  static double ux, uy, len, mmx, mmy, tx, ty, x1, y1, x2, y2;
  double angDif;
  char line[1024];
  static int count = 0;
  static double old_dx = 0, old_dy = 0;

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
  if (ai->st.state == 0 || ai->st.state == 100 || ai->st.state == 200) // Initial set up - find own, ball, and opponent blobs
  {
    // Carry out self id process.
    fprintf(stderr, "Initial state, self-id in progress...\n");

    id_bot(ai, blobs);
    if ((ai->st.state % 100) != 0) // The id_bot() routine will change the AI state to initial state + 1
    {                              // if robot identification is successful.

      if (ai->st.self->cx >= 512)
        ai->st.side = 1;
      else
        ai->st.side = 0; // This sets the side the bot thinks as its own side 0->left, 1->right
      BT_all_stop(0);

      fprintf(stderr, "Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n", ai->st.self->cx, ai->st.self->cy, ai->st.smx, ai->st.smy, ai->st.sdx, ai->st.sdy, ai->st.state);

      if (ai->st.self != NULL)
      {
        // This checks that the motion vector and the blob direction vector
        // are pointing in the same direction. If they are not (the dot product
        // is less than 0) it inverts the blob direction vector so it points
        // in the same direction as the motion vector.
        if (((ai->st.smx * ai->st.sdx) + (ai->st.smy * ai->st.sdy)) < 0)
        {
          ai->st.self->dx *= -1.0;
          ai->st.self->dy *= -1.0;
          ai->st.sdx *= -1;
          ai->st.sdy *= -1;
        }
        old_dx = ai->st.sdx;
        old_dy = ai->st.sdy;
      }

      if (ai->st.opp != NULL)
      {
        // Checks motion vector and blob direction for opponent. See above.
        if (((ai->st.omx * ai->st.odx) + (ai->st.omy * ai->st.ody)) < 0)
        {
          ai->st.opp->dx *= -1;
          ai->st.opp->dy *= -1;
          ai->st.odx *= -1;
          ai->st.ody *= -1;
        }
      }
    }

    // Initialize BotInfo structures
    tempDPhead = ai->DPhead;
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
    // fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
    ai->DPhead = tempDPhead;
    track_agents(ai, blobs); // Currently, does nothing but endlessly track
    update_global(ai);
    if (enemy_goal_x == -1)
    {
      return;
    }
    (*state_functions[ai->st.state])(ai);
    // ai->DPhead = addVector(ai->DPhead, self_x, self_y, self_dx, self_dy, 200, 255,0,0);
    ai->DPhead = addCross(ai->DPhead, target_x, target_y, 30, 0, 0, 255); // Draws target cross
    ai->DPhead = addCross(ai->DPhead, enemy_goal_x, sy / 2, 30, 0, 255, 0);
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, self_dx, self_dy, 300, 255, 0, 0);
    // IF ANY PREVIOUS VALUES NEEDED
    // SET THEM HERE
    prev_self_dx = self_dx;
    prev_self_dy = self_dy;
    prev_self_x = self_x;
    prev_self_y = self_y;
    prev_ball_x = ball_x;
    prev_ball_y = ball_y;
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

//////////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS (do not change state within any of the helpers, always pass mode)
//////////////////////////////////////////////////////////////////////////////////////
void update_global(struct RoboAI *ai)
{
  // reset error flag
  ERROR_FLAG = 0;
  // line 504 sets the right heading direction by moving self for a few frames
  // check if both robot and ball visible
  if (ai != NULL)
  {
    int mode = ai->st.state / 100;
    if (enemy_goal_x == -1)
    {
      enemy_goal_x = sx * (1 - ai->st.side);
    }
    if (ai->st.selfID && ai->st.self != NULL)
    {
      self_x = ai->st.self->cx;
      self_y = ai->st.self->cy;

      if (!self_dx && !self_dy)
      {
        self_dx = ai->st.self->mx;
        self_dy = ai->st.self->my;
      }
      else
      {
        if (abs(find_angle(1, prev_self_dx, prev_self_dy, ai->st.self->dx, ai->st.self->dy)) < 1.5)
        {
          self_dx = ai->st.self->dx;
          self_dy = ai->st.self->dy;
        }
        else
        {
          self_dx = -ai->st.self->dx;
          self_dy = -ai->st.self->dy;
        }
      }
      fprintf(stderr, "the robot is facing: %f, %f\n", self_dx, self_dy);
    }
    else
    {
      fprintf(stderr, "ROBOT IS NOT VISIBLE\n");
      ERROR_FLAG = 1;
    }
    if (ai->st.ballID && ai->st.ball != NULL)
    {
      ball_x = ai->st.ball->cx;
      ball_y = ai->st.ball->cy;
    }
    else
    {
      fprintf(stderr, "BALL IS NOT VISIBLE\n");
      ERROR_FLAG = 1;
    }
    if (mode == 0)
    {
      if (ai->st.oppID && ai->st.opp != NULL)
      {
        enemy_x = ai->st.opp->cx;
        enemy_y = ai->st.opp->cy;
      }
      else
      {
        fprintf(stderr, "ENEMY NOT VISIBLE IN PLAY SOCCER MODE\n");
        ERROR_FLAG = 1;
      }
    }
    else
    {
      if (ai->st.oppID && ai->st.opp != NULL)
      {
        fprintf(stderr, "GET THE ENEMY OFF THE FIELD\n");
        ERROR_FLAG = 1;
      }
    }
  }
  else
  {
    fprintf(stderr, "ai points to NULL\n");
    ERROR_FLAG = 1;
  }

  if (!ERROR_FLAG)
  {
    // fprintf(stderr,"Global variables have been updated\n");
  }
}
double norm(int mode, double x, double y)
{
  // for now I don't think we need a mode specific norm
  return sqrt(pow(x, 2) + pow(y, 2));
}

double find_angle(int mode, double x1, double y1, double x2, double y2)
{
  return atan2(x1 * y2 - x2 * y1, x1 * x2 + y1 * y2);
}
//////////////////////////////////////////////////////////////////////////////////////
// PENALTY FUNCTIONS
// all these functions work under the following assumptions
// 1. There is no enemy on the field (it ignores the opp blob)
// 2. The ball and robot are in a feasible orientation (we can reach the offset and shoot)
// 3. The action of moving to the offset will not move the ball
//////////////////////////////////////////////////////////////////////////////////////

void penalty_select_target(struct RoboAI *ai)
{
  // reset error flag
  ERROR_FLAG = 0;
  // identify vector from enemy goal centre to the ball
  double vec2ball_x = ball_x - enemy_goal_x;
  double vec2ball_y = ball_y - (sy / 2);
  double distance_to_goal = norm(1, vec2ball_x, vec2ball_y);
  target_x = ball_x + (penalty_offset_threshold) * (vec2ball_x / distance_to_goal);
  target_y = ball_y + (penalty_offset_threshold) * (vec2ball_y / distance_to_goal);
  // although we can assume target is feasible, just double check
  ERROR_FLAG = (target_x <= side_buffer || target_x >= (sx - side_buffer) || target_y <= side_buffer || target_y >= (sy - side_buffer));
  if (!ERROR_FLAG)
  {
    fprintf(stderr, "Target feasible, proceeding to next stage\n");
    ai->st.state = T[ai->st.state][PENALTY_TARGET_SELECTED];
  }
  else
  {
    // infeasible target
    fprintf(stderr, "Target is infeasible, request another orientation\n");
    ai->st.state = T[ai->st.state][PENALTY_TARGET_INFEASIBLE];
  }
}

void penalty_get_to_target(struct RoboAI *ai)
{
  // reset error flag
  ERROR_FLAG = 0;
  // the penalty PID can be slow, theres no race to get the ball
  // assumes the path to target will not move the ball

  if (norm(1, target_x - self_x, target_y - self_y) <= penalty_proximity_threshold)
  {
    // stop moving
    BT_all_stop(1);
    // reached target
    fprintf(stderr, "Target reached, proceeding to align with the goal\n");
    ai->st.state = T[ai->st.state][PENALTY_TARGET_REACHED];
  }
  else
  {
    // find vector to target, and angle (error) wrt to currrent motion direction
    double vec2tgt_x = target_x - self_x;
    double vec2tgt_y = target_y - self_y;
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, vec2tgt_x, vec2tgt_y, 200, 0, 255, 0); // Vector to target
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, enemy_goal_x - self_x, (sy / 2) - self_y, 1000, 255, 255, 0);// Vector to enemy goal
    double curr_err = find_angle(1, self_dx, self_dy, vec2tgt_x, vec2tgt_y);
    if (M_PI / 2 - abs(curr_err) < 0.4)
    {
      fprintf(stderr, "ROTATING INSTEAD OF PID\n");
      int turn = (curr_err < 0) ? 1 : -1;
      BT_turn(MOTOR_D, turn * penalty_turn_power, MOTOR_A, turn * -penalty_turn_power);
    }
    else
    {
      double delta_err = curr_err - prev_err;
      double int_err = updateInt(1, ai, curr_err);
      prev_err = curr_err;

      // PID HERE (MORE FINE TUNED THAN THE PLAY SOCCER ONE) @JACKON fix plz
      fprintf(stderr, "penalty_get_to_target PID: e: %f de: %f, integral e: %f\n", curr_err, delta_err, int_err);
      double u = penalty_kp * curr_err + penalty_kd * delta_err + penalty_ki * int_err; // error b/w -pi and pi
      fprintf(stderr, "pid result u: %f\n", u);

      // double temp = u > -30 ? u : -30;
      // u = 30 > temp ? temp : 30;
      // penalty_drive_left -= u;
      // penalty_drive_right += u;
      // penalty_drive_left = std::fmin(100, std::fmax(-100, penalty_drive_left));
      // penalty_drive_right = std::fmin(100, std::fmax(-100, penalty_drive_right));
      // fprintf(stderr, "left_motor_speed: %d right_motor_speed: %d\n", penalty_drive_left, penalty_drive_right);
      double drive_left = std::fmin(100.0, std::fmax(-100.0, penalty_drive_left - u));
      double drive_right = std::fmin(100.0, std::fmax(-100.0, penalty_drive_right + u));
      fprintf(stderr, "left_motor_speed: %f right_motor_speed: %f\n", drive_left, drive_right);
      BT_turn(MOTOR_D, drive_left, MOTOR_A, drive_right);
    }
  }
}

void penalty_align_to_goal(struct RoboAI *ai)
{
  printf("Aligning to goal\n");
  // reset error flag
  ERROR_FLAG = 0;
  double angle = find_angle(1, self_dx, self_dy, enemy_goal_x - self_x, (sy / 2) - self_y);
  if (abs(angle) <= penalty_alignment_threshold)
  {
    // stop turning
    BT_all_stop(1);
    // aligned
    fprintf(stderr, "Robot has aligned to the goal, proceeding to shoot the ball\n");
    ai->st.state = T[ai->st.state][PENALTY_ALIGNED];
  }
  else
  {
    // need to turn, no need for a PID here, just turn at medium-ish pace
    // MAKE SURE YOU TURN IN PLACE (centre of rotation b/w wheels)
    // turn right  = 1, turn left = -1
    int turn = (angle < 0) ? 1 : -1;
    BT_turn(MOTOR_D, turn * penalty_turn_power, MOTOR_A, turn * -penalty_turn_power);
  }
}

void penalty_shoot(struct RoboAI *ai)
{
  // reset error flag
  ERROR_FLAG = 0;
  printf("penalty_shoot_counter: %d\n", penalty_shoot_counter);
  if (penalty_shoot_counter > penalty_shoot_threshold)
  {
    // set counter back to 0
    penalty_shoot_counter = 0;
    BT_all_stop(1);
    // kick finished
    fprintf(stderr, "Kick finished, proceeding to end state\n");
    ai->st.state = T[ai->st.state][PENALTY_SHOOT_COMPLETE];
  }
  else
  {
    // Launch control
    BT_turn(MOTOR_D, -40 - penalty_shoot_counter * 10, MOTOR_A, -40 - penalty_shoot_counter * 10);
    penalty_shoot_counter++;
  }
}

void penalty_finish(struct RoboAI *ai)
{
  // reset anything that needs to be reset
  // maybe have some audio playing?
  // if we score do a celebration?
  // successfully exit
  exit(1);
}

double updateInt(int mode, struct RoboAI *ai, double curr_err)
{
  fprintf(stderr, "start updateInt\n");
  double sum = 0;
  for (int k = PID_TIME - 1; k > 0; k--)
  {
    pastError[k] = pastError[k - 1];
    sum += pastError[k];
  }
  pastError[0] = curr_err;
  return sum;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kickoff_setup(struct RoboAI *ai)
{
  ERROR_FLAG = 0;
  // set the target
  double delta_x = self_x - ball_x; //ball_x - enemy_goal_x;
  double delta_y = self_y - ball_y; //ball_y - sy / 2;
  double length = norm(1, delta_x, delta_y);
  target_x = ball_x + delta_x * kickoff_offset_threshold / length;
  target_y = ball_y + delta_y * kickoff_offset_threshold / length;

  ai->st.state = T[ai->st.state][SUCCESS];
}

void kickoff_head_to_center(struct RoboAI *ai)
{
  // the penalty PID can be slow, theres no race to get the ball
  // assumes the path to target will not move the ball
  ERROR_FLAG = 0;
  // set the target
  double delta_x = self_x - ball_x; //ball_x - enemy_goal_x;
  double delta_y = self_y - ball_y; //ball_y - sy / 2;
  double length = norm(1, delta_x, delta_y);
  target_x = ball_x + delta_x * kickoff_offset_threshold / length;
  target_y = ball_y + delta_y * kickoff_offset_threshold / length;


  if (norm(1, target_x - self_x, target_y - self_y) <= kickoff_center_proximity_threshold)
  {
    // reached target
    fprintf(stderr, "Target reached, proceeding to align with the goal\n");
    ai->st.state = T[ai->st.state][SUCCESS];
    return;
  }
  else
  {
    // find vector to target, and angle (error) wrt to currrent motion direction
    double vec2tgt_x = target_x - self_x;
    double vec2tgt_y = target_y - self_y;
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, vec2tgt_x, vec2tgt_y, 200, 0, 255, 0); // Vector to target
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, enemy_goal_x - self_x, (sy / 2) - self_y, 1000, 255, 255, 0);
    double curr_err = find_angle(1, self_dx, self_dy, vec2tgt_x, vec2tgt_y);
    double delta_err = curr_err - prev_err;
    double int_err = updateInt(1, ai, curr_err);
    prev_err = curr_err;

    // PID HERE (MORE FINE TUNED THAN THE PLAY SOCCER ONE) @JACKON fix plz
    fprintf(stderr, "kickoff_head_to_center PID: e: %f de: %f, integral e: %f\n", curr_err, delta_err, int_err);
    double u = kickoff_kp * curr_err + kickoff_kd * delta_err + kickoff_ki * int_err; // error b/w -pi and pi
    fprintf(stderr, "pid result u: %f\n", u);

    // double temp = u > -30 ? u : -30;
    // u = 30 > temp ? temp : 30;
    // penalty_drive_left -= u;
    // penalty_drive_right += u;
    // penalty_drive_left = std::fmin(100, std::fmax(-100, penalty_drive_left));
    // penalty_drive_right = std::fmin(100, std::fmax(-100, penalty_drive_right));
    // fprintf(stderr, "left_motor_speed: %d right_motor_speed: %d\n", penalty_drive_left, penalty_drive_right);
    double drive_left = std::fmin(100.0, std::fmax(-100.0, kickoff_drive_left - u));
    double drive_right = std::fmin(100.0, std::fmax(-100.0, kickoff_drive_right + u));
    fprintf(stderr, "left_motor_speed: %f right_motor_speed: %f\n", drive_left, drive_right);
    BT_turn(MOTOR_D, drive_left, MOTOR_A, drive_right);
  }
}

void kickoff_kick(struct RoboAI *ai)
{
  ERROR_FLAG = 0;

  kickoff_kick_counter += 1;

  // set the target
  target_x = enemy_goal_x;
  target_y = sy / 2;

  // the penalty PID can be slow, theres no race to get the ball
  // assumes the path to target will not move the ball

  if (kickoff_kick_counter >= kickoff_kick_counter_threshold)
  {
    // reached target
    fprintf(stderr, "Kickoff_kick_finished, proceeding to default\n");
    ai->st.state = T[ai->st.state][SUCCESS];
    BT_all_stop(1);
    kickoff_kick_counter = 0;
    return;
  }
  else
  {
    // find vector to target, and angle (error) wrt to currrent motion direction
    double vec2tgt_x = target_x - self_x;
    double vec2tgt_y = target_y - self_y;
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, vec2tgt_x, vec2tgt_y, 200, 0, 255, 0); // Vector to target
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, enemy_goal_x - self_x, (sy / 2) - self_y, 1000, 255, 255, 0);
    double curr_err = find_angle(1, self_dx, self_dy, vec2tgt_x, vec2tgt_y);
    double delta_err = curr_err - prev_err;
    double int_err = updateInt(1, ai, curr_err);
    prev_err = curr_err;

    // PID HERE (MORE FINE TUNED THAN THE PLAY SOCCER ONE) @JACKON fix plz
    fprintf(stderr, "kickoff_kick PID: e: %f de: %f, integral e: %f\n", curr_err, delta_err, int_err);
    double u = kickoff_shoot_kp * curr_err + kickoff_shoot_kd * delta_err + kickoff_shoot_ki * int_err; // error b/w -pi and pi
    fprintf(stderr, "pid result u: %f\n", u);

    // double temp = u > -30 ? u : -30;
    // u = 30 > temp ? temp : 30;
    // penalty_drive_left -= u;
    // penalty_drive_right += u;
    // penalty_drive_left = std::fmin(100, std::fmax(-100, penalty_drive_left));
    // penalty_drive_right = std::fmin(100, std::fmax(-100, penalty_drive_right));
    // fprintf(stderr, "left_motor_speed: %d right_motor_speed: %d\n", penalty_drive_left, penalty_drive_right);
    double drive_left = std::fmin(100.0, std::fmax(-100.0, kickoff_shoot_drive_left - u));
    double drive_right = std::fmin(100.0, std::fmax(-100.0, kickoff_shoot_drive_right + u));
    fprintf(stderr, "left_motor_speed: %f right_motor_speed: %f\n", drive_left, drive_right);
    BT_turn(MOTOR_D, drive_left, MOTOR_A, drive_right);
  }
}

int attack_mode(struct RoboAI *ai){
  return (enemy_goal_x == 0 ? (self_x-ball_x>=200):(ball_x-self_x>=200)) ? 1 : 0;
}

int defend_mode(struct RoboAI *ai){
  return (enemy_goal_x==0) ? (ball_x-self_x>=0):(self_x-ball_x>=0);
}

int tackle_mode(struct RoboAI *ai){
  double enemy_dist = norm(1, ball_x-enemy_x, ball_y-enemy_y);
  double self_dist = norm(1, ball_x-self_x, ball_y-self_y);
  return defend_mode(ai) && (self_dist/enemy_dist>=1.5 && self_dist >= 500);
}

void robosoccer_select_tactic(struct RoboAI *ai)
{
  printf("Selecting tactic\n");
  // double enemy_to_ball_x = ball_x - enemy_x;
  // double enemy_to_ball_y = ball_y - enemy_y;
  // double enemy_to_ball_norm = norm(1, enemy_to_ball_x, enemy_to_ball_y);

  // double self_to_ball_x = ball_x - self_x;
  // double self_to_ball_y = ball_y - self_y;
  // double self_to_ball_norm = norm(1, self_to_ball_x, self_to_ball_y);

  // if (self_to_ball_norm <= 150 && enemy_to_ball_norm <= 150) { // If we are both close to the ball, regardless of who is closer, go into tackle mode
  //   ai->st.state = T[ai->st.state][TACKLE];
  // } else if (enemy_to_ball_norm / self_to_ball_norm == 0.5) { // If the enemy is a certain ratio closer to the ball than us, go into defense mode
  //   ai->st.state = T[ai->st.state][DEFEND];
  // } else { // Else, we have a pretty decent chanse of beating the enemy to the ball, so go into attack mode
  //   ai->st.state = T[ai->st.state][ATTACK];
  // }

  if (attack_mode(ai))
  {
    ai->st.state = T[ai->st.state][ATTACK];
  }
  else if(defend_mode(ai))
  {
    double enemy_dist = norm(1, ball_x-enemy_x, ball_y-enemy_y);
    double self_dist = norm(1, ball_x-self_x, ball_y-self_y);
    // we need to be relatively far away from the ball than the enemy
    // so check relative distance to ball, and absolute dist to ball
    if (tackle_mode(ai))
    {
      ai->st.state = T[ai->st.state][DEFEND];
    }
    else
    {
      // we are close enough to the ball or closer to the ball than the enemy
      // go to tackle mode
      ai->st.state = T[ai->st.state][DEFEND];
      // ai->st.state = T[ai->st.state][TACKLE];
    }

  }
  else
  {
    ai->st.state = T[ai->st.state][DEFEND];
  }

}

int select_target(struct RoboAI *ai){
  // if within the buffer along the wall, set the ball as target else set the offset as target
  int returnValue = 0;
  double delta_x = ball_x - enemy_goal_x;
  double delta_y = ball_y - sy / 2.0;
  double length = norm(1, delta_x, delta_y);
  target_x = ball_x + delta_x * kick_distance / length;
  target_y = ball_y + delta_y * kick_distance / length;
  
  if (target_x < side_buffer || target_x > (sx - side_buffer) || target_y < side_buffer || target_y > (sy - side_buffer))
  {
    target_x = ball_x;
    target_y = ball_y;
    returnValue = 1;
    printf("Target: ball\n");
  } else {
    printf("Target: off ball\n");
  }

  printf("Target (x, y): (%f, %f)\n", target_x, target_y);
  return returnValue;
}

int get_to_target(struct RoboAI *ai, double target_threshold){
  // if 
  if (norm(1, target_x - self_x, target_y - self_y) <= target_threshold)
  {
    // reached target
    fprintf(stderr, "Target reached, proceeding to align with the goal\n");
    // ai->st.state = T[ai->st.state][SUCCESS];
    return 1;
  }
  else
  {
    // find vector to target, and angle (error) wrt to currrent motion direction
    double vec2tgt_x = target_x - self_x;
    double vec2tgt_y = target_y - self_y;
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, vec2tgt_x, vec2tgt_y, 200, 0, 255, 0); // Vector to target
    ai->DPhead = addVector(ai->DPhead, self_x, self_y, enemy_goal_x - self_x, (sy / 2) - self_y, 1000, 255, 255, 0);
    double curr_err = find_angle(1, self_dx, self_dy, vec2tgt_x, vec2tgt_y);
    double delta_err = curr_err - prev_err;
    double int_err = updateInt(1, ai, curr_err);
    prev_err = curr_err;

    // PID HERE (MORE FINE TUNED THAN THE PLAY SOCCER ONE) @JACKON fix plz
    fprintf(stderr, "get_to_target PID: e: %f de: %f, integral e: %f\n", curr_err, delta_err, int_err);
    double u = vs_kp * curr_err + vs_kd * delta_err + vs_ki * int_err; // error b/w -pi and pi
    fprintf(stderr, "pid result u: %f\n", u);
    
    double drive_left = std::fmin(100.0, std::fmax(-100.0, vs_drive_left - u));
    double drive_right = std::fmin(100.0, std::fmax(-100.0, vs_drive_right + u));
    fprintf(stderr, "left_motor_speed: %f right_motor_speed: %f\n", drive_left, drive_right);
    BT_turn(MOTOR_D, drive_left, MOTOR_A, drive_right);
    return 0;
  }

}

int ball_moved(struct RoboAI *ai)
{
  return (norm(1, ball_x - prev_ball_x, ball_y - prev_ball_y) > ball_moved_threshold) ? 1 : 0;
}


void attack(struct RoboAI *ai)
{
  fprintf(stderr,"state %d attack_state\n", ai->st.state);
  if (check_is_stuck(ai)) {return;};
  
  // if (!(attack_mode(ai))) {
  //   // go back to find stragety 
  //   // ai->st.state = T[ai->st.state][SELECT_TACTIC];
  //   return;
  // }
  // need to check if enemy in the way of getting to the target
  double distance_from_enemy = abs((ball_x-self_x)*(self_y-enemy_y)-(self_x-enemy_x)*(enemy_y-self_y))/norm(0, ball_x-self_x, ball_y-self_y);
  if (distance_from_enemy < 200 && norm(0, enemy_x-self_x, enemy_y-self_y) < 300)
  {
    // TODO don't change states
    // curve around the enemy
    double tangent_y = enemy_y-self_y;
    double tangent_x = self_x-enemy_y;
    if(self_y>sy/2)
    {
      tangent_y *= -1;
      tangent_x *= -1;
    }
    double length = norm(0, tangent_x, tangent_y);
    tangent_x/=length;
    tangent_y/=length;
    target_x = self_x+300*tangent_x;
    target_y = self_y+300*tangent_y;
    get_to_target(ai, 100);
    ai->st.state = T[ai->st.state][RECALCULATE];
    return;
  }
  else
  {
    int flick = select_target(ai);
    if (flick){
      if (get_to_target(ai, flick_distance)){
        ai->st.state = T[ai->st.state][FLICK];
      }
    } else {
      if (get_to_target(ai, vs_proximity_threshold)){
        ai->st.state = T[ai->st.state][SUCCESS];
      }
    }
  }
}

void align_to_ball(struct RoboAI *ai)
{
  fprintf(stderr,"state %d align_to_ball\n", ai->st.state);
  if (check_is_stuck(ai)) {return;};
  // reset error flag
  ERROR_FLAG = 0;
  printf("Aligning to ball\n");
  double angle = find_angle(1, self_dx, self_dy, ball_x - self_x, ball_y - self_y);
  if (abs(angle) <= vs_alignment_threshold)
  {
    // stop turning
    BT_all_stop(1);
    // aligned
    fprintf(stderr, "Robot has aligned to the goal, proceeding to shoot the ball\n");
    ai->st.state = T[ai->st.state][SUCCESS];
  }
  else
  {
    // need to turn, no need for a PID here, just turn at medium-ish pace
    // MAKE SURE YOU TURN IN PLACE (centre of rotation b/w wheels)
    // turn right  = 1, turn left = -1
    int turn = (angle < 0) ? 1 : -1;
    BT_turn(MOTOR_D, turn * vs_turn_power, MOTOR_A, turn * -vs_turn_power);
  }
}

void align_to_goal(struct RoboAI *ai)
{
  fprintf(stderr,"state %d align_to_goal\n", ai->st.state);
  if (check_is_stuck(ai)) {return;};
  // reset error flag
  ERROR_FLAG = 0;
  double angle = find_angle(1, self_dx, self_dy, enemy_goal_x - self_x, (sy / 2) - self_y);
  if (abs(angle) <= vs_alignment_threshold)
  {
    // stop turning
    BT_all_stop(1);
    // aligned
    fprintf(stderr, "Robot has aligned to the goal, proceeding to shoot the ball\n");
    ai->st.state = T[ai->st.state][SUCCESS];
  }
  else
  {
    // need to turn, no need for a PID here, just turn at medium-ish pace
    // MAKE SURE YOU TURN IN PLACE (centre of rotation b/w wheels)
    // turn right  = 1, turn left = -1
    int turn = (angle < 0) ? 1 : -1;
    BT_turn(MOTOR_D, turn * vs_turn_power, MOTOR_A, turn * -vs_turn_power);
  }
}

void shoot(struct RoboAI *ai)
{
  fprintf(stderr,"state %d shoot\n", ai->st.state);
  // reset error flag
  ERROR_FLAG = 0;
  printf("penalty_shoot_counter: %f\n", attack_counter);
  if (attack_counter > attack_counter_threshold)
  {
    // set counter back to 0
    attack_counter = 0;
    BT_all_stop(1);
    // kick finished
    fprintf(stderr, "Kick finished, proceeding to end state\n");
    ai->st.state = T[ai->st.state][SUCCESS];
  }
  else
  {
    // keep driving forward
    // use counter for launch control
    // Identify whether a PID is necessary because the ball has
    // inertia, and if it does, make it have very little influence
    // @ALTON
    BT_turn(MOTOR_D, -70 - penalty_shoot_counter * 10, MOTOR_A, -70 - penalty_shoot_counter * 10);
    attack_counter++;
  }
}

void defend(struct RoboAI *ai)
{
  // logic
  // as long as the y-diff b/defendw ball and self is > 200
  // book it to our goal
  // turn towards ball and move towards it
  // if y-diff becomes less than 200
  // then target becomes ball
  // prepare for flick
  int got_to_target;
  fprintf(stderr,"state %d defend\n", ai->st.state);
  fprintf(stderr,"defend mode %d\n", defend_mode(ai));
  fprintf(stderr,"tackle_mode %d\n", tackle_mode(ai));
  // if (!(defend_mode(ai)||tackle_mode(ai))){
if (attack_mode(ai)){
    ai->st.state = T[ai->st.state][SELECT_TACTIC];
    return;
  }
  if (check_is_stuck(ai)) {return;};
  // if (abs(self_y-ball_y) < 200)
  // {
  //   // run to own goal
  //   target_x = sx-enemy_goal_x;
  //   target_y = sy/2;
  // }
  // else
  // {
    // prepare for flick
    target_x = ball_x;
    target_y = ball_y;
  // }
  if(get_to_target(ai, flick_distance))
  {
    BT_all_stop(1);
    ai->st.state = T[ai->st.state][SUCCESS];
  }
}

void tackle(struct RoboAI *ai)
{
  fprintf(stderr,"state %d tackling\n", ai->st.state);
  // Collision detection
  // Calculate target where the ball is in between the target and the enemy goal
  // Get to  
  // Choose target (target will be ball if ball is too close to wall)
  // Get to the ball
  // Flick if the target is the ball
  ai->st.state = T[ai->st.state][SUCCESS];
}

void flick(struct RoboAI *ai)
{
  printf("state %d flick\n\n\n\n", ai->st.state);
  if (check_is_stuck(ai)) {return;};
  if (flick_counter > flick_counter_threshold)
  {
    BT_all_stop(1);
    flick_counter = 0;
    ai->st.state = T[ai->st.state][SUCCESS];
    printf("Flick finished\n");
    return;
  }
  int above = self_y <= ball_y;
  int enemy_to_the_right = (enemy_goal_x == sx);
  if ((above && enemy_to_the_right) || (!above && !enemy_to_the_right))
  {
    // If bot is above the ball and our net is left, or our bot is below the ball and our net is right, spin CCW
    BT_turn(MOTOR_D, 100, MOTOR_A, -100);
    printf("Flicking CCW\n");
  }
  else
  {
    // If bot is below the ball and our net is left, or our bot is above the ball and our net is right, spin CW
    BT_turn(MOTOR_D, -100, MOTOR_A, 100);
    printf("Flicking CW\n");
  }
  printf("Self Y: %f, Ball Y: %f, Our net X: %f\n", self_y, ball_y, (sx - enemy_goal_x));

  flick_counter++;
}

int check_is_stuck(struct RoboAI *ai)
{
  fprintf(stderr,"Running unstuck()\n");
  if (stuck_counter > stuck_counter_threshold)
  {
    // Reverse for a little bit of time
    if (reverse_counter > reverse_counter_threshold) {
      BT_all_stop(1);
      reverse_counter = 0;
      stuck_counter = 0;
      // maybe we don't need to transition
      // ai->st.state = T[ai->st.state][SUCCESS];
      return 0;
    }

    BT_turn(MOTOR_D, 100, MOTOR_A, 100);
    reverse_counter++;
    return 1;
  }

  // Check if we are stuck and increment stuck_counter if so
  if (abs(self_x - prev_self_x) < 10 && abs(self_y - prev_self_y) < 10) {
    stuck_counter++;
  } else {
    stuck_counter = 0;
  }
  return 0;
}

// void unstuck(struct RoboAI *ai) {
//   printf("Running unstuck()\n");
//   if (stuck_counter > stuck_counter_threshold)
//   {
//     // Reverse for a little bit of time
//     if (reverse_counter > reverse_counter_threshold) {
//       BT_all_stop(1);
//       reverse_counter = 0;
//       stuck_counter = 0;
//       ai->st.state = T[ai->st.state][SUCCESS];
//       return;
//     }

//     BT_turn(MOTOR_D, 100, MOTOR_A, 100);
//     reverse_counter++;
//   }

//   // Check if we are stuck and increment stuck_counter if so
//   if (abs(self_x - prev_self_x) < 10 && abs(self_y - prev_self_y) < 10) {
//     stuck_counter++;
//   } else {
//     stuck_counter = 0;
//   }
// }