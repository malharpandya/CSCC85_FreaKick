
#include "EV3_Localization.h"

typedef double model[4][3][3];

double driveForwardModelFacingUp[4][3][3] = {
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 1, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        }
};

double driveForwardModelFacingRight[4][3][3] = {
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {1, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        }
};

double driveForwardModelFacingDown[4][3][3] = {
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 1, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        }
};

double driveForwardModelFacingLeft[4][3][3] = {
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}
                                        },
                                        {
                                            {0, 0, 0},
                                            {0, 0, 1},
                                            {0, 0, 0}
                                        }
};


model* driveForwardModel[4] = {
                                    &driveForwardModelFacingUp,
                                    &driveForwardModelFacingRight,
                                    &driveForwardModelFacingDown,
                                    &driveForwardModelFacingLeft
                                };

double turnLeftModelFacingUp[4][3][3] =     {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.03, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.8, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.10, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.07, 0}
                                      }
                                    };
                                    
double turnLeftModelFacingRight[4][3][3] =     {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0.3, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0.07, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0.8, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0.10, 0, 0},
                                       {0, 0, 0}
                                      }
                                    };


double turnLeftModelFacingDown[4][3][3] =     {
                                      // Facing up
                                      {
                                       {0, 0.1, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0.7, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0.3, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0.8, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      }
                                    };

double turnLeftModelFacingLeft[4][3][3] =     {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.8},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.1},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.07},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.03},
                                       {0, 0, 0}
                                      }
                                    };

model* turnLeftModel[4] = {
    &turnLeftModelFacingUp,
    &turnLeftModelFacingRight,
    &turnLeftModelFacingDown,
    &turnLeftModelFacingLeft
};

double turnRightModelFacingUp[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.03, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.07, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.1, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.8, 0}
                                      }
                                    };

double turnRightModelFacingRight[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0.8, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0.03, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0.07, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0.1, 0, 0},
                                       {0, 0, 0}
                                      }
                                    };

double turnRightModelFacingDown[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0.3, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0.8, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0.1, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0.07, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      }
                                    };

double turnRightModelFacingLeft[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.07},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.03},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.8},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.1},
                                       {0, 0, 0}
                                      }
                                    };           

model* turnRightModel[4] = {
    &turnRightModelFacingUp,
    &turnRightModelFacingRight,
    &turnRightModelFacingDown,
    &turnRightModelFacingLeft
};


double turnAroundModelFacingUp[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.07, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.03, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.8, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0.1, 0}
                                      }
                                    };

double turnAroundModelFacingRight[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0.1, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0.07, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0.03, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0.8, 0, 0},
                                       {0, 0, 0}
                                      }
                                    };

double turnAroundModelFacingDown[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0.8, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0.1, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0.07, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0.03, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}
                                      }
                                    };

double turnAroundModelFacingLeft[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.03},
                                       {0, 0, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.8},
                                       {0, 0, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.1},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.07},
                                       {0, 0, 0}
                                      }
                                    };           
// TODO Crunchy dynamic generate the motion model (NESW clockwise rotation and 3*3 90 deg rotation clockwise)
// TODO for sensor model, have 0.8 for each correct colour and 0.2 for each incorrect colour. Each colour can also have different success rate
model* turnAroundModel[4] = {
    &turnAroundModelFacingUp,
    &turnAroundModelFacingRight,
    &turnAroundModelFacingDown,
    &turnAroundModelFacingLeft
};

model** getMotionModel(int action){
    if (action == 0){
        return driveForwardModel;
    } else if (action == 1) {
        return turnRightModel;
    } else if (action == 2) {
        return turnAroundModel;
    } else if (action == 3) {
        return turnLeftModel;
    } else {
        printf("Invalid action");
        exit(1);
    }
}


double convol(double ***beliefs, model *motionModel, int i, int j, int k) {
    int padding = 1;
    int mj = 1; // Motion model j center
    int mi = 1;
    double sum = 0;
    for (int dir = 0; dir < 4; dir++) { // IDT we need the direction
        for (int j_offset = -1; j_offset < 2 ; j_offset++) {
            for (int i_offset = -1; i_offset < 2; i_offset++) {
                sum += *motionModel[dir][mj+j_offset][mi+i_offset] * beliefs[j+j_offset][i+i_offset][dir];//beliefs[(i+i_offset)+((j+j_offset)*sx)][dir];
            }
        }
    }
    return sum;
}

void normailize(double ***beliefs){
    double sum = 0;
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++)
                sum += beliefs[j][i][k];
    
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++)
                beliefs[j][i][k] /= sum;

}

double updateBeliefBasedOnAction(double **beliefs, int action) {
    /*
        action  0: forward
                1: right turn
                2: 180 turn
                3: left turn
    */
    model **motionModel = getMotionModel(action);
    int padding = 1;
    // use sx and sy 
    // double myBeliefs[sy+2*padding][sx+2*padding][4];
    double ***myBeliefs;
    myBeliefs = (double***) calloc((sy+2*padding), sizeof *myBeliefs);
    for (int j = 0; j < (sy+2*padding); j++){
        myBeliefs[j] = (double**) calloc((sx+2*padding), sizeof *myBeliefs[j]);
        for (int i = 0; i < sx+2*padding; i++)
            myBeliefs[j][i] = (double*) calloc(4, sizeof *myBeliefs[j][i]);
    }

    // double newBeliefs[sy][sx][4]; //TODO calloc and free it all
    double ***newBeliefs;
    newBeliefs = (double ***) calloc(sy, sizeof *newBeliefs);
    for (int j = 0; j < sy; j++){
        newBeliefs[j] = (double**) calloc(sx, sizeof *newBeliefs[j]);
        for (int i = 0; i < sx; i++)
            newBeliefs[j][i] = (double*) calloc(4, sizeof *newBeliefs[j][i]);
    }
    
    // memset(myBeliefs, 0, sizeof(double) * (sx+2*padding) * (sy+2*padding) * 4);
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++)
                myBeliefs[j+padding][i+padding][k] = beliefs[i+(j*sx)][k];
    // Made a copy of belief 
    
    for (int facing_direction = 0; facing_direction < 4; facing_direction++){
        for (int j = 0; j < sy; j++) {
            for (int i = 0; i < sx; i++) {
                newBeliefs[j][i][facing_direction] = convol(myBeliefs, *(motionModel+facing_direction), i, j, facing_direction);
            }
        }
    }
    normailize(newBeliefs);
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++)
                beliefs[i+(j*sx)][k] = newBeliefs[j][i][k];
}

void updateperceptionModel();

void updateBeliefBasedOnSensorReading(){
    for (int k = 0; k < 4; k++) {
        for (int j=0; j<sy; j++)
            for (int i=0; i<sx; i++)
                break;
    }
}

void updateBelief(double **beliefs, int action, double intersectionReadings[4]) {
    /*
        beliefs is of format [][4]
        int action [0..3]
        intersectionReadings [forward left, forward right, rear left, rear right]
    */
    
    // What is the true direction of the colour reading (convert it)



    // TODO what relationship between a multidimensional matrix and its pointer address

}


void test(){
    fprintf(stderr, "Can we access sx sy %d %d\n", sx, sy);
}