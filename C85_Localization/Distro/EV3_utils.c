// #define debug	
#include "EV3_Localization.h"
#include "EV3_utils.h"

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

void printOutMotionModel(model *motionModel) {
    fprintf(stderr, "Printing out Motion Model\n");
    for (int k=0; k<4; k++)
    {
        for (int j=0; j<3; j++)
        {
            for (int i=0; i<3; i++) 
                fprintf(stderr,"[%d][%d][%d]%f ",k, j, i, (*motionModel)[k][j][i]);
                // fprintf(stderr,"%f ", (*motionModel)[k][j][i]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr,"\n");
    }
}

double convol(double ***belief, model *motionModel, int i, int j, int k) {
    int padding = 1;
    int mj = 1; // Motion model j center
    int mi = 1;
    i += padding;
    j += padding;
    double sum = 0;
    // fprintf(stderr, "convol %d %d %d\n", k, j, i);
    // printOutMyBeliefs(belief);
    // printf("motionModel pointer: %p\n", motionModel);
    // printOutMotionModel(motionModel);


    // printOutMotionModel(&turnRightModelFacingRight);
    for (int dir = 0; dir < 4; dir++) { // IDT we need the direction
        for (int j_offset = -1; j_offset < 2 ; j_offset++) {
            for (int i_offset = -1; i_offset < 2; i_offset++) {
                // fprintf(stderr, "%d %d %d - ", dir, mj+j_offset, mi+i_offset);
                // fprintf(stderr,"[%d][%d][%d] - %f %f\n",dir, mj+j_offset, mi+i_offset, (*motionModel)[dir][mj+j_offset][mi+i_offset], belief[j+j_offset][i+i_offset][dir]);//beliefs[(i+i_offset)+((j+j_offset)*sx)][dir]);
                // printf("motionModel pointer: %p\n", motionModel);
                sum += ((*motionModel)[dir][mj+j_offset][mi+i_offset]) * belief[j+j_offset][i+i_offset][dir];//beliefs[(i+i_offset)+((j+j_offset)*sx)][dir];
                // fprintf(stderr, "in sum: %f\n", sum);
            }
        }
    }
    // printf("sum: %f\n", sum);
    return sum;
}

// void normalize(double ***belief){
//     double sum = 0;
//     for (int j=0; j<sy; j++)
//         for (int i=0; i<sx; i++)
//             for (int k=0; k<4; k++){
//                 if (belief[j][i][k] == 0.0) {belief[j][i][k]=0.0001;}
//                 sum += belief[j][i][k];
//             }
                
    
//     for (int j=0; j<sy; j++)
//         for (int i=0; i<sx; i++)
//             for (int k=0; k<4; k++)
//                 belief[j][i][k] /= sum;

// }

// normailze Paco's belief
void normalize(){
    double sum = 0;
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++){
                if (beliefs[i+(j*sx)][k] == 0.0) {beliefs[i+(j*sx)][k]=0.0001;}
                sum += beliefs[i+(j*sx)][k];
            }
                
    
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++)
                beliefs[i+(j*sx)][k] /= sum;
}

double updateBeliefBasedOnAction(int action) {
    /*
        action  0: forward
                1: right turn
                2: 180 turn
                3: left turn
    */
    model **motionModel = getMotionModel(action);
    // printf("action::: %d\n", action);
    // printOutMotionModel(*motionModel);
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

    // printOutMyBeliefs(myBeliefs);

    // double newBeliefs[sy][sx][4]; //TODO calloc and free it all
    double ***newBeliefs;
    newBeliefs = (double ***) calloc(sy, sizeof *newBeliefs);
    for (int j = 0; j < sy; j++){
        newBeliefs[j] = (double**) calloc(sx, sizeof *newBeliefs[j]);
        for (int i = 0; i < sx; i++)
            newBeliefs[j][i] = (double*) calloc(4, sizeof *newBeliefs[j][i]);
    }

    // printOutNewBeliefs(newBeliefs);
    
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

    // printOutNewBeliefs(newBeliefs);
    // normalize(newBeliefs);
    for (int j=0; j<sy; j++)
        for (int i=0; i<sx; i++)
            for (int k=0; k<4; k++)
                beliefs[i+(j*sx)][k] = newBeliefs[j][i][k];
    normalize();

    for (int j = 0; j < (sy+2*padding); j++){
        for (int i = 0; i < sx+2*padding; i++)
            free(myBeliefs[j][i]);
        free(myBeliefs[j]);
    }
    free(myBeliefs);

    for (int j = 0; j < sy; j++){
        for (int i = 0; i < sx; i++)
            free(newBeliefs[j][i]);
        free(newBeliefs[j]);
    }
    free(newBeliefs);
    return 0;
}

void updateperceptionModel();

void rotateSensorReadings(int readings[4]) {
    int temp = readings[0];
    readings[0] = readings[3];
    readings[3] = readings[2];
    readings[2] = readings[1];
    readings[1] = temp;
    /*
        int temp = readings[0];
    readings[0] = readings[1];
    readings[1] = readings[3];
    readings[3] = readings[2];
    readings[2] = temp;
    */
}

double readingsMatchMap(int readings[4], int j,int i){
    double updateFactor = 1.0;
    for (int b = 0; b < 4 ; b++) {
        // printf("%d ", map[i+j*sx][b]);
        updateFactor *= (map[i+j*sx][b] == readings[b]) ? 0.8 : 0.2;
    }
    return updateFactor;
}

void updateBeliefBasedOnSensorReading(int readings[4]){
    for (int k = 0; k < 4; k++) {
        for (int j=0; j<sy; j++) {
            for (int i=0; i<sx; i++) {
                // printf("k: %d j: %d i: %d \n", k, j, i);
                beliefs[i+(j*sx)][k] *= readingsMatchMap(readings, j, i);
            }
        }
        // printf("Rotating\n");
        rotateSensorReadings(readings);
        for (int c = 0; c < 4; c++) {printf("%d",readings[c]);}
    }
    normalize();
}

void printOutBeliefs() {
    fprintf(stderr, "Printing out Belief Table\n");
    for (int k=0; k<4; k++)
    {
        for (int j=0; j<sy; j++)
        {
            for (int i=0; i<sx; i++)
                fprintf(stderr,"%f ",beliefs[i+(j*sx)][k]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr,"\n");
    }
}

void printOutMyBeliefs(double ***myBeliefs){
    fprintf(stderr, "Printing out My Belief Table\n");
    int padding = 1;
    for (int k=0; k<4; k++)
    {
        for (int j=0; j<sy+2*padding; j++)
        {
            for (int i=0; i<sx+2*padding; i++)
                fprintf(stderr,"%f ",myBeliefs[j][i][k]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr,"\n");
    }
}

void printOutNewBeliefs(double ***newBeliefs){
    fprintf(stderr, "Printing out New Belief Table\n");
    for (int k=0; k<4; k++)
    {
        for (int j=0; j<sy; j++)
        {
            for (int i=0; i<sx; i++)
                fprintf(stderr,"%f ",newBeliefs[j][i][k]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr,"\n");
    }
}



void updateBelief(int action, int intersectionReadings[4]) {
    /*
        beliefs is of format [][4]
        int action [0..3]
        intersectionReadings [forward left, forward right, rear left, rear right]
    */
    
    // What is the true direction of the colour reading (convert it)

    #ifdef debug 
        action = 0; // Go forward 
        intersectionReadings[0] = 6; // White
        intersectionReadings[1] = 3; // Green 
        intersectionReadings[2] = 2; // Blue
        intersectionReadings[3] = 2; // Blue
    #endif
    fprintf(stderr, "action: %d, readings %d %d %d %d\n", action, intersectionReadings[0],intersectionReadings[1], intersectionReadings[2],intersectionReadings[3]);
    // printOutBeliefs();

    updateBeliefBasedOnAction(action);

    // printOutBeliefs();


    updateBeliefBasedOnSensorReading(intersectionReadings);
    printOutBeliefs();


    // TODO what relationship between a multidimensional matrix and its pointer address

}


void test(){
    fprintf(stderr, "Can we access sx sy %d %d\n", sx, sy);
}