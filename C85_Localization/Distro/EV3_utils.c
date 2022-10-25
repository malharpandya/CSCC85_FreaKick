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


double* driveForwardModel[4] = {
                                    driveForwardModelFacingUp,
                                    driveForwardModelFacingRight,
                                    driveForwardModelFacingDown,
                                    driveForwardModelFacingLeft
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

double *turnLeftModel = {
    turnLeftModelFacingUp,
    turnLeftModelFacingRight,
    turnLeftModelFacingDown,
    
}

double turnRightModelFacingRight[4][3][3] =    {
                                      // Facing up
                                      {
                                       {0, 0, 0},
                                       {0, 0, 0.9},
                                       {0, 0.1, 0}
                                      },
                                      // Facing right
                                      {
                                       {0, 0, 0},
                                       {0.1, 0, 0},
                                       {0, 0.9, 0}
                                      },
                                      // Facing bottom
                                      {
                                       {0, 0.1, 0},
                                       {0.9, 0, 0},
                                       {0, 0, 0}
                                      },
                                      // Facing left
                                      {
                                       {0, 0.9, 0},
                                       {0, 0, 0.1},
                                       {0, 0, 0}
                                      }
                                    };