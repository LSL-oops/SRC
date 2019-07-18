//
// Created by bfa-oren on 19-7-8.
//

#ifndef SOFT_BODY_CONTROL_PYBINDINTERFACE_H
#define SOFT_BODY_CONTROL_PYBINDINTERFACE_H

#include "minivector.h"
#include "SoftBodySimulator.h"

SoftBodySimulator *simulator;

void initializeSimulator(char *msh_filename,char *config_filename);
void updateRestShapeConfiguration(Mat3d *scaling_matrix);
void doTimeStep(Vec3d *p);
void reset();


#endif //SOFT_BODY_CONTROL_PYBINDINTERFACE_H
