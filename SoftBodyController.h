//
// Created by bfa-oren on 19-7-5.
//

#ifndef SOFT_BODY_CONTROL_SOFTBODYCONTROLLER_H
#define SOFT_BODY_CONTROL_SOFTBODYCONTROLLER_H


#include "SoftBodySimulator.h"
#include "volumetricMesh.h"
#include <glad/glad.h>

class SoftBodyController {
public:
    SoftBodyController(VolumetricMesh *vm);
    ~SoftBodyController();

    void scalingRestShape(bool scaling);
    void DoTimeStep();
    void updateGPUVertexBuffer(const GLuint VBO[2]);
protected:
    int NV;
    SoftBodySimulator* simulator;
    Vec3d *p;
 //   Vec3d *q;
    Vec3d *r;
};


#endif //SOFT_BODY_CONTROL_SOFTBODYCONTROLLER_H
