#include "SoftBodyController.h"
#include "volumetricMesh.h"

SoftBodyController::SoftBodyController(VolumetricMesh *vm){ ;
//    q = (Vec3d *)calloc(vm->getNumVertices(),sizeof(Vec3d));
    r = (Vec3d *)calloc(vm->getNumVertices(),sizeof(Vec3d));
    p = (Vec3d *)calloc(vm->getNumVertices(),sizeof(Vec3d));
    memcpy(r,vm->getVertices()->data(),sizeof(double)*vm->getNumVertices()*3);
    NV = vm->getNumVertices();
    simulator = new SoftBodySimulator(vm);
}

SoftBodyController::~SoftBodyController(){
    free(simulator);
}

void SoftBodyController::DoTimeStep(){
    simulator->DotimeStep(p,r);
}

void SoftBodyController::scalingRestShape(bool scaling){
    simulator->updateState(scaling);
}

void SoftBodyController::updateGPUVertexBuffer(const GLuint VBOs[2]){
    Vec3d *p_gpu = reinterpret_cast<Vec3d *>(glMapNamedBuffer(VBOs[1],GL_WRITE_ONLY));
    for(int i = 0;i < NV;++i)
        p_gpu[i] = p[i];
    glUnmapNamedBuffer(VBOs[1]);

    Vec3d *r_gpu = reinterpret_cast<Vec3d *>(glMapNamedBuffer(VBOs[0],GL_WRITE_ONLY));
    for(int i = 0;i < NV;++i)
        r_gpu[i] = r[i];
    glUnmapNamedBuffer(VBOs[0]);
}