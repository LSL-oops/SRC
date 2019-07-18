//
// Created by bfa-oren on 19-7-5.
//

#ifndef SOFT_BODY_CONTROL_SOFTBODYSIMULATOR_H
#define SOFT_BODY_CONTROL_SOFTBODYSIMULATOR_H
#include "volumetricMesh.h"
#include "sparseMatrix.h"
#include "integratorBase.h"
#include "IsotropicHyperelasticFEMWithRestShapeControl.h"
#include "implicitBackwardEulerSparseWithRestShapeControl.h"
#include "RestShapeController.h"

class SoftBodySimulator {
public:
    SoftBodySimulator(VolumetricMesh *vm);
    ~SoftBodySimulator(){};
    void DotimeStep(Vec3d *q,Vec3d *r = NULL);
    void updateState(bool scaling);
    void updateRestShapeWithScalingMatrix(const Mat3d &sm);
protected:
    SparseMatrix *massMatrix;
    VolumetricMesh* mesh;
    RestShapeController *rsc;
    IsotropicHyperelasticFEMWithRestShapeControl* fem;
    ImplicitBackwardEulerSparseWithRestShapeControl * integrator;
    int *fixed_dofs;
    int num_fixed_dofs;
    Vec3d *r;
    Vec3d *q;
    Vec3d *scaler;

    double scaling_param = 0.0;

    const double max_scaling = 0.5;
    const double scaling_step = 0.01;
    const double pressing_step = -0.01;
    const double min_scaling = -0.8;
};


#endif //SOFT_BODY_CONTROL_SOFTBODYSIMULATOR_H
