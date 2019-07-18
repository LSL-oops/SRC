//
// Created by bfa-oren on 19-7-7.
//

#ifndef SOFT_BODY_CONTROL_IMPLICITBACKWARDEULERSPARSEWITHRESTSHAPECONTROL_H
#define SOFT_BODY_CONTROL_IMPLICITBACKWARDEULERSPARSEWITHRESTSHAPECONTROL_H
#include "implicitBackwardEulerSparse.h"
#include "minivector.h"

class ImplicitBackwardEulerSparseWithRestShapeControl : public ImplicitBackwardEulerSparse {
public:
    ImplicitBackwardEulerSparseWithRestShapeControl(int r,double timestep,SparseMatrix * massMatrix, ForceModel *forceModel,int numConstrainedDOFs = 0,int *constrainedDOFs = NULL,double dampingMassCoef=0.0,double dampingStiffnessCoef=0.0,int maxIterations = 1,double epsilon = 1E-6,int numSolverThreads=0):
        ImplicitBackwardEulerSparse(r,timestep,massMatrix,forceModel,numConstrainedDOFs,constrainedDOFs,dampingMassCoef,dampingStiffnessCoef,maxIterations,epsilon,numSolverThreads){

    }

    ~ImplicitBackwardEulerSparseWithRestShapeControl(){}

    void updateDisplacement(const Vec3d *delata_q,double scaling_ratio,int NV);

};


#endif //SOFT_BODY_CONTROL_IMPLICITBACKWARDEULERSPARSEWITHRESTSHAPECONTROL_H
