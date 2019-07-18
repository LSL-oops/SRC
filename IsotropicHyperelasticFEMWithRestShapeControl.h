//
// Created by bfa-oren on 19-7-7.
//

#ifndef SOFT_BODY_CONTROL_ISOTROPICHYPERELASTICFEMWITHRESTSHAPECONTROL_H
#define SOFT_BODY_CONTROL_ISOTROPICHYPERELASTICFEMWITHRESTSHAPECONTROL_H

#include "isotropicHyperelasticFEM.h"
#include "RestShapeController.h"
#include "minivector.h"

class IsotropicHyperelasticFEMWithRestShapeControl : public IsotropicHyperelasticFEM{
public:
    IsotropicHyperelasticFEMWithRestShapeControl(TetMesh *tetMesh,IsotropicMaterial *isotropicMaterial, double inversionThreshhold=-DBL_MAX,bool addGravity= false,double g = 9.81): IsotropicHyperelasticFEM(tetMesh,isotropicMaterial,inversionThreshhold,addGravity,g){}
    ~IsotropicHyperelasticFEMWithRestShapeControl(){}
    void updateRestShape(const RestShapeController *rsc);
    void ComputeTetVolumesWithRestShapeControl(const RestShapeController *rsc);
    void ComputeAreaWeightedVertexNormalsWithRestShapeControl(const RestShapeController *rsc);
    void PrepareDeformGradWithRestShapeControl(const RestShapeController *rsc);
};


#endif //SOFT_BODY_CONTROL_ISOTROPICHYPERELASTICFEMWITHRESTSHAPECONTROL_H
