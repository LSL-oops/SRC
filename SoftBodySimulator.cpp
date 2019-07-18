//
// Created by bfa-oren on 19-7-5.
//
#include "generateMassMatrix.h"
#include "SoftBodySimulator.h"
#include "volumetricMeshLoader.h"
#include "implicitBackwardEulerSparseWithRestShapeControl.h"
#include "isotropicHyperelasticFEMStencilForceModel.h"
#include "IsotropicHyperelasticFEMWithRestShapeControl.h"
#include "neoHookeanIsotropicMaterial.h"
#include "StVKIsotropicMaterial.h"
#include "forceModelAssembler.h"

#include "configHeader.h"

const double timestep = 0.001;
int fixed_vertices[] = {51,52,103,104,155,156,207,208};
int num_fixed_vertices = 8;

extern const int num_active_elements = 150;
extern int active_elements[] = {
        0,1,2,3,4,5,6,7,8,9,
        10,11,12,13,14,15,16,17,18,19,
        20,21,22,23,24,25,26,27,28,29,
        30,31,32,33,34,35,36,37,38,39,
        40,41,42,43,44,45,46,47,48,49,
        50,51,52,53,54,55,56,57,58,59,
        60,61,62,63,64,65,66,67,68,69,
        70,71,72,73,74,75,76,77,78,79,
        80,81,82,83,84,85,86,87,88,89,
        90,91,92,93,94,95,96,97,98,99,
        100,101,102,103,104,105,106,107,108,109,
        110,111,112,113,114,115,116,117,118,119,
        120,121,122,123,124,125,126,127,128,129,
        130,131,132,133,134,135,136,137,138,139,
        140,141,142,143,144,145,146,147,148,149
};


bool update_state = true;

SoftBodySimulator::SoftBodySimulator(VolumetricMesh *mesh){
    this->mesh = mesh;
    this->rsc = new RestShapeController(num_active_elements,active_elements);
    GenerateMassMatrix::computeMassMatrix(mesh,&massMatrix,true);
    r = mesh->getVertices();
    //scaler = (Vec3d*)calloc(mesh->getNumVertices(),sizeof(Vec3d));

//    Vec3d* q = (Vec3d*)calloc(mesh->getNumVertices(),sizeof(Vec3d));
/*
    for(int i = 0;i < mesh->getNumVertices();++i){
        scaler[i][1] = r[i][1] - 1.0f;
        q[i][1] = scaler[i][1]*scaling_param;
        r[i][1] -= q[i][1];
    }
*/
    IsotropicMaterial *material = new NeoHookeanIsotropicMaterial((TetMesh*)mesh,1,500);
    fem = new IsotropicHyperelasticFEMWithRestShapeControl((TetMesh*)mesh,material);
    IsotropicHyperelasticFEMStencilForceModel *stencil = new IsotropicHyperelasticFEMStencilForceModel(fem);
    ForceModelAssembler *assembler = new ForceModelAssembler(stencil);
    ForceModel *forceModel = assembler;

    num_fixed_dofs = num_fixed_vertices*3;
    fixed_dofs = (int*)malloc(sizeof(int)*num_fixed_dofs);
    for(int i = 0;i < num_fixed_vertices;++i){
        int fv_index = fixed_vertices[i];
        fixed_dofs[i*3 + 0] = fv_index*3 - 3;
        fixed_dofs[i*3 + 1] = fv_index*3 - 2;
        fixed_dofs[i*3 + 2] = fv_index*3 - 1;
    }

    integrator = new ImplicitBackwardEulerSparseWithRestShapeControl(3*mesh->getNumVertices(),timestep,massMatrix,forceModel,num_fixed_dofs,fixed_dofs,0.01,0.0,1,1e-6,0);//   integrator->SetState(reinterpret_cast<double *>(initial_q),NULL);
    integrator->ResetToRest();
#ifdef STATIC_SOLVER
    integrator->UseStaticSolver(true);
#endif
}

void SoftBodySimulator::DotimeStep(Vec3d *p,Vec3d *rest_v){
//    int N = mesh->getNumVertices()*3;
    integrator->DoTimestep();
    //integrator->ResetToRest();
    //memset(q,0,sizeof(double)*N);

    Vec3d *q_s = reinterpret_cast<Vec3d*>(integrator->Getq());

    for(int i = 0;i < mesh->getNumVertices();++i) {
        p[i] = q_s[i] + r[i];
        if(rest_v)
            rest_v[i] = r[i];
    }
#ifdef WHOLE_ACTIVE_SETTING
    for(int i = 0;i < mesh->getNumVertices();++i)
        rest_v[i][1] -= scaler[i][1]*scaling_param;
#endif

#ifdef SELECTED_ACTIVE_SETTING
    if(rest_v) {
        for (int i = 0; i < num_active_elements; ++i) {
            int ele = active_elements[i];
            Mat3d sm = rsc->getControllingParam(ele);
            int v0 = mesh->getVertexIndex(ele, 0);
            int v1 = mesh->getVertexIndex(ele, 1);
            int v2 = mesh->getVertexIndex(ele, 2);
            int v3 = mesh->getVertexIndex(ele, 3);

            Vec3d ref0 = Vec3d(r[v0][0],1.0f,r[v0][2]);
            Vec3d ref1 = Vec3d(r[v1][0],1.0f,r[v1][2]);
            Vec3d ref2 = Vec3d(r[v2][0],1.0f,r[v2][2]);
            Vec3d ref3 = Vec3d(r[v3][0],1.0f,r[v3][2]);

            rest_v[v0] = ref0 + sm*(r[v0] - ref0);
            rest_v[v1] = ref1 + sm*(r[v1] - ref1);
            rest_v[v2] = ref2 + sm*(r[v2] - ref2);
            rest_v[v3] = ref3 + sm*(r[v3] - ref3);
        }
    }

#endif
    update_state = true;
}

void SoftBodySimulator::updateState(bool scaling) {
    int NV = mesh->getNumVertices();
    double step = 0;
    if(update_state){
        if(scaling && scaling_param < max_scaling){
            if(scaling_param + scaling_step > max_scaling)
                step = max_scaling - scaling_param;
            else
                step = scaling_step;
        }
        if(!scaling && scaling_param > min_scaling){
            if(scaling_param + pressing_step < min_scaling)
                step = min_scaling - scaling_param;
            else
                step = pressing_step;
        }
    }

    scaling_param += step;
/*
    for(int i = 0;i < mesh->getNumVertices();++i){
        integrator->Getq()[i*3 + 1] += scaler[i][1]*step;
        r[i][1] -= scaler[i][1]*step;
    }
*/
    rsc->setAllControllingParam(Mat3d(1.0f,0.0f,0.0f,\
                                      0.0f,1.0f + scaling_param,0.0f,\
                                      0.0f,0.0f,1.0f));
    fem->updateRestShape(rsc);
    update_state = false;
}