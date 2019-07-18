//
// Created by bfa-oren on 19-7-7.
//
#include "IsotropicHyperelasticFEMWithRestShapeControl.h"
#include "configHeader.h"
#include "RestShapeController.h"

void IsotropicHyperelasticFEMWithRestShapeControl::updateRestShape(const RestShapeController *rsc){
//    int numElements = tetMesh->getNumElements();
//    int numVertices = tetMesh->getNumVertices();

//    Vec3d *r = tetMesh->getVertices();
//   Vec3d *rest_vec = reinterpret_cast<Vec3d *>(restVerticesPosition);

#ifdef WHOLE_ACTIVE_SETTING
    for(int i = 0;i < numVertices;++i){
        r[i] -= scaler[i]*scaling_ratio;
//        rest_vec[i] = r[i];
    }

    ComputeTetVolumes();
    ComputeAreaWeightedVertexNormals();
    PrepareDeformGrad();
    Compute_dFdU();

    for(int i = 0;i < numVertices;++i)
        r[i] = rest_vec[i];
#endif

#ifdef SELECTED_ACTIVE_SETTING
    ComputeTetVolumesWithRestShapeControl(rsc);
    ComputeAreaWeightedVertexNormalsWithRestShapeControl(rsc);
    PrepareDeformGradWithRestShapeControl(rsc);
    Compute_dFdU();
#endif

}

void IsotropicHyperelasticFEMWithRestShapeControl::ComputeTetVolumesWithRestShapeControl(const RestShapeController *rsc) {
    int N = rsc->getNumActiveEles();
    for(int i = 0;i < N;++i){
        int ele = rsc->getActiveEles(i);
        double vscaling = det(rsc->getControllingParam(ele));
        tetVolumes[ele] *= vscaling;
    }
}

void IsotropicHyperelasticFEMWithRestShapeControl::ComputeAreaWeightedVertexNormalsWithRestShapeControl(
        const RestShapeController *rsc) {
    int N = rsc->getNumActiveEles();
    for(int i = 0;i < N;++i){
        int el = rsc->getActiveEles(i);
        Mat3d sm = rsc->getControllingParam(i);
        Vec3d va = tetMesh->getVertex(el, 0);
        Vec3d vb = tetMesh->getVertex(el, 1);
        Vec3d vc = tetMesh->getVertex(el, 2);
        Vec3d vd = tetMesh->getVertex(el, 3);
        Vec3d ref = (va + vb + vc + vd)/4;

        va = sm*(va - ref) + ref;
        vb = sm*(vb - ref) + ref;
        vc = sm*(vc - ref) + ref;
        vd = sm*(vd - ref) + ref;

        Vec3d vca = vc - va;
        Vec3d vba = vb - va;
        Vec3d vda = vd - va;
        Vec3d vdb = vd - vb;
        Vec3d vcb = vc - vb;

        // compute normals for the four faces: acb, adc, abd, bcd
        Vec3d acbNormal = cross(vca, vba);
        Vec3d adcNormal = cross(vda, vca);
        Vec3d abdNormal = cross(vba, vda);
        Vec3d bcdNormal = cross(vcb, vdb);

        // if the tet vertices abcd form a positive orientation, the normals are now correct
        // otherwise, we need to flip them
        double orientation = dot(vd-va, cross(vb-va, vc-va));
        if (orientation < 0)
        {
            acbNormal *= -1.0;
            adcNormal *= -1.0;
            abdNormal *= -1.0;
            bcdNormal *= -1.0;
        }

        // triangle area = 0.5 |u x v|
        double acbArea = 0.5 * sqrt(dot(acbNormal, acbNormal));
        double adcArea = 0.5 * sqrt(dot(adcNormal, adcNormal));
        double abdArea = 0.5 * sqrt(dot(abdNormal, abdNormal));
        double bcdArea = 0.5 * sqrt(dot(bcdNormal, bcdNormal));

        // normalize
        acbNormal.normalize();
        adcNormal.normalize();
        abdNormal.normalize();
        bcdNormal.normalize();

        areaWeightedVertexNormals[4*el+0] = (acbArea * acbNormal + adcArea * adcNormal + abdArea * abdNormal) / 3.0;
        areaWeightedVertexNormals[4*el+1] = (acbArea * acbNormal + abdArea * abdNormal + bcdArea * bcdNormal) / 3.0;
        areaWeightedVertexNormals[4*el+2] = (acbArea * acbNormal + adcArea * adcNormal + bcdArea * bcdNormal) / 3.0;
        areaWeightedVertexNormals[4*el+3] = (adcArea * adcNormal + abdArea * abdNormal + bcdArea * bcdNormal) / 3.0;
    }
}

void IsotropicHyperelasticFEMWithRestShapeControl::PrepareDeformGradWithRestShapeControl(const RestShapeController *rsc){
    for(int i = 0;i < rsc->getNumActiveEles();++i){
        int el = rsc->getActiveEles(i);
        Mat3d sm = rsc->getControllingParam(i);
        Vec3d va = tetMesh->getVertex(el, 0);
        Vec3d vb = tetMesh->getVertex(el, 1);
        Vec3d vc = tetMesh->getVertex(el, 2);
        Vec3d vd = tetMesh->getVertex(el, 3);
        Vec3d ref = (va + vb + vc + vd)/4;

        va = sm*(va - ref) + ref;
        vb = sm*(vb - ref) + ref;
        vc = sm*(vc - ref) + ref;
        vd = sm*(vd - ref) + ref;

        Vec3d dm1 = vd - va;
        Vec3d dm2 = vd - vb;
        Vec3d dm3 = vd - vc;

        Mat3d tmp(  dm1[0], dm2[0], dm3[0],\
                    dm1[1], dm2[1], dm3[1],\
                    dm1[2], dm2[2], dm3[2]);

        dmInverses[el] = inv(tmp);
    }
}
