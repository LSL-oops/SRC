//
// Created by bfa-oren on 19-7-9.
//

#include "RestShapeController.h"
#include <iostream>

RestShapeController::RestShapeController(char *controller_filename){

}

RestShapeController::RestShapeController(const int _nActiveEles,const int *_activeEles){
    nActiveEles = _nActiveEles;
    activeEles = (int *)calloc(_nActiveEles,sizeof(int));
    memcpy(activeEles,_activeEles,sizeof(int)*nActiveEles);
    ele_params = (Mat3d*)calloc(nActiveEles,sizeof(Mat3d));
    for(int i = 0;i < nActiveEles;++i)
        ele_params[i] = Mat3d(1.0f);
}

void RestShapeController::setAllControllingParam(Mat3d param){
    for(int i = 0;i < nActiveEles;++i){
        int ele = activeEles[i];
        setEleControllingParam(i,param);
    }
}
