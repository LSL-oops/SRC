//
// Created by bfa-oren on 19-7-9.
//

#ifndef SOFT_BODY_CONTROL_RESTSHAPECONTROLLER_H
#define SOFT_BODY_CONTROL_RESTSHAPECONTROLLER_H

#include "minivector.h"

class RestShapeController {
public:
    RestShapeController(char *controller_filename);
    RestShapeController(const int nActiveEles,const int *activeEles);
    inline void setEleControllingParam(int ele,Mat3d param){ele_params[ele] = param;}
    void setAllControllingParam(Mat3d param);
    inline int getNumActiveEles() const{return nActiveEles;}
    inline int getActiveEles(int ele_index) const{return activeEles[ele_index];}
    inline Mat3d getControllingParam(int ele) const {return ele_params[ele];}
protected:
    Mat3d shared_param;
    Mat3d *ele_params;
    int nActiveEles;
    int *activeEles;
};


#endif //SOFT_BODY_CONTROL_RESTSHAPECONTROLLER_H
