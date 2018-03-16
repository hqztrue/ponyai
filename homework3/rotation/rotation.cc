// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/rotation/rotation.h"

namespace homework3 {

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
    Eigen::Vector3d rpy = rotation.eulerAngles(0, 1, 2);
    cout<<"rpy"<<rpy[0]<<" "<<rpy[1]<<" "<<rpy[2]<<endl;
    //Eigen::Matrix3d mat = Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ())*Eigen::AngleAxisf(-0.2, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitX());
    //cout<<mat<<endl;
    return rpy;
    //return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
    Eigen::AngleAxisd d;
    d.fromRotationMatrix(rotation);
    return d;
    //return Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
}
}  // namespace homework3

