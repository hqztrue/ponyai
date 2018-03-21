// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/rotation/rotation.h"

namespace homework3 {

bool near(double x, double y, double eps=1e-6){
    return fabs(x-y)<eps;
}

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
    const double PI=atan(1.0)*4;
    /*Eigen::Vector3d rpy = rotation.eulerAngles(0, 1, 2);
    cout<<"rpy"<<rpy[0]<<" "<<rpy[1]<<" "<<rpy[2]<<endl;
    return rpy;*/
    //Eigen::Matrix3d mat = Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ())*Eigen::AngleAxisf(-0.2, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitX());
    //cout<<mat<<endl;
    //return Eigen::Vector3d(0.0, 0.0, 0.0);
    double alpha, beta, gamma;
    alpha = atan(-rotation(0, 1)/rotation(0, 0));
    beta = asin(rotation(0, 2));
    gamma = atan(-rotation(1, 2)/rotation(2, 2));
    for (int i=0;i<2;++i){
        for (int j=0;j<2;++j){
            for (int k=0;k<2;++k){
                if (near(rotation(0,0),cos(alpha)*cos(beta))&&
                    near(rotation(1,0),cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma))&&
                    near(rotation(1,1),-sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma))&&
                    //near(rotation(1,2),-cos(beta)*sin(gamma))&&
                    near(rotation(2,0),-cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma))&&
                    near(rotation(2,1),sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma))&&
                    near(rotation(2,2),cos(beta)*cos(gamma))
                )goto end;
                gamma=PI-gamma;
            }
            beta=PI-beta;
        }
        alpha=alpha+PI;
    }
    puts("error");//exit(0);
    end:;
    //printf("%.5lf %.5lf %.5lf\n",alpha, beta, gamma);
    return Eigen::Vector3d(alpha, beta, gamma);
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
    /*Eigen::AngleAxisd d;
    d.fromRotationMatrix(rotation);
    printf("%.5lf %.5lf %.5lf %.5lf\n",d.angle(), d.axis().x(), d.axis().y(), d.axis().z());
    cout<<d.matrix()<<endl;
    return d;*/
    double c = (rotation(0,0)+rotation(1,1)+rotation(2,2)-1)/2;
    if (near(c,1.0)){
        return Eigen::AngleAxisd(0, Eigen::Vector3d(1,0,0));
    }
    double t=1-c, s=sqrt(1-c*c), x=sqrt((rotation(0,0)-c)/t), y=0, z=0;
    for (int i=0;i<2;++i){
        for (int j=0;j<2;++j){
            y=(rotation(0,1)+rotation(1,0))/(2*t*x);
            z=(rotation(0,2)+rotation(2,0))/(2*t*x);
            //printf("%.8lf %.8lf %.8lf %.8lf\n", c, x, y, z);
            if (near(rotation(0,1),t*x*y-z*s, 1e-3)&&
                near(rotation(0,2),t*x*z+y*s, 1e-3)&&
                near(rotation(1,2),t*y*z-x*s, 1e-3)
                )goto end;
            x*=-1;
        }
        s*=-1;
    }
    puts("error");//exit(0);
    end:
    double angle = acos(c);
    if (!near(sin(angle),s))angle=-angle;
    //printf("%.5lf %.5lf %.5lf %.5lf\n", angle, x, y, z);
    return Eigen::AngleAxisd(angle, Eigen::Vector3d(x,y,z));
    //return Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
}
}  // namespace homework3


