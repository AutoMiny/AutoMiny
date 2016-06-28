#include "kalman_3d.h"


kalman::kalman()
{
   //inizialize the dimension of the matrices 
    //Eigen::MatrixXd kalman_V(7,7);
    kalman_V=Eigen::MatrixXd::Identity(7,7);
    kalman_V <<0.001,0,0,0,0,0,0,0,0.001,0,0,0,0,0,0,0,0.001,0,0,0,0,0,0,0,0.007,0,0,0,0,0,0,0,0.007,0,0,0,0,0,0,0,0.007,0,0,0,0,0,0,0,0.007;
    //kalman_W=Eigen::MatrixXd::Identity(3*startP.rows(),3*startP.rows());
    kalman_P=Eigen::MatrixXd::Zero(7,7); 

     
    //kalman_K=Eigen::MatrixXd::Zeros(7,3*startP.rows());
    //kalman_y= Eigen::VectorXd::Identity(3*startP.rows());
   // kalman_S=Eigen::MatrixXd::Identity(3*startP.rows(),3*startP.rows()); 
   // kalman_x=OptimalRigidTransformation(startP,finalP);

    first=0;

}



 void kalman::prediction(Eigen::MatrixXd startP)
 {
    kalman_P +=kalman_V;
    double a,b,c;
    double tx=kalman_x[0];
    double ty=kalman_x[1];
    double tz=kalman_x[2];
    double qx=kalman_x[3];
    double qy=kalman_x[4];
    double qz=kalman_x[5];
    double qw=kalman_x[6];
    
    kalman_h.resize(3*startP.rows());
    kalman_H.resize(3*startP.rows(),7);
    
    for (int i=0;i<startP.rows();i++){
        a=startP(i,0);
        b=startP(i,1);
        c=startP(i,2);

        kalman_h(3*i)=(1-2*qy*qy-2*qz*qz)*a+2*(qx*qy-qw*qz)*b+2*(qx*qz+qw*qy)*c+ tx;
        kalman_h(3*i+1)=2*(qx*qy+qw*qz)*a+(1-2*qx*qx-2*qz*qz)*b+2*(qy*qz-qw*qx)*c+ty;
        kalman_h(3*i+2)=2*(qx*qz-qw*qy)*a+2*(qy*qz+qw*qx)*b+(1-2*qx*qx-2*qy*qy)*c+tz;

        kalman_H(3*i,0)=1;
        kalman_H(3*i,1)=0;
        kalman_H(3*i,2)=0;
        kalman_H(3*i,3)=2*b*qy+2*c*qz;
        kalman_H(3*i,4)=-4*a*qy+2*b*qx+2*c*qw;
        kalman_H(3*i,5)=-4*a*qz-2*b*qw+2*c*qx;
        kalman_H(3*i,6)=-2*b*qz+2*c*qy;
           
        kalman_H(3*i+1,0)=0;
        kalman_H(3*i+1,1)=1;
        kalman_H(3*i+1,2)=0;
        kalman_H(3*i+1,3)=2*a*qy-4*b*qx-2*c*qw;
        kalman_H(3*i+1,4)=2*a*qx+2*c*qz;
        kalman_H(3*i+1,5)=2*a*qw-4*b*qz+2*c*qy;
        kalman_H(3*i+1,6)=2*a*qz-2*c*qx;
           
        kalman_H(3*i+2,0)=0;
        kalman_H(3*i+2,1)=0;
        kalman_H(3*i+2,2)=1;
        kalman_H(3*i+2,3)=2*a*qz+2*b*qw-4*c*qx;
        kalman_H(3*i+2,4)=-2*a*qw+2*b*qz-4*c*qy;
        kalman_H(3*i+2,5)=2*a*qx+2*b*qy;
        kalman_H(3*i+2,6)=-2*a*qy+2*b*qx;
    }
}

Eigen::VectorXd kalman::update(Eigen::MatrixXd finalP)
{  
    Eigen::MatrixXd kalman_W = Eigen::MatrixXd::Identity(3*finalP.rows(),3*finalP.rows())*0.3;
    Eigen::MatrixXd kalman_S(3*finalP.rows(),3*finalP.rows());
    Eigen::VectorXd kalman_y(3*finalP.rows());
    Eigen::MatrixXd kalman_K(7,3*finalP.rows());
    
    int k=0;
    for (int i=0;i<finalP.rows();i++)
        for(int j=0;j<3;j++)
            kalman_y[k++]=finalP(i,j);

    kalman_y = kalman_y-kalman_h;

    kalman_S = (kalman_H*kalman_P)*kalman_H.transpose() + kalman_W;

    kalman_K=(kalman_P*kalman_H.transpose())*kalman_S.inverse();

    //std::cout << "The inverse of S is:" <<std::endl<<kalman_S.inverse() << endl;
     
    kalman_x += kalman_K * kalman_y;
    for (int i=3;i<7;i++)
    {
       kalman_x(i)=kalman_x(i)/(sqrt(kalman_x(3)*kalman_x(3)+kalman_x(4)*kalman_x(4)+kalman_x(5)*kalman_x(5)+kalman_x(6)*kalman_x(6)));
    }
    

    kalman_P -= kalman_K*kalman_H*kalman_P;
    
    ROS_INFO("kalman_x %G",kalman_x(0));

    return kalman_x;
 }
 