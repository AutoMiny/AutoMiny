#include "kalman_2d.h"


kalman::kalman()
{
   //inizialize the dimension of the matrices 
    //Eigen::MatrixXd kalman_V(3,3);
    kalman_V=Eigen::MatrixXd::Identity(3,3);
    kalman_V <<0.07,0,0,0,0.07,0,0,0,0.07;
    kalman_P=Eigen::MatrixXd::Zero(3,3);
    first=0;
}



 void kalman::prediction(Eigen::MatrixXd startP)
 {
    kalman_P +=kalman_V;
    double Fx,Fy,Fz;

    double tx=kalman_x[0];
    double ty=kalman_x[1];
    double theta=kalman_x[2];
    
    kalman_h.resize(2*startP.rows());
    kalman_H.resize(2*startP.rows(),3);
    
    for (int i=0;i<startP.rows();i++){
        Fx=startP(i,0);
        Fy=startP(i,1);
       // ROS_INFO("%i=%G\n",i,Fx);
        // kalman_h(2*i)=(Fx-tx)+theta*(Fy-ty);
        // kalman_h(2*i+1)=-theta*(Fx-tx)+(Fy-ty);


        // kalman_H(2*i,0)=theta*(Fx-tx)-(Fy-ty);
        // kalman_H(2*i,1)=-1;
        // kalman_H(2*i,2)=-theta;
           
        // kalman_H(2*i+1,0)=(Fx-tx)+theta*(Fy-ty);
        // kalman_H(2*i+1,1)=theta;
        // kalman_H(2*i+1,2)=-1;



        kalman_h(2*i)=cos(theta)*Fx-sin(theta)*Fy+tx;
        kalman_h(2*i+1)=sin(theta)*Fx+cos(theta)*Fy+ty;
   
        kalman_H(2*i,0)=1;
        kalman_H(2*i,1)=0;
        kalman_H(2*i,2)=sin(theta)*Fx+cos(theta)*Fy;
             
        kalman_H(2*i+1,0)=0;
        kalman_H(2*i+1,1)=1;
        kalman_H(2*i+1,2)=-cos(theta)*Fx+sin(theta)*Fy;

    }
}
/*Eigen::VectorXd kalman::updateFeauters(Eigen::MatrixXd finalP)
{
    for (int i=0;i<4;i++)


}*/

Eigen::VectorXd kalman::update(Eigen::MatrixXd finalP)
{  
    Eigen::MatrixXd kalman_W = Eigen::MatrixXd::Identity(2*finalP.rows(),2*finalP.rows())*0.3;
    Eigen::MatrixXd kalman_S(2*finalP.rows(),2*finalP.rows());
    Eigen::VectorXd kalman_y(2*finalP.rows());
    Eigen::MatrixXd kalman_K(3,2*finalP.rows());
    
    int k=0;
    for (int i=0;i<finalP.rows();i++)
        for(int j=0;j<2;j++)
        {
            kalman_y[k++]=finalP(i,j);
            //ROS_INFO("%i,kalman_y %G",k,kalman_y[k-1]);

        }


    kalman_y = kalman_y-kalman_h;


    kalman_S = (kalman_H*kalman_P)*kalman_H.transpose() + kalman_W;

    kalman_K=(kalman_P*kalman_H.transpose())*kalman_S.inverse();

    //std::cout << "The inverse of S is:" <<std::endl<<kalman_S.inverse() << endl;
     
    kalman_x += kalman_K * kalman_y;
    //kalman_x(0)=kalman_x(0);

    kalman_P -= kalman_K*kalman_H*kalman_P;

   // ROS_INFO("kalman_x %G",kalman_x(0));

    return kalman_x;
 }
 
