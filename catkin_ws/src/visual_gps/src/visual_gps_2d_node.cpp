#include <fstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include "kalman_2d.h"
#include <math.h>

#include "geometry_msgs/Transform.h"
#include <Eigen/Eigen>
#include <cmvision/Blobs.h>

namespace transformations {

class online_tf
{
  private:
    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers
    ros::Subscriber read_map_positions;
 
    tf::TransformBroadcaster tf_broadcaster_;
    
    ros::Publisher pub_transform_;
   
    std::string map_file_path;
    std::string camera_file_path;
    std::string start_frame;
    std::string end_frame;
    std::vector<int> marker_id;

    Eigen::MatrixXd map_points;
    Eigen::MatrixXd camera_points;
    Eigen::MatrixXd read_points;
    

    kalman filter;
    Eigen::VectorXd kalman_result;

    Eigen::VectorXd transfParameters;
    void read_map_coordinates(std::string,int);
    //void read_camera_coordinates(std::string,int);

  public:
//
    ros::Publisher pub_markers_;
    visualization_msgs::MarkerArray feature_markers_;
    // callback functions
    // callback functions
    void calculate_tf(const cmvision::Blobs& blobsIn);
   
   void OptimalRigidTransformation(Eigen::MatrixXd worldP, Eigen::MatrixXd cameraP);


    // constructor
    online_tf(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~"), filter()
    {
        priv_nh_.param<std::string>("map_file", map_file_path, "");
        priv_nh_.param<std::string>("camera_file", camera_file_path, "");
        priv_nh_.param<std::string>("start_frame", start_frame, "");
        priv_nh_.param<std::string>("end_frame", end_frame, "");


        int max_id=-1;
        marker_id.resize(argc-1);
        for (int i=0;i<argc-1;i++)
        {   marker_id[i]=std::atoi(argv[i+1]);
            if (marker_id[i]>max_id)
                max_id=marker_id[i];
        }

        pub_transform_= nh.advertise<geometry_msgs::Transform>(nh.resolveName("Transform"), 10);
        pub_markers_= nh.advertise<visualization_msgs::MarkerArray>(nh.resolveName("/Features_markers"), 1);
        
        read_map_coordinates(map_file_path,max_id);
        //read_camera_coordinates(camera_file_path,max_id);
        transfParameters.resize(7);
                // subscribe to topics
        //read_map_positions = nh_.subscribe(nh_.resolveName("/phase_space_markers"),10, &online_tf::calculate_tf,this);
        read_map_positions = nh.subscribe(nh_.resolveName("blobs/filtered"), 1000, &online_tf::calculate_tf,this);
        //calculate_tf();
    }
    void inizialize_markers(visualization_msgs::Marker&);
    //! Empty stub
    ~online_tf() {}

};

void online_tf::inizialize_markers(visualization_msgs::Marker& marker)
{
    marker.header.frame_id = "/world";
    marker.ns = "/Features";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

void online_tf::read_map_coordinates(std::string map_file_path,int max_id)
{
    std::ifstream f_in(map_file_path.c_str());
    int id;
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker mark;
    map_points=Eigen::MatrixXd::Zero(max_id+1,4);

    if (!f_in)
        ROS_ERROR("Error in opening file %s",map_file_path.c_str());
     //ROS_ERROR("Error in reading file %s",f_in>>);
    while(f_in>>id)
    {   

        if (!(f_in>>map_points(id,0)>>map_points(id,1)>>map_points(id,2)))
            ROS_ERROR("Error in reading file %s",map_file_path.c_str());

        std::cout<<id<<" "<<map_points(id,0)<<" "<<map_points(id,1)<<" "<<map_points(id,2)<<std::endl;
        inizialize_markers(mark);  
        mark.pose.position.x = map_points(id,0);
        mark.pose.position.y = map_points(id,1);
        mark.pose.position.z = map_points(id,2);
        mark.id = id;
        if (int(id)==0)
            mark.color.r = 1;
        if (int(id)==1)
            mark.color.g = 1;
        if (int(id)==2)
            mark.color.b = 1;
        feature_markers_.markers.push_back(mark);
    }
    pub_markers_.publish(feature_markers_);
}

// void online_tf::read_camera_coordinates(std::string camera_file_path,int max_id)
// {
//     std::ifstream f_in(camera_file_path.c_str());
//     int id;

//     camera_points=Eigen::MatrixXd::Zero(max_id+1,4);

//     if (!f_in)
//         ROS_ERROR("Error in opening file %s",camera_file_path.c_str());
//     while(f_in>>id)
//     {   //std::cout<<"id1 "<<id<<std::endl;
//         if (!(f_in>>camera_points(id,0)>>camera_points(id,1)>>camera_points(id,2)))
//             ROS_ERROR("Error in reading file %s",camera_file_path.c_str());
//         //ROS_INFO("here %i=%G\n",id,camera_points(id,0));

//     }
// }


// this function is called when a new message is received at the topic_name_you_want_to_subscribe_to
void online_tf::calculate_tf(const cmvision::Blobs& blobsIn)
{   tf::Transform tr;
   //pub_markers_.publish(feature_markers_);
    if (blobsIn.blob_count<2)
        return;
    std::vector<Eigen::Vector2d> read_points;//=map_points.block(0,0,numrows,2);

    std::vector<Eigen::Vector2d> camera_points;//=camera_points.block(0,0,numrows,2);


     int numrows=blobsIn.blob_count;
    int id=0;
    for (int i = 0; i < blobsIn.blob_count; ++i) {
        // TODO: rule to drop false positives?

        if (blobsIn.blobs[i].name=="RedRectangle")
            read_points.push_back(Eigen::Vector2d(map_points(0,0),map_points(0,1)).transpose());
        else if (blobsIn.blobs[i].name=="BlueRectangle")
            read_points.push_back(Eigen::Vector2d(map_points(1,0),map_points(1,1)).transpose());
        else if (blobsIn.blobs[i].name=="GreenRectangle")
            read_points.push_back(Eigen::Vector2d(map_points(2,0),map_points(2,1)).transpose());
        else if (blobsIn.blobs[i].name=="PurpleRectangle")
            read_points.push_back(Eigen::Vector2d(map_points(3,0),map_points(3,1)).transpose());
        double x_normalized=(320.0-blobsIn.blobs[i].x)*(0.759);//261 cm/310 pixel
        double y_normalized=(blobsIn.blobs[i].y-240.0)*(0.70); //261 cm/ 356 pixel  // 65cm/110 pixel
         //double x_normalized=(blobsIn.blobs[i].x-320.0)*(243.0/320);
        //double y_normalized=(blobsIn.blobs[i].y-240.0)*(169.0/240);
        camera_points.push_back(Eigen::Vector2d(x_normalized,y_normalized).transpose());
    //  ROS_INFO("here %i x,%G=%G\n",i,camera_points[i][0],read_points[i][0]);
    //  ROS_INFO("here %i y,%G=%G\n",i,camera_points[i][1],read_points[i][1]);
       
    }
    
    Eigen::MatrixXd worldP, cameraP;
    worldP.resize(blobsIn.blob_count, 2);
    cameraP.resize(blobsIn.blob_count, 2);

    for(int i = 0; i < blobsIn.blob_count; ++i)
    {
        worldP.row(i) = read_points[i];
        cameraP.row(i) = camera_points[i];
    }
    
    // for (int i=0;i<numrows;i++)
    // {
    //     ROS_INFO(" id %i,start %G= final %G\n",i,worldP(i,1),cameraP(i,1));
    //     ROS_INFO(" id %i,start %G= final %G\n",i,worldP(i,0),cameraP(i,0));
    //     ROS_INFO(" id %i,start %G= final %G\n",i,worldP(i,2),cameraP(i,2));
    // }
    
    // if (filter.getFirst()==0)
    // {   
    //     filter.setFirst(1);
    //     OptimalRigidTransformation(cameraP,worldP);
    //      ROS_INFO("here ");
    //     filter.setKalman_x(Eigen::Vector3d(transfParameters(0),transfParameters(1),transfParameters(5)));
    //    // printf("%f\n", transfParameters(0));
    // }       
    // else
    // {   
    //     //OptimalRigidTransformation(cameraP,worldP);
    //     filter.prediction(worldP);
    //     kalman_result=filter.update(cameraP);
    //     float yaw_ =kalman_result[3];
    //     Eigen::Matrix3d matYaw = Eigen::Matrix3d::Zero();
    //     matYaw << cos(yaw_), -sin(yaw_), 0.0f,
    //     sin(yaw_), cos(yaw_), 0.0f,  //z
    //     0.0f, 0.0f, 1.0f;
    //     Eigen::Quaterniond mat_rot(matYaw);
    //     transfParameters<<kalman_result[0],kalman_result[1],0,mat_rot.x(),mat_rot.y(),mat_rot.z(),mat_rot.w();
    // }

    OptimalRigidTransformation(worldP,cameraP);
    tr.setOrigin( tf::Vector3(transfParameters(0),transfParameters(1),transfParameters(2)));
    tr.setRotation( tf::Quaternion(transfParameters(3),transfParameters(4),transfParameters(5),transfParameters(6)));


    
    geometry_msgs::Transform msg_t;

   // msg_t.header.stamp = ros::Time::now();

    //tf::transformTFToMsg(tr,msg_t.Transf);
    tf::transformTFToMsg(tr,msg_t);
    pub_transform_.publish(msg_t);


    tf_broadcaster_.sendTransform(tf::StampedTransform(tr, ros::Time::now(), start_frame.c_str(), end_frame.c_str()));
 
}

void online_tf::OptimalRigidTransformation(Eigen::MatrixXd worldP, Eigen::MatrixXd cameraP)
{   
    if (worldP.rows()!=cameraP.rows())
    {   ROS_ERROR("The number of rows of worldP and finalP have to be the same");
        exit(1);
    }

    Eigen::RowVector2d centroid_worldP=Eigen::RowVector2d::Zero(); 
    Eigen::RowVector2d centroid_cameraP=Eigen::RowVector2d::Zero(); //= mean(B);
    double numerator,denominator,yaw_;
    numerator=0;
    denominator=0;
    yaw_=0;
    //calculate the mean
    for (int i=0;i<worldP.rows();i++)
    {   centroid_worldP=centroid_worldP+worldP.row(i);
        centroid_cameraP=centroid_cameraP+cameraP.row(i);       
    }
    centroid_worldP=centroid_worldP/worldP.rows();
    centroid_cameraP=centroid_cameraP/worldP.rows();
    
    for (int i=0;i<worldP.rows();i++)
    {   worldP.row(i)=worldP.row(i)-centroid_worldP;
        cameraP.row(i)=cameraP.row(i)-centroid_cameraP;
        numerator= numerator+cameraP(i,0)*worldP(i,1)-cameraP(i,1)*worldP(i,0);
        denominator=denominator+cameraP(i,0)*worldP(i,0)+cameraP(i,1)*worldP(i,1);   
    }
    
    yaw_=atan2(numerator,denominator);

    Eigen::Matrix2d matYaw = Eigen::Matrix2d::Zero();
    matYaw << cos(yaw_), -sin(yaw_),
    sin(yaw_), cos(yaw_);
        
    Eigen::Vector2d trasl;
    trasl=centroid_worldP.transpose()-matYaw*centroid_cameraP.transpose();
  
    ROS_ERROR("yaw %G: x %G: y %G:",yaw_,trasl(0),trasl(1));
    Eigen::Matrix3d R_new = Eigen::Matrix3d::Identity();
    R_new.block<2,2>(0,0)=matYaw;
	Eigen::Quaterniond mat_rot(R_new);  
    //mat_rot.setRPY(0,0,-yaw_);


    transfParameters<<trasl(0),trasl(1),0,mat_rot.x(),mat_rot.y(),mat_rot.z(),mat_rot.w();
    //kalman_result<<trasl(0),trasl(1),RPY[2];
}

} // namespace package_name

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_tf");
        
    //ROS_ERROR("Nargs %d val=%s e % s", argc,argv[0],argv[1]);

    ros::NodeHandle nh;

    transformations::online_tf node(nh,argc,argv);

    while(ros::ok())
    {
        
        node.pub_markers_.publish(node.feature_markers_);
        //node.calculate_tf();
 
    ros::spinOnce();
    }

    return 0;
}
