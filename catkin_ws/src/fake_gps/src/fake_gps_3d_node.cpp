#include <fstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Transform.h"
#include "kalman_3d.h"

#include "fake_gps/Transform.h"
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
    

    kalman filter;

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
   
   void OptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP);


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
   
    /*
    for (int i=0;i<msg.markers.size();i++)
    {   //check if the markers id is in the list and if it has not be read yet
        if((find(marker_id.begin(), marker_id.end(), msg.markers[i].id) != marker_id.end())) // && read_points((msg.markers[i].id,0)==0))   
        //if (read_points((msg.markers[i].id,0)==0))
        {   read_points(numrows,0)=msg.markers[i].id;
            read_points(numrows,1)=msg.markers[i].pose.position.x;
            read_points(numrows,2)=msg.markers[i].pose.position.y;
            read_points(numrows,3)=msg.markers[i].pose.position.z;
            numrows++;
        }

    }   */

    if (blobsIn.blob_count<3)
        return;
    std::vector<Eigen::Vector3d> read_points;//=map_points.block(0,0,numrows,2);

    std::vector<Eigen::Vector3d> camera_points;//=camera_points.block(0,0,numrows,2);


     int numrows=blobsIn.blob_count;
    int id=0;
    for (int i = 0; i < blobsIn.blob_count; ++i) {
        // TODO: rule to drop false positives?

        if (blobsIn.blobs[i].name=="RedRectangle")
            read_points.push_back(Eigen::Vector3d(map_points(0,0),map_points(0,1),0).transpose());
        else if (blobsIn.blobs[i].name=="BlueRectangle")
            read_points.push_back(Eigen::Vector3d(map_points(1,0),map_points(1,1),0).transpose());
        else if (blobsIn.blobs[i].name=="GreenRectangle")
            read_points.push_back(Eigen::Vector3d(map_points(2,0),map_points(2,1),0).transpose());
        double x_normalized=(blobsIn.blobs[i].x-320.0)*(243.0/320);
        double y_normalized=(blobsIn.blobs[i].y-240.0)*(169.0/240);

        camera_points.push_back(Eigen::Vector3d(x_normalized,y_normalized,0).transpose());
        ROS_INFO("here %i,%G=%G\n",i,camera_points[i][0],read_points[i][0]);

       // camera_points.push_back(Eigen::Vector3d(blobsIn.blobs[i].x,blobsIn.blobs[i].y,0).transpose());
        
    }
    
    Eigen::MatrixXd startP, finalP;
    startP.resize(blobsIn.blob_count, 3);
    finalP.resize(blobsIn.blob_count, 3);

    for(int i = 0; i < blobsIn.blob_count; ++i)
    {
        startP.row(i) = read_points[i];
        finalP.row(i) = camera_points[i];
    }
    for (int i=0;i<numrows;i++)
    {
       
    //appo.row(i)=map_points.row(camera_points(i,0));
       // ROS_INFO("here %i,%G=%G\n",i,camera_points(i,0),read_points(i,0));
    }
    
    // for (int i=0;i<numrows;i++)
    // {
    //     ROS_INFO(" id %i,start %G= final %G\n",i,startP(i,1),finalP(i,1));
    //     ROS_INFO(" id %i,start %G= final %G\n",i,startP(i,0),finalP(i,0));
    //     ROS_INFO(" id %i,start %G= final %G\n",i,startP(i,2),finalP(i,2));
    // }
    
    // if (filter.getFirst()==0)
    // {   
    //     filter.setFirst(1);
    //     OptimalRigidTransformation(finalP,startP);
    //     filter.setKalman_x(transfParameters);
    //    // printf("%f\n", transfParameters(0));


    // }       
    // else
    // {   
        
    //     //OptimalRigidTransformation(finalP,startP);
    //     filter.prediction(finalP);      
    //     transfParameters=filter.update(startP);
    // }

    OptimalRigidTransformation(finalP,startP);
    tr.setOrigin( tf::Vector3(transfParameters(0),transfParameters(1),transfParameters(2)));
    tr.setRotation( tf::Quaternion(transfParameters(3),transfParameters(4),transfParameters(5),transfParameters(6)));


    geometry_msgs::Transform msg_t;

   // msg_t.header.stamp = ros::Time::now();

    //tf::transformTFToMsg(tr,msg_t.Transf);
    tf::transformTFToMsg(tr,msg_t);
    pub_transform_.publish(msg_t);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tr, ros::Time::now(), start_frame.c_str(), end_frame.c_str()));
 
}

void online_tf::OptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP)
{   
    Eigen::Matrix4d transf;
    
    if (startP.rows()!=finalP.rows())
    {   ROS_ERROR("The number of rows of startP and finalP have to be the same");
        exit(1);
    }

    Eigen::RowVector3d centroid_startP=Eigen::RowVector3d::Zero(); 
    Eigen::RowVector3d centroid_finalP=Eigen::RowVector3d::Zero(); //= mean(B);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    //calculate the mean
    for (int i=0;i<startP.rows();i++)
    {   centroid_startP=centroid_startP+startP.row(i);
        centroid_finalP=centroid_finalP+finalP.row(i);
    }
    
    centroid_startP=centroid_startP/startP.rows();
    centroid_finalP=centroid_finalP/startP.rows();

    for (int i=0;i<startP.rows();i++)
        H=H+(startP.row(i)-centroid_startP).transpose()*(finalP.row(i)-centroid_finalP);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
   
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd R=V*U.transpose();
     if (R.determinant()<0)
         V.col(2)=V.col(2)*(-1);
     
     R=V*U.transpose();



    Eigen::Matrix4d C_A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d C_B = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d R_new = Eigen::Matrix4d::Identity();
            
    C_A.block<3,1>(0,3)=-centroid_startP.transpose();
    R_new.block<3,3>(0,0)=R;
    
    C_B.block<3,1>(0,3)=centroid_finalP.transpose();


    transf = C_B * R_new * C_A;

    //std::cout<<"trans: "<<transf<<std::endl;

    Eigen::Quaterniond mat_rot(transf.block<3,3>(0,0));

    Eigen::Vector3d trasl=transf.block<3,1>(0,3).transpose();

    transfParameters<<trasl(0),trasl(1),trasl(2),mat_rot.x(),mat_rot.y(),mat_rot.z(),mat_rot.w();


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
