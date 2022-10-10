#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "std_msgs/Bool.h"

point_cloud_transport::Publisher pub;
int message_num = 0;

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    message_num++;
    std::cout<<"publish: "<<message_num<<std::endl;
    pub.publish(msg);
}

//void new_save_callback(const std_msgs::BoolPtr& msg){
//    if(msg->data){
//        new_record = true;
//        std::cout<<"get new save msg!"<<std::endl;
//    }
//}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;


    std::string topic_name;
    nh.param< std::string >( "record/topic", topic_name, std::string("/rslidar_points") );
    std::cout<<"subscribe topic name: "<<topic_name<<std::endl;

    ros::Subscriber sub = nh.subscribe(topic_name, 100, Callback);
//    ros::Subscriber new_save_sub = nh.subscribe("/video_recorder/new_save", 10, new_save_callback);

    point_cloud_transport::PointCloudTransport pct(nh);
    pub = pct.advertise("/point_cloud_compress", 100);
    std::cout<<"start recorder!!!"<<std::endl;

    ros::spin();
    return 0;
}
