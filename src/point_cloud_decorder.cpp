#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


typedef pcl::PointXYZINormal PointType;

ros::Publisher rs_pub;


void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
//    pcl::PointCloud< PointType > pl_orig;
//    pcl::fromROSMsg( *msg, pl_orig );
//
//    for ( int i = 0; i < pl_orig.points.size(); i++ )
//    {
//        printf( "[%d] (%.2f, %.2f, %.2f) \r\n", i, pl_orig.points[ i ].x, pl_orig.points[ i ].y,
//                    pl_orig.points[ i ].z);
//
//    }
    rs_pub.publish(msg);
//    std::cout << "Message received, number of points is: " << msg->width*msg->height << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;


    std::string topic_name;
    nh.param< std::string >( "decode/topic", topic_name, std::string("/points_cloud") );
    std::cout<<"publish topic name: "<<topic_name<<std::endl;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Subscriber sub = pct.subscribe("/point_cloud_compress", 1000, Callback);
    rs_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1000);

    std::cout<<"start decode!!!"<<std::endl;
    ros::spin();

    return 0;
}

