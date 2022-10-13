#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "std_msgs/Bool.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

point_cloud_transport::Publisher pub;
int message_num = 0;

typedef pcl::PointXYZINormal PointType;
namespace rslidar_ros {

    struct EIGEN_ALIGN16 Point {
            PCL_ADD_POINT4D;
            uint8_t intensity;
            uint16_t ring;
            double timestamp;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
(float, x, x)
(float, y, y)
(float, z, z)
(uint8_t, intensity, intensity)
// use std::uint32_t to avoid conflicting with pcl::uint32_t
(std::uint16_t, ring, ring)
(double, timestamp, timestamp)
)

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg){
//    message_num++;
//    std::cout<<"publish: "<<message_num<<std::endl;
    pcl::PointCloud< PointType > pl_processed;
    pcl::PointCloud< rslidar_ros::Point > pl_orig;
    pcl::fromROSMsg( *msg, pl_orig );
    pl_processed.clear();
    pl_processed.reserve( pl_orig.points.size() );

    double time_stamp = msg->header.stamp.toSec();  // last point time
    double start_time_stamp = pl_orig.points[0].timestamp;
    for ( int i = 0; i < pl_orig.points.size(); i++ )
    {
        if(isnan(pl_orig.points[ i ].x) || isnan(pl_orig.points[ i ].y) || isnan(pl_orig.points[ i ].z)){
            continue;
        }

        PointType       added_pt;
        added_pt.x = pl_orig.points[ i ].x;
        added_pt.y = pl_orig.points[ i ].y;
        added_pt.z = pl_orig.points[ i ].z;
        added_pt.intensity = float(pl_orig.points[ i ].intensity);
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;

        added_pt.curvature = (pl_orig.points[ i ].timestamp - start_time_stamp)*1000;  //why? curvature unit : ms

        pl_processed.points.push_back( added_pt );

    }
    pl_processed.height = 1;
    pl_processed.width = pl_processed.size();
    auto ct = ros::Time().fromSec(start_time_stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl_processed, output );
//    output.header.frame_id = "/rslidar";
    output.header = msg->header;
    output.header.stamp = ct;
    pub.publish(output);
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
