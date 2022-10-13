#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


typedef pcl::PointXYZINormal PointType;

ros::Publisher rs_pub;
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


void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud< rslidar_ros::Point > pl_orig;
    pcl::fromROSMsg( *msg, pl_orig );

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = pct.subscribe("/rslidar_points", 1000, Callback);

    std::cout<<"start save"<<std::endl;
    ros::spin();

    return 0;
}


