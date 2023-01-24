#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ply_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // directory containing ply files
    std::string directory = "/path/to/ply/files";

    while (ros::ok()) {
        // iterate through all files in the directory
        for (const auto &entry : std::filesystem::directory_iterator(directory)) {
            // check if file is a ply file
            if (entry.path().extension() == ".ply") {
                // read the ply file
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PLYReader reader;
                reader.read(entry.path().string(), *cloud);

                // convert the pointcloud to a ROS message
                sensor_msgs::PointCloud2 msg;
                pcl::toROSMsg(*cloud, msg);

                // publish the message
                pub.publish(msg);
                ros::spinOnce();
            }
        }
        ros::Duration(1.0).sleep();
    }

    return 0;
}