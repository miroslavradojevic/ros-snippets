#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// http://wiki.ros.org/pcl_ros
// https://answers.ros.org/question/263649/pcd-files-into-cloud-of-points/

typedef std::vector<std::string> StringVec;

struct PathLeafString
{
    std::string operator()(const boost::filesystem::directory_entry &entry) const
    {
        return entry.path().leaf().string();
    }
};

void readDirectory(const std::string &name, StringVec &readList)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(readList), PathLeafString());
}

// https://stackoverflow.com/questions/874134/find-out-if-string-ends-with-another-string-in-c
bool hasEnding(std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length())
    {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    }
    else
    {
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_to_pointcloud_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // read parameters
    std::string pcdDir = "";
    int freq = 5;

    nh.getParam("/pcd_dir", pcdDir);
    nh.getParam("/freq", freq);

    ROS_INFO("pcd_directory=%s", pcdDir.c_str());
    ROS_INFO("freq=%d", freq);

    // TODO: check if the directory pcdDir exists

    StringVec listDir;
    readDirectory(pcdDir, listDir);
    ROS_INFO("Found %d files in %s\n", (int)listDir.size(), pcdDir.c_str());
    
    // print names
    // std::copy(listDir.begin(), listDir.end(), std::ostream_iterator<std::string>(std::cout, "\n")); 

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Rate loop_rate(freq); // freq: rate in Hz

    int counter = 0;

    while (nh.ok() && counter < listDir.size())
    {
        std::string fullPath = pcdDir + listDir[counter];

        // check if the file has ".pcd" extension
        if (hasEnding(fullPath, ".pcd"))
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(fullPath, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file\n");
                return (-1);
            }

            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);

            // pcl_conversions::toPCL(ros::Time::now(), output.header.stamp);
            pub.publish(output);

            ROS_INFO("counter=%04d/%04d %s", counter, (int)(listDir.size() - 1), fullPath.c_str());
        }
        else
        {
            ROS_INFO("%s did not have .pcd extension", fullPath.c_str());
        }

        counter++;

        ros::spinOnce();
        loop_rate.sleep();
    }
}