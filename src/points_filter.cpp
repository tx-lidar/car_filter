#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
ros::Publisher pub;

float min_x, min_y, min_z, max_x, max_y, max_z;
string points_topic = "";
string filter_topic = "";
//bool negative;
   
void BoxFiltercallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //1. 读取雷达点云数据
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(*input, *cloud);
   //std::cout << "waiting lidar data" << std::endl;
   
   
   // 2.立方体滤波器
   pcl::CropBox<pcl::PointXYZ> box_filter;
   box_filter.setInputCloud(cloud);
   
   //std::cout << "2" << std::endl;

   //box_filter.setMin(Eigen::Vector4f(-1.6,-0.6,-100,1)); //设置最小点
   //box_filter.setMax(Eigen::Vector4f(0.6,0.6,100,1));//设置最大点

   box_filter.setMin(Eigen::Vector4f(min_x,min_y,min_z,1)); //设置最小点
   box_filter.setMax(Eigen::Vector4f(max_x,max_y,max_z,1));//设置最大点
   std::cout << "1" << std::endl;
   box_filter.setNegative(true);
   box_filter.filter(*cloud);
   
   sensor_msgs::PointCloud2 output;
   pcl::toROSMsg(*cloud, output);
   
   //3. 发布过滤后的点云消息
   pub.publish(output);

}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "box_filter_node");
   ros::NodeHandle nh("~");
   

   nh.getParam("min_x", min_x);
   std::cout << "min_x:" << min_x << std::endl;

   nh.getParam("min_y", min_y);
   std::cout << "min_y:" << min_x << std::endl;

   nh.getParam("min_z", min_z);
   std::cout << "min_z:" << min_x << std::endl;

   nh.getParam("max_x", max_x);
   std::cout << "max_x:" << min_x << std::endl;

   nh.getParam("max_y", max_y);
   std::cout << "max_y:" << min_x << std::endl;

   nh.getParam("max_z", max_z);
   std::cout << "max_z:" << min_x << std::endl;

   
   nh.getParam("points_topic", points_topic);
   std::cout << "points_topic:" << points_topic << std::endl;
   nh.getParam("filter_topic", filter_topic);
   std::cout << "filter_topic:" << filter_topic << std::endl;
      // nh.getParam("min_x", min_x, -1.6);
   // nh.getParam("min_y", min_y, -0.6);
   // nh.getParam("min_z", min_z, -100);
   // nh.getParam("max_x", max_x, 0.6);
   // nh.getParam("max_y", max_y, 0.6);
   // nh.getParam("max_z", max_z, 100);
   //订阅雷达点云话题
   ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(points_topic, 1, BoxFiltercallback);
   //发布过滤后的点云消息
   pub = nh.advertise<sensor_msgs::PointCloud2>(filter_topic, 1);
   std::cout << "filter points publish " << std::endl;
   ros::spin();

   return 0;


}
