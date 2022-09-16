#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
//#include <pcl/memory.h>
//#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

struct tPointWithClassID
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  std::uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  std::uint8_t class_id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(tPointWithClassID,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(
                                      uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(
                                      uint16_t, ambient, ambient)(uint32_t, range, range)(uint8_t, class_id, class_id))

static float dx = 40;
static float dy = 50;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  static int i = 0;
  pcl::PointCloud<tPointWithClassID> scan;
  pcl::fromROSMsg(*input, scan);
  for (auto& pt : scan)
  {
    pt.x += dx;
    pt.y += dy;
  }
  pcl::io::savePCDFileASCII("/home/wxxs6p/repo/ros-playaround/results/wx_" + std::to_string(i) + ".pcd", scan);
  std::cout << "Saved " << scan.points.size() << " points to wx_" << i << ".pcd, dx: " << dx << std::endl;
  i++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  ros::NodeHandle nh;
  nh.param<float>("x_translation", dx, 0.0);
  nh.param<float>("y_translation", dy, 0.0);
  ros::Subscriber sub = nh.subscribe("/point_cloud", 100000, callback);

  if (sub)
  {
    std::cout << "Now subscribe to topic" << std::endl;
  }

  ros::spin();
  return 0;
}