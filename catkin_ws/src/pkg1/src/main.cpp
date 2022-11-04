#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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


struct EIGEN_ALIGN16 OsPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(OsPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

static float dx = 40;
static float dy = 50;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  static int i = 0;
  // pcl::PointCloud<tPointWithClassID> scan;
  // pcl::fromROSMsg(*input, scan);
  // for (auto& pt : scan)
  // {
  //   pt.x += dx;
  //   pt.y += dy;
  // }
  // pcl::PCLPointCloud2 scan;
  pcl::PointCloud<OsPoint> scan;
  pcl::fromROSMsg(*input, scan);
  pcl::io::savePCDFileBinaryCompressed("/home/wxxs6p/repo/ros-playaround/os2_sensor_frame/os2_" + padZeros(i) + ".pcd", scan);
  // std::cout << "Saved " << scan.points.size() << " points to os2_" << i << ".pcd, dx: " << dx << std::endl;
  std::cout << "Saved os2_" << i << ".pcd, dx: " << dx << std::endl;
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