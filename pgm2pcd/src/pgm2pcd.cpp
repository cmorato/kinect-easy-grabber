/**
 * Convert a pgm 16 bit depth file to the pcd format
 */
#include <iostream>
#include <stdexcept>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
//#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/median.hpp>
using namespace boost::accumulators;

#include "raw_image.h"

using namespace std;

// sequences: /media/LG External HDD/Windows/kinect/secuencias/pachi_walk/depth/video_depth_1.pgm

#define SEQ_PATH "/media/LG External HDD/Windows/kinect/secuencias/pachi_walk/depth/"
#define PCD_FILE "test_pcd"

int main (int argc, char** argv){

  std::cout << "hello world" << std::endl;

  RawImage di;
  bool rimage = false;
  try{
    di.loadImage(SEQ_PATH"video_depth_1.pgm");
    if(di.getDepth() != 2)
      throw std::runtime_error("Not a 16 bit image");
    rimage = true;
  }catch (std::exception& e) {
//    throw std::runtime_error(e.what());
    rimage = false;
  }

  // define accumulator
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if(rimage)
  {
    accumulator_set<ushort, stats<tag::min, tag::max, tag::median > > acc;
    // Fill in the cloud data
    cloud.width = di.getWidth();
    cloud.height = di.getHeight();
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    ushort* imdata_ptr = di.get16bitData();
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      ushort depth = imdata_ptr[i];;
      cloud.points[i].x = i % di.getWidth();
      cloud.points[i].y = i / di.getHeight();
      if(depth!=0) cloud.points[i].z = depth;
      else cloud.points[i].z = std::numeric_limits<double>::quiet_NaN();

      acc(depth);
    }
    std::cout << "min: "
              << extract_result<tag::min>(acc)
              << std::endl;
    std::cout << "max: "
              << extract_result<tag::max>(acc)
              << std::endl;
    std::cout << "med: "
              << extract_result<tag::median>(acc)
              << std::endl;

    pcl::io::savePCDFileASCII(PCD_FILE".pcd", cloud);
    std::cerr << "Saved " << cloud.points.size()
        << " data points to "PCD_FILE".pcd." << std::endl;
    system("pcd2ply "PCD_FILE".pcd "PCD_FILE".ply > /dev/null");
  }
  else
  {
    std::cout << "Loading point from pcd file..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (PCD_FILE ".pcd", *pcloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file " PCD_FILE ".pcd \n");
    }
    std::cout << "Loaded "
              << pcloud->width * pcloud->height
              << " data points from" PCD_FILE ".pcd"
              << std::endl;
    cloud = *pcloud;

    cloud.is_dense = false;

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      ushort depth = cloud.points[i].z;
      if(depth==0)
        cloud.points[i].z = std::numeric_limits<double>::quiet_NaN();
    }

    pcl::io::savePCDFileASCII(PCD_FILE"_nan.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size()
        << " data points to "PCD_FILE"_nan.pcd." << std::endl;
    system("pcd2ply "PCD_FILE"_nan.pcd "PCD_FILE"_nan.ply > /dev/null");
  }

  return (0);
}
