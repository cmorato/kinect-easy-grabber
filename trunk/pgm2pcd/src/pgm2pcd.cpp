/**
 * Convert a pgm 16 bit depth file to the pcd format
 */
#include <iostream>
#include <stdexcept>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "raw_image.h"

//using namespace std;

// sequences: /media/LG External HDD/Windows/kinect/secuencias/pachi_walk/depth/video_depth_1.pgm

#define SEQ_PATH "/media/LG External HDD/Windows/kinect/secuencias/pachi_walk/depth/"

int main (int argc, char** argv){

  std::cout << "hello world" << std::endl;

  RawImage di;

  try{
    di.loadImage(SEQ_PATH"video_depth_1.pgm");
  }catch (std::exception& e) {
    throw std::runtime_error(e.what());
  }

  if(di.getDepth() != 2)
    throw std::runtime_error("Not a 16 bit image");

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = di.getWidth();
  cloud.height = di.getHeight();
  cloud.is_dense = false; //TODO check if nan points have to be added
  cloud.points.resize(cloud.width * cloud.height);

  ushort* imdata_ptr = di.get16bitData();
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = i % di.getWidth();
    cloud.points[i].y = i / di.getHeight();
    cloud.points[i].z = imdata_ptr[i];
  }

  pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size()
      << " data points to test_pcd.pcd." << std::endl;

//
//  for (size_t i = 0; i < cloud.points.size(); ++i)
//    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y
//        << " " << cloud.points[i].z << std::endl;
//

  return (0);
}
