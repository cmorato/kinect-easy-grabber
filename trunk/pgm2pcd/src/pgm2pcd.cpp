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

#define SEQ_PATH "/media/LG External HDD/Windows/kinect/secuencias/pachi_walk/"
#define PCD_FILE "test_pcd"

void depth2PCD()
{
  RawImage di;
  try{
    di.loadImage(SEQ_PATH"depth/video_depth_1.pgm");
    if(di.getDepth() != 2)
      throw std::runtime_error("Not a 16 bit image");
  }catch (std::exception& e) {
//    throw std::runtime_error(e.what());
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
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
//  std::cout << "min: "
//            << extract_result<tag::min>(acc)
//            << std::endl;
//  std::cout << "max: "
//            << extract_result<tag::max>(acc)
//            << std::endl;
//  std::cout << "med: "
//            << extract_result<tag::median>(acc)
//            << std::endl;

  pcl::io::savePCDFile(PCD_FILE".pcd", cloud);
  std::cerr << "Saved " << cloud.points.size()
      << " data points to "PCD_FILE".pcd." << std::endl;
  system("pcd2ply "PCD_FILE".pcd "PCD_FILE".ply > /dev/null");

}

void
depthAndRGB2PCD(const char* dimage_fname, const char* cimage_fname, const char* map_fname, const char* pcdout_fname)
{
  // Depth image ---------------------
  RawImage di;
  try{
    di.loadImage(dimage_fname);
    if(di.getDepth() != 2)
      throw std::runtime_error("Not a 16 bit image");
  }catch (std::exception& e) {
    throw std::runtime_error(e.what());
  }
  cout << "Depth image " << di.getComments() << endl;
  // RGB image ---------------------------
  RawImage ci;
  try{
    ci.loadImage(cimage_fname);
  }catch (std::exception& e) {
    throw std::runtime_error(e.what());
  }
  cout << "RGB image " << ci.getComments() << endl;
  // MAP between RGB and 3D data -------------------------------
  ifstream fmap(map_fname, ios::binary);
  if(!fmap.good()) throw std::runtime_error("Unable to open file.");

  int map_size = ci.getWidth()*ci.getHeight()*2;
  int* color_coordinates = new int[map_size];

  int map_width = 0, map_height = 0;
  fmap >> map_width; fmap >> map_height;
  fmap.ignore(1); //skip '\n'

  fmap.read((char*)color_coordinates, map_size*sizeof(int));

  cout << fmap.gcount() << " bytes read" << endl;
  cout << map_size*sizeof(int) << " bytes excepted to be read" << endl;

  if(fmap.gcount() != (unsigned)(map_size*sizeof(int)) || ci.getWidth() != map_width || ci.getHeight() != map_height)
     throw std::runtime_error("Error reading map file. [ERR_CODE 1].");
  if(fmap.fail())
   throw std::runtime_error("Error reading map file. [ERR_CODE 2].");
  if(fmap.eof())
   throw std::runtime_error("Error reading map file. [ERR_CODE 3].");

  fmap.close();
  //--------------------------------
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // Fill in the cloud data
  cloud.width = di.getWidth();
  cloud.height = di.getHeight();
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  ushort* pdepth = di.get16bitData();
  uchar* pcimage = ci.getData();
  for(int y=0; y < di.getHeight(); y++)
    for(int x=0; x < di.getWidth(); x++)
    {
      pcl::PointXYZRGB point;
      int dindex = y*di.getWidth() + x;
      ushort depth = pdepth[dindex];

//      if(depth!=0) point.z = depth;
//      else point.z = std::numeric_limits<double>::quiet_NaN();

      point.z = depth;
      point.x = x; point.y = y;

      int rgb_x = color_coordinates[dindex*2];
      int rgb_y = color_coordinates[dindex*2 +1];
      if(rgb_x >= 0 && rgb_y >=0 && rgb_x < ci.getWidth() && rgb_y < ci.getHeight())
      {
        int rgb_index = rgb_y*ci.getWidth() +  rgb_x;
        point.r = pcimage[rgb_index*3];
        point.g = pcimage[rgb_index*3+1];
        point.b = pcimage[rgb_index*3+2];
      }

      cloud(x,y) = point;
    }

  pcl::io::savePCDFile(pcdout_fname, cloud);
  std::cerr << "Saved " << cloud.points.size()
           << " data points to "<< pcdout_fname << std::endl;
  system((std::string("pcd2ply")+" "+std::string(pcdout_fname)+" "+std::string(pcdout_fname)+".ply > /dev/null").c_str());
}

void
addRGB2PCD(const char* pcd_input, const char* pcd_output, const char* rgb_image, const char* map_file)
{
  // RGB image ---------------------------
  RawImage ci;
  try{
    ci.loadImage(rgb_image);
  }catch (std::exception& e) {
    throw std::runtime_error(e.what());
  }
  // 3D data ---------------------------
  std::cout << "Loading points from pcd file " << pcd_input << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_input, *pcloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n",pcd_input);
  }
  std::cout << "Loaded "
           << pcloud->width * pcloud->height
           << " data points from " PCD_FILE ".pcd"
           << std::endl;
  // MAP between RGB and 3D data -------------------------------
  ifstream fmap(map_file, ios::binary);
  if(!fmap.good())
      throw std::runtime_error("Unable to open file.");

  int map_size = ci.getWidth()*ci.getHeight()*2;
  int* color_coordinates = new int[map_size];

  int map_width = 0, map_height = 0;
  fmap >> map_width; fmap >> map_height;
  fmap.ignore(1); //skip '\n'

  fmap.read((char*)color_coordinates, map_size*sizeof(int));

  cout << fmap.gcount() << " bytes read" << endl;
  cout << map_size*sizeof(int) << " bytes excepted to be read" << endl;

  if( fmap.gcount() != (unsigned)(map_size*sizeof(int)) || ci.getWidth() != map_width || ci.getHeight() != map_height )
      throw std::runtime_error("Error reading map file. [ERR_CODE 1].");
  if(fmap.fail())
    throw std::runtime_error("Error reading map file. [ERR_CODE 2].");
  if(fmap.eof())
    throw std::runtime_error("Error reading map file. [ERR_CODE 3].");

  fmap.close();

  //----------------------
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = pcloud->width;
  cloud.height = pcloud->height;
  cloud.is_dense = pcloud->is_dense;
  cloud.points.resize(cloud.width * cloud.height);

  uchar* imdata = ci.getData();

  for (size_t i = 0; i < pcloud->points.size(); ++i)
  {
    pcl::PointXYZRGB point;
    point.x = pcloud->points[i].x;
    point.y = pcloud->points[i].y;
    point.z = pcloud->points[i].z;
//    point.z = 0;
    if(point.x < 0 || point.x >= pcloud->width || point.y < 0 || point.y >= pcloud->height )
      continue;
    int depth_index = point.y*pcloud->width + point.x;
    int rgb_x = color_coordinates[depth_index*2];
    int rgb_y = color_coordinates[depth_index*2 +1];
    if(rgb_x >= 0 && rgb_y >=0 && rgb_x < ci.getWidth() && rgb_y < ci.getHeight())
    {
      int rgb_index = rgb_y*ci.getWidth() +  rgb_x;
      point.r = imdata[rgb_index*3];
      point.g = imdata[rgb_index*3+1];
      point.b = imdata[rgb_index*3+2];
    }

    cloud.points[i] = point;
  }

  pcl::io::savePCDFile(pcd_output, cloud);
  std::cerr << "Saved " << cloud.points.size()
           << " data points to "<< pcd_output << std::endl;
//  system("pcd2ply "PCD_FILE"_rgb.pcd "PCD_FILE"_rgb.ply > /dev/null");
//  system((std::string("pcd2ply")+" "+std::string(pcd_input)+" "+std::string(pcd_output)+".ply > /dev/null").c_str());
}

void filterZeroDepth()
{
  std::cout << "Loading points from pcd file " << PCD_FILE ".pcd" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (PCD_FILE ".pcd", *pcloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file " PCD_FILE ".pcd \n");
  }
  std::cout << "Loaded "
            << pcloud->width * pcloud->height
            << " data points from" PCD_FILE ".pcd"
            << std::endl;

  pcloud->is_dense = false;

  for (size_t i = 0; i < pcloud->points.size(); ++i) {
    ushort depth = pcloud->points[i].z;
    if(depth==0)
      pcloud->points[i].z = std::numeric_limits<double>::quiet_NaN();
  }

  pcl::io::savePCDFile(PCD_FILE"_nan.pcd", *pcloud);
  std::cerr << "Saved " << pcloud->points.size()
            << " data points to "PCD_FILE"_nan.pcd." << std::endl;
  system("pcd2ply "PCD_FILE"_nan.pcd "PCD_FILE"_nan.ply > /dev/null");
}

int main (int argc, char** argv){

  std::cout << "hello world" << std::endl;

//  depth2PCD();
//  filterZeroDepth();
  depthAndRGB2PCD(SEQ_PATH"depth/video_depth_1.pgm",SEQ_PATH"rgb/video_rgb_1.ppm",SEQ_PATH"map/video_map_1.coord",PCD_FILE"_rgb_v2.pcd");
//  addRGB2PCD(PCD_FILE".pcd",PCD_FILE"_rgb.pcd",SEQ_PATH"rgb/video_rgb_1.ppm",SEQ_PATH"map/video_map_1.coord");

  return (0);
}
