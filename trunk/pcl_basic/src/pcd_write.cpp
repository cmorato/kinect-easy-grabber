//============================================================================
// http://www.pointclouds.org/documentation/tutorials/writing_pcd.php
//============================================================================

/*
 *
 * Defines:
 * -DEIGEN_USE_NEW_STDVECTOR
 * -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
 * -DFLANN_STATIC
 *
 * Flags:
 * -Wno-deprecated
 * -march=native
 *
 * Includes:
 * -I/usr/include/vtk-5.8
 * -I/usr/include/pcl-1.5
 * -I/usr/include/eigen3
 * -I/usr/include/openni
 * -I/usr/include/qhull
 *
 * Linker Flags
 * -Wno-deprecated
 * -rdynamic
 * -Wl,-Bstatic
 *
 * Libraries
 * -lboost_system-mt
 * -lboost_filesystem-mt
 * -lboost_thread-mt
 * -lpthread
 * -lboost_date_time-mt
 * -lboost_iostreams-mt
 * -lpcl_common
 * -lpcl_octree
 * -lOpenNI
 * /usr/lib/libvtkCommon.so.5.8.0
 * /usr/lib/libvtkRendering.so.5.8.0
 * /usr/lib/libvtkHybrid.so.5.8.0
 * -lpcl_io
 * -lpcl_sample_consensus
 * -lflann_cpp_s
 * -lpcl_kdtree
 * -lpcl_search
 * -lpcl_features
 * -lqhull
 * -lpcl_surface
 * -lpcl_segmentation
 * -lpcl_filters
 * -lpcl_tracking
 * -lpcl_keypoints
 * -lpcl_visualization
 * -lpcl_registration
 * -lpcl_apps
 * /usr/lib/libvtkParallel.so.5.8.0
 * /usr/lib/libvtkRendering.so.5.8.0
 * /usr/lib/libvtkGraphics.so.5.8.0
 * /usr/lib/libvtkImaging.so.5.8.0
 * /usr/lib/libvtkIO.so.5.8.0
 * /usr/lib/libvtkFiltering.so.5.8.0
 * /usr/lib/libvtkCommon.so.5.8.0
 * -lm
 * /usr/lib/libvtksys.so.5.8.0
 * -ldl
 * -Wl,-rpath,/usr/lib/openmpi/lib
 * -Wl,-rpath-link,/usr/lib/openmpi/lib
 *
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main (int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i) {
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size()
			<< " data points to test_pcd.pcd." << std::endl;

	for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y
				<< " " << cloud.points[i].z << std::endl;

	return (0);
}
