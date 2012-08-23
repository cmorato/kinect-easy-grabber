//============================================================================
// Name        : pcdvideo.cpp
// Author      : Pachi
//============================================================================

//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//#include <boost/thread/mutex.hpp>
#include <iostream>

using namespace std;



class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer() :
			viewer("PCL Cloud Viewer"),
			pcloud(new pcl::PointCloud<pcl::PointXYZRGB>) {	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
	}

	void run() {
//		pcl::Grabber* interface = new pcl::OpenNIGrabber();

//		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
//				boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

//		interface->registerCallback(f);
//		interface->start();

		while (!viewer.wasStopped()) {
//			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

//		interface->stop();
	}

	void run2() {

		viewer.showCloud(pcloud);

		while (!viewer.wasStopped())
		{
		}

	}

	void loadCloud(const string& pcd_file)
	{
	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, *pcloud) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read pcd file.\n");
		return;
	  }
	  std::cout << "Loaded "
				<< pcloud->width * pcloud->height
				<< " data points from "
				<< pcd_file
				<< std::endl;
	}

//private:
	pcl::visualization::CloudViewer viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud;
};

void
showCloud(const std::string& pcd_file) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read pcd file.\n");
		return;
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from "
			<< pcd_file
			<< std::endl;

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {
	}
}

int main(int argc, char** argv) {

	if(argc == 2)
	{
		std::cout << "Input params:" << std::endl;
		std::cout << argv[1] << std::endl;
	}
	else
	{
		std::cerr << "Wrong params" << std::endl;
		return -1;
	}

	cout << "Starting viewer ..." << endl;

	SimpleOpenNIViewer v;
	v.loadCloud(argv[1]);
	v.run2();



	return 0;
}

