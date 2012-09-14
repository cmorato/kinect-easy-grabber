//============================================================================
// Name        : pcdvideo.cpp
// Author      : Pachi
//============================================================================

//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//#include <boost/thread/mutex.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>

using namespace std;

#define QCLOUD_SIZE 10

std::string cloud_path;

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

//-------------------------------------
//-------------------------------------
//-------------------------------------

class SimpleOpenNIViewer {
public:

	typedef pcl::PointXYZRGB PointType;
	typedef pcl::PointCloud<PointType>::Ptr CloudType;


	SimpleOpenNIViewer() {
		pviewer = NULL;
	}
	~SimpleOpenNIViewer() {
		delete pviewer;
	}


	void run() {

		pviewer = new pcl::visualization::CloudViewer("PCL Cloud Viewer");

		struct timespec slptm;
		slptm.tv_sec = 0;
		slptm.tv_nsec = 30000000L;

		vector<CloudType>::iterator itb = qcloud.begin();
		vector<CloudType>::iterator ite = qcloud.end();
		vector<CloudType>::iterator it = itb;
		while (!pviewer->wasStopped() && !qcloud.empty())
		{
			CloudType cl = *it++;
			pviewer->showCloud(cl);
			if(it == ite) it = itb;

//			nanosleep(&slptm,NULL);
		}

	}

	void loadCloud(const string& pcd_file, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud)
	{
	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, *pcloud) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read pcd file.\n");
		return;
	  }
//	  std::cout << "Loaded "
//				<< pcloud->width * pcloud->height
//				<< " data points from "
//				<< pcd_file
//				<< std::endl;
	}

	void loadClouds(const string& dir, size_t first, size_t last)
	{
		cout << "Loading " << (last - first +1) << " clouds ..." << endl;

		time_t start,end;
		double ttot = 0;
        for(size_t i=first; i <= last; i++)
		{
            stringstream pcd_file;
            pcd_file << dir << "/video_pcd_" << i << ".pcd";
//            cout << pcd_file.str() << endl;

            time(&start);
			CloudType cl(new pcl::PointCloud<pcl::PointXYZRGB>);
			loadCloud(pcd_file.str(), cl);
			time(&end);
			ttot += difftime (end,start);

			cout << "Clouds left: " << (last-i) << " Time left: " << (ttot/(i-first+1)*(last-i)/60) << " min" << endl;

			qcloud.push_back(cl);
		}
	}

  private:
	pcl::visualization::CloudViewer* pviewer;
	vector<CloudType> qcloud;
};


int main(int argc, char** argv) {

	if(argc == 2)
	{
		std::cout << "Input params:" << std::endl;
		std::cout << argv[1] << std::endl;
	}
	else
	{
        std::cerr << "Not enough params" << std::endl;
		return -1;
	}

	cout << "Starting viewer ..." << endl << endl;

	SimpleOpenNIViewer v;

	cloud_path = argv[1];
	v.loadClouds(cloud_path,1,100);

	cout << "Showing clouds ..." << endl << endl;

	v.run();

	return 0;
}

