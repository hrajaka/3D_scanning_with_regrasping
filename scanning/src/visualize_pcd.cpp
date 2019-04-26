#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

// load the data from a file
PointCloudPtr load_data(const char *filename)
{
    PointCloudPtr cloud(new PointCloud);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>
            (filename, *cloud) == -1)
    {
        PCL_ERROR("Coudln't read file\n");
    }

    return cloud;
}

int main (int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "usage: visualize_pcd filename" << endl;
        return -1;
    }
    const char *filename = argv[1];
    PointCloudPtr cloud = load_data(filename);

    cout << cloud << endl;
    
    // visualize point cloud
    pcl::visualization::CloudViewer viewer("Point cloud");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }

    return 0;
}

