#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>


int main()
{
  // Loading stuff //
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pawn (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());

  pcl::io::loadPCDFile ("ros_table_cleaned.pcd", *cloud_table);
  pcl::io::loadPCDFile ("ros_pawn_cleaned.pcd", *cloud_pawn);

  // Creating the viewer //
  pcl::visualization::PCLVisualizer *viewer;
  viewer = new pcl::visualization::PCLVisualizer();
  viewer->addPointCloud(cloud_pawn, "cloud");


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }



  float threshold = 0.04;


  // Double for loop to find close points //
  int counter = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it_pawn = cloud_pawn->begin(); it_pawn!= cloud_pawn->end(); it_pawn++)
  {
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it_table = cloud_table->begin(); it_table!= cloud_table->end(); it_table++)
    {
      float distance = sqrt(pow(it_table->x - it_pawn->x, 2) + pow(it_table->y - it_pawn->y, 2) + pow(it_table->z - it_pawn->z, 2));
      if (distance < threshold)
      {
        outliers->indices.push_back(counter);
        break;
      }

    }
    counter++;
  }

  // Removal of the close points //
  extract.setInputCloud(cloud_pawn);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*cloud_pawn);

  // Computing the transform between the point cloud and the camera (only translation) //
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_pawn, centroid);

  printf("Position of centroid: %f %f %f\n", centroid(0), centroid(1), centroid(2));

  // Saving etc //
  pcl::PCDWriter pcd_writer;
  pcd_writer.write("pawn.pcd", *cloud_pawn);

  // // Creating the viewer //
  // pcl::visualization::PCLVisualizer *viewer;
  // viewer = new pcl::visualization::PCLVisualizer();
  // viewer->addPointCloud(cloud_pawn, "cloud");
  //
  //
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  // }

  return 0;
}
