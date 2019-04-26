#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/uniform_sampling.h>
// #include <pcl/keypoints/uniform_sampling.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

int
 main (int argc, char** argv)
{

  if (argc != 3)
  {
    std::cout << "Please provide a filename for input file and output file" << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  pcl::io::loadPCDFile (argv[1], *cloud);

  std::cout << "Initial number of points: " << cloud->size () << std::endl;


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud (cloud_filtered);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-1.0, 1.0);
  pass_x.filter (*cloud_filtered);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud (cloud_filtered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-1, 0.25);
  pass_y.filter (*cloud_filtered);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud (cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.0, 2.0);
  pass_z.filter (*cloud_filtered);

  printf("PassThrough finished\n");

  // // Reducing size by uniformly sampling
  // pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
  // uniform_sampling.setInputCloud (cloud_filtered);
  // uniform_sampling.setRadiusSearch (0.003);
  // uniform_sampling.compute (*cloud_filtered);

  // printf("UniformSampling finished\n");

  // Create the filtering object (remove lonely points)
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (15);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  printf("StatisticalOutlierRemoval finished\n");

  std::cout << "Final number of points: " << cloud_filtered->size () << std::endl;


  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (argv[2], *cloud_filtered, false);

  // Creating the viewer //
  pcl::visualization::PCLVisualizer *viewer;
  viewer = new pcl::visualization::PCLVisualizer();
  viewer->addPointCloud(cloud_filtered, "cloud");


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }


  return (0);
}
