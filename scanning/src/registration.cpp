#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

// Artificially translate a point cloud for testing
pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  // initialze output point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  // create transformation
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // translation
  transform.translation() << 0.01, 0.01, 0.01;
  // rotation about z axis
  //float rot_angle = - 10 * 3.141592 / 180; // 10 degrees
  //transform.rotate(Eigen::AngleAxisf(rot_angle, Eigen::Vector3f::UnitZ()));

  // apply transformation
  pcl::transformPointCloud(*input_cloud, *output_cloud, transform);

  return output_cloud;
}

// Perform ICP to return transformation
Eigen::Matrix4f perform_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_src);
  icp.setInputTarget(cloud_target);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*cloud_final);

  Eigen::Matrix4f icp_transform = icp.getFinalTransformation();

  return icp_transform;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Please provide a filename for point cloud 1" << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud_1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 = transform_cloud(cloud_1);

  // Estimate the transformation from cloud 2 to cloud 1
  Eigen::Matrix4f transformation = perform_icp(cloud_2, cloud_1);
  std::cout << transformation << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*cloud_2, *transformed_cloud_2, transformation);

  // Creating the viewer //
  pcl::visualization::PCLVisualizer *viewer;
  viewer = new pcl::visualization::PCLVisualizer();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
      cloud_1, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(
      cloud_2, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(
      transformed_cloud_2, 255, 255, 0);
  viewer->addPointCloud(cloud_1, red, "cloud 1");
  viewer->addPointCloud(cloud_2, green, "cloud 2");
  viewer->addPointCloud(transformed_cloud_2, yellow, "transformed cloud 2");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce ();
  }

  return 0;
}


