#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <cmath>

int
 main (int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << "Please provide a filename for point cloud 1 and a filename for point cloud 2" << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile (argv[1], *cloud_1);
  pcl::io::loadPCDFile (argv[2], *cloud_2);

  // ICP stuff //

  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setInputSource(cloud_1);
  // icp.setInputTarget(cloud_2);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  // // pcl::PointCloud<pcl::PointXYZ> cloud_final;
  // icp.align(*cloud_final);
  //
  // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  // icp.getFitnessScore() << std::endl;
  // Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
  // std::cout << icp_transform << std::endl;
  //
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud (*cloud_1, *transformed_cloud, icp_transform);

  // Other method: manually fitting //

  // putting the point clouds in the world frame
  Eigen::Vector4f centroid_1;
  pcl::compute3DCentroid(*cloud_1, centroid_1);
  Eigen::Vector4f centroid_2;
  pcl::compute3DCentroid(*cloud_2, centroid_2);


  // centering the point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_world (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_world (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f camera_transform_1 = Eigen::Affine3f::Identity();
  camera_transform_1.translation() << -centroid_1(0), -centroid_1(1), -centroid_1(2);

  Eigen::Affine3f camera_transform_2 = Eigen::Affine3f::Identity();
  camera_transform_2.translation() << -centroid_2(0), -centroid_2(1), -centroid_2(2);

  pcl::transformPointCloud (*cloud_1, *cloud_1_world, camera_transform_1);
  pcl::transformPointCloud (*cloud_2, *cloud_2_world, camera_transform_2);

  // rotating the point clouds so that they are properly oriented
  float camera_rot_angle = -3.141592 / 2;
  Eigen::Affine3f camera_transform_rot = Eigen::Affine3f::Identity();
  camera_transform_rot.rotate (Eigen::AngleAxisf (camera_rot_angle, Eigen::Vector3f::UnitX()));

  pcl::transformPointCloud (*cloud_1_world, *cloud_1_world, camera_transform_rot);
  pcl::transformPointCloud (*cloud_2_world, *cloud_2_world, camera_transform_rot);


  // translating the points clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_translated (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_translated (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f small_translation = Eigen::Affine3f::Identity();
  small_translation.translation() << 0, -0.05, 0;
  pcl::transformPointCloud (*cloud_1_world, *cloud_1_translated, small_translation);
  pcl::transformPointCloud (*cloud_2_world, *cloud_2_translated, small_translation);

  // rotating manually the second point cloud about +z
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_rotated (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f object_transform = Eigen::Affine3f::Identity();
  object_transform.translation() << 0, 0, 0;
  float object_rot_angle = - 10 * 3.141592 / 180;
  object_transform.rotate (Eigen::AngleAxisf (object_rot_angle, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud (*cloud_2_translated, *cloud_2_rotated, object_transform);


  // Creating the viewer //
  pcl::visualization::PCLVisualizer *viewer;
  viewer = new pcl::visualization::PCLVisualizer();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_1_translated, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_2_rotated, 0, 255, 0);

  viewer->addPointCloud(cloud_1_translated, red, "cloud_1");
  viewer->addPointCloud(cloud_2_rotated, green, "cloud_2");
  // viewer->addCoordinateSystem(1.0);


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }

 return (0);
}
