#include <iostream>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_cleaner(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cleaned (new pcl::PointCloud<pcl::PointXYZ>);

  std::cout << "  Initial number of points: " << cloud->size () << std::endl;

  // Create the PassThrough filter (removing points outside dimensions bounds)
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud (cloud);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-2.0, 2.0);
  pass_x.filter (*cloud_cleaned);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud (cloud_cleaned);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-2, 2);
  pass_y.filter (*cloud_cleaned);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud (cloud_cleaned);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.2, 1.3);
  pass_z.filter (*cloud_cleaned);

  std::cout << "  PassThrough finished " << std::endl;
  std::cout << "  Current number of points: " << cloud_cleaned->size () << std::endl;

  // Create the filtering object (remove lonely points)
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_cleaned);
  sor.setMeanK (15);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_cleaned);

  std::cout << "  StatisticalOutlierRemoval finished " << std::endl;

  std::cout << "  Final number of points: " << cloud_cleaned->size () << std::endl;

  return cloud_cleaned;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr background_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr background, pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
  std::cout << "  Background point cloud: " << background->points.size() << " points" << std::endl;
	std::cout << "  Object point cloud: " << object->points.size() << " points" << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
  float threshold = 0.04;

  // Double for loop to find close points //
  int counter = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it_object = object->begin(); it_object!= object->end(); it_object++)
  {
    if (counter % 2000 == 0)
    {
      std::cout << "  Looking at point number: " << counter << std::endl;
    }

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it_background = background->begin(); it_background!= background->end(); it_background++)
    {
      float distance = sqrt(pow(it_background->x - it_object->x, 2) + pow(it_background->y - it_object->y, 2) + pow(it_background->z - it_object->z, 2));
      if (distance < threshold)
      {
        outliers->indices.push_back(counter);
        break;
      }
    }
    counter++;
  }

  // Removal of the close points //
  extract.setInputCloud(object);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*object);

  return object;
}


pcl::PolygonMesh generate_mesh(char* name)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (name, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.03);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(0);
  gp3.setMaximumAngle(M_PI);
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  return triangles;
}


int
 main (int argc, char** argv)
{

  if (argc != 5)
  {
    std::cout << "Please provide a filename for input object file, input background file, cleaned file, isolated file" << std::endl;
    return -1;
  }
  // Creating the viewer, writer and useful point clouds //
  // pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer());
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr background (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cleaned(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr background_cleaned(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_isolated(new pcl::PointCloud<pcl::PointXYZ>);


  // Loading the raw point clouds of the and the background
  std::cout << "\nLoading point clouds" << std::endl;
  pcl::io::loadPCDFile (argv[1], *object);
  pcl::io::loadPCDFile (argv[2], *background);

  // Removing useless regions and lonely points and saving
  std::cout << "\nCleaning point clouds" << std::endl;
  object_cleaned = point_cloud_cleaner(object);
  background_cleaned = point_cloud_cleaner(background);
  writer.write<pcl::PointXYZ> (argv[3], *object_cleaned, false);

  // Removing background and saving
  std::cout << "\nRemoving background" << std::endl;
  object_isolated = background_removal(background, object_cleaned);
  writer.write<pcl::PointXYZ> (argv[4], *object_isolated, false);

  // viewer->addPointCloud(object_isolated, "cloud");
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  // }

  // // Generating the obj file and saving
  // std::cout << "\nGenerating obj file" << std::endl;
  // pcl::PolygonMesh mesh = generate_mesh(argv[4]);
  // pcl::io::saveOBJFile (argv[5], mesh);


  return (0);
}
