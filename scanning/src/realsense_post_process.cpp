/*
Take an image from the Realsense,
save filtered and unfiltered versions to pcd files
*/

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr rs2_to_pcl(const rs2::points &points)
{
    PointCloud::Ptr cloud(new PointCloud);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto vertex = points.get_vertices();
    for (int i = 0; i < cloud->width * cloud->height; i++)
    {
        cloud->points[i].x = vertex->x;
        cloud->points[i].y = vertex->y;
        cloud->points[i].z = vertex->z;
        vertex++;
    }

    return cloud;
}

void visualize_cloud(const PointCloud::Ptr cloud)
{
    pcl::visualization::CloudViewer viewer("Point Cloud");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}

int main(int argc, char **argv)
{
    bool decimation = true;
    bool threshold = true;
    bool spatial = true;
    bool temporal = false;
    bool hole_filling = false;

    /*
    // Parse command line arguments
    try
    {
        options_description desc{"Options"};
        desc.add_options()
            ("help,h", "Help screen")
            ("decimation,d", "use decimation filter")
            ("threshold,t", "use threshold filter")
            ("spatial,s", "use spatial filter")
            ("temporal,T", "use temporal filter")
            ("hole_filling,H", "use hole-filling filter")
        ;
        variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);

        if (vm.count("help"))
        {
            cout  << desc << endl;
            return 0;
        }
        if (vm.count("decimation"))
        {
            cout << "using decimation filter" << endl;
            decimation = true;
        }
        if (vm.count("threshold"))
        {
            cout << "using threshold filter" << endl;
            threshold = true;
        }
        if (vm.count("spatial"))
        {
            cout << "using spatial filter" << endl;
            spatial = true;
        }
        if (vm.count("temporal"))
        {
            cout << "using temporal filter" << endl;
            temporal = true;
        }
        if (vm.count("hole_filling"))
        {
            cout << "using hole-filling" << endl;
            hole_filling = true;
        }
    }
    catch (const error &e)
    {
        cout << e.what() << endl;
        return 0;
    }
    */

    bool disparity = spatial || temporal;

    rs2::pipeline pipe;
    pipe.start();

    rs2::pointcloud pc;
    rs2::points points;
    rs2::points points_filtered;

    // Decimation filter and options
    rs2::decimation_filter decimation_filter;
    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    // Threshold filter and options
    rs2::threshold_filter threshold_filter;
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1f);
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.5f);

    // Disparity transform and inverse
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    // Spatial filter and options
    rs2::spatial_filter spatial_filter;
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 2);
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 2);
    
    // Temporal filter and options
    rs2::temporal_filter temporal_filter;
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.25f);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temporal_filter.set_option(RS2_OPTION_HOLES_FILL, 2);

    // Hole-filling filter and options
    rs2::hole_filling_filter hole_filling_filter;
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 0);

    rs2::frameset frames = pipe.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::depth_frame depth_filtered = depth;

    if (decimation)
        depth_filtered = decimation_filter.process(depth_filtered);
    if (threshold)
        depth_filtered = threshold_filter.process(depth_filtered);
    if (disparity)
        depth_filtered = depth_to_disparity.process(depth_filtered);
    if (spatial)
        depth_filtered = spatial_filter.process(depth_filtered);
    if (temporal)
        depth_filtered = temporal_filter.process(depth_filtered);
    if (disparity)
        depth_filtered = disparity_to_depth.process(depth_filtered);
    if (hole_filling)
        depth_filtered = hole_filling_filter.process(depth_filtered);

    // Convert depth to point cloud
    points = pc.calculate(depth);
    points_filtered = pc.calculate(depth_filtered);

    PointCloud::Ptr cloud = rs2_to_pcl(points);
    PointCloud::Ptr cloud_filtered = rs2_to_pcl(points_filtered);

    pcl::io::savePCDFileASCII("rs_raw.pcd", *cloud);
    pcl::io::savePCDFileASCII("rs_filtered.pcd", *cloud_filtered);

    // Open visualizer for both point clouds
    visualize_cloud(cloud);
    visualize_cloud(cloud_filtered);

    return 0;
}
