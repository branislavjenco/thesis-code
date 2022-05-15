/*
Created by Branislav Jenco on 14.05.2021.

Takes a point cloud saved in PCD with labels and transforms the labels (which
are meant to be laser angles corresponding to the VLP16 laser angles) to laser ids, based
on the VLP16 manual
*/

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <iostream>
#include "utils.h"

typedef pcl::PointCloud<pcl::PointXYZL> point_cloud;



int
main(int argc, char **argv)
{
    point_cloud::Ptr cloud(new point_cloud);

    char opt_f[] = "-f";
    std::string input_file;
    pcl::console::parse_argument(argc, (char **) argv, opt_f, input_file);

    std::cout << "Loading point cloud" << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZL>(input_file, *cloud);
    for (auto &point: cloud->points) {
        point.label = angle_to_color(point.label);
    }

    std::cout << "Outputting  point cloud" << std::endl;
    std::string new_filename = input_file.substr(0, (input_file.length() - 4));
    new_filename = new_filename + "_color.pcd";
    pcl::io::savePCDFileBinary<pcl::PointXYZL>(new_filename, *cloud);

}
