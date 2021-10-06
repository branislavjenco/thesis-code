//
// Created by Branislav Jenco on 14.05.2021.
// Simply takes a point cloud saved in PCD with labels and transforms the labels (which
// are meant to be laser angles corresponding to the VLP16 laser angles) to laser ids, based
// on the VLP16 manual

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZL> point_cloud;

std::vector<int32_t> laser_angles_vlp16{-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

uint32_t
laser_id_to_color(int32_t laser_id) {
    auto color = (float_t) laser_id / laser_angles_vlp16.size();
    color = (uint32_t) (color * 255.0);
    return color;
}

uint8_t
angle_to_color(uint32_t label) {
    auto angle = (int32_t) label;
    auto itr = std::find(laser_angles_vlp16.begin(), laser_angles_vlp16.end(), angle);
    if (itr != laser_angles_vlp16.cend()) {
        int32_t idx = std::distance(laser_angles_vlp16.begin(), itr);
        return laser_id_to_color(idx);
    }
    else {
        std::cout << "Element not found";
    }
}


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
