//
// Created by Branislav Jenco on 30.05.2021.
// Takes a point cloud saved in PCD, segments it and finds the centroids of the segments

//
// Created by Branislav Jenco on 30.05.2021.
// Takes a point cloud saved in PCD, segments it and finds the centroids of the segments

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/auto_io.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>
#include "utils.h"


#include <iostream>
#include <string>
#include <cmath>
#include <queue>

typedef pcl::PointCloud<pcl::PointXYZL> point_cloud;


struct classcomp
{
    bool operator() (const pcl::PointXYZL& a, const pcl::PointXYZL& b) const
    {
        return a.z < b.z;
    }
};


void
read_distances_file(const point_cloud::Ptr &cloud, const std::string &filename) {
    std::ifstream file(filename);
    std::string   line;
    while(std::getline(file, line))
    {
        std::stringstream   linestream(line);
        std::string         data;
        float x, y, z, res, l;
        int32_t angle;
        linestream >> res >> x >> y >> z >> l >> angle;
        pcl::PointXYZL point(x, y, z, angle_to_color(angle));
        cloud->push_back(point);
    }
}

void
visualize(const point_cloud::Ptr &cloud, const pcl::ModelCoefficients::Ptr plane_coeff)
{
    std::cout << "Visualizing" << std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> color(cloud);
    viz.addPointCloud<pcl::PointXYZL>(cloud, color, "point cloud");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
    viz.addPlane(*plane_coeff);
    viz.addCoordinateSystem();
    while (!viz.wasStopped ())
    {
        viz.spinOnce ();
    }
    viz.close();
}

void
visualize(const point_cloud::Ptr &cloud)
{
    std::cout << "Visualizing" << std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> color(cloud);
    viz.addPointCloud<pcl::PointXYZL>(cloud, color, "point cloud");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
    while (!viz.wasStopped ())
    {
        viz.spinOnce ();
    }
    viz.close();
}

int
color_to_laser_id(int color)
{
    float laser_id;
    laser_id = (float) color / 255 * laser_angles_vlp16.size();
    return ceil(laser_id);
}

Eigen::VectorXf
fit_plane(const point_cloud::Ptr &cloud)
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZL>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZL>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZL> ransac(model_p);
    ransac.setDistanceThreshold(.01);

    std::cout << "Computing plane" << std::endl;
    ransac.computeModel();
    ransac.refineModel();
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    return coeff;
}


void
relabel_color_to_id(point_cloud::Ptr &cloud) {
    for (auto &point: cloud->points) {
        point.label = color_to_laser_id(point.label);
    }
}

void
distribute_by_label(const point_cloud::Ptr &cloud, const int num, std::vector<point_cloud::Ptr> &clouds) {
    for (int i = 0; i < num; i++) {
        point_cloud::Ptr c(new point_cloud);
        clouds.push_back(c);
    }

    // Distribute the points from the initial cloud into their respective clouds
    for (auto &point: cloud->points) {
        clouds[point.label]->push_back(point);
    }
}


int
main(int argc, char **argv)
{
    point_cloud::Ptr cloud(new point_cloud);

    char opt_f[] = "-f";
    char opt_o[] = "-o";
    char opt_t[] = "-t";
    std::string input_file;
    std::string output_file;
    bool real;
    pcl::console::parse_argument(argc, (char **) argv, opt_f, input_file);
    pcl::console::parse_argument(argc, (char **) argv, opt_o, output_file);
    pcl::console::parse_argument(argc, (char **) argv, opt_t, real);

    std::cout << "Loading point cloud" << std::endl;
    if (real) {
        read_distances_file(cloud, input_file);
    } else {
        pcl::io::loadPCDFile<pcl::PointXYZL>(input_file, *cloud);
    }
    relabel_color_to_id(cloud);

    // Making a point cloud for each laser
    std::vector<point_cloud::Ptr> laser_clouds;


    // Distributing into new point clouds based on label
    distribute_by_label(cloud, 16, laser_clouds);

    // Fit a plane to the original point cloud, get it's model coeffs
    auto plane_parameters = fit_plane(cloud);
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients ());
    plane_coeff->values.resize (4);
    plane_coeff->values[0] = plane_parameters.x ();
    plane_coeff->values[1] = plane_parameters.y ();
    plane_coeff->values[2] = plane_parameters.z ();
    plane_coeff->values[3] = plane_parameters.w ();

    // Add a 90deg rotated plane
    pcl::ModelCoefficients::Ptr rotated_plane_coeff(new pcl::ModelCoefficients ());
    rotated_plane_coeff->values.resize (4);
    rotated_plane_coeff->values[0] = plane_parameters.x ();
    rotated_plane_coeff->values[1] = plane_parameters.y ();
    rotated_plane_coeff->values[2] = plane_parameters.z ();
    rotated_plane_coeff->values[3] = plane_parameters.w ();
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf((90.0*M_PI) / 180, Eigen::Vector3f::UnitZ())); // Y for floor, Z for wall
    pcl::transformPlane(rotated_plane_coeff, plane_coeff, transform); // For some reason, it seems like the in/out coeffs are switched


    // Get centroids
    std::vector<std::vector<double>> centroid_distances_by_laser;
    int c = 0;
    for (auto &laser_cloud: laser_clouds)
    {
//        // I dont quite understand why I did this projection
//        point_cloud::Ptr laser_cloud_plane_proj(new point_cloud);
//        pcl::ProjectInliers<pcl::PointXYZL> proj;
//        proj.setModelType (pcl::SACMODEL_PLANE);
//        proj.setInputCloud (laser_cloud);
//        proj.setModelCoefficients (plane_coeff);
//        proj.filter (*laser_cloud_plane_proj);
//
//        point_cloud::Ptr laser_cloud_line_proj(new point_cloud);
//        pcl::ProjectInliers<pcl::PointXYZL> proj2;
//        proj.setModelType (pcl::SACMODEL_PLANE);
//        proj.setInputCloud (laser_cloud_plane_proj);
//        proj.setModelCoefficients (rotated_plane_coeff);
//        proj.filter (*laser_cloud_line_proj);

        // Segment individual clouds
        pcl::search::KdTree<pcl::PointXYZL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZL>);
        tree->setInputCloud(laser_cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZL> extraction;
        extraction.setInputCloud(laser_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        extraction.setClusterTolerance(0.025); // found works best with around 0.04 with virtual scan, 0.06 for real
        extraction.setSearchMethod(tree);
        extraction.extract(cluster_indices);
        std::cout << "Total number of points " << laser_cloud->size() << std::endl;
        std::cout << "Num of clusters " << cluster_indices.size() << std::endl;
        std::vector<point_cloud::Ptr> cluster_clouds;


        // Label each cluster in the cloud based
        int count = 0;
        for (auto &c: cluster_indices)
        {
            for (auto &idx: c.indices)
            {
                laser_cloud->at(idx).label = count;
            }
            count = count + 1;
        }

        if (c < 1) {
//            visualize(laser_cloud, rotated_plane_coeff);
//            visualize(laser_cloud_plane_proj, rotated_plane_coeff);
//            visualize(laser_cloud_line_proj, rotated_plane_coeff);
            c++;
        }
        // Distribute based on the new label
        distribute_by_label(laser_cloud, cluster_indices.size(), cluster_clouds);

        // Get centroids
        std::vector<pcl::PointXYZL> centroids;
        for (auto &cc: cluster_clouds)
        {
            pcl::PointXYZL centroid;
            pcl::computeCentroid<pcl::PointXYZL, pcl::PointXYZL>(*cc, centroid);
            centroids.push_back(centroid);
        }

        point_cloud::Ptr centroid_cloud(new point_cloud);
        for (auto &c: centroids)
        {
            std::cout << c.getVector4fMap() << std::endl;
            centroid_cloud->push_back(c);
        }

        // Sort by the Z coordinate (this only works for wall)
        std::priority_queue<pcl::PointXYZL, std::vector<pcl::PointXYZL>, classcomp> priQue;
        for (auto & centroid : centroids) {
            priQue.push(centroid);
        }

        // Get distance to the fitted plane
        std::vector<double> distances;
        while (!priQue.empty())
        {
            const pcl::PointXYZL &centroid = priQue.top();
            double distance_to_plane = pcl::pointToPlaneDistanceSigned(centroid, plane_parameters);
            distances.push_back(distance_to_plane);
            priQue.pop();
        }
        centroid_distances_by_laser.push_back(distances);
    }

    // Save the centroid distances
    ofstream myfile;
    myfile.open(output_file);

    int laser_id = 0;
    for (auto &distances : centroid_distances_by_laser)
    {
        myfile << "Laser id: " << laser_id << std::endl;
        for (auto distance: distances) {
            myfile << distance << std::endl;
        }
        laser_id = laser_id + 1;
    }
    myfile.close();
}
