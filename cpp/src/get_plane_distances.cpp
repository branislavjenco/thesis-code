//
// Created by Branislav Jenco on 21.09.2020.
// Crop point cloud with minX/Y/Z and maxX/Y/Z boundaries, fit a plane and output a file with signed
// distances between the fitted plane and points.
// Assuming the cropped point cloud is of a flat surface to which a plane can be fitted.
// The input must be a PointXYZL point cloud (with labels)
//

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <iostream>
#include <fstream>

typedef pcl::PointCloud<pcl::PointXYZL> point_cloud;

struct DistanceData
{
    double distance_signed;
    int laser_id;
    float *data;
};

std::vector<float>
get_floats_from_string(const std::string &str, char delimiter)
{
    std::vector<float> internal;
    std::stringstream ss(str);
    std::string token;

    while (getline(ss, token, delimiter))
    {
        internal.push_back(std::stof(token));
    }

    return internal;
}

void
visualize(const point_cloud::Ptr &cloud)
{
    std::cout << "Visualizing" << std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> color(cloud);
    viz.addPointCloud<pcl::PointXYZL>(cloud, color, "point cloud");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
    while (!viz.wasStopped ())
    {
        viz.spinOnce ();
    }
    viz.close();
}

void
save_distances(const std::vector<DistanceData> &dists, const std::string &output_file)
{
    std::cout << "Writing distances to file" << std::endl;
    ofstream myfile;
    myfile.open(output_file);

    for (auto &dist : dists)
    {
        myfile << dist.distance_signed << " " << dist.data[0] << " " << dist.data[1] << " " << dist.data[2] << " "
               << dist.data[3] << " " << dist.laser_id << std::endl;
    }
    myfile.close();
}

std::vector<DistanceData>
get_distances(const point_cloud::Ptr &cloud)
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZL>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZL>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZL> ransac(model_p);
    ransac.setDistanceThreshold(.01);

    std::cout << "Computing plane" << std::endl;
    ransac.computeModel();
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);


    std::cout << "Getting distances from plane" << std::endl;
    std::vector<DistanceData> dists;
    for (auto &it : *cloud)
    {
        DistanceData d{};
        d.distance_signed = pcl::pointToPlaneDistanceSigned(it, coeff);
        d.laser_id = it.label;
        d.data = it.data;
        dists.emplace_back(d);
    }

    return dists;
}

void
crop_point_cloud(const point_cloud::Ptr &input,
                 const point_cloud::Ptr &output,
                 std::vector<float> &x_boundaries,
                 std::vector<float> &y_boundaries,
                 std::vector<float> &z_boundaries,
                 std::vector<float> &rotation_args)
{
    Eigen::Vector3f rotation(rotation_args[0], rotation_args[1], rotation_args[2]);
    std::cout << "Cropping point cloud" << std::endl;


    pcl::CropBox<pcl::PointXYZL> boxFilter;
    boxFilter.setRotation(rotation);
    boxFilter.setMin(Eigen::Vector4f(x_boundaries[0], y_boundaries[0], z_boundaries[0], 1.0));
    boxFilter.setMax(Eigen::Vector4f(x_boundaries[1], y_boundaries[1], z_boundaries[1], 1.0));
    boxFilter.setInputCloud(input);
    boxFilter.filter(*output);

    std::cout << output->height << std::endl;
    std::cout << output->width << std::endl;
    std::cout << output->points.size() << std::endl;
}

void
fill_values(const std::string &message, std::vector<float> &current_values)
{
    std::string user_input;
    std::vector<float> updated_values(current_values.size());

    std::cout << message << " Current: ";
    for (auto &v : current_values)
    {
        std::cout << v << " ";
    }
    std::cout << "Leave empty to leave as is." << std::endl;

    std::getline(std::cin, user_input);
    if (user_input.empty())
    {
        std::cout << "Using old values." << std::endl;
        updated_values = current_values;
    }
    else
    {
        std::vector<float> user_values = get_floats_from_string(user_input, ' ');
        updated_values = user_values;
        std::cout << "Using new values: ";
        for (auto &v : updated_values)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }
    current_values = updated_values;
}

void interactive_crop(const point_cloud::Ptr &input,
                      const point_cloud::Ptr &output) {

    std::vector<float> x_boundaries{-10.0, 10.0};
    std::vector<float> y_boundaries{-10.0, 10.0};
    std::vector<float> z_boundaries{-10.0, 10.0};
    std::vector<float> rotation_args{0.0, 0.0, 0.0};
    std::vector<std::pair<std::string, std::vector<float>>> params;
    params.emplace_back("Set <MinX MaxX> values.", x_boundaries);
    params.emplace_back("Set <MinY MaxY> values.", y_boundaries);
    params.emplace_back("Set <MinZ MaxZ> values.", z_boundaries);
    params.emplace_back("Set <RotX RotY RotZ> values.", rotation_args);
    std::string answer;
    bool stop = false;


    while (!stop)
    {
        for (auto &param: params)
        {
            fill_values(param.first, param.second);
        }
        crop_point_cloud(input, output, params[0].second, params[1].second, params[2].second, params[3].second);

        visualize(output);

        std::cout << "Write out distances file?" << std::endl;
        std::getline(std::cin, answer);
        if (answer == "y")
        {
            stop = true;
        }
        else
        {
            stop = false;
            output->clear();
        }
    }
}

int
main(int argc, char **argv)
{
    point_cloud::Ptr cloud(new point_cloud);
    point_cloud::Ptr cropped(new point_cloud);

    char opt_f[] = "-f";
    char opt_o[] = "-o";
    char opt_s[] = "-s";
    char opt_c[] = "-c";
    std::string input_file;
    std::string output_file;
    bool skip = false;
    bool output_cloud = false;
    pcl::console::parse_argument(argc, (char **) argv, opt_f, input_file);
    pcl::console::parse_argument(argc, (char **) argv, opt_o, output_file);
    pcl::console::parse_argument(argc, (char **) argv, opt_s, skip);
    pcl::console::parse_argument(argc, (char **) argv, opt_c, output_cloud);

    std::cout << "Skip: " << skip << std::endl;
    std::cout << "Output cloud: " << output_cloud << std::endl;

    std::cout << "Loading point cloud" << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZL>(input_file, *cloud);

    if (skip) {
        cropped = cloud;
    } else {
        interactive_crop(cloud, cropped);
    }

    auto distances = get_distances(cropped);
    save_distances(distances, output_file);
    if (output_cloud) {
        std::cout << "Outputting cropped point cloud" << std::endl;
        std::string new_filename = input_file.substr(0, (input_file.length() - 4));
        new_filename = new_filename + "_crop.pcd";
        visualize(cropped);
        pcl::io::savePCDFileBinary<pcl::PointXYZL>(new_filename, *cropped);
    }

}
