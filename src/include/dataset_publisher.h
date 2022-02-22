#ifndef OPENVSLAM_ROS_DATASET_PUBLISHER_H
#define OPENVSLAM_ROS_DATASET_PUBLISHER_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <experimental/filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace openvslam_ros
{

class dataset_publisher
{
private:
public:
    std::vector<std::string> paths;

    dataset_publisher(const std::string &dataset_dir)
    {
        std::cout << "dataset dir: " << dataset_dir << std::endl;
        if(!std::experimental::filesystem::exists(dataset_dir))
        {
            std::cerr << "dataset_dir does not exists: " << dataset_dir << std::endl;
            std::abort();
        }

        sort(dataset_dir);
    }

    ~dataset_publisher(){}


    void sort(const std::string &dataset_dir)
    {
        for (const auto & file : std::experimental::filesystem::directory_iterator(dataset_dir))
            paths.push_back(file.path());

        std::sort(paths.begin(), paths.end());

        // for(auto path_it=paths.begin(); path_it!=paths.end(); ++path_it)
        //     std::cout << *path_it << std::endl;;
    }


    static std::string basename(const std::string& path) {
        return path.substr(path.find_last_of('/') + 1);
    }

    std::string basename(int i) {
        return paths.at(i).substr(paths.at(i).find_last_of('/') + 1);
    }

    size_t size(){return paths.size();}
    std::string at(size_t i) {return paths.at(i);}

};


} // namespace openvslam_ros


#endif