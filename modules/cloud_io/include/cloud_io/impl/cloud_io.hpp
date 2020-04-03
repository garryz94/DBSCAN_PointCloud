#ifndef SEM2VEC_CLOUD_IO_HPP
#define SEM2VEC_CLOUD_IO_HPP

#include "cloud_io/cloud_io.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/make_shared.hpp>
#include <glog/logging.h>

namespace sem2vec
{
template <typename PointT>
CloudIO<PointT>::CloudIO()
{
    //initialize format_map_: only support pcd and ply now
    format_map_["pcd"] = 1;
    format_map_["ply"] = 2;
}

template <typename PointT>
int CloudIO<PointT>::readCloud(PointTCloudPtr& cloud, const std::string& file_name, const std::string& format)
{
    if(cloud == NULL)
    {
        cloud = boost::make_shared<PointTCloud>(*(new PointTCloud));
    }
    cloud->clear();
    if(format_map_.count(format) == 0)
    {
        LOG(ERROR) << "ERROR: unknown cloud format";
        return -1;
    }
    switch (format_map_[format])
    {
        case 1: //pcd format
            if(pcl::io::loadPCDFile<PointT>(file_name, *cloud) == -1)
            {
                LOG(ERROR) << "ERROR: load file " << file_name;
                return -1;
            }
            break;
        case 2: //ply format
            if(pcl::io::loadPLYFile<PointT>(file_name, *cloud) == -1)
            {
                LOG(ERROR) << "ERROR: load file " << file_name;
                return -1;
            }
            break;
        default:
            LOG(ERROR) << "ERROR: unknown cloud format";
            return -1;
    }
    return cloud->points.size();
}

template <typename PointT>
int CloudIO<PointT>::readCloud(PointTCloudPtr& cloud, const std::string& file_name)
{
    int32_t dot_pos = file_name.find_last_of(".");
    if(dot_pos == -1)
    {
        LOG(ERROR) << "ERROR: invalid file_name: " << file_name;
        return -1;
    }
    std::string format = file_name.substr(dot_pos + 1);
    return readCloud(cloud, file_name, format);
}

template <typename PointT>
bool CloudIO<PointT>::writeCloud(const PointTCloudPtr& cloud, const std::string& file_name, const std::string& format)
{
    if(cloud == NULL || cloud->points.empty())
    {
        LOG(ERROR) << "ERROR: invalid input cloud";
        return false;
    }
    int32_t dot_pos = file_name.find_last_of(".");
    if(dot_pos == -1)
    {
        LOG(ERROR) << "ERROR: invalid file_name: " << file_name;
        return false;
    }
    std::string file_format = file_name.substr(dot_pos + 1);
    if(file_format != format)
    {
        LOG(ERROR) << "ERROR: format does not match: " << file_name << " vs " << format;
        return false;
    }
    switch (format_map_[format])
    {
        case 1:
            if(pcl::io::savePCDFileBinary(file_name, *cloud) == -1)
            {
                LOG(ERROR) << "ERROR: save file " << file_name;
                return false;
            }
            break;
        case 2:
            if(pcl::io::savePLYFileBinary(file_name, *cloud) == -1)
            {
                LOG(ERROR) << "ERROR: save file " << file_name;
                return false;
            }
            break;
        default:
            LOG(ERROR) << "ERROR: unknown cloud format";
            return false;
    }
    return true;
}

template <typename PointT>
bool CloudIO<PointT>::writeCloud(const PointTCloudPtr& cloud, const std::string& file_name)
{
    int32_t dot_pos = file_name.find_last_of(".");
    if(dot_pos == -1)
    {
        LOG(ERROR) << "ERROR: invalid file_name: " << file_name;
        return false;
    }
    std::string format = file_name.substr(dot_pos + 1);
    return writeCloud(cloud, file_name, format);
}

template <typename PointT>
bool CloudIO<PointT>::writeClouds(const std::vector<PointTCloudPtr>& clouds, const std::string& out_dir,
                                  const std::string& format, const std::string& fname)
{
    if(clouds.empty())
    {
        LOG(ERROR) << "input is empty";
        return false;
    }
    if(!boost::filesystem::exists(out_dir))
    {
        LOG(INFO) << out_dir << " doesn't exist, now create...";
        boost::filesystem::create_directories(out_dir);
    }
    for(uint32_t i = 0; i < clouds.size(); i++)
    {
        if(clouds[i] == NULL)
        {
            LOG(ERROR) << "ERROR: clouds[" << i << "] == NULL";
            return false;
        }
        std::string file_name;
        if(out_dir[out_dir.size() - 1] == '/')
        {
            file_name = out_dir + fname + std::to_string(i);
        }
        else
        {
            file_name = out_dir + "/" + fname + std::to_string(i);
        }
        switch (format_map_[format])
        {
            case 1:
                file_name += ".pcd";
                break;
            case 2:
                file_name += ".ply";
                break;
            default:
                LOG(ERROR) << "unknown out_format";
                return false;
        }
        if(!clouds[i]->points.empty())
        {
            LOG(INFO) << i + 1 << " / " << clouds.size() << " writing file to " << file_name;
            if(!writeCloud(clouds[i], file_name, format))
            {
                LOG(ERROR) << "Can't write " << file_name;
                return false;
            }
        }
    }
    return true;
}
}

#endif
