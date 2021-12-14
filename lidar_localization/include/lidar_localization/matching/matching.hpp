/*
 * @Description: 地图匹配定位算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_
#define LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"

namespace lidar_localization
{
    class Matching
    {
    public:
        Matching();

        bool Update(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose);
        bool SetGNSSPose(const Eigen::Matrix4f &init_pose);

        /**
         * @brief 过滤global_map，将结果输出，并且将has_new_global_map_设为false
         * 
         * @param global_map[out]  
         */
        void GetGlobalMap(CloudData::CLOUD_PTR &global_map);
        CloudData::CLOUD_PTR &GetLocalMap();
        CloudData::CLOUD_PTR &GetCurrentScan();
        bool HasInited();
        bool HasNewGlobalMap();
        bool HasNewLocalMap();

    private:
        bool InitWithConfig();
        bool InitDataPath(const YAML::Node &config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
        bool InitBoxFilter(const YAML::Node &config_node);

        bool SetInitPose(const Eigen::Matrix4f &init_pose);
        bool InitGlobalMap();
        bool ResetLocalMap(float x, float y, float z);

    private:
        // 记录地图路径
        std::string map_path_ = "";
        // box滤波器
        std::shared_ptr<BoxFilter> box_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        // 局部滤波器
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        // 全局滤波器
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        // 配准算法
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        // 记录经过矩阵变换后的点云数据
        CloudData::CLOUD_PTR current_scan_ptr_;
        Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

        // 初始化标记，当收到gnss数据大于等于3时才为true
        bool has_inited_ = false;
        bool has_new_global_map_ = false;
        bool has_new_local_map_ = false;
    };
}

#endif