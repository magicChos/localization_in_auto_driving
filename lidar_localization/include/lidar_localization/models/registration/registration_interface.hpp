/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization
{
    class RegistrationInterface
    {
    public:
        virtual ~RegistrationInterface() = default;

        virtual bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) = 0;
        /**
         * @brief 执行扫描匹配
         * 
         * @param input_source 
         * @param predict_pose[in] 初始位姿 
         * @param result_cloud_ptr 对齐的点云 
         * @param result_pose 输出的位姿信息
         * @return true 
         * @return false 
         */
        virtual bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                               const Eigen::Matrix4f &predict_pose,
                               CloudData::CLOUD_PTR &result_cloud_ptr,
                               Eigen::Matrix4f &result_pose) = 0;
        virtual float GetFitnessScore() = 0;
    };
}

#endif