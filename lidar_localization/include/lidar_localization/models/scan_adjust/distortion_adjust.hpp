/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:38:12
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization
{
    class DistortionAdjust
    {
    public:
        void SetMotionInfo(float scan_period, VelocityData velocity_data);
        
        /**
         * @brief 点云畸变校正
         * 
         * @param input_cloud_ptr 
         * @param output_cloud_ptr 
         * @return true 
         * @return false 
         */
        bool AdjustCloud(CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &output_cloud_ptr);

    private:
        inline Eigen::Matrix3f UpdateMatrix(float real_time);

    private:
        // lidar扫描周期
        float scan_period_;
        // 记录线速度
        Eigen::Vector3f velocity_;
        // 记录角速度
        Eigen::Vector3f angular_rate_;
    };
} // namespace lidar_slam
#endif