/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization
{
    class NDTRegistration : public RegistrationInterface
    {
    public:
        NDTRegistration(const YAML::Node &node);
        NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

        bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;

        /**
         * @brief 
         * 
         * @param input_source 输入源点云
         * @param predict_pose 初始位姿
         * @param result_cloud_ptr 输出对齐后的点云
         * @param result_pose  源点云到起始帧的坐标变换
         * @return true 
         * @return false 
         */
        bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                       const Eigen::Matrix4f &predict_pose,
                       CloudData::CLOUD_PTR &result_cloud_ptr,
                       Eigen::Matrix4f &result_pose) override;
        float GetFitnessScore() override;

    private:
        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    private:
        pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    };
}

#endif