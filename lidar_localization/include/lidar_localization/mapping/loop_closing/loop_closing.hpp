/*
 * @Description: 闭环检测算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_
#define LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/key_frame.hpp"
#include "lidar_localization/sensor_data/loop_pose.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization
{
    class LoopClosing
    {
    public:
        LoopClosing();

        /**
         * @brief 利用关键帧和gnss信息完成更新
         * 
         * @param key_frame 关键帧
         * @param key_gnss gnss信息
         * @return true 
         * @return false 
         */
        bool Update(const KeyFrame key_frame, const KeyFrame key_gnss);

        bool HasNewLoopPose();
        LoopPose &GetCurrentLoopPose();

    private:
        bool InitWithConfig();
        bool InitParam(const YAML::Node &config_node);
        bool InitDataPath(const YAML::Node &config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);

        /**
         * @brief 检测最近的关键帧索引
         * 
         * @param key_frame_index[out] 
         * @return true 
         * @return false 
         */
        bool DetectNearestKeyFrame(int &key_frame_index);

        /**
         * @brief 和最近的关键帧执行配准
         * 
         * @param key_frame_index[in] 最近邻关键帧的索引
         * @return true 
         * @return false 
         */
        bool CloudRegistration(int key_frame_index);

        /**
         * @brief 
         * 
         * @param key_frame_index[in] 输入的最近邻关键帧索引 
         * @param map_cloud_ptr[out] 
         * @param map_pose[out]       gnss中key_frame_index对应的帧位姿
         * @return true 
         * @return false 
         */
        bool JointMap(int key_frame_index, CloudData::CLOUD_PTR &map_cloud_ptr, Eigen::Matrix4f &map_pose);
        
        /**
         * @brief 加载最新关键帧点云数据
         * 
         * @param scan_cloud_ptr[out] 保存最新帧的点云
         * @param scan_pose[out]      最新帧的gnss信息
         * @return true 
         * @return false 
         */
        bool JointScan(CloudData::CLOUD_PTR &scan_cloud_ptr, Eigen::Matrix4f &scan_pose);

        /**
         * @brief 
         * 
         * @param map_cloud_ptr[in]  局部地图
         * @param scan_cloud_ptr[in] 当前扫描地图
         * @param scan_pose[in]      当前gnss位姿
         * @param result_pose[out]   输出相对位姿
         * @return true 
         * @return false 
         */
        bool Registration(CloudData::CLOUD_PTR &map_cloud_ptr,
                          CloudData::CLOUD_PTR &scan_cloud_ptr,
                          Eigen::Matrix4f &scan_pose,
                          Eigen::Matrix4f &result_pose);

    private:
        std::string key_frames_path_ = "";
        // map是以历史帧为中心，往前后时刻各选取extend_frame_num个关键帧，放在一起拼接成的
        int extend_frame_num_ = 3;
        // 防止检测过于频繁，每隔loop_step个关键帧检测一次闭环
        int loop_step_ = 10;
        // 过于小的闭环没有意义，所以只有两帧之间的关键帧个数超出这个值再做检测
        int diff_num_ = 100;
        // 检测区域，只有两帧距离小于这个值，才做闭环匹配
        float detect_area_ = 10.0;
        float fitness_score_limit_ = 2.0;

        std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
        // 配准对象指针
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        // 记录所有关键帧
        std::deque<KeyFrame> all_key_frames_;
        // 记录所有关键gnss
        std::deque<KeyFrame> all_key_gnss_;

        // 记录当前回环位姿
        LoopPose current_loop_pose_;
        bool has_new_loop_pose_ = false;
    };
}

#endif