/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#pragma once

#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cloud_convert_interface.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tools/point_types.h"
#include "common/cloudMap.hpp"
#include "common/mutexDeque.hpp"

namespace zjloc
{

    /**
     * 预处理雷达点云
     *
     * 将Velodyne, ouster, avia等数据转到FullCloud
     * 该类由MessageSync类持有，负责将收到的雷达消息与IMU同步并预处理后，再交给LO/LIO算法
     */
    class CloudConvert2 : public CloudConvertInterface
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum class LidarType
        {
            AVIA = 1,    // 大疆的固态雷达
            VELO32,      // Velodyne 32线
            OUST64,      // ouster 64线
            ROBOSENSE16, //  速腾16线
            PANDAR,
        };

        CloudConvert2() = default;
        ~CloudConvert2() = default;

        CloudConvert2(const CloudConvert2 &) = delete;
        CloudConvert2 &operator=(const CloudConvert2 &) = delete;

        void AddData(Eigen::Vector3d in) override;

        /**
         * 处理livox avia 点云
         * @param msg
         * @param pcl_out
         */
        void
        Process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                std::vector<std::vector<point3D>> &pcl_out,
                std::vector<double> &v_t);

        /**
         * 处理sensor_msgs::PointCloud2点云
         * @param msg
         * @param pcl_out
         */
        void Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                     std::vector<std::vector<point3D>> &pcl_out,
                     std::vector<double> &v_t);

        /// 从YAML中读取参数
        void LoadFromYAML(const std::string &yaml);

        //  返回激光的时间
        double getTimeSpan() { return timespan_; }

        LidarType lidar_type_ = LidarType::AVIA; // 雷达类型

    private:
        void reset();
        void adaCut();
        void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
        void Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void PandarHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

        std::vector<double> v_timestamp;
        std::vector<point3D> cloud_out_;           //  输出点云
        std::vector<std::vector<point3D>> v_cloud; //  out cloud

        MutexDeque<Eigen::Vector3d> vecDeque;

        int point_filter_num_ = 1; // 跳点
        double blind = 0.1;
        int sweep_cut_num = 1.0;
        double delta_time = 0.1;
        int degenerate_window_num = 5;

        double timespan_;
    };
} // namespace zjloc