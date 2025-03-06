#include "cloud_convert2.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
// #include <execution>

namespace zjloc
{

    void CloudConvert2::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                                std::vector<std::vector<point3D>> &pcl_out,
                                std::vector<double> &v_t)
    {
        reset();

        AviaHandler(msg);
        pcl_out = v_cloud;
        v_t = v_timestamp;
    }

    void CloudConvert2::Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                std::vector<std::vector<point3D>> &pcl_out,
                                std::vector<double> &v_t)
    {
        reset();

        switch (lidar_type_)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

        case LidarType::PANDAR:
            PandarHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        pcl_out = v_cloud;
        v_t = v_timestamp;
    }

    void CloudConvert2::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        int plsize = msg->point_num;

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = msg->points.back().offset_time / tm_scale;

        delta_time = timespan_ / sweep_cut_num;

        v_timestamp.resize(sweep_cut_num);
        for (int i = 0; i < sweep_cut_num; i++)
            v_timestamp[i] = /*msg->header.stamp.toSec() +*/ (i + 1) * delta_time; //  delta time

        // std::cout << "span:" << timespan_ << ",0: " << msg->points[0].offset_time / tm_scale
        //           << " , 100: " << msg->points[100].offset_time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(msg->points[i].x) &&
                  std::isfinite(msg->points[i].y) &&
                  std::isfinite(msg->points[i].z)))
                continue;

            if (msg->points[i].offset_time / tm_scale > timespan_)
                std::cout << "------" << __FUNCTION__ << ", " << __LINE__ << ", error timespan:" << timespan_ << " < " << msg->points[i].offset_time << std::endl;

            if (i % point_filter_num_ != 0)
                continue;

            double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                           msg->points[i].z * msg->points[i].z;
            if (range > 250 * 250 || range < blind * blind)
                continue;

            if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                point3D point_temp;
                point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                point_temp.point = point_temp.raw_point;
                point_temp.relative_time = msg->points[i].offset_time / tm_scale; // curvature unit: ms
                point_temp.intensity = msg->points[i].reflectivity;

                point_temp.timestamp = headertime + point_temp.relative_time;

                int id = (msg->points[i].offset_time / tm_scale) / delta_time; //  get id
                if (id < 0 || id >= sweep_cut_num)
                {
                    // std::cout << "ERROR, id = " << id << std::endl;
                    id = id - 1;
                }

                point_temp.alpha_time = point_temp.relative_time / delta_time - id;
                point_temp.timespan = delta_time;
                point_temp.ring = msg->points[i].line;

                v_cloud[id].push_back(point_temp);
                // cloud_out_.push_back(point_temp);
            }
        }
    }

    void CloudConvert2::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = pl_orig.points.back().t / tm_scale;
        delta_time = timespan_ / sweep_cut_num;

        v_timestamp.resize(sweep_cut_num);
        for (int i = 0; i < sweep_cut_num; i++)
            v_timestamp[i] = /*msg->header.stamp.toSec() +*/ (i + 1) * delta_time; //  delta time

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale
        //           << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;

            int id = (pl_orig.points[i].t / tm_scale) / delta_time; //  get id
            if (id < 0 || id >= sweep_cut_num)
            {
                // std::cout << "ERROR, id = " << id << std::endl;
                id = id - 1;
            }

            point_temp.alpha_time = point_temp.relative_time / delta_time - id;
            point_temp.timespan = delta_time;
            point_temp.ring = pl_orig.points[i].ring;

            v_cloud[id].push_back(point_temp);
            // cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert2::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();

        double headertime = msg->header.stamp.toSec();
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;
        delta_time = timespan_ / sweep_cut_num;

        v_timestamp.resize(sweep_cut_num);
        for (int i = 0; i < sweep_cut_num; i++)
            v_timestamp[i] = /*msg->header.stamp.toSec() +*/ (i + 1) * delta_time; //  delta time

        // std::cout << timespan_ << std::endl;

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = pl_orig.points[i].timestamp;

            int id = point_temp.relative_time / delta_time; //  get id
            if (id < 0 || id >= sweep_cut_num)
            {
                // std::cout << "ERROR, id = " << id << std::endl;
                id = id - 1;
            }

            point_temp.alpha_time = point_temp.relative_time / delta_time - id;
            point_temp.timespan = delta_time;
            point_temp.ring = pl_orig.points[i].ring;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            v_cloud[id].push_back(point_temp);
        }
    }

    void CloudConvert2::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6 - nclt  or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().time / tm_scale;
        delta_time = timespan_ / sweep_cut_num;

        v_timestamp.resize(sweep_cut_num);
        for (int i = 0; i < sweep_cut_num; i++)
            v_timestamp[i] = /*msg->header.stamp.toSec() +*/ (i + 1) * delta_time; //  delta time

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale
        //           << " , 300: " << pl_orig.points[300].time / tm_scale
        //           << ", 1300: " << pl_orig.points[1300].time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;

            int id = point_temp.relative_time / delta_time; //  get id
            if (id < 0 || id >= sweep_cut_num)
            {
                // std::cout << "ERROR, id = " << id << std::endl;
                id = id - 1;
            }

            point_temp.alpha_time = point_temp.relative_time / delta_time - id;
            point_temp.timespan = delta_time;
            point_temp.ring = pl_orig.points[i].ring;

            v_cloud[id].push_back(point_temp);
        }
    }

    void CloudConvert2::PandarHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<pandar_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6

        auto time_list_pandar = [&](pandar_ros::Point &point_1, pandar_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;
        delta_time = timespan_ / sweep_cut_num;

        v_timestamp.resize(sweep_cut_num);
        for (int i = 0; i < sweep_cut_num; i++)
            v_timestamp[i] = /*msg->header.stamp.toSec() +*/ (i + 1) * delta_time; //  delta time

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
        //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;

            int id = point_temp.relative_time / delta_time; //  get id
            if (id < 0 || id >= sweep_cut_num)
            {
                // std::cout << "ERROR, id = " << id << std::endl;
                id = id - 1;
            }

            point_temp.alpha_time = point_temp.relative_time / delta_time - id;
            point_temp.timespan = delta_time;
            point_temp.ring = pl_orig.points[i].ring;

            v_cloud[id].push_back(point_temp);
        }
    }

    void CloudConvert2::AddData(Eigen::Vector3d in)
    {
        // TODO: 考虑处理效率差异，增加时间戳
        vecDeque.push_back(in);
        if (vecDeque.size() > degenerate_window_num)
            vecDeque.pop_front();
    }

    void CloudConvert2::reset()
    {
        v_cloud.reserve(sweep_cut_num);
        if (v_cloud.size() != sweep_cut_num)
        {
            v_cloud.reserve(sweep_cut_num);
            for (int i = 0; i < sweep_cut_num; i++)
                v_cloud.push_back(std::vector<point3D>());
        }

        for (int i = 0; i < sweep_cut_num; i++)
            std::vector<point3D>().swap(v_cloud[i]);
    }

    void CloudConvert2::adaCut()
    {
    }

    void CloudConvert2::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        point_filter_num_ = yaml["preprocess"]["point_filter_num"].as<int>();
        sweep_cut_num = yaml["preprocess"]["sweep_cut_num"].as<int>();
        blind = yaml["preprocess"]["blind"].as<double>();
        degenerate_window_num = yaml["preprocess"]["degenerate_window_num"].as<double>();
        LOG(INFO) << "Sweep cut num: " << sweep_cut_num << ", degenerate_window_num: "
                  << degenerate_window_num << std::endl;

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else if (lidar_type == 5)
        {
            lidar_type_ = LidarType::PANDAR;
            LOG(INFO) << "Using Pandar LIdar";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
    }

} // namespace zjloc
