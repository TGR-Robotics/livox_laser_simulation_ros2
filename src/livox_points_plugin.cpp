/**
 * @file livox_points_plugin.cpp
 * @brief Livox激光雷达仿真插件实现文件
 * 
 * 该文件实现了在Gazebo中模拟Livox激光雷达的功能，支持多种Livox型号，
 * 并发布CustomMsg和PointCloud2两种格式的点云数据。
 * 
 * 主要功能：
 * 1. 从CSV文件读取真实激光雷达扫描模式
 * 2. 在Gazebo中进行射线追踪仿真
 * 3. 发布Livox专用的CustomMsg格式点云数据
 * 4. 发布标准PointCloud2格式点云数据
 * 5. 支持FAST_LIO等SLAM算法
 */

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <rclcpp/logging.hpp>

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>  // 用于存储最新的激光扫描数据到laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace gazebo
{

    // 注册Gazebo传感器插件
    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    /**
     * @brief 构造函数
     */
    LivoxPointsPlugin::LivoxPointsPlugin() {}

    /**
     * @brief 析构函数
     */
    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    /**
     * @brief 将CSV数据转换为旋转信息
     * 
     * 该函数将CSV文件中的扫描数据转换为AviaRotateInfo结构体，
     * 用于后续的射线追踪计算。
     * 
     * @param datas CSV文件中的原始数据，每行包含[时间, 方位角, 天顶角]
     * @param avia_infos 输出的旋转信息向量
     * @param min_vertical_angle 最小垂直角度（弧度）
     * @param max_vertical_angle 最大垂直角度（弧度）
     */
    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos, double min_vertical_angle, double max_vertical_angle)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;  // 度转弧度转换因子
        RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "convertDataToRotateInfo data size is %ld!", datas.size());
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                // 将天顶角转换为俯仰角（elevation）
                // 物理关系：elevation = 90° - zenith
                // 这里使用 yaw = zenith - 90° 来转换
                auto yaw = data[2] * deg_2_rad - M_PI_2;
                
                // CSV 列说明：
                // - data[0]: 时间 (Time/s)
                // - data[1]: 方位角 (Azimuth/deg) - 水平旋转角度
                // - data[2]: 天顶角 (Zenith/deg) - 从垂直方向测量的角度
                // 
                // 角度转换关系：
                // - 俯仰角 0°（水平） ⇒ zenith = 90°
                // - 俯仰角 -7°（略向下） ⇒ zenith = 90° - (-7°) = 97°
                // 
                // 过滤指定垂直角度范围内的光束
                if (-yaw > min_vertical_angle && -yaw < max_vertical_angle) {
                    avia_infos.emplace_back();
                    avia_infos.back().time = data[0];                    // 时间戳
                    avia_infos.back().azimuth = data[1] * deg_2_rad;     // 方位角（弧度）
                    avia_infos.back().zenith = yaw;                      // 俯仰角（弧度，右手坐标系）
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
            }
        }
    }

    /**
     * @brief 插件加载函数
     * 
     * 该函数在Gazebo加载传感器时被调用，负责：
     * 1. 读取CSV扫描模式文件
     * 2. 初始化ROS2发布器
     * 3. 设置射线追踪参数
     * 4. 创建激光雷达的物理碰撞体
     * 
     * @param _parent 传感器指针
     * @param sdf SDF元素指针，包含传感器配置参数
     */
    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        // 初始化ROS2节点
        node_ = gazebo_ros::Node::Get(sdf);
        
        // 读取CSV扫描模式文件
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        
        // 保存SDF指针和解析传感器参数
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");
        auto verticalElem = scanElem->GetElement("vertical");
        
        // 获取垂直扫描角度范围
        min_vertical_angle = verticalElem->Get<double>("min_angle");
        max_vertical_angle = verticalElem->Get<double>("max_angle");

        // 设置传感器和话题信息
        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        // 解析父级和子级名称
        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        // 初始化Gazebo传输节点
        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        
        // 创建ROS2发布器
        // PointCloud2发布器 - 用于标准ROS2点云处理
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + "_PointCloud2", 10);
        // CustomMsg发布器 - 用于FAST_LIO等SLAM算法
        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);
        // LaserScan发布器 - 用于传统激光扫描处理
        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        // 转换CSV数据为旋转信息
        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos, min_vertical_angle, max_vertical_angle);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();
        // 加载基础射线插件
        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        
        // 获取父级实体
        parentEntity = this->world->EntityByName(_parent->ParentName());
        
        // 创建激光雷达的物理碰撞体用于射线追踪
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        
        // 创建自定义的多射线形状
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        
        // 获取采样和降采样参数
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);
        
        // 初始化射线形状
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        
        // 获取距离范围参数
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        
        // 根据CSV数据创建射线：每帧覆盖整个FOV，按固定步长均匀抽样
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        const int64_t ray_count = std::max<int64_t>(1, samplesStep / downSample);
        const int64_t stride = std::max<int64_t>(1, maxPointSize / ray_count);
        for (int64_t r = 0; r < ray_count; ++r)
        {
            int64_t index = (r * stride) % maxPointSize;
            auto &rotate_info = aviaInfos[static_cast<size_t>(index)];

            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }

    /**
     * @brief 处理新的激光扫描数据
     * 
     * 该函数在每次激光扫描更新时被调用，负责：
     * 1. 执行射线追踪计算
     * 2. 处理扫描结果并转换为点云数据
     * 3. 发布CustomMsg和PointCloud2格式的消息
     * 
     * 这是插件的核心函数，实现了从射线追踪到点云发布的完整流程。
     */
    void LivoxPointsPlugin::OnNewLaserScans()
    {
        // 检查射线形状是否已初始化
        if (rayShape)
        {
            std::vector<std::pair<int, AviaRotateInfo>> points_pair;
            
            // 初始化射线扫描点对
            InitializeRays(points_pair, rayShape);
            
            // 更新射线追踪，执行实际的碰撞检测
            rayShape->Update();

            // 创建激光扫描消息并设置时间戳
            // 使用 Gazebo 仿真时间（gazebo::common::Time，sec + nsec 精度），
            // 避免依赖 ROS /clock topic 带来的颗粒度问题。
            // 参考 gazebo_ros_imu_sensor 插件用 sensor->LastUpdateTime() 的做法。
            const gazebo::common::Time sim_time = raySensor->LastMeasurementTime();
            msgs::Set(laserMsg.mutable_time(), sim_time);
            msgs::LaserScan *scan = laserMsg.mutable_scan();
            InitializeScan(scan);

            const uint64_t now_ns =
                static_cast<uint64_t>(sim_time.sec) * 1000000000ULL +
                static_cast<uint64_t>(sim_time.nsec);
            const builtin_interfaces::msg::Time stamp =
                gazebo_ros::Convert<builtin_interfaces::msg::Time>(sim_time);

            livox_ros_driver2::msg::CustomMsg pp_livox;

            // 设置 CustomMsg 头部字段
            const std::string frame = raySensor->Name();
            for (size_t fi = 0;
                 fi < std::min(frame.size(),
                               static_cast<size_t>(livox_ros_driver2::msg::CustomMsg::MAX_ID_LENGTH));
                 ++fi) {
                    pp_livox.frame_id[fi] = static_cast<uint8_t>(frame[fi]);
            }
            pp_livox.publish_time_ns = now_ns;
            pp_livox.timebase_ns = now_ns;
            pp_livox.lidar_id = 0;
            pp_livox.rsvd[0] = pp_livox.rsvd[1] = pp_livox.rsvd[2] = 0;
            // RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "publish time ns: %ld world sim time: %f ros time: %ld", now_ns / 1000, 1000 * world->SimTime().Double(), node_->get_clock()->now().nanoseconds() / 1000);
            boost::chrono::high_resolution_clock::time_point start_time = boost::chrono::high_resolution_clock::now();

            // 创建PointCloud消息用于发布PointCloud2类型消息
            sensor_msgs::msg::PointCloud cloud;
            cloud.header.stamp = stamp;
            cloud.header.frame_id = raySensor->Name();
            auto &clouds = cloud.points;

            // 遍历射线扫描点对，处理每个扫描点
            for (auto &pair : points_pair)
            {
                // 获取射线的距离和反射强度
                uint32_t out_idx = pp_livox.point_num;
                // if (out_idx >= 21000) {
                //     // 缓冲区已满，直接发布并开始新窗口
                //     RCLCPP_WARN_THROTTLE(rclcpp::get_logger("LivoxPointsPlugin"),
                //         *node_->get_clock(), 5000,
                //         "Accum buffer full (%u pts), force-publishing.", max_pts);
                //     break;
                // }
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);

                // 处理超出范围的数据
                if (range >= RangeMax())
                {
                    range = 0;  // 超出最大范围，设为0
                }
                else if (range <= RangeMin())
                {
                    range = 0;  // 小于最小范围，设为0
                }

                // 计算点云数据
                auto rotate_info = pair.second;
                {
                    // 根据旋转信息计算射线方向
                    ignition::math::Quaterniond ray;
                    ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith,
                                                       rotate_info.azimuth));
                    auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                    auto point = range * axis;

                    // // 填充CustomMsg点云消息
                    // livox_ros_driver2::msg::CustomPoint p;
                    // p.x = point.X();
                    // p.y = point.Y();
                    // p.z = point.Z();
                    // p.reflectivity = intensity;  // 反射强度
                    // offset_time：相对于本积累窗口起始的纳秒偏移
                                        // 计算时间戳偏移量（用于Livox的时间同步）
                    boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();
                    boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(
                                            end_time - start_time);
                    // const double dt_sec = rotate_info.time - t0_sec;
                    // const uint64_t frame_offset_ns =
                    //     (now_ns >= accum_start_ns_) ? (now_ns - accum_start_ns_) : 0ULL;
                    // const uint64_t intra_ns =
                    //     static_cast<uint64_t>(std::max(0.0, dt_sec * 1e9));
                    pp_livox.offset_times[out_idx] = elapsed_time.count();
                    // static_cast<uint32_t>(std::min<uint64_t>(frame_offset_ns + intra_ns, UINT32_MAX));

                    pp_livox.xs[out_idx] = static_cast<float>(point.X());
                    pp_livox.ys[out_idx] = static_cast<float>(point.Y());
                    pp_livox.zs[out_idx] = static_cast<float>(point.Z());
                    pp_livox.reflectivitys[out_idx] = static_cast<uint8_t>(
                    std::min(255.0, std::max(0.0, static_cast<double>(intensity))));
                    pp_livox.tags[out_idx] = 0;
                    pp_livox.lines[out_idx] = 0;
                    pp_livox.point_num = out_idx + 1;
                    // 填充PointCloud点云消息
                    clouds.emplace_back();
                    clouds.back().x = point.X();
                    clouds.back().y = point.Y();
                    clouds.back().z = point.Z();
                }
            }

            // 发布LaserScan消息（如果有连接）
            if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);

            // 设置点云数据数量并发布CustomMsg消息
            // pp_livox.point_num = count;
            custom_pub->publish(pp_livox);

            // 发布PointCloud2类型消息
            sensor_msgs::msg::PointCloud2 cloud2;
            sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
            cloud2.header = cloud.header;
            cloud2_pub->publish(cloud2);
        }
    }

    /**
     * @brief 初始化射线
     * 
     * 该函数根据CSV扫描模式数据初始化射线，用于每次扫描更新。
     * 它实现了非重复扫描模式，通过currStartIndex跟踪当前扫描位置。
     * 
     * @param points_pair 输出的射线索引和旋转信息对
     * @param ray_shape 射线形状对象
     */
    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose();
        
        // 每帧按固定步长在整个FOV上均匀抽样，samples/downsample 表示一帧
        auto ray_size = rays.size();
        points_pair.reserve(ray_size);
        const int64_t ray_count = static_cast<int64_t>(ray_size);
        const int64_t stride = std::max<int64_t>(1, maxPointSize / std::max<int64_t>(1, ray_count));

        for (int64_t r = 0; r < ray_count; ++r)
        {
            int64_t index = (r * stride) % maxPointSize;
            auto &rotate_info = aviaInfos[static_cast<size_t>(index)];

            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();

            rays[static_cast<size_t>(r)]->SetPoints(start_point, end_point);
            points_pair.emplace_back(static_cast<int>(r), rotate_info);
        }
    }

    /**
     * @brief 初始化激光扫描消息
     * 
     * 该函数初始化LaserScan消息的基本参数，包括角度范围、距离范围等。
     * 主要用于传统激光扫描格式的兼容性。
     * 
     * @param scan 激光扫描消息指针
     */
    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // 设置激光扫描的世界坐标位姿
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        
        // 设置水平角度参数
        scan->set_angle_min(AngleMin().Radian());        // 最小水平角度
        scan->set_angle_max(AngleMax().Radian());        // 最大水平角度
        scan->set_angle_step(AngleResolution());         // 水平角度分辨率
        scan->set_count(RangeCount());                   // 水平扫描点数

        // 设置垂直角度参数
        scan->set_vertical_angle_min(VerticalAngleMin().Radian());  // 最小垂直角度
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());  // 最大垂直角度
        scan->set_vertical_angle_step(VerticalAngleResolution());   // 垂直角度分辨率
        scan->set_vertical_count(VerticalRangeCount());             // 垂直扫描点数

        // 设置距离参数
        scan->set_range_min(RangeMin());  // 最小测量距离
        scan->set_range_max(RangeMax());  // 最大测量距离

        // 清空并初始化距离和强度数据
        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        // 初始化距离和强度数组
        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);      // 距离数据初始化为0
                scan->add_intensities(0); // 强度数据初始化为0
            }
        }
    }

    // ==================== 角度相关工具函数 ====================
    
    /**
     * @brief 获取最小水平角度
     * @return 最小水平角度（弧度）
     */
    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    /**
     * @brief 获取最大水平角度
     * @return 最大水平角度（弧度）
     */
    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    // ==================== 距离相关工具函数 ====================
    
    /**
     * @brief 获取最小测量距离（已弃用）
     * @return 最小测量距离（米）
     */
    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    /**
     * @brief 获取最小测量距离
     * @return 最小测量距离（米）
     */
    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    /**
     * @brief 获取最大测量距离（已弃用）
     * @return 最大测量距离（米）
     */
    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    /**
     * @brief 获取最大测量距离
     * @return 最大测量距离（米）
     */
    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    // ==================== 分辨率相关工具函数 ====================
    
    /**
     * @brief 获取水平角度分辨率（已弃用）
     * @return 水平角度分辨率（弧度）
     */
    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    /**
     * @brief 获取水平角度分辨率
     * @return 水平角度分辨率（弧度）
     */
    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    /**
     * @brief 获取距离分辨率（已弃用）
     * @return 距离分辨率（米）
     */
    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    /**
     * @brief 获取距离分辨率
     * @return 距离分辨率（米）
     */
    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    // ==================== 射线数量相关工具函数 ====================
    
    /**
     * @brief 获取射线数量（已弃用）
     * @return 射线数量
     */
    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    /**
     * @brief 获取射线数量
     * @return 射线数量
     */
    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    /**
     * @brief 获取水平扫描点数（已弃用）
     * @return 水平扫描点数
     */
    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    /**
     * @brief 获取水平扫描点数
     * @return 水平扫描点数
     */
    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    // ==================== 垂直扫描相关工具函数 ====================
    
    /**
     * @brief 获取垂直射线数量（已弃用）
     * @return 垂直射线数量
     */
    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    /**
     * @brief 获取垂直射线数量
     * @return 垂直射线数量
     */
    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    /**
     * @brief 获取垂直扫描点数（已弃用）
     * @return 垂直扫描点数
     */
    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    /**
     * @brief 获取垂直扫描点数
     * @return 垂直扫描点数
     */
    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    /**
     * @brief 获取最小垂直角度
     * @return 最小垂直角度（弧度）
     */
    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    /**
     * @brief 获取最大垂直角度
     * @return 最大垂直角度（弧度）
     */
    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    /**
     * @brief 获取垂直角度分辨率（已弃用）
     * @return 垂直角度分辨率（弧度）
     */
    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    /**
     * @brief 获取垂直角度分辨率
     * @return 垂直角度分辨率（弧度）
     */
    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}
