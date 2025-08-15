//
// Created by shibo zhao on 2020-09-27.
//
#pragma once
#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>


#include "utility.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include "super_odometry/utils/Twist.h"
#include "super_odometry/container/MapRingBuffer.h"
#include "super_odometry/config/parameter.h"
#include "super_odometry/tic_toc.h"
#include <glog/logging.h>
#include "super_odometry/sensor_data/imu/imu_data.h"


namespace super_odometry {

    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
    using FrameId = std::uint64_t;

    struct imuPreintegration_config{
        float imuAccNoise;
        float imuAccBiasN;
        float imuGyrNoise;
        float imuGyrBiasN;
        float imuGravity;
        float lidar_correction_noise;
        float smooth_factor;
        bool  use_imu_roll_pitch;
        SensorType sensor;

        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;
    };

    class imuPreintegration : public rclcpp::Node {
    public:

        imuPreintegration(const rclcpp::NodeOptions & options);

        static constexpr double delta_t = 0;
        static constexpr double imu_laser_timedelay= 0.8;

    public:
        void initInterface();

        bool readParameters();

        void laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

        void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw);

        void initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose);

        void process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose);

        bool build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);

        void repropagate_imuodometry(double currentCorrectionTime);

        bool failureDetection(const gtsam::Vector3 &velCur,
                         const gtsam::imuBias::ConstantBias &biasCur);

        void obtainCurrodometry(nav_msgs::msg::Odometry::SharedPtr &odomMsg, double &currentCorrectionTime,
                           gtsam::Pose3 &lidarPose,
                           int &currentResetId);

        void integrate_imumeasurement(double currentCorrectionTime);

        void reset_graph();

        void resetOptimization();

        void resetParams();

        bool handleIMUInitialization(const sensor_msgs::msg::Imu::SharedPtr&imu_raw, 
        sensor_msgs::msg::Imu& thisImu);


        void updateAndPublishPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu);

        void publishTransform(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu);

        void prepareOdometryMessage(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu, const gtsam::NavState &currentState);

        void publishOdometry(const sensor_msgs::msg::Imu& thisImu, const gtsam::NavState& currentState, nav_msgs::msg::Odometry &odometry);

        void publishTransformsAndPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu);

        void processTiming(const sensor_msgs::msg::Imu& thisImu);

        void initializeImu(const sensor_msgs::msg::Imu::SharedPtr& imu_raw);

        void correctLivoxGravity(sensor_msgs::msg::Imu& thisImu);


        sensor_msgs::msg::Imu
        imuConverter(const sensor_msgs::msg::Imu &imu_in);

        template<typename T>
        double secs(T msg) {
            return msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
        }


    private:

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
        

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubHealthStatus;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;


        rclcpp::CallbackGroup::SharedPtr cb_group_;
    public:
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::Vector noiseModelBetweenBias;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorOpt_;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorImu_;
        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
        gtsam::NavState prevStateOdom;
        gtsam::imuBias::ConstantBias prevBiasOdom;
        gtsam::ISAM2 optimizer;
        gtsam::NonlinearFactorGraph graphFactors;
        gtsam::Values graphValues;
        gtsam::Pose3 lidarodom_w_pre;
        gtsam::Pose3 lidarodom_w_cur;


    public:
        //Modify the extrinsic matrxi between laser and imu, laser and camera
        gtsam::Pose3 imu2cam;
        gtsam::Pose3 cam2Lidar;
        gtsam::Pose3 imu2Lidar;
        gtsam::Pose3 lidar2Imu;

    public:
        MapRingBuffer<Imu::Ptr> imuBuf;
        std::deque<sensor_msgs::msg::Imu> imuQueOpt;
        std::deque<sensor_msgs::msg::Imu> imuQueImu;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> lidarOdomBuf;
        std::mutex mBuf;
        Imu::Ptr imu_Init = std::make_shared<Imu>();

    public:
        bool systemInitialized = false;
        bool doneFirstOpt = false;
        bool health_status = true;
        bool imu_init_success = false;

       
        Eigen::Quaterniond firstImu;
        Eigen::Vector3d gyr_pre;

        double first_imu_time_stamp;
        double last_processed_lidar_time = -1;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;
        int key = 1;
        int imuPreintegrationResetId = 0;
        int frame_count = 0;

        enum IMU_STATE : uint8_t {
        FAIL=0,    //lose imu information 
        SUCCESS=1, //Obtain the good imu data 
        UNKNOW=2
        };  

        IMU_STATE RESULT;
        nav_msgs::msg::Odometry::SharedPtr cur_frame = nullptr;
        nav_msgs::msg::Odometry::SharedPtr last_frame = nullptr;
        imuPreintegration_config config_;

    // --- Lever-arm compensation related (dynamic, filter-based) ---
    bool enable_lever_arm_compensation_ = true;      // runtime param
    double gyro_lp_alpha_ = 0.1;                     // low-pass factor for gyro (0-1)
    double angular_acc_lp_alpha_ = 0.1;              // low-pass factor for angular acceleration
    double last_imu_converter_time_ = -1.0;          // last timestamp used for dt
    Eigen::Vector3d gyr_filtered_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_acc_filtered_ = Eigen::Vector3d::Zero();

    // --- Accelerometer bias online estimation (static detection) ---
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_static_sum_ = Eigen::Vector3d::Zero();
    int static_sample_count_ = 0;
    int static_sample_required_ = 200;          // number of consecutive static samples to update bias
    double static_gyr_thresh_ = 0.02;            // rad/s threshold for static
    double static_acc_dev_thresh_ = 0.15;        // m/s^2 acceptable deviation of |acc| from g
    bool acc_bias_initialized_ = false;          // flag once bias computed

    // --- Adaptive gravity / unit detection ---
    bool gravity_scale_determined_ = false;      // whether we identified if accel is in g or m/s^2
    double detection_gravity_scale_ = 0.0;       // 1.0 if in g units else config_.imuGravity
    double acc_norm_sum_ = 0.0;                  // accumulation for gravity detection
    int acc_norm_count_ = 0;                     // sample count for gravity detection
    int acc_norm_required_ = 120;                // ~0.6s @200Hz

    // --- dt anomaly tracking ---
    int dt_anomaly_count_ = 0;
    int dt_anomaly_log_count_ = 0;
    long long dt_total_count_ = 0;              // total processed dt samples (for ratio)
    bool lever_arm_auto_disabled_ = false;       // auto disable lever-arm compensation after repeated anomalies
    // --- Additional runtime state (Priority A+B) ---
    double last_raw_imu_time_ = -1.0;            // for early monotonic gating in imuHandler
    int dropped_converter_out_of_order_count_ = 0; // count of dropped raw IMU due to non-monotonic stamps
    bool gravity_scaled_applied_ = false;        // whether we have multiplied g-unit data into m/s^2
    bool acc_bias_injected_ = false;             // whether bias has been injected into GTSAM integrators

    // --- Path publishing control ---
    bool publish_path_without_subscriber_ = true; // always publish path regardless of subscription count
    double path_publish_interval_ = 0.1;          // seconds between path samples
    double last_path_time_ = -1.0;                // last time a path pose was appended
    uint32_t path_time_rewind_count_ = 0;         // count of time rewinds for path gating
    nav_msgs::msg::Path imuPath_;                 // accumulated path (shared instead of static inside function)
    // --- Path debug counters ---
    uint32_t path_skip_not_init_count_ = 0;       // skipped because IMU not initialized
    uint32_t path_skip_not_optimized_count_ = 0;  // skipped because first optimization not done
    uint32_t path_skip_interval_count_ = 0;       // skipped because interval not reached
    uint32_t path_skip_no_subscriber_count_ = 0;  // skipped because no subscriber and flag disabled

    };

}

#endif // IMUPREINTEGRATION_H
