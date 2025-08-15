//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/ImuPreintegration/imuPreintegration.h"


namespace super_odometry {

    imuPreintegration::imuPreintegration(const rclcpp::NodeOptions & options)
    : Node("imu_preintegration_node", options) {
    }
    
    void imuPreintegration::initInterface() {
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        // rclcpp::QoS imu_qos(10);
        // imu_qos.best_effort();  // Use BEST_EFFORT reliability
        // imu_qos.keep_last(10);  // Keep last 10 messages

        auto imu_qos = rclcpp::SensorDataQoS().keep_last(400);
        imu_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        RCLCPP_INFO(get_logger(), "\033[1;32mUsing imu_qos.reliability in %s: %s\033[0m",
            get_name(), (imu_qos.reliability() == rclcpp::ReliabilityPolicy::Reliable) ? "RELIABLE" : "BEST_EFFORT");

        RCLCPP_INFO(get_logger(), "\033[1;32mUsing imu_qos.reliability in imuPreintegration initInterface: %s\033[0m",
           (imu_qos.reliability() == rclcpp::ReliabilityPolicy::Reliable) ? "RELIABLE" : "BEST_EFFORT");

        if (!readGlobalparam(shared_from_this())) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read global parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readParameters()) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read local parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read calibration parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "[SuperOdometry::imuPreintegration] use_imu_rol_pitch:  %d", config_.use_imu_roll_pitch);

        //subscribe and publish relevant topics
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, imu_qos,
            std::bind(&imuPreintegration::imuHandler, this,
                        std::placeholders::_1), sub_options);
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 5,
            std::bind(&imuPreintegration::laserodometryHandler, this,
                        std::placeholders::_1), sub_options);

        pubImuOdometry = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/state_estimation", 10);
        pubHealthStatus = this->create_publisher<std_msgs::msg::Bool>(
            ProjectName+"/state_estimation_health", 1);
        pubImuPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/imuodom_path", 1);
        
        // set relevant parameter
        std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(config_.imuGravity);
        
        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) *
                                   pow(1e-4, 2); // error committed in integrating position from velocities

        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m

        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);                      // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6,
                                                             1e-1);                    // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, config_.lidar_correction_noise); // meter


        noiseModelBetweenBias = (gtsam::Vector(6)
                << config_.imuAccBiasN,
                config_.imuAccBiasN, config_.imuAccBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN)
                .finished();
        imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias); // setting up the IMU integration for IMU message
        imuIntegratorOpt_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias); // setting up the IMU integration for optimization

        //set extrinsic matrix for laser and imu
        if (PROVIDE_IMU_LASER_EXTRINSIC) {
            lidar2Imu = gtsam::Pose3(gtsam::Rot3(imu_laser_R), gtsam::Point3(imu_laser_T));
            imu2Lidar = lidar2Imu.inverse();
        } else {
            imu2cam = gtsam::Pose3(gtsam::Rot3(imu_camera_R), gtsam::Point3(imu_camera_T));
            cam2Lidar = gtsam::Pose3(gtsam::Rot3(cam_laser_R), gtsam::Point3(cam_laser_T));
            imu2Lidar = imu2cam.compose(cam2Lidar);
            lidar2Imu = imu2Lidar.inverse();
        }

        // ==== TEMP: Zero IMU-LiDAR translation to test lever-arm induced velocity drift ====
        // Set TEST_ZERO_IMU_LASER_T to false (or remove this block) after finishing diagnostics.
        constexpr bool TEST_ZERO_IMU_LASER_T = true; // <--- toggle here
        if (TEST_ZERO_IMU_LASER_T) {
            lidar2Imu = gtsam::Pose3(lidar2Imu.rotation(), gtsam::Point3(0, 0, 0));
            imu2Lidar = lidar2Imu.inverse();
            RCLCPP_WARN(this->get_logger(), "[imuPreintegration] TEST MODE: imu_laser translation forced to zero.");
        }
        // =====================================================================================
    }
    

    bool imuPreintegration::readParameters()
    {
        this->declare_parameter<float>("imu_preintegration_node.acc_n", 1e-3);
        this->declare_parameter<float>("imu_preintegration_node.acc_w", 1e-3);
        this->declare_parameter<float>("imu_preintegration_node.gyr_n", 1e-6);
        this->declare_parameter<float>("imu_preintegration_node.gyr_w", 1e-6);
        this->declare_parameter<float>("imu_preintegration_node.g_norm", 9.80511);
        this->declare_parameter<float>("imu_preintegration_node.lidar_correction_noise",0.01);
        this->declare_parameter<float>("imu_preintegration_node.smooth_factor",0.9);
        this->declare_parameter<bool>("imu_preintegration_node.use_imu_roll_pitch", false);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_x_limit", 1.0);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_y_limit", 1.0);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_z_limit", 1.0);
        // New parameters for adaptive handling
        this->declare_parameter<bool>("imu_preintegration_node.enable_lever_arm_compensation", true);
        this->declare_parameter<double>("imu_preintegration_node.static_gyr_thresh", 0.05);
        this->declare_parameter<double>("imu_preintegration_node.static_acc_dev_thresh", 0.05);
        this->declare_parameter<int>("imu_preintegration_node.static_sample_required", 300);
        this->declare_parameter<bool>("imu_preintegration_node.auto_detect_gravity_scale", true);
        this->declare_parameter<double>("imu_preintegration_node.gravity_detect_time", 0.6); // seconds
        this->declare_parameter<double>("imu_preintegration_node.angular_acc_lp_alpha", 0.05);
        this->declare_parameter<double>("imu_preintegration_node.gyro_lp_alpha", 0.05);
        // Path publishing parameters
        this->declare_parameter<bool>("imu_preintegration_node.publish_path_without_subscriber", true);
        this->declare_parameter<double>("imu_preintegration_node.path_publish_interval", 0.1);

        config_.imuAccNoise = this->get_parameter("imu_preintegration_node.acc_n").as_double();
        config_.imuAccBiasN = this->get_parameter("imu_preintegration_node.acc_w").as_double();
        config_.imuGyrNoise = this->get_parameter("imu_preintegration_node.gyr_n").as_double();
        config_.imuGyrBiasN = this->get_parameter("imu_preintegration_node.gyr_w").as_double();
        config_.imuGravity = this->get_parameter("imu_preintegration_node.g_norm").as_double();
        config_.lidar_correction_noise = this->get_parameter("imu_preintegration_node.lidar_correction_noise").as_double();
        config_.smooth_factor = this->get_parameter("imu_preintegration_node.smooth_factor").as_double();
        // config_.use_imu_roll_pitch = this->get_parameter("imu_preintegration_node.use_imu_roll_pitch").as_bool();
        config_.imu_acc_x_limit = this->get_parameter("imu_preintegration_node.imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = this->get_parameter("imu_preintegration_node.imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = this->get_parameter("imu_preintegration_node.imu_acc_z_limit").as_double();
        config_.use_imu_roll_pitch = USE_IMU_ROLL_PITCH;
        config_.imu_acc_x_limit = IMU_ACC_X_LIMIT;
        config_.imu_acc_y_limit = IMU_ACC_Y_LIMIT;
        config_.imu_acc_z_limit = IMU_ACC_Z_LIMIT;

        // Apply new parameters
        enable_lever_arm_compensation_ = this->get_parameter("imu_preintegration_node.enable_lever_arm_compensation").as_bool();
        static_gyr_thresh_ = this->get_parameter("imu_preintegration_node.static_gyr_thresh").as_double();
        static_acc_dev_thresh_ = this->get_parameter("imu_preintegration_node.static_acc_dev_thresh").as_double();
        static_sample_required_ = this->get_parameter("imu_preintegration_node.static_sample_required").as_int();
        bool auto_detect_gravity = this->get_parameter("imu_preintegration_node.auto_detect_gravity_scale").as_bool();
        double gravity_detect_time = this->get_parameter("imu_preintegration_node.gravity_detect_time").as_double();
        angular_acc_lp_alpha_ = this->get_parameter("imu_preintegration_node.angular_acc_lp_alpha").as_double();
        gyro_lp_alpha_ = this->get_parameter("imu_preintegration_node.gyro_lp_alpha").as_double();
        publish_path_without_subscriber_ = this->get_parameter("imu_preintegration_node.publish_path_without_subscriber").as_bool();
        path_publish_interval_ = this->get_parameter("imu_preintegration_node.path_publish_interval").as_double();
        if (path_publish_interval_ < 0.02) path_publish_interval_ = 0.02; // clamp to reasonable lower bound
        acc_norm_required_ = std::max(10, (int)(gravity_detect_time * 200.0)); // assume 200Hz

        RCLCPP_INFO(this->get_logger(), "[imuPreintegration] Params: lever_arm=%d static_gyr=%.4f static_acc_dev=%.4f static_req=%d auto_g=%d", 
            enable_lever_arm_compensation_ ? 1:0, static_gyr_thresh_, static_acc_dev_thresh_, static_sample_required_, auto_detect_gravity ? 1:0);

        if (SENSOR == "livox") {
            config_.sensor = SensorType::LIVOX;
        } else if (SENSOR == "velodyne") {
            config_.sensor = SensorType::VELODYNE;
        } else if (SENSOR == "ouster") {
            config_.sensor = SensorType::OUSTER;
        }   

        return true;

    }

    void imuPreintegration::resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void imuPreintegration::resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }


    void imuPreintegration::reset_graph() {

        // get updated noise before reset
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise, updatedVelNoise, updatedBiasNoise;

        try
        {
            updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to reset graph. No marginal covariance key");
            updatedPoseNoise = priorPoseNoise;
            updatedVelNoise = priorVelNoise;
            updatedBiasNoise = priorBiasNoise;
        }

        // reset graph
        resetOptimization();
        // add pose
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        key = 1;
    }

    void imuPreintegration::initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose) {
        resetOptimization();

        while (!imuQueOpt.empty()) {
            if (secs(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
                lastImuT_opt = secs(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        prevPose_ = lidarPose.compose(lidar2Imu);

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   priorPoseNoise);
        graphFactors.add(priorPose);

        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    priorVelNoise);
        graphFactors.add(priorVel);

        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);

        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);

        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        key = 1;
        systemInitialized = true;
    }

    void imuPreintegration::integrate_imumeasurement(double currentCorrectionTime) {
        // 1. integrate imu data and optimize

        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = secs(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_opt);
                lastImuT_opt = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;
               
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                imuQueOpt.pop_front();
            }
            else
                break;
        }

    }


    bool imuPreintegration::build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {


        // add laser pose prior factor

        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);

        // insert predicted values
        gtsam::NavState propState_ =
                imuIntegratorOpt_->predict(prevState_, prevBias_);
        auto diff  = curPose.translation() - propState_.pose().translation();

        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,
                                                     correctionNoise);
        graphFactors.add(pose_factor);
                // add imu factor to graph
        
        const gtsam::PreintegratedImuMeasurements &preint_imu =
                dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
                        *imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key),
                                    B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(
                        sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        
  
        // optimize
        bool systemSolvedSuccessfully = false;
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            systemSolvedSuccessfully = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException &) {
            systemSolvedSuccessfully = false;
            RCLCPP_WARN(this->get_logger(), "Update failed due to underconstrained call to isam2 in imuPreintegration");
        }

        graphFactors.resize(0);
        graphValues.clear();

        if (systemSolvedSuccessfully) {

            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        }        
        return systemSolvedSuccessfully;
    }

    void imuPreintegration::repropagate_imuodometry(double currentCorrectionTime) {
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        double lastImuQT = -1;
        while (!imuQueImu.empty() && secs(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
            lastImuQT = secs(&imuQueImu.front());
            imuQueImu.pop_front();
        }

        if (!imuQueImu.empty()) {
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            for (int i = 0; i < (int)imuQueImu.size(); ++i) {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = secs(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 200.0) :(imuTime - lastImuQT);
                lastImuQT = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;

                imuIntegratorImu_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), 
                    dt);
                lastImuQT = imuTime;
            }
        }
    }

    void imuPreintegration::process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose) {

        // reset graph for speed
        if (key > 100) {
            reset_graph();
        }

        // 1. integrate_imumeasurement
        integrate_imumeasurement(currentCorrectionTime);

        lidarodom_w_cur = relativePose;

        // 2. build_graph
        bool successOptimization = build_graph(lidarodom_w_cur, currentCorrectionTime);
        
        // 3. check optimization
        if (failureDetection(prevVel_, prevBias_) || !successOptimization) {
            RCLCPP_WARN(
            this->get_logger(),
            "[imuPreintegration::process_imu_odometry] failureDetected "
            "(successOptimization=%s vel=%.3f biasAccNorm=%.3f biasGyrNorm=%.3f) -> resetting",
            successOptimization ? "true" : "false",
            prevVel_.norm(),
            prevBias_.accelerometer().norm(),
            prevBias_.gyroscope().norm());
            resetParams();
            return;
        }

        // 4. reprogate_imuodometry
        repropagate_imuodometry(currentCorrectionTime);
        ++key;

        doneFirstOpt = true;
    }

    bool imuPreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            RCLCPP_WARN(this->get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());

        if (ba.norm() > 2.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuPreintegration::laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mBuf);

        cur_frame = odomMsg;
        double lidarOdomTime = secs(odomMsg);

        if (imuQueOpt.empty()) {
            static int empty_warn = 0;
            if (empty_warn < 20 || empty_warn % 100 == 0) {
                RCLCPP_WARN(this->get_logger(), "[imuPreintegration::laserodometryHandler] imuQueOpt empty at lidar t=%.6f (can't init system yet)", lidarOdomTime);
            }
            ++empty_warn;
            return;
        }

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                                              gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false) {
            initial_system(lidarOdomTime, lidarPose);
            RCLCPP_INFO(this->get_logger(), "[imuPreintegration::laserodometryHandler] System initialized (key=%d imuQueOpt=%zu)", key, imuQueOpt.size());
            return;
        }

        TicToc Optimization_time;
        //1. process imu odometry
        process_imu_odometry(lidarOdomTime, lidarPose);

        // 2. safe landing process
        if (imuQueImu.empty()) {
            RESULT = IMU_STATE::FAIL;
            health_status = false;
            // publish health + return (or handle gracefully)
            std_msgs::msg::Bool msg; msg.data = health_status; pubHealthStatus->publish(msg);
            last_frame = cur_frame;
            last_processed_lidar_time = lidarOdomTime;
            RCLCPP_WARN(this->get_logger(), "[imuPreintegration::laserodometryHandler] IMU queue empty - cannot proceed with odometry processing");
            return;
        }
        double latest_imu_time = secs(&imuQueImu.back());

        if (lidarOdomTime - latest_imu_time < imu_laser_timedelay) {
            RESULT = IMU_STATE::SUCCESS;
            health_status = true;
          
            if((int)odomMsg->pose.covariance[0] == 1) {
                RESULT = IMU_STATE::FAIL;
            }
         
        } else {
            health_status = false;
            if (cur_frame != nullptr && last_frame != nullptr) {
                Eigen::Vector3d velocity_curr;
                velocity_curr.x() =
                    (cur_frame->pose.pose.position.x - last_frame->pose.pose.position.x) /
                    (secs(cur_frame) - secs(last_frame));
                velocity_curr.y() = (cur_frame->pose.pose.position.y - last_frame->pose.pose.position.y) /
                                    (secs(cur_frame) - secs(last_frame));
                velocity_curr.z() = (cur_frame->pose.pose.position.z - last_frame->pose.pose.position.z) /
                                    (secs(cur_frame) - secs(last_frame));

                RCLCPP_INFO(this->get_logger(), "LOOSE CONNECTION WITH IMU DRIVER, PLEASE CHECK HARDWARE!!");
                RESULT = IMU_STATE::FAIL;
                health_status = false;
                nav_msgs::msg::Odometry odometry;

                std_msgs::msg::Bool health_status_msg;
                health_status_msg.data = health_status;
                pubHealthStatus->publish(health_status_msg);
            }
        }

        last_frame = cur_frame;
        last_processed_lidar_time = lidarOdomTime;
    }
    //TODO: need to consider the extrinsic matrix of imu and lidar
    sensor_msgs::msg::Imu imuPreintegration::imuConverter(const sensor_msgs::msg::Imu &imu_in) {
        sensor_msgs::msg::Imu imu_out = imu_in;
        
        Eigen::Matrix3d imu_laser_R_Gravity;
        imu_laser_R_Gravity=imu_Init->imu_laser_R_Gravity;

        Eigen::Vector3d rpy;
        rpy=imu_Init->rotationMatrixToRPY(imu_laser_R_Gravity);
 
        // rotate gyro and acc only when sensor is livox
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                imu_in.angular_velocity.z);
        gyr=imu_laser_R_Gravity*gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();


        // rotate acceleration (body frame)
        Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                            imu_in.linear_acceleration.y,
                            imu_in.linear_acceleration.z);
        acc = imu_laser_R_Gravity * acc;
        
        // // Early timestamp monotonic gating (raw) ---
        // double imu_raw_time = imu_in.header.stamp.sec + imu_in.header.stamp.nanosec * 1e-9;

        // if (last_raw_imu_time_ >= 0 && imu_raw_time <= last_raw_imu_time_) {
        //     ++dropped_converter_out_of_order_count_;
        //     if (dropped_converter_out_of_order_count_ <= 20 || dropped_converter_out_of_order_count_ % 5 == 0) {
        //         double rewind = last_raw_imu_time_ - imu_raw_time;
        //         RCLCPP_WARN(this->get_logger(),
        //             "[imuPreintegration::imuConverter] drop non-monotonic raw t=%.9f last=%.9f rewind=%.6f count=%d",
        //             imu_raw_time, last_raw_imu_time_, rewind, dropped_converter_out_of_order_count_);
        //     }
        //     if (imu_out.linear_acceleration_covariance[0] == 0.0) {
        //         imu_out.linear_acceleration_covariance[0] = -1.0; // sentinel
        //     }
        //     return imu_out; // early exit
        // }

        // last_raw_imu_time_ = imu_raw_time;

        // ===========================================================================================================================
        // ===========================================================================================================================
        // // TODO: REMOVE debug info
        // // RCLCPP_INFO(this->get_logger(), "[imuPreintegration::imuConverter] last_raw_imu_time_ = %.9f", last_raw_imu_time_);
        // // Dynamic lever-arm compensation with actual dt and filtering (optional)
        // double t_now = imu_raw_time;
        // double dt = (last_imu_converter_time_ < 0) ? 0.0 : (t_now - last_imu_converter_time_);

        // // TODO: REMOVE debug info
        // // RCLCPP_INFO(this->get_logger(), "[imuPreintegration::imuConverter] last_imu_converter_time_ = %.9f", last_imu_converter_time_);

        // last_imu_converter_time_ = t_now;
        // bool dt_valid = (dt > 0 && dt < 0.02); // expect ~200Hz
        
        // ===========================================================================================================================
            // TODO: REMOVE debug info
        // RCLCPP_INFO(this->get_logger(),
        //     "[imuPreintegration::imuConverter] t=%.9f dt=%.6f dt_valid=%d last_raw=%.9f last_conv=%.9f enable_lever=%d auto_disabled=%d gyr=(%.4f %.4f %.4f)|%.4f acc_rot=(%.4f %.4f %.4f)|%.4f imuQueOpt=%zu imuQueImu=%zu key=%d systemInit=%d doneFirstOpt=%d poses=%zu",
        //     t_now, dt, dt_valid?1:0, last_raw_imu_time_, last_imu_converter_time_,
        //     enable_lever_arm_compensation_?1:0, lever_arm_auto_disabled_?1:0,
        //     gyr.x(), gyr.y(), gyr.z(), gyr.norm(),
        //     acc.x(), acc.y(), acc.z(), acc.norm(),
        //     imuQueOpt.size(), imuQueImu.size(), key, systemInitialized?1:0, doneFirstOpt?1:0, imuPath_.poses.size());

        // if (!dt_valid && dt != 0.0) {
        //     dt_anomaly_count_++;
        //     if (++dt_anomaly_log_count_ % 1 == 0) {
        //         double ratio = (dt_total_count_ > 0) ? (double)dt_anomaly_count_ / (double)dt_total_count_ : 0.0;
        //         RCLCPP_WARN(this->get_logger(), "[imuPreintegration] DT anomaly count=%d total=%lld ratio=%.3f last_dt=%.5f", dt_anomaly_count_, dt_total_count_, ratio, dt);
        //     }
        //     if (dt_anomaly_count_ > 1 && enable_lever_arm_compensation_ && !lever_arm_auto_disabled_) {
        //         lever_arm_auto_disabled_ = true;
        //         enable_lever_arm_compensation_ = false;
        //         RCLCPP_WARN(this->get_logger(), "[imuPreintegration] Auto-disabled lever-arm compensation due to repeated dt anomalies");
        //     }
        // }

        // // Low-pass filter measurements to suppress high frequency noise
        // if (enable_lever_arm_compensation_) {
        //     if (dt_valid) {
        //         // First-order IIR filter
        //         gyr_filtered_ = (1.0 - gyro_lp_alpha_) * gyr_filtered_ + gyro_lp_alpha_ * gyr;
        //         Eigen::Vector3d angular_acc = (gyr - gyr_pre) / dt; // raw angular acceleration
        //         angular_acc_filtered_ = (1.0 - angular_acc_lp_alpha_) * angular_acc_filtered_ + angular_acc_lp_alpha_ * angular_acc;
        //         // Compensation terms: alpha x (-T) + w x (w x -T)
        //         // Use filtered angular acceleration for stability
        //         acc += angular_acc_filtered_.cross(-imu_laser_T) + gyr_filtered_.cross(gyr_filtered_.cross(-imu_laser_T));
                
        //         RCLCPP_INFO(this->get_logger(), "[imuPreintegration::imuConverter] acc=(%.4f %.4f %.4f)|%.4f gyr=(%.4f %.4f %.4f)|%.4f",
        //             acc.x(), acc.y(), acc.z(), acc.norm(),
        //             gyr.x(), gyr.y(), gyr.z(), gyr.norm());
        //     }
        //     // else: skip compensation if dt invalid
        // }

        // Online gravity detection, static detection & accelerometer bias estimation moved behind IMU init & data sanity gating
        // bool maybe_static = false; // for logging below
        // if (imu_init_success) {
        //     // 1) Gravity scale detection (avoid early zero-average): accumulate only valid norms
        //     if (!gravity_scale_determined_) {
        //         double acc_norm = acc.norm();
        //         // Ignore unrealistically small norms (e.g. zeroed before correction), and very large spikes
        //         if (acc_norm > 0.5 && acc_norm < 30.0) {
        //             acc_norm_sum_ += acc_norm;
        //             ++acc_norm_count_;
        //         }
        //         if (acc_norm_count_ >= acc_norm_required_) {
        //             double avg = acc_norm_sum_ / acc_norm_count_;
        //             // After Livox correction acceleration should be in m/s^2 already
        //             if (avg > 7.0 && avg < 12.5) {
        //                 detection_gravity_scale_ = config_.imuGravity; // treat as m/s^2
        //                 RCLCPP_INFO(this->get_logger(), "[imuPreintegration] Gravity scale fixed (m/s^2) avg=%.3f using g=%.3f", avg, detection_gravity_scale_);
        //             } else if (avg > 0.8 && avg < 1.2) {
        //                 // Data appears in g-units; convert to m/s^2 once
        //                 detection_gravity_scale_ = config_.imuGravity; // we will scale measurements
        //                 gravity_scaled_applied_ = true;
        //                 RCLCPP_WARN(this->get_logger(), "[imuPreintegration] Gravity detected in g-units avg=%.3f -> scaling all future acc by %.3f", avg, config_.imuGravity);
        //             } else {
        //                 // Out-of-expected range; keep collecting (extend requirement)
        //                 acc_norm_required_ += 50;
        //                 RCLCPP_WARN(this->get_logger(), "[imuPreintegration] Gravity detection deferred avg=%.3f collected=%d new_required=%d", avg, acc_norm_count_, acc_norm_required_);
        //             }
        //             if (acc_norm_count_ >= acc_norm_required_) {
        //                 gravity_scale_determined_ = true;
        //                 RCLCPP_INFO(this->get_logger(), "[imuPreintegration] Gravity detection finalized mode=%s avg_norm=%.3f", gravity_scaled_applied_?"scaled(ms2)":"native(ms2)", (acc_norm_sum_/acc_norm_count_));
        //             }
        //         }
        //     }
        //     double g_norm = detection_gravity_scale_ > 0 ? detection_gravity_scale_ : config_.imuGravity;

        //     // If scaling decided (original data in g), scale current acc (and implicitly future because detection complete)
        //     if (gravity_scale_determined_ && gravity_scaled_applied_) {
        //         acc *= (config_.imuGravity / g_norm); // g_norm will equal config_.imuGravity after assignment; safe even if repeated
        //         g_norm = config_.imuGravity;
        //     }

        //     // 2) Static detection & bias estimation only after gravity scale determined and with valid dt
        //     bool can_estimate_bias = gravity_scale_determined_ && dt_valid;
        //     double gyr_mag = gyr.norm();
        //     double acc_mag = acc.norm();
        //     double acc_dev = std::fabs(acc_mag - g_norm);
        //     if (can_estimate_bias && gyr_mag < static_gyr_thresh_ && acc_dev < static_acc_dev_thresh_) {
        //         maybe_static = true;
        //         acc_static_sum_ += acc;
        //         ++static_sample_count_;
        //     } else if (can_estimate_bias) {
        //         // Instead of a hard reset, decay the counter to allow brief disturbances
        //         static_sample_count_ = std::max(0, static_sample_count_ - 5);
        //         if (static_sample_count_ == 0) acc_static_sum_.setZero();
        //     }

        //     if (can_estimate_bias && !acc_bias_initialized_ && static_sample_count_ >= static_sample_required_) {
        //         Eigen::Vector3d acc_mean = acc_static_sum_ / static_sample_count_;
        //         Eigen::Vector3d gravity_dir = acc_mean.normalized();
        //         Eigen::Vector3d expected_acc = gravity_dir * g_norm;
        //         acc_bias_ = acc_mean - expected_acc;
        //         acc_bias_initialized_ = true;
        //         RCLCPP_INFO(this->get_logger(), "[imuPreintegration] Acc bias initialized: %.5f %.5f %.5f (samples=%d)",
        //                     acc_bias_.x(), acc_bias_.y(), acc_bias_.z(), static_sample_count_);
        //     }

        //     if (acc_bias_initialized_) {
        //         acc -= acc_bias_;
        //         // Inject into preintegration biases once, then future GTSAM integration aware of bias
        //         if (!acc_bias_injected_) {
        //             // Current prevBias_ only has zero biases; create new bias with accelerometer part set
        //             gtsam::Vector3 new_acc_bias(acc_bias_.x(), acc_bias_.y(), acc_bias_.z());
        //             gtsam::Vector3 gyr_bias = prevBias_.gyroscope();
        //             prevBias_ = gtsam::imuBias::ConstantBias(new_acc_bias, gyr_bias);
        //             prevBiasOdom = prevBias_;
        //             imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        //             imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        //             acc_bias_injected_ = true;
        //             RCLCPP_INFO(this->get_logger(), "[imuPreintegration] Injected accelerometer bias into preintegrators (%.5f %.5f %.5f)", acc_bias_.x(), acc_bias_.y(), acc_bias_.z());
        //         }
        //     }

        //     // 3) Residual logging ~1Hz
        //     static int residual_log_count = 0;
        //     if (++residual_log_count % 200 == 0) {
        //         Eigen::Vector3d acc_residual = acc;
        //         double horiz = std::sqrt(acc_residual.x()*acc_residual.x() + acc_residual.y()*acc_residual.y());
        //         double ratio = (dt_total_count_ > 0) ? (double)dt_anomaly_count_ / (double)dt_total_count_ : 0.0;
        //         RCLCPP_INFO(this->get_logger(),
        //                     "[imuPreintegration] acc_residual |a|=%.4f horiz=%.4f gyr=%.4f bias=(%.4f %.4f %.4f) static=%d dt=%.4f scaled=%d dropped_raw=%d anomalies=%d total=%lld ratio=%.3f",
        //                     acc_residual.norm(), horiz, gyr_mag,
        //                     acc_bias_.x(), acc_bias_.y(), acc_bias_.z(), maybe_static ? 1 : 0, dt,
        //                     gravity_scaled_applied_?1:0, dropped_converter_out_of_order_count_, dt_anomaly_count_, dt_total_count_, ratio);
        //     }
        // }

        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();


        // rotate roll pitch yaw
        Eigen::Quaterniond q(imu_in.orientation.w, imu_in.orientation.x,
                             imu_in.orientation.y, imu_in.orientation.z);

        q.normalize();


        Eigen::Quaterniond q_extrinsic;
        q_extrinsic=Eigen::Quaterniond(imu_laser_R_Gravity);

        Eigen::Quaterniond q_new;

        q_new=q*q_extrinsic;

        q_new.normalize();

        imu_out.orientation.x = q_new.x();
        imu_out.orientation.y = q_new.y();
        imu_out.orientation.z = q_new.z();
        imu_out.orientation.w = q_new.w();

        gyr_pre = gyr;

        return imu_out;
    }


   void imuPreintegration::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {
    std::lock_guard<std::mutex> lock(mBuf);
    
    // 1. Pre-process IMU data
    sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
    // Removed crash-causing assert: converter can legitimately return unchanged acceleration for dropped/non-monotonic samples or identity rotation.
    // assert(imu_raw->linear_acceleration.x != thisImu.linear_acceleration.x);

    bool acc_unchanged = (imu_raw->linear_acceleration.x == thisImu.linear_acceleration.x &&
                          imu_raw->linear_acceleration.y == thisImu.linear_acceleration.y &&
                          imu_raw->linear_acceleration.z == thisImu.linear_acceleration.z);
    if (acc_unchanged) {
        static int unchanged_cnt = 0;
        // Distinguish reasons: non-monotonic drop sentinel vs identity rotation vs pre-init
        bool dropped_non_monotonic = (thisImu.linear_acceleration_covariance[0] == -1.0);
        if (unchanged_cnt < 50 || unchanged_cnt % 200 == 0) {
            RCLCPP_WARN(this->get_logger(),
                "[imuPreintegration::imuHandler] Acc unchanged (cnt=%d, dropped=%d, imu_init=%d) t=%.9f raw_t=%.9f extrinsic_is_identity=%d",
                unchanged_cnt, dropped_non_monotonic?1:0, imu_init_success?1:0, secs(&thisImu), secs(imu_raw.get()),
                (imu_Init && imu_laser_R.isIdentity()) ? 1:0);
        }
        ++unchanged_cnt;
        if (dropped_non_monotonic) {
            // Skip further processing for dropped samples early to avoid polluting timing statistics
            return;
        }
    }

    // 2. Handle IMU initialization for LIVOX sensor
    if (!handleIMUInitialization(imu_raw, thisImu)) {
        return;
    }

    // 3. Process timing and queue management
    processTiming(thisImu);

    // 4. Early return if first optimization not done
    if (!doneFirstOpt) {
        static int waiting_log = 0;
        if (waiting_log < 50 || waiting_log % 500 == 0) {
            RCLCPP_INFO(this->get_logger(), "[imuPreintegration::imuHandler] Waiting for first optimization: systemInitialized=%d imuQueOpt=%zu imuQueImu=%zu path_poses=%zu", systemInitialized?1:0, imuQueOpt.size(), imuQueImu.size(), imuPath_.poses.size());
        }
        ++waiting_log;
        return;
    }

    // 5. Prepare and publish odometry
    gtsam::NavState currentState =imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
    nav_msgs::msg::Odometry odometry;
    publishOdometry(thisImu, currentState, odometry);
    publishTransformsAndPath(odometry,  thisImu);     
   
   }

   bool imuPreintegration::handleIMUInitialization(const sensor_msgs::msg::Imu::SharedPtr&imu_raw, 
   sensor_msgs::msg::Imu& thisImu) {   

    if (!imu_init_success) {
        initializeImu(imu_raw);
    }

    if (imu_init_success && config_.sensor == SensorType::LIVOX) {
        correctLivoxGravity(thisImu);
    }
    
    return imu_init_success; 

   }

   void imuPreintegration::initializeImu(const sensor_msgs::msg::Imu::SharedPtr& imu_raw) {
    Imu::Ptr imudata = std::make_shared<Imu>();
    imudata->time = imu_raw->header.stamp.sec + imu_raw->header.stamp.nanosec * 1e-9;
    imudata->acc = Eigen::Vector3d(imu_raw->linear_acceleration.x,
                                  imu_raw->linear_acceleration.y,
                                  imu_raw->linear_acceleration.z);
    imudata->gyr = Eigen::Vector3d(imu_raw->angular_velocity.x,
                                  imu_raw->angular_velocity.y,
                                  imu_raw->angular_velocity.z);
    imudata->q_w_i = Eigen::Quaterniond(imu_raw->orientation.w,
                                       imu_raw->orientation.x,
                                       imu_raw->orientation.y,
                                       imu_raw->orientation.z);

    imuBuf.addMeas(imudata, imudata->time);

    double first_imu_time = 0.0;
    imuBuf.getFirstTime(first_imu_time);

    if (imudata->time - first_imu_time > 1.0) {
        imu_Init->imuInit(imuBuf);
        imu_init_success = true;
        imuBuf.clean(imudata->time);
      std::cout<<"IMU Initialization Process Finish! "<<std::endl;
    }
}


void imuPreintegration::correctLivoxGravity(sensor_msgs::msg::Imu& thisImu) {
    const double gravity = 9.8105;
    Eigen::Vector3d acc(thisImu.linear_acceleration.x,
                       thisImu.linear_acceleration.y,
                       thisImu.linear_acceleration.z);
    acc = acc * gravity / imu_Init->acc_mean.norm();
    thisImu.linear_acceleration.x = acc.x();
    thisImu.linear_acceleration.y = acc.y();
    thisImu.linear_acceleration.z = acc.z();
}


void imuPreintegration::processTiming(const sensor_msgs::msg::Imu& thisImu) {
    const double imuTime = secs(&thisImu);
    dt_total_count_++;  // total samples seen (including those dropped below)

    // First valid sample: initialize timestamp and bail early.
    if (lastImuT_imu < 0.0) {
        RCLCPP_INFO(this->get_logger(),
                    "[imuPreintegration] First IMU sample received t=%.9f",
                    imuTime);
        lastImuT_imu = imuTime;
        return;
    }

    // Compute raw delta-time relative to last accepted IMU timestamp.
    const double raw_dt = imuTime - lastImuT_imu;

    // Reject non-monotonic (<= 0) samples outright.
    if (raw_dt <= 0.0) {
        dt_anomaly_count_++;
        const double ratio = static_cast<double>(dt_anomaly_count_) /
                             std::max(1.0, static_cast<double>(dt_total_count_));
        RCLCPP_WARN(this->get_logger(),
                    "[imuPreintegration] Dropped non-monotonic IMU sample "
                    "(dt=%.9f imu=%.9f last=%.9f count=%d total=%lld ratio=%.3f)",
                    raw_dt, imuTime, lastImuT_imu,
                    dt_anomaly_count_, dt_total_count_, ratio);
        return;  // do not advance lastImuT_imu; do not enqueue
    }

    // Extremely small positive dt: ignore (likely clock jitter / duplicate)
    if (raw_dt < 1e-5) {
        dt_anomaly_count_++;
        const double ratio = static_cast<double>(dt_anomaly_count_) /
                             std::max(1.0, static_cast<double>(dt_total_count_));
        RCLCPP_WARN(this->get_logger(),
                    "[imuPreintegration] Extremely small IMU dt=%.9f "
                    "(imu=%.9f last=%.9f) count=%d total=%lld ratio=%.3f",
                    raw_dt, imuTime, lastImuT_imu,
                    dt_anomaly_count_, dt_total_count_, ratio);
        return;  // do not advance; do not enqueue
    }

    // Sanitize dt for downstream integration.
    double used_dt = raw_dt;

    // Very large gap: accept sample but cap dt for integration safety.
    if (raw_dt > 0.1) {
        dt_anomaly_count_++;
        const double ratio = static_cast<double>(dt_anomaly_count_) /
                             std::max(1.0, static_cast<double>(dt_total_count_));
        RCLCPP_WARN(this->get_logger(),
                    "[imuPreintegration] Large IMU gap dt=%.5f "
                    "count=%d total=%lld ratio=%.3f (capping to 0.01)",
                    raw_dt, dt_anomaly_count_, dt_total_count_, ratio);
        used_dt = 0.01;  // conservative cap to reduce velocity jump
    }
    // Keep within safe integration bounds (your chosen window).
    else if (raw_dt < 0.001 || raw_dt > 0.05) {
        RCLCPP_WARN(this->get_logger(), 
                   "[imuPreintegration] IMU dt out of safe bounds (%.5f), clamping to 0.005",
                   raw_dt);
        used_dt = 0.005;
    }

    // At this point the sample is accepted: advance time and enqueue.
    lastImuT_imu = imuTime;

    // Enqueue IMU for consumers. (Queues carry the message; dt is tracked separately.)
    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);
    static uint64_t timing_log = 0;
    // TODO: REMOVE DEBUG OUTPUT
    if (timing_log % 500 == 0) {
        RCLCPP_INFO(this->get_logger(),
            "[imuPreintegration::processTiming]  imuTime=%.9f raw_dt=%.6f used_dt=%.6f lastImuT_imu=%.9f dt_anomalies=%d total=%lld imuQueOpt=%zu imuQueImu=%zu systemInit=%d doneFirstOpt=%d key=%d",
            imuTime, raw_dt, used_dt, lastImuT_imu,
            dt_anomaly_count_, dt_total_count_, imuQueOpt.size(), imuQueImu.size(),
            systemInitialized?1:0, doneFirstOpt?1:0, key);
    }
    ++timing_log;
}



void imuPreintegration::publishOdometry(
    const sensor_msgs::msg::Imu& thisImu,
    const gtsam::NavState& currentState, nav_msgs::msg::Odometry &odometry) {
    
    prepareOdometryMessage(odometry, thisImu, currentState);
    
    if (frame_count++ % 4 == 0) {
        pubImuOdometry->publish(odometry);
    }

    // Publish health status
    std_msgs::msg::Bool health_status_msg;
    health_status_msg.data = health_status;
    pubHealthStatus->publish(health_status_msg);
}



void imuPreintegration::publishTransformsAndPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu) { 
    publishTransform(odometry, thisImu);
    updateAndPublishPath(odometry,thisImu);
}

void imuPreintegration::publishTransform(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu){
    
    tf2_ros::TransformBroadcaster br(this);
    geometry_msgs::msg::TransformStamped transform_stamped_;
    tf2::Transform transform;
    transform_stamped_.header.stamp  = thisImu.header.stamp;
    transform_stamped_.header.frame_id = WORLD_FRAME;
    transform_stamped_.child_frame_id = SENSOR_FRAME;
    
    tf2::Quaternion q;
    transform.setOrigin(tf2::Vector3(odometry.pose.pose.position.x, 
    odometry.pose.pose.position.y, odometry.pose.pose.position.z));

    q.setW(odometry.pose.pose.orientation.w);
    q.setX(odometry.pose.pose.orientation.x);
    q.setY(odometry.pose.pose.orientation.y);
    q.setZ(odometry.pose.pose.orientation.z);
    transform.setRotation(q);
    transform_stamped_.transform = tf2::toMsg(transform);
    if(frame_count%4==0)
        br.sendTransform(transform_stamped_);
}

    void imuPreintegration::updateAndPublishPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu){ 
    static nav_msgs::msg::Path imuPath;
    
    static double last_path_time = -1;
        double curimuTime = secs(&thisImu);
        // Monotonic gating for path (avoid time going backwards)
        if (last_path_time_ >= 0 && curimuTime < last_path_time_) {
            ++path_time_rewind_count_;
            RCLCPP_WARN(this->get_logger(), "[imuPreintegration] Path time rewind (cur=%.9f last=%.9f)", curimuTime, last_path_time_);
            return;
        }
        static int path_state_log = 0;
        if (path_state_log < 100 || path_state_log % 500 == 0) {
            RCLCPP_INFO(this->get_logger(), "[imuPreintegration::updateAndPublishPath] Attempt path append (poses=%zu) doneFirstOpt=%d systemInit=%d subscribers=%zu", imuPath_.poses.size(), doneFirstOpt?1:0, systemInitialized?1:0, pubImuPath->get_subscription_count());
        }
        ++path_state_log;

        // Interval satisfied OR first time (last_path_time_ < 0)
        last_path_time_ = curimuTime;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = thisImu.header.stamp;
        pose_stamped.header.frame_id = WORLD_FRAME;
        pose_stamped.pose = odometry.pose.pose;
        imuPath_.poses.push_back(pose_stamped);

        bool has_subscriber = pubImuPath->get_subscription_count() != 0;
        if (!(publish_path_without_subscriber_ || has_subscriber)) {
            if (++path_skip_no_subscriber_count_ % 1 == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[imuPreintegration] Path built but not published (no subscriber) count=%u", 
                    path_skip_no_subscriber_count_);
            }
            // return;
        }

        // while (!imuPath.poses.empty() &&
        //         abs(secs(&imuPath.poses.front()) -
        //             secs(&imuPath.poses.back())) > 3.0)
        //     imuPath.poses.erase(imuPath.poses.begin());
        imuPath_.header.stamp = thisImu.header.stamp;
        imuPath_.header.frame_id = WORLD_FRAME;
        pubImuPath->publish(imuPath_);
        // if (imuPath_.poses.size() % 100 == 0) {
        RCLCPP_DEBUG(this->get_logger(), "[imuPreintegration] Path published poses=%zu", imuPath_.poses.size());
        // }
    }

void imuPreintegration::prepareOdometryMessage( nav_msgs::msg::Odometry &odometry, 
const sensor_msgs::msg::Imu &thisImu, const gtsam::NavState &currentState){
    
    Eigen::Quaterniond q_w_curr;    
    if (config_.use_imu_roll_pitch) {
        q_w_curr = Eigen::Quaterniond(thisImu.orientation.w, thisImu.orientation.x, thisImu.orientation.y, thisImu.orientation.z);
    } else {
        q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
                                      currentState.quaternion().y(), currentState.quaternion().z());
    }

    gtsam::Rot3 imuRot(q_w_curr);
    gtsam::Pose3 imuPose = gtsam::Pose3(imuRot, currentState.position());
    gtsam::Pose3 lidarPoseOpt = imuPose.compose(imu2Lidar);

    Eigen::Vector3d velocity_w_curr(currentState.velocity().x(), currentState.velocity().y(),
                                    currentState.velocity().z());
    Eigen::Vector3d velocity_curr = currentState.quaternion().inverse() * velocity_w_curr;
    
    
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = WORLD_FRAME;
    odometry.child_frame_id = SENSOR_FRAME;
    
    Eigen::Quaterniond q_w_lidar(lidarPoseOpt.rotation().toQuaternion().w(), lidarPoseOpt.rotation().toQuaternion().x(),
                                    lidarPoseOpt.rotation().toQuaternion().y(), lidarPoseOpt.rotation().toQuaternion().z());
    q_w_lidar.normalized();

    odometry.pose.pose.position.x = lidarPoseOpt.translation().x();
    odometry.pose.pose.position.y = lidarPoseOpt.translation().y();
    odometry.pose.pose.position.z = lidarPoseOpt.translation().z();
    odometry.pose.pose.orientation.x = q_w_lidar.x();
    odometry.pose.pose.orientation.y = q_w_lidar.y();
    odometry.pose.pose.orientation.z = q_w_lidar.z();
    odometry.pose.pose.orientation.w = q_w_lidar.w();

    odometry.twist.twist.linear.x = velocity_curr.x(); 
    odometry.twist.twist.linear.y = velocity_curr.y();; 
    odometry.twist.twist.linear.z = velocity_curr.z();; 
    odometry.twist.twist.angular.x =
            thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y =
            thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z =
            thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    odometry.pose.covariance[0] = double(RESULT);

    odometry.pose.covariance[1] = prevBiasOdom.accelerometer().x();
    odometry.pose.covariance[2] = prevBiasOdom.accelerometer().y();
    odometry.pose.covariance[3] = prevBiasOdom.accelerometer().z();
    odometry.pose.covariance[4] = prevBiasOdom.gyroscope().x();
    odometry.pose.covariance[5] = prevBiasOdom.gyroscope().y();
    odometry.pose.covariance[6] = prevBiasOdom.gyroscope().z();
    odometry.pose.covariance[7] = config_.imuGravity;
}


} // end namespace super_odometry