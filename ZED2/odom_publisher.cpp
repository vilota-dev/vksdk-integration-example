#include <vk_sdk/capnp/Shared.hpp>
#include <vk_sdk/capnp/odometry3d.capnp.h>
#include <vk_sdk/Sdk.hpp>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <chrono>
#include <atomic>

#include <sl/Camera.hpp>

static std::atomic<int64_t> time_offset;

bool update_offset_to_realtime(size_t try_count) {
    // Nanoseconds threhsold to detect a context-switch happening in-between calls.
    const int64_t THRESHOLD = 10'000;

    while (try_count-- > 0) {
        auto mono_tp0 = std::chrono::steady_clock::now();
        auto real_tp1 = std::chrono::system_clock::now();
        auto mono_tp2 = std::chrono::steady_clock::now();

        auto mono_t0 = std::chrono::nanoseconds(mono_tp0.time_since_epoch()).count();
        auto real_t1 = std::chrono::nanoseconds(real_tp1.time_since_epoch()).count();
        auto mono_t2 = std::chrono::nanoseconds(mono_tp2.time_since_epoch()).count();
        std::cout << real_t1 << std::endl;

        auto diff_a = real_t1 - mono_t0;
        auto diff_b = mono_t2 - real_t1;

        auto twice_overhead = diff_a + diff_b;

        if (twice_overhead < THRESHOLD) {
            auto twice_offset = diff_a - diff_b;

            time_offset.store(twice_offset / 2, std::memory_order_release);
            return true;
        }
    }

    return false;
}

int main(int argc, char **argv) {

    auto visualkit = vkc::VisualKit::create(std::nullopt);
    if (visualkit == nullptr)
    {
        std::cout << "Failed to create visualkit connection." << std::endl;
        return -1;
    }

    const std::string odomTopic = "ZED2/vio_odom";

    // Output from VIO to ECAL
    auto odomReceiver = visualkit->sink().obtain(odomTopic, vkc::Type<vkc::Odometry3d>());
    if (odomReceiver == nullptr) {
        std::cout << "Failed to create publisher to " << odomTopic << std::endl;
        return -1;
    }

    visualkit->sink().start();

    std::thread t_odom([&]() {
        std::uint64_t odomSeq = 0;

        int32_t vio_failure = -1;
        uint32_t vio_failure_count = 0;
        uint64_t vio_turning_to_failure_last_stamp = 0;

        // Create a ZED camera object
        sl::Camera zed;

        // Set configuration parameters
        sl::InitParameters init_parameters;
        init_parameters.camera_resolution = sl::RESOLUTION::HD720; // Use HD720 or HD1200 video mode (default fps: 60)
        init_parameters.camera_fps = 0; // 0 will set to highest available fps
        init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
        init_parameters.coordinate_units = sl::UNIT::METER; // Set units in meters
        init_parameters.sensors_required = true;
        
        // Open the camera
        auto returned_state = zed.open(init_parameters);
        if (returned_state != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Error " << returned_state << ", exit program.\n";
            return EXIT_FAILURE;
        }

        // Enable positional tracking with default parameters
        sl::PositionalTrackingParameters tracking_parameters;
        tracking_parameters.enable_area_memory = false; // Disable VSLAM
        returned_state = zed.enablePositionalTracking(tracking_parameters);
        if (returned_state != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Error " << returned_state << ", exit program.\n";
            return EXIT_FAILURE;
        }

        sl::Pose zed_pose;
        sl::Timestamp last_imu_ts = 0;

        // Check if the camera is a ZED M and therefore if an IMU is available
        bool zed_has_imu = zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE);
        sl::SensorsData sensor_data;
        sl::PositionalTrackingStatus status;

        while (true) {
            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

                zed.getSensorsData(sensor_data, sl::TIME_REFERENCE::CURRENT);
                // Get the pose of the left eye of the camera with reference to the world frame
                zed.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD); 

                // get the translation information
                auto zed_translation = zed_pose.getTranslation();
                // get the orientation information
                auto zed_orientation = zed_pose.getOrientation();

                // Stabilize the offset between systemclock and steadyclock
                if (!update_offset_to_realtime(2)){
                    std::cerr << "Failed to update offset!" << std::endl;
                }

                // Get the offset and monotonic time
                auto t_ns_offset = time_offset.load(std::memory_order_acquire);
                auto t_ns_mono = zed_pose.timestamp.getNanoseconds() - t_ns_offset; // Convert global time to nanoseconds then substract with offset
                // std::cout << "\nOffset: " << t_ns_offset << std::endl;
                // std::cout << "Monotonic Time: " << t_ns_mono << std::endl;

                if (odomSeq % 10 == 0) {
                    // Display the translation and timestamp
                    std::cout << std::setprecision(3) << std::fixed << "Camera Translation: {" << zed_translation << "}, Orientation: {" << zed_orientation
                            << "}, timestamp: " << zed_pose.timestamp.getNanoseconds() << "ns" << std::endl;
                }
                // Status and statistics
                // std::cout << status.tracking_fusion_status << std::endl; // Show which positional tracking fusion is used
                // std::cout << status.odometry_status << std::endl; // Show current state of Visual-Inertial Odometry (VIO) tracking between the previous frame and the current frame
                // std::cout << zed_pose.pose_covariance << std::endl; // float[36] pose covariance matrix of translation
                // std::cout << zed_pose.pose_confidence << std::endl; // Confidence/quality of the pose estimation for the target frame. Range [0,100]
                // std::cout << sensor_data.imu.pose_covariance << std::endl; // Covariance matrix of imu

                // Calculate norm of pose covariance
                // Convert the float[36] array to an Eigen 6x6 matrix
                Eigen::Map<Eigen::Matrix<float, 6, 6>> cov_matrix(zed_pose.pose_covariance);
                // Eigen::Map<Eigen::Matrix<float, 3, 3>> imu_cov_matrix(sensor_data.imu.pose_covariance);
                // Compute the Frobenius norm (magnitude) of the covariance matrix
                float covariance_norm = cov_matrix.norm();

                float vision_failure_detection_metric = (100.0f - zed_pose.pose_confidence) / 100; // inverse map to [1,0], 1:Failed
                bool turning_to_failure = false;
                bool damped_turning_to_failure = false;

                if (vio_failure < 0)
                    vio_failure = 0;
                else if (vio_failure == 0 && vision_failure_detection_metric > 0.99) {
                    vio_failure = 1;
                    turning_to_failure = true;
                }
                else if (vio_failure == 1 && vision_failure_detection_metric < 0.9) {
                    vio_failure = 0;
                }

                if (turning_to_failure && t_ns_mono - vio_turning_to_failure_last_stamp > 1e9) {
                    damped_turning_to_failure = true;
                    vio_failure_count++;
                }

                if (damped_turning_to_failure)
                    vio_turning_to_failure_last_stamp = t_ns_mono;
                
                auto mmb = std::make_unique<capnp::MallocMessageBuilder>();
                vkc::Odometry3d::Builder msg = mmb->getRoot<vkc::Odometry3d>();
                std::int64_t nowTns = std::chrono::steady_clock::now().time_since_epoch().count();

                auto header = msg.getHeader();
                header.setSeq(odomSeq);
                header.setStampMonotonic(t_ns_mono);
                header.setClockOffset(t_ns_offset);
                header.setLatencyDevice(nowTns - t_ns_mono);
                header.setClockDomain(vkc::Header::ClockDomain::MONOTONIC);

                // Initialize Sophus::SE3 from pose_data
                Sophus::SE3d::Point translation(zed_translation.tx,
                                    zed_translation.ty,
                                    zed_translation.tz);

                Sophus::SE3d::QuaternionType quaternion(zed_orientation.ow,
                                            zed_orientation.ox,
                                            zed_orientation.oy,
                                            zed_orientation.oz);

                // Normalize the quaternion (optional for safety)
                quaternion.normalize();

                // Create the SE3 object
                Sophus::SE3d T_w_b(quaternion, translation);

                Sophus::Matrix3<double> R_nwu_eus;
                // change of coordinates from EUS to NWU
                Sophus::SE3d T_nwu_eus;
                R_nwu_eus << 0, 0, -1,
                            -1, 0, 0,
                            0, 1, 0;

                T_nwu_eus.setRotationMatrix(R_nwu_eus);
                T_nwu_eus.translation().setZero();

                Sophus::SE3d T_nwu_frd;
                T_nwu_frd = T_nwu_eus * T_w_b * T_nwu_eus.inverse();

                auto position = msg.getPose().getPosition();
                position.setX(T_nwu_frd.translation().x());
                position.setY(T_nwu_frd.translation().y());
                position.setZ(T_nwu_frd.translation().z());

                auto orientation = msg.getPose().getOrientation();
                orientation.setW(T_nwu_frd.unit_quaternion().w());
                orientation.setX(T_nwu_frd.unit_quaternion().x());
                orientation.setY(T_nwu_frd.unit_quaternion().y());
                orientation.setZ(T_nwu_frd.unit_quaternion().z());
                
                msg.setBodyFrame(vkc::Odometry3d::BodyFrame::NWU);
                msg.setReferenceFrame(vkc::Odometry3d::ReferenceFrame::NWU);
                // not implementing velocity output yet
                // msg.setVelocityFrame(vkc::Odometry3d::VelocityFrame::NONE);
                msg.initPoseCovariance(1);
                msg.getPoseCovariance().set(0, covariance_norm);
                msg.setMetricVisionFailureLikelihood(vision_failure_detection_metric);
                // msg.setMetricInertialFailureLikelihood(imu_failure_detection_metric); // havent implement
                // msg.setEstimatedFailureModeDrift(estimatedFailureModeDrift); // havent implement
                msg.setMetricFailureVio(vio_failure);

                msg.setResetCounter(vio_failure_count);

                odomReceiver->handle(odomTopic, vkc::Message(vkc::Shared<vkc::Odometry3d>(std::move(mmb))));
                odomSeq++;
            }
        }

        // Disable positional tracking and close the camera
        zed.disablePositionalTracking();
        zed.close();
        return EXIT_SUCCESS;
    });
    vkc::waitForCtrlCSignal();
    visualkit->sink().stop(false);
    return 0;
}
