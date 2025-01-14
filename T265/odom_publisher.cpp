#include "vk_sdk/capnp/Shared.hpp"
#include "vk_sdk/capnp/odometry3d.capnp.h"
#include <iostream>
#include <memory>
#include <vk_sdk/Sdk.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <chrono>
#include <atomic>

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "intel-utils.hpp"

// using namespace std;
// using namespace rs2;


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

int main(int argc, char *argv[])
{
    auto visualkit = vkc::VisualKit::create(std::nullopt);
    if (visualkit == nullptr)
    {
        std::cout << "Failed to create visualkit connection." << std::endl;
        return -1;
    }

    const std::string odomTopic = "T265/vio_odom";

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

        try
        {
            // rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);

            std::string serial;
            if (!device_with_streams({ RS2_STREAM_POSE}, serial))
                return EXIT_SUCCESS;

            // Declare RealSense pipeline, encapsulating the actual device and sensors
            rs2::pipeline pipe;
            // Create a configuration for configuring the pipeline with a non default profile
            rs2::config cfg;

            if (!serial.empty())
                cfg.enable_device(serial);

            // Add pose stream
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

            rs2::context ctx;

            auto device = ctx.query_devices()[0];
            device.hardware_reset();

            std::cout << "Device " << device.get_info(RS2_CAMERA_INFO_NAME) << " connected" << std::endl;

            rs2::sensor sensor = device.query_sensors()[0];
            if (sensor.supports(rs2_option::RS2_OPTION_ENABLE_POSE_JUMPING)){
                sensor.set_option(rs2_option::RS2_OPTION_ENABLE_POSE_JUMPING, 0.0f);
                std::cout << "Pose Jumping is now disabled for " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            }

            std::cout << "Enable Pose Jumping" << std::endl;
            // Get a human readable description of the option
            const char* description = sensor.get_option_description(rs2_option::RS2_OPTION_ENABLE_POSE_JUMPING);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(rs2_option::RS2_OPTION_ENABLE_POSE_JUMPING);
            std::cout << "       Current Value : " << current_value << std::endl;

            // not available
            // sensor.set_option(rs2_option::RS2_OPTION_GLOBAL_TIME_ENABLED, 0);

            // Start pipeline with chosen configuration
            pipe.start(cfg);

            // Main loop
            while (true)
            {
                // Wait for the next set of frames from the camera
                auto frames = pipe.wait_for_frames();

                // Stabilize the offset between systemclock and steadyclock
                if (!update_offset_to_realtime(2)){
                    std::cerr << "Failed to update offset!" << std::endl;
                }
                // Get the offset and monotonic time
                auto t_ns_offset = time_offset.load(std::memory_order_acquire);
                auto t_ns_mono = (frames.get_timestamp()*1'000'000) - t_ns_offset; // Convert global time to nanoseconds then substract with offset
                // std::cout << "\nOffset: " << t_ns_offset << std::endl;
                // std::cout << "Monotonic Time: " << t_ns_mono << std::endl;

                // Get a frame from the pose stream
                auto f = frames.first_or_default(RS2_STREAM_POSE);
                // Cast the frame to pose_frame and get its data
                auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

                // Print the x, y, z values of the translation, relative to initial position
                std::cout << "\r" << std::string(60, ' ') << "\r"; // Clears the line
                std::cout << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
                    pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::flush;
                
                // std::cout <<"Tracker confidence: " << pose_data.tracker_confidence << std::endl; // Tracking confidence 0:Failed 1:Low 2:Medium 3:High
                
                float vision_failure_detection_metric = 1.0f - ((pose_data.tracker_confidence)/(3-0)); // Inverse values and map to [0,1], 1:failed
                bool turning_to_failure = false;
                bool damped_turning_to_failure = false;

                if (vio_failure < 0)
                    vio_failure = 0;
                else if (vio_failure == 0 && vision_failure_detection_metric >= 0.99) {
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
                header.setClockOffset(t_ns_offset);
                header.setStampMonotonic(t_ns_mono);
                header.setLatencyDevice(nowTns - t_ns_mono);
                header.setClockDomain(vkc::Header::ClockDomain::MONOTONIC);

                // Initialize Sophus::SE3 from pose_data
                Sophus::SE3d::Point translation(pose_data.translation.x,
                                    pose_data.translation.y,
                                    pose_data.translation.z);

                Sophus::SE3d::QuaternionType quaternion(pose_data.rotation.w,
                                            pose_data.rotation.x,
                                            pose_data.rotation.y,
                                            pose_data.rotation.z);

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
                // not implementing pose covariance yet
                msg.initPoseCovariance(1);
                // msg.getPoseCovariance().set(0, estimatedCovariance); // not available
                msg.setMetricVisionFailureLikelihood(vision_failure_detection_metric); 
                // msg.setMetricInertialFailureLikelihood(imu_failure_detection_metric); // not available
                // msg.setEstimatedFailureModeDrift(estimatedFailureModeDrift); // havent implement
                msg.setMetricFailureVio(vio_failure);

                msg.setResetCounter(vio_failure_count);

                odomReceiver->handle(odomTopic, vkc::Message(vkc::Shared<vkc::Odometry3d>(std::move(mmb))));
                
            }
            pipe.stop();
            return EXIT_SUCCESS;
        }
        catch (const rs2::error & e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return EXIT_FAILURE;
        }

        odomSeq++;
    });
    

    // visualkit->sink().stop(false);

    vkc::waitForCtrlCSignal();
    visualkit->sink().stop(false);
    return 0;
}