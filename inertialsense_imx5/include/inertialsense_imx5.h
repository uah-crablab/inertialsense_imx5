#ifndef INERTIALSENSE_IMX5_H
#define INERTIALSENSE_IMX5_H

#include <iostream>
#include <string.h>

#include "InertialSense.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "inertialsense_msgs/msg/pre_int_imu.hpp"

#define FIRMWARE_VERSION_CHAR0 1
#define FIRMWARE_VERSION_CHAR1 10
#define FIRMWARE_VERSION_CHAR2 0

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple)           \
    IS_.BroadcastBinaryData(DID, __periodmultiple,                      \
        [this](InertialSense *i, p_data_t *data, int pHandle)           \
        {                                                               \
            /* ROS_INFO("Got message %d", DID);*/                       \
            this->__cb_fun(DID, reinterpret_cast<__type *>(data->buf)); \
        })

class InertialSenseROS  : public rclcpp::Node
{
    public:
        InertialSenseROS(bool configFlashParameters = true);

        void update(); 

        void load_params_srv(); 
        
        void connect(); //L82
        bool firmware_compatiblity_check(); //L83
        void configure_flash_parameters(); //L85
        void configure_data_streams(); //L90
        void start_log(); //L93
        void reset_device(); //L99

        // Serial Port Configuration
        std::string port_; 
        int baudrate_; 
        bool initialized_; //L105
        bool log_enabled_; //L106
        std::string frame_id_; //L109

        // ROS Stream handling
        typedef struct
        {
            bool enabled = false;
            // not sure how to declare 'blank' publishers in ROS2.
            // Declaring publishers seperately instead
            // ros::Publisher pub; 
            // ros::Publisher pub2;
            // ros::Publisher pub3;
            int period_multiple = 1;
            
        } ros_stream_t;

        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub;
        rclcpp::Publisher<inertialsense_msgs::msg::PreIntIMU>::SharedPtr preint_IMU_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU_RAW_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU1_RAW_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU2_RAW_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU3_RAW_pub;

        void mag_callback(eDataIDs DID, const magnetometer_t *const msg); //L189
        void baro_callback(eDataIDs DID, const barometer_t *const msg);   //L190 
        void preint_IMU_callback(eDataIDs DID, const pimu_t *const msg);  //L191
        void IMU_callback(eDataIDs DID, const imu_t *const msg);
        void preint_IMU_MAG_callback(eDataIDs DID, const pimu_mag_t *const msg);
        void IMU_MAG_callback(eDataIDs DID, const imu_mag_t *const msg);
        void IMU_RAW_callback(eDataIDs DID, const imu_t *const msg); 
        void IMU3_RAW_callback(eDataIDs DID, const imu3_t *const msg); 


        ros_stream_t IMU_; //L214
        ros_stream_t IMU_RAW_; 
        ros_stream_t IMU3_RAW_; 
        ros_stream_t mag_; //L215
        ros_stream_t preint_IMU_; //L216
        ros_stream_t baro_; //L217
        ros_stream_t preint_IMU_MAG_;
        ros_stream_t IMU_MAG_; 
        bool magStreaming_ = false; //L238
        bool baroStreaming_ = false; //L239
        bool preintImuStreaming_ = false; //L240
        bool imuStreaming_ = false; //L241
        bool preintImuMagStreaming_ = false;
        bool imuMagStreaming_ = false; 
        bool imuRawStreaming_ = false;
        bool imu3RawStreaming_ = false;
        bool data_streams_enabled_;  //L257

        /**
         * @brief ros_time_from_start_time
         * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
         * @return equivalent ros::Time
         */
        rclcpp::Time ros_time_from_start_time(const double time); //L313 
        
        double INS_local_offset_ = 0.0;  // Current estimate of the uINS start time in ROS time seconds (L330)
        bool got_first_message_ = false; // Flag to capture first uINS start time guess (L331)
        
        sensor_msgs::msg::FluidPressure baro_msg;
        sensor_msgs::msg::Imu imu_msg; //L364
        inertialsense_msgs::msg::PreIntIMU preintIMU_msg; //L381

        // Connection to the imx5
        InertialSense IS_;

        // flash parameters 

        int ins_nav_dt_ms_; //L393
        int imu_dt_ms_; 
        float magDeclination_ ; //L400
        int insDynModel_; //L401
        int ioConfigBits_; // EVB2: GPS1 Ser1 F9P, GPS2 disabled F9P, PPS G8
        

        // ros2 added variables
        bool startup; 
        template<typename T>  void set_flash_config(std::string ros2_param_name, uint32_t offset) __attribute__ ((optimize(0)));
        void get_flash_config();
}; 

#endif
