#include "inertialsense_imx5.h"


InertialSenseROS::InertialSenseROS(bool configFlashParameters) : Node("inertial_sense")
{
    load_params_srv();

    RCLCPP_INFO(get_logger(),"Connecting Sensor... \n");
    connect();
    
    //Check protocol and firmware version (Checks for V1.10 Firmware)
    if( !firmware_compatiblity_check())
    {
        RCLCPP_FATAL(get_logger(), "Protocol version of ROS node does not match device protocol!");
    }
    
    IS_.StopBroadcasts(true); 
    configure_data_streams(); 
    IS_.SavePersistent();

    if (configFlashParameters)
    {   // Set imx5 flash parameters after everything thing else so uINS flash write processor stall doesn't interfere.
        configure_flash_parameters(); 
    }

    if (log_enabled_)
    {
        start_log();    // Start log should always happen last.
    }

    initialized_ = true;
}

void InertialSenseROS::load_params_srv()
{
    // ROS Config
    RCLCPP_INFO(get_logger(), "Load Param Server");
    frame_id_ = declare_parameter<std::string>("frame_id","body");
    log_enabled_ = declare_parameter<bool>("enable_log",false);

    // Enable Sensors
    preint_IMU_MAG_.enabled = declare_parameter<bool>("stream_preint_IMU_mag_",false); // DID_PIMU_MAG
    IMU_MAG_.enabled = declare_parameter<bool>("stream_IMU_mag",true); // DID_IMU_MAG
    IMU_.enabled = declare_parameter<bool>("stream_IMU",false); // DID_IMU
    preint_IMU_.enabled = declare_parameter<bool>("stream_preint_IMU",false); // DID_PIMU
    IMU_RAW_.enabled = declare_parameter<bool>("stream_IMU_raw",false); // DID_IMU_RAW
    IMU3_RAW_.enabled = declare_parameter<bool>("stream_IMU3_raw",false); // DID_IMU3_RAW
    baro_.enabled = declare_parameter<bool>("stream_baro",true); // DID_BAROMETER
    mag_.enabled = declare_parameter<bool>("stream_mag",false); // DID_MAGNETOMETER

    // Inertialsense Sensor Config 
    port_ = declare_parameter<std::string>("port","/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate",921600);
    ins_nav_dt_ms_ = declare_parameter<int>("navigation_dt_ms",10); //Navigation Sampling Period, used for PIMU and IMU
    imu_dt_ms_ = declare_parameter<int>("imu_dt_ms",1); //IMU Sampling Period, used for IMU_RAW and IMU_RAW3. Set imu_dt_ms to 1 if using IMU or PIMU
    magDeclination_ = declare_parameter<float>("declination", 0.0f); 

    if (ins_nav_dt_ms_ <= 8)
    {
        RCLCPP_WARN(get_logger(), "navigation_dt_ms must be >=8ms... setting navigation_dt_ms to 8ms");
        ins_nav_dt_ms_ = 8; 
    }
    // Checks:

    // If we are using 'filtered' IMU data (e.g. PIMU / IMU), we want to set imu_dt_ms = 1 for maximum filter 
    // performance. By extension, we do not need 'raw' IMU data (IMU_RAW / IMU3_RAW), especially sampled at 1 kHz. 
    // Therefore, IMU_RAW and IMU3_RAW is disabled. 
    if (preint_IMU_.enabled || IMU_.enabled)
    {
        IMU_RAW_.enabled = false;
        IMU3_RAW_.enabled = false; 
        imu_dt_ms_ = 1; // set to 1 ms to get the 'best' IMU and PIMU data possible. 
        RCLCPP_INFO(get_logger(), "ins_nav_dt_ms_ set to 1 ms for best PIMU and PIMU results"); 
    }

    // If want 'RAW' IMU data (typically if a sampling period (dT) 'smaller' than ins_nav_dt_ms_ is desired, then we do not care 
    // about 'filtered' IMU data (e.g. PIMU / IMU) being potentially less accurate), since we are using IMU_RAW / IMU3_RAW data. 
    // Might as well disable PIMU and IMU streams since they are not used. 
    if (IMU_RAW_.enabled || IMU3_RAW_.enabled)
    {
        preint_IMU_.enabled = false;
        IMU_.enabled = false; 
        IMU_.enabled = false; 
    }

    // Default to using 'filtered' IMU data. 
    if ((preint_IMU_.enabled || IMU_.enabled) && (IMU_RAW_.enabled || IMU3_RAW_.enabled))
    {
        IMU_RAW_.enabled = false;
        IMU3_RAW_.enabled = false; 
        imu_dt_ms_ = 1; // set to 1 ms to get the 'best' IMU and PIMU data possible. 
        RCLCPP_WARN(get_logger(), "Both filterd (IMU/PIMU) and RAW (IMU_RAW/IMU3_RAW) data streams are enabled... Disabling RAW Data Streams"); 
    }

    if (preint_IMU_MAG_.enabled && IMU_MAG_.enabled)
    {
        IMU_MAG_.enabled = false; // you should only run either IMU_MAG or IMU_MAG, not both.
        preint_IMU_.enabled = false; //preint_IMU will be handled by PIMU_MAG
        mag_.enabled = false; // mag will be handled with IMU_MAG. 
        imu_dt_ms_ = 1; // set to 1 ms to get the 'best' IMU and PIMU data possible. 
        RCLCPP_WARN(get_logger(), "You can only choose eithe PIMU_MAG or IMU_MAG. Defaulting to PIMU_MAG"); 
    }

    if (preint_IMU_MAG_.enabled)
    {
        IMU_MAG_.enabled = false; // you should only run either IMU_MAG or IMU_MAG, not both.
        preint_IMU_.enabled = false; //preint_IMU will be handled by PIMU_MAG
        mag_.enabled = false; // mag will be handled with IMU_MAG. 
        imu_dt_ms_ = 1; // set to 1 ms to get the 'best' IMU and PIMU data possible. 
        RCLCPP_WARN(get_logger(), "Both filterd (IMU/PIMU) and RAW (IMU_RAW/IMU3_RAW) data streams are enabled... Disabling RAW Data Streams"); 
    }

    if (IMU_MAG_.enabled)
    {
        preint_IMU_MAG_.enabled = false; // you should only run either IMU_MAG or IMU_MAG, not both.
        IMU_.enabled = false; // you should only run either IMU_MAG or IMU_MAG, not both. 
        mag_.enabled = false; // mag will be handled with IMU_MAG. 
        imu_dt_ms_ = 1; // set to 1 ms to get the 'best' IMU and PIMU data possible. 
        RCLCPP_WARN(get_logger(), "Both filterd (IMU/PIMU) and RAW (IMU_RAW/IMU3_RAW) data streams are enabled... Disabling RAW Data Streams"); 
    }
}

void InertialSenseROS::configure_data_streams() 
{
    // Set up the magnetometer ROS stream
    if (mag_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable Mag data stream.");
        SET_CALLBACK(DID_MAGNETOMETER, magnetometer_t, mag_callback, mag_.period_multiple);
    }    

    // Set up the barometer ROS stream
    if (baro_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable baro data stream.");
        SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback, baro_.period_multiple);
    }

    // Set up the preintegrated IMU (coning and sculling integral) ROS stream
    if (preint_IMU_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable preint IMU data stream.");
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, preint_IMU_.period_multiple);
    }
    
    // Set up the IMU ROS stream
    if(IMU_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable IMU data stream.");
        SET_CALLBACK(DID_IMU, imu_t, IMU_callback, IMU_.period_multiple);
    } 

    // Set up the PIMU_MAG ROS stream
    if(preint_IMU_MAG_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable IMU data stream.");
        SET_CALLBACK(DID_PIMU_MAG, pimu_mag_t, preint_IMU_MAG_callback, preint_IMU_MAG_.period_multiple);
    } 

    // Set up the PIMU_MAG ROS stream
    if(IMU_MAG_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable IMU data stream.");
        SET_CALLBACK(DID_IMU_MAG, imu_mag_t, IMU_MAG_callback, IMU_MAG_.period_multiple);
    } 

    // Set up the IMU_RAW ROS stream
    if(IMU_RAW_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable IMU data stream.");
        SET_CALLBACK(DID_IMU_RAW, imu_t, IMU_RAW_callback, IMU_RAW_.period_multiple);
    } 

    // Set up the IMU3_RAW ROS stream
    if(IMU3_RAW_.enabled)
    {
        RCLCPP_INFO(get_logger(), "Attempting to enable IMU data stream.");
        SET_CALLBACK(DID_IMU3_RAW, imu3_t, IMU3_RAW_callback, IMU3_RAW_.period_multiple);
    } 


    RCLCPP_INFO(get_logger(), "Data Streaming...");
}

void InertialSenseROS::start_log()
{
    std::string filename = getenv("HOME");
    filename += "/Documents/Inertial_Sense/Logs/" + cISLogger::CreateCurrentTimestamp();
    RCLCPP_INFO_STREAM(get_logger(), "InertialSenseROS: Creating log in " << filename << " folder");
    IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_GROUND_VEHICLE);
}

void InertialSenseROS::connect()
{
    /// Connect to the uINS
    RCLCPP_INFO(get_logger(), "Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    if (!IS_.Open(port_.c_str(), baudrate_))
    {
        RCLCPP_FATAL(get_logger(),"inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
        exit(0);
    }

    else
    {
        // Print if Successful
        RCLCPP_INFO(get_logger(), "Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
    }
}

bool InertialSenseROS::firmware_compatiblity_check()
{
    if( IS_.GetDeviceInfo().protocolVer[0] != PROTOCOL_VERSION_CHAR0 || \
            IS_.GetDeviceInfo().protocolVer[1] != PROTOCOL_VERSION_CHAR1 || \
            IS_.GetDeviceInfo().protocolVer[2] != PROTOCOL_VERSION_CHAR2 || \
            IS_.GetDeviceInfo().protocolVer[3] != PROTOCOL_VERSION_CHAR3 || \
            IS_.GetDeviceInfo().firmwareVer[0] != FIRMWARE_VERSION_CHAR0 || \
            IS_.GetDeviceInfo().firmwareVer[1] != FIRMWARE_VERSION_CHAR1 || \
            IS_.GetDeviceInfo().firmwareVer[2] != FIRMWARE_VERSION_CHAR2)
    {
        return false;
    }

    else
    {
        return true;
    }
}

void InertialSenseROS::configure_flash_parameters()
{    
    get_flash_config();
    bool reboot = false;

    nvm_flash_cfg_t current_flash_cfg;
    IS_.GetFlashConfig(current_flash_cfg);  
    
    //startupNavDtMs
    if (ins_nav_dt_ms_ != current_flash_cfg.startupNavDtMs)
    {
        RCLCPP_INFO(get_logger(), "ReConfiguring startupNavDtMs\n"); 
        set_flash_config<int>("navigation_dt_ms", offsetof(nvm_flash_cfg_t, startupNavDtMs));
        reboot = true; 
    }

    //startupImuDtMs
    if (imu_dt_ms_ != current_flash_cfg.startupImuDtMs)
    {
        RCLCPP_INFO(get_logger(), "ReConfiguring startupImuDtMs\n"); 
        set_flash_config<int>("imu_dt_ms", offsetof(nvm_flash_cfg_t, startupImuDtMs));
        reboot = true; 
    }

    //magDeclination
    if (magDeclination_ != current_flash_cfg.magDeclination)
    {
        RCLCPP_INFO(get_logger(), "ReConfiguring magDeclination\n"); 
        set_flash_config<int>("declination", offsetof(nvm_flash_cfg_t, magDeclination));
        reboot = true; 
    }

    if  (reboot)
    {
        RCLCPP_INFO(get_logger(), "Sensor Disconnecting..."); 
        reset_device();
        RCLCPP_WARN(get_logger(), "Shutting Down Node... (You may have to unplug and replug your sensor)");
        rclcpp::shutdown();
    }

    else 
    {
        RCLCPP_INFO(get_logger(), "Sensors Successfully Configured."); 
    }

}

template <typename T>
void InertialSenseROS::set_flash_config(std::string ros2_param_name, uint32_t offset)
{
    T tmp;
    this->get_parameter(ros2_param_name, tmp);
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&tmp), sizeof(T), offset);
}

void InertialSenseROS::get_flash_config()
{
    nvm_flash_cfg_t current_flash_cfg;
    IS_.GetFlashConfig(current_flash_cfg);    
    std::cout << "ins_nav_dt_ms_: " << "ros2 param: " << ins_nav_dt_ms_ << " imx5_param: " << current_flash_cfg.startupNavDtMs << std::endl; 
    std::cout << "imu_dt_ms_: " << "ros2 param: " << imu_dt_ms_ << " imx5_param: " << current_flash_cfg.startupImuDtMs << std::endl; 
    std::cout << "magDeclination_: " << "ros2 param: " << magDeclination_ << " imx5_param: " << current_flash_cfg.magDeclination << std::endl; 
}

void InertialSenseROS::update()
{
	IS_.Update();
}

void InertialSenseROS::mag_callback(eDataIDs DID, const magnetometer_t *const msg)
{
    if (!magStreaming_)
    {
        RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
        if (mag_.enabled)
            mag_pub = create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
    }

    magStreaming_ = true;
    sensor_msgs::msg::MagneticField mag_msg;
    // mag_msg.header.stamp = get_clock()->now();
    mag_msg.header.stamp = ros_time_from_start_time(msg->time);
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = msg->mag[0];
    mag_msg.magnetic_field.y = msg->mag[1];
    mag_msg.magnetic_field.z = msg->mag[2];

    mag_pub->publish(mag_msg);
}

void InertialSenseROS::baro_callback(eDataIDs DID, const barometer_t *const msg)
{
    if (!baroStreaming_)
    {
        RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
        if (baro_.enabled)
            baro_pub = create_publisher<sensor_msgs::msg::FluidPressure>("baro", 1);
    }

    baroStreaming_ = true;
    // baro_msg.header.stamp = get_clock()->now();
    baro_msg.header.stamp = ros_time_from_start_time(msg->time);
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = msg->bar;
    baro_msg.variance = msg->barTemp;

    baro_pub->publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(eDataIDs DID, const pimu_t *const msg)
{   
    if (preint_IMU_.enabled)
    {   
        if (!preintImuStreaming_)
        {   
            RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
            preint_IMU_pub = create_publisher<inertialsense_msgs::msg::PreIntIMU>("preint_imu", 1);
        }
        preintImuStreaming_ = true;
        // preintIMU_msg.header.stamp = get_clock()->now(); 
        preintIMU_msg.header.stamp = ros_time_from_start_time(msg->time);
        preintIMU_msg.header.frame_id = frame_id_;
        preintIMU_msg.dtheta.x = msg->theta[0];
        preintIMU_msg.dtheta.y = msg->theta[1];
        preintIMU_msg.dtheta.z = msg->theta[2];

        preintIMU_msg.dvel.x = msg->vel[0];
        preintIMU_msg.dvel.y = msg->vel[1];
        preintIMU_msg.dvel.z = msg->vel[2];

        preintIMU_msg.dt = msg->dt;

        preint_IMU_pub->publish(preintIMU_msg); 
    }
}

void InertialSenseROS::IMU_callback(eDataIDs DID, const imu_t *const msg)
{      
    if (IMU_.enabled)
    {
        if (!imuStreaming_)
        {
            RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
            IMU_pub = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
        }
        imuStreaming_ = true;
        // imu_msg.header.stamp = get_clock()->now(); 
        imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->I.pqr[0];
        imu_msg.angular_velocity.y = msg->I.pqr[1];
        imu_msg.angular_velocity.z = msg->I.pqr[2];
        imu_msg.linear_acceleration.x = msg->I.acc[0];
        imu_msg.linear_acceleration.y = msg->I.acc[1];
        imu_msg.linear_acceleration.z = msg->I.acc[2];
        
        IMU_pub->publish(imu_msg);
    } 
}

void InertialSenseROS::preint_IMU_MAG_callback(eDataIDs DID, const pimu_mag_t *const msg)
{
    if (preint_IMU_MAG_.enabled)
    {
        if (!preintImuMagStreaming_)
        {
            RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
            preint_IMU_pub = create_publisher<inertialsense_msgs::msg::PreIntIMU>("preint_imu", 1);
            mag_pub = create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
        }
        preintImuMagStreaming_ = true;
        sensor_msgs::msg::MagneticField mag_msg;

        // preintIMU_msg.header.stamp = get_clock()->now(); 
        preintIMU_msg.header.stamp = ros_time_from_start_time(msg->pimu.time);
        preintIMU_msg.header.frame_id = frame_id_;
        preintIMU_msg.dtheta.x = msg->pimu.theta[0];
        preintIMU_msg.dtheta.y = msg->pimu.theta[1];
        preintIMU_msg.dtheta.z = msg->pimu.theta[2];

        preintIMU_msg.dvel.x = msg->pimu.vel[0];
        preintIMU_msg.dvel.y = msg->pimu.vel[1];
        preintIMU_msg.dvel.z = msg->pimu.vel[2];

        preintIMU_msg.dt = msg->pimu.dt;

        preint_IMU_pub->publish(preintIMU_msg); 

        mag_msg.header.stamp = ros_time_from_start_time(msg->pimu.time);
        mag_msg.header.frame_id = frame_id_;
        mag_msg.magnetic_field.x = msg->mag.mag[0];
        mag_msg.magnetic_field.y = msg->mag.mag[1];
        mag_msg.magnetic_field.z = msg->mag.mag[2];
        mag_pub->publish(mag_msg); 
    } 

}


void InertialSenseROS::IMU_MAG_callback(eDataIDs DID, const imu_mag_t *const msg)
{    
    if (IMU_MAG_.enabled)
    {
        if (!imuMagStreaming_)
        {
            RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
            IMU_pub = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
            mag_pub = create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
        }
        imuMagStreaming_ = true;
        // imu_msg.header.stamp = get_clock()->now(); 
        sensor_msgs::msg::MagneticField mag_msg;

        imu_msg.header.stamp = ros_time_from_start_time(msg->mag.time);
        imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->imu.I.pqr[0];
        imu_msg.angular_velocity.y = msg->imu.I.pqr[1];
        imu_msg.angular_velocity.z = msg->imu.I.pqr[2];
        imu_msg.linear_acceleration.x = msg->imu.I.acc[0];
        imu_msg.linear_acceleration.y = msg->imu.I.acc[1];
        imu_msg.linear_acceleration.z = msg->imu.I.acc[2];
        IMU_pub->publish(imu_msg);

        mag_msg.header.stamp = ros_time_from_start_time(msg->mag.time);
        mag_msg.header.frame_id = frame_id_;
        mag_msg.magnetic_field.x = msg->mag.mag[0];
        mag_msg.magnetic_field.y = msg->mag.mag[1];
        mag_msg.magnetic_field.z = msg->mag.mag[2];
        mag_pub->publish(mag_msg); 
    } 
}

void InertialSenseROS::IMU_RAW_callback(eDataIDs DID, const imu_t *const msg)
{      
    if (IMU_RAW_.enabled)
    {
        if (!imuRawStreaming_)
        {
            RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
            IMU_RAW_pub = create_publisher<sensor_msgs::msg::Imu>("imu_raw", 1);
        }
        imuRawStreaming_ = true;
        // imu_msg.header.stamp = get_clock()->now(); 
        imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->I.pqr[0];
        imu_msg.angular_velocity.y = msg->I.pqr[1];
        imu_msg.angular_velocity.z = msg->I.pqr[2];
        imu_msg.linear_acceleration.x = msg->I.acc[0];
        imu_msg.linear_acceleration.y = msg->I.acc[1];
        imu_msg.linear_acceleration.z = msg->I.acc[2];
        
        IMU_RAW_pub->publish(imu_msg);
    } 
}

void InertialSenseROS::IMU3_RAW_callback(eDataIDs DID, const imu3_t *const msg)
{      
    if (IMU3_RAW_.enabled)
    {
        if (!imu3RawStreaming_)
        {
            RCLCPP_INFO(get_logger(), "%s response received", cISDataMappings::GetDataSetName(DID));
            IMU1_RAW_pub = create_publisher<sensor_msgs::msg::Imu>("imu1_raw", 1);
            IMU2_RAW_pub = create_publisher<sensor_msgs::msg::Imu>("imu2_raw", 1);
            IMU3_RAW_pub = create_publisher<sensor_msgs::msg::Imu>("imu3_raw", 1);

        }
        imu3RawStreaming_ = true;
        // imu_msg.header.stamp = get_clock()->now(); 
        imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->I[0].pqr[0];
        imu_msg.angular_velocity.y = msg->I[0].pqr[1];
        imu_msg.angular_velocity.z = msg->I[0].pqr[2];
        imu_msg.linear_acceleration.x = msg->I[0].acc[0];
        imu_msg.linear_acceleration.y = msg->I[0].acc[1];
        imu_msg.linear_acceleration.z = msg->I[0].acc[2];
        IMU1_RAW_pub->publish(imu_msg);

        // imu_msg.header.stamp = get_clock()->now(); 
        // imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        // imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->I[1].pqr[0];
        imu_msg.angular_velocity.y = msg->I[1].pqr[1];
        imu_msg.angular_velocity.z = msg->I[1].pqr[2];
        imu_msg.linear_acceleration.x = msg->I[1].acc[0];
        imu_msg.linear_acceleration.y = msg->I[1].acc[1];
        imu_msg.linear_acceleration.z = msg->I[1].acc[2];
        IMU2_RAW_pub->publish(imu_msg);

        // imu_msg.header.stamp = get_clock()->now(); 
        // imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        // imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->I[2].pqr[0];
        imu_msg.angular_velocity.y = msg->I[2].pqr[1];
        imu_msg.angular_velocity.z = msg->I[2].pqr[2];
        imu_msg.linear_acceleration.x = msg->I[2].acc[0];
        imu_msg.linear_acceleration.y = msg->I[2].acc[1];
        imu_msg.linear_acceleration.z = msg->I[2].acc[2];
        IMU3_RAW_pub->publish(imu_msg);
    } 
}

void InertialSenseROS::reset_device()
{
    // send reset command
    system_command_t reset_command;
    reset_command.command = 99;
    reset_command.invCommand = ~reset_command.command;
    IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t*>(&reset_command), sizeof(system_command_t), 0);

    RCLCPP_WARN(get_logger(), "Device reset required.\n\nDisconnecting from device.");
    sleep(2);
    IS_.Close();
}

rclcpp::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
  rclcpp::Time rostime(0);
  
  // Otherwise, estimate the uINS boot time and offset the messages
  if (!got_first_message_)
  {
    got_first_message_ = true;
    INS_local_offset_ = this->now().seconds() - time;
  }
  else // low-pass filter offset to account for drift
  {
    double y_offset = this->now().seconds() - time;
    INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
  }
  // Publish with ROS time
  rostime = rclcpp::Time((INS_local_offset_ + time)*1e9);
  
  return rostime;
}