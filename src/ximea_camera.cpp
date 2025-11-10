#include "ximea_camera/ximea_camera.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <filesystem>
#include <vector>
#include <sstream>
#include <fstream>
 

namespace { // anon
    int roundDown(int value, int increment)
    {
        return (value / increment) * increment;
    }
    int roundUp(int value, int increment)
    {
        return roundDown(value + increment - 1, increment);
    }
    
} // anon

namespace ximea_camera {


std::map<std::string, int> XimeaROSCam::PixelFormatMap = {
    {"XI_MONO8",      XI_MONO8},
    {"XI_MONO16",     XI_MONO16},
    {"XI_RGB24",      XI_RGB24},
    {"XI_RGB32",      XI_RGB32},
    {"XI_RAW8",       XI_RAW8},
    {"XI_RAW16",      XI_RAW16}
};

std::string_view XimeaROSCam::getImageEncoding() 
{
    int colour_filter_array = -1;
    get(XI_PRM_COLOR_FILTER_ARRAY, colour_filter_array);

    switch(this->pixel_format_)
    {
    case XI_MONO8:
        return sensor_msgs::image_encodings::MONO8;
    case XI_MONO16:
        return sensor_msgs::image_encodings::MONO16;
    case XI_RGB24:
        return sensor_msgs::image_encodings::BGR8; // check this!
    case XI_RGB32:
        return sensor_msgs::image_encodings::BGRA8; // check this!
    case XI_RAW8:
        switch(colour_filter_array)
        {
        case XI_CFA_NONE:
            return sensor_msgs::image_encodings::MONO8;
        case XI_CFA_BAYER_RGGB:
            return sensor_msgs::image_encodings::BAYER_RGGB8;
        case XI_CFA_BAYER_BGGR:
            return sensor_msgs::image_encodings::BAYER_BGGR8;
        case XI_CFA_BAYER_GRBG:
            return sensor_msgs::image_encodings::BAYER_GRBG8;
        case XI_CFA_BAYER_GBRG:
            return sensor_msgs::image_encodings::BAYER_GBRG8;
        }
        break;
    case XI_RAW16:
        switch(colour_filter_array)
        {
        case XI_CFA_NONE:
            return sensor_msgs::image_encodings::MONO16;
        case XI_CFA_BAYER_RGGB:
            return sensor_msgs::image_encodings::BAYER_RGGB16;
        case XI_CFA_BAYER_BGGR:
            return sensor_msgs::image_encodings::BAYER_BGGR16;
        case XI_CFA_BAYER_GRBG:
            return sensor_msgs::image_encodings::BAYER_GRBG16;
        case XI_CFA_BAYER_GBRG:
            return sensor_msgs::image_encodings::BAYER_GBRG16;
        }
        break;
    }
    return ""; // unknown
}


XimeaROSCam::XimeaROSCam(const rclcpp::NodeOptions& options) 
        :rclcpp::Node("ximea_camera", options)
{
    // hijack the timer to delay initialising the class which requires shared_from_this()
    xi_open_device_cb_ = create_wall_timer( std::chrono::seconds{0},
            std::bind(&XimeaROSCam::initialize, this));
}

XimeaROSCam::~XimeaROSCam() {
    // Init variables
    XI_RETURN xi_stat;

    RCLCPP_INFO(this->get_logger(), "Shutting down ximea_camera node...");
    // Stop acquisition and close device if handle is available
    if (this->xi_h_ != NULL) {
        // Stop image acquisition
        this->is_active_ = false;
        xi_stat = xiStopAcquisition(this->xi_h_);

        // Close camera device
        xiCloseDevice(this->xi_h_);
        this->xi_h_ = NULL;

        RCLCPP_INFO_STREAM(this->get_logger(), "Closed device: " << this->cam_serialno_);
    }
    RCLCPP_INFO(this->get_logger(), "ximea_camera node shutdown complete.");

    // To avoid warnings
    (void)xi_stat;
}
 
XI_RETURN XimeaROSCam::get(const char* prm, int& value, bool suppress_warn){
    XI_RETURN xi_stat = xiGetParamInt(this->xi_h_, prm, &value);
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiGetParamInt " << prm << " returned " << xi_stat);
    return xi_stat;
}
XI_RETURN XimeaROSCam::get(const char* prm, uint64_t& value, bool suppress_warn){
    uint32_t size = sizeof(value);
    XI_PRM_TYPE type = xiTypeInteger64;
    XI_RETURN xi_stat = xiGetParam(this->xi_h_, prm, &value, &size, &type);
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiGetParam type xiTypeInteger64 " << prm << " returned " << xi_stat);
    return xi_stat;
}
XI_RETURN XimeaROSCam::get(const char* prm, float& value, bool suppress_warn){
    XI_RETURN xi_stat = xiGetParamFloat(this->xi_h_, prm, &value);
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiGetParamFloat " << prm << " returned " << xi_stat);
    return xi_stat;
}
XI_RETURN XimeaROSCam::get(const char* prm, std::string& value, bool suppress_warn){
    char buf[256];
    XI_RETURN xi_stat = xiGetParamString(this->xi_h_, prm, buf, sizeof(buf));
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiGetParamString " << prm << " returned " << xi_stat);
    else
        value = std::string(buf);       
    return xi_stat;
}

XI_RETURN XimeaROSCam::set(const char* prm, int value, bool suppress_warn){
    XI_RETURN xi_stat = xiSetParamInt(this->xi_h_, prm, value);
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiSetParamInt " << prm << " returned " << xi_stat);
    return xi_stat;
}
XI_RETURN XimeaROSCam::set(const char* prm, float value, bool suppress_warn){
    XI_RETURN xi_stat = xiSetParamFloat(this->xi_h_, prm, value);
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiSetParamFloat " << prm << " returned " << xi_stat);
    return xi_stat;
}
XI_RETURN XimeaROSCam::set(const char* prm, const std::string& value, bool suppress_warn){
    XI_RETURN xi_stat = xiSetParamString(this->xi_h_, prm, 
            const_cast<char*>(value.c_str()), value.size()); // cheeky const cast!
    if(xi_stat != XI_OK && !suppress_warn)
        RCLCPP_WARN_STREAM(this->get_logger(), "xiSetParamString " << prm << " returned " << xi_stat);
    return xi_stat;
}


// onInit() - on the initialization of the nodelet (not the class)
void XimeaROSCam::initialize() {
    // Report start of function
    RCLCPP_INFO(this->get_logger(), "Initializing Camera Node ... ");

    // Camera initialization
    this->initCam();

    // Diagnostics
    this->initDiagnostics();

    // Publishers and Subscriptions
    this->initPubs();

    // Timer callback for camera capture (after camera is initialized)
    this->initTimers();

    // Initialize image storage parameters
    this->initStorage();

    // Report end of function
    RCLCPP_INFO(this->get_logger(), "... Camera Node Initialized. Waiting for Input...");
}

// initNodeHandles() - initialize the private/public node handles
void XimeaROSCam::initDiagnostics() 
{
    if (enable_diagnostics_) 
    {
        diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
        diag_updater_->setHardwareID(cam_name_);

        frequency_min_ = pub_frequency_ - pub_frequency_tolerance_;
        frequency_max_ = pub_frequency_ + pub_frequency_tolerance_;
        cam_pub_diag_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
                std::string{this->get_namespace()} + "/image_raw",
                *diag_updater_,
                diagnostic_updater::FrequencyStatusParam( &frequency_min_, &frequency_max_, 0.0, 20),
                diagnostic_updater::TimeStampStatusParam( age_min_, age_max_));
    }
}

// initPubs() - initialize the publishers
void XimeaROSCam::initPubs() {
    // Report start of function
    RCLCPP_INFO(this->get_logger(), "Loading Publishers ... ");

    this->cam_img_counter_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
            "image_count", 10);

    this->publish_xi_image_info_ = this->declare_parameter("publish_xi_image_info", false);
    RCLCPP_INFO_STREAM(this->get_logger(), "publish_xi_image_info: " << this->publish_xi_image_info_);

    if(this->publish_xi_image_info_) {
      this->cam_xi_image_info_pub_ =
        this->create_publisher<ximea_camera_interfaces::msg::XiImageInfo>(
          "xi_image_info", 10);
    }

    // Report end of function
    RCLCPP_INFO(this->get_logger(), "... Publishers Loaded. ");
}

// initTimers() - initialize the timers
void XimeaROSCam::initTimers() {
    // Report start of function
    RCLCPP_INFO(this->get_logger(), "Loading Timers ... ");

    // Load camera polling callback timer ((Ensure that with multiple cameras,
    // each time is about 2 seconds spaced apart)
    this->xi_open_device_cb_ =
        this->create_wall_timer(
            std::chrono::duration<double>(this->poll_time_),
            std::bind(&XimeaROSCam::openDeviceCb, this));
    RCLCPP_INFO(this->get_logger(), "xi_open_device_cb_ created");

    // Load camera frame capture callback timer
    this->t_frame_cb_ =
        this->create_wall_timer(
            std::chrono::duration<double>(this->poll_time_frame_),
            std::bind(&XimeaROSCam::frameCaptureCb, this));
    RCLCPP_INFO(this->get_logger(), "t_frame_cb_ created");

    // Report end of function
    RCLCPP_INFO(this->get_logger(), "... Timers Loaded.");
}

void XimeaROSCam::initStorage() {
    RCLCPP_INFO(this->get_logger(), "Loading Image Storage ... ");

    this->image_directory_ = this->declare_parameter("image_directory", std::string("NO_PATH"));
    RCLCPP_INFO_STREAM(this->get_logger(), "image_directory: " << this->image_directory_);
    
    this->calib_mode_ = this->declare_parameter("calib_mode", false);
    RCLCPP_INFO_STREAM(this->get_logger(), "calib_mode_: " << this->calib_mode_);


    // Initialize directory paths
    namespace fs = std::filesystem;
    fs::path main_dir {this->image_directory_};

    if (this->calib_mode_) {
        this->trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "camera/save_image", 10,
            std::bind(&XimeaROSCam::triggerCb, this, std::placeholders::_1));

        // directory that holds calibration images
        fs::path calib_dir = main_dir / (this->cam_name_ + "/calib/");
        this->png_path_ = calib_dir.string();
        if (!fs::create_directories(calib_dir))
        {
            // failed to create directory, exit ROS and explain.
            RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: unable to create directory: " << this->png_path_);
            RCLCPP_ERROR_STREAM(this->get_logger(), "Please make sure that the image_directory "
                         << "parameter is set to a folder with the proper "
                         << "permissions in the config file.");
            rclcpp::shutdown();
        }

        this->save_trigger_ = false;
    }


    RCLCPP_INFO(this->get_logger(), "Image Storage Loaded.");
}

void XimeaROSCam::initCam() {
    RCLCPP_INFO(this->get_logger(), "Loading Camera Configuration.");

    // Assume that all of the config is embedded in the camera private namespace
    // Load all parameters and store them into their corresponding vars

    //      -- apply camera name --
    this->cam_name_ = this->declare_parameter("cam_name", std::string("INVALID"));
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_name: " << this->cam_name_);
    //      -- apply camera specific parameters --
    this->cam_serialno_ = this->declare_parameter("serial_no", std::string());
    RCLCPP_INFO_STREAM(this->get_logger(), "serial number: " << this->cam_serialno_);
    this->cam_user_id_ = this->declare_parameter("user_id", std::string());
    RCLCPP_INFO_STREAM(this->get_logger(), "user id: " << this->cam_user_id_);
    this->cam_frameid_ = this->declare_parameter("frame_id", std::string("camera"));
    RCLCPP_INFO_STREAM(this->get_logger(), "frame id: " << this->cam_frameid_);
    this->cam_calib_file_ = this->declare_parameter("calib_file", std::string("INVALID"));
    RCLCPP_INFO_STREAM(this->get_logger(), "calibration file: " << this->cam_calib_file_);
    this->poll_time_ = this->declare_parameter("poll_time", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "poll_time: " << this->poll_time_);
    this->poll_time_frame_ = this->declare_parameter("poll_time_frame", 0.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "poll_time_frame: " << this->poll_time_frame_);

    // Diagnostics
    this->enable_diagnostics_ = this->declare_parameter("enable_diagnostics", true);
    RCLCPP_INFO_STREAM(this->get_logger(), "enable_diagnostics: " << this->enable_diagnostics_);
    this->pub_frequency_ = this->declare_parameter("pub_frequency", 10.0);
    RCLCPP_INFO_STREAM(this->get_logger(), "pub_frequency: " << this->pub_frequency_);
    this->pub_frequency_tolerance_ = this->declare_parameter("pub_frequency_tolerance", 0.3);
    RCLCPP_INFO_STREAM(this->get_logger(), "pub_frequency_tolerance: " << this->pub_frequency_tolerance_);
    this->age_max_ = this->declare_parameter("data_age_max", 0.1);
    RCLCPP_INFO_STREAM(this->get_logger(), "data_age_max: " << this->age_max_);

    //      -- apply image format parameters --
    this->declare_parameter("format", std::string{});

    //      -- apply bandwidth parameters --
    this->cam_num_in_bus_ = this->declare_parameter("num_cams_in_bus", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_num_in_bus_: " << this->cam_num_in_bus_);
    this->cam_bw_safetyratio_ = this->declare_parameter("bw_safetyratio", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_bw_safetyratio_: " << this->cam_bw_safetyratio_);

    //      -- apply triggering parameters --
    this->cam_trigger_mode_ = this->declare_parameter("cam_trigger_mode", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_trigger_mode_: " << this->cam_trigger_mode_);
    this->cam_hw_trigger_edge_ = this->declare_parameter("hw_trigger_edge", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_hw_trigger_edge_: " << this->cam_hw_trigger_edge_);

    //      -- apply framerate (software cap) parameters --
    this->cam_framerate_control_ = this->declare_parameter("frame_rate_control", false);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_framerate_control_: " << this->cam_framerate_control_);
    this->cam_framerate_set_ = this->declare_parameter("frame_rate_set", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_framerate_set_: " << this->cam_framerate_set_);
    this->cam_img_cap_timeout_ = this->declare_parameter("img_capture_timeout", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_img_cap_timeout_: " << this->cam_img_cap_timeout_);

    //      -- apply exposure parameters --
    this->cam_autoexposure_ = this->declare_parameter("auto_exposure", false);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_autoexposure_: " << this->cam_autoexposure_);
    this->cam_manualgain_ = this->declare_parameter("manual_gain", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_manualgain_: " << this->cam_manualgain_);
    this->cam_exposure_time_ = this->declare_parameter("exposure_time", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_exposure_time_: " << this->cam_exposure_time_);
    this->cam_autoexposure_priority_ = this->declare_parameter("auto_exposure_priority", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_autoexposure_priority_: "
        << this->cam_autoexposure_priority_);
    this->cam_autotime_limit_ = this->declare_parameter("auto_time_limit", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_autotime_limit_: " << this->cam_autotime_limit_);
    this->cam_autogain_limit_ = this->declare_parameter("auto_gain_limit", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_autogain_limit_: " << this->cam_autogain_limit_);

    //      -- apply white balance parameters --
    this->cam_white_balance_mode_ = this->declare_parameter("white_balance_mode", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_white_balance_mode_: "
        << this->cam_white_balance_mode_);
    this->cam_white_balance_coef_r_ = this->declare_parameter("white_balance_coef_red", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_white_balance_coef_r_: "
        << this->cam_white_balance_coef_r_);
    this->cam_white_balance_coef_g_ = this->declare_parameter("white_balance_coef_green", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_white_balance_coef_g_: "
        << this->cam_white_balance_coef_g_);
    this->cam_white_balance_coef_b_ = this->declare_parameter("white_balance_coef_blue", -1.0f);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_white_balance_coef_b_: "
        << this->cam_white_balance_coef_b_);

    //      -- apply ROI parameters --
    this->cam_roi_left_ = this->declare_parameter("roi_left", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_left_: " << this->cam_roi_left_);
    this->cam_roi_top_ = this->declare_parameter("roi_top", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_top_: " << this->cam_roi_top_);
    this->cam_roi_width_ = this->declare_parameter("roi_width", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_width_: " << this->cam_roi_width_);
    this->cam_roi_height_ = this->declare_parameter("roi_height", -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_height_: " << this->cam_roi_height_);

    // Other basic init values
    this->is_active_ = false;
    this->xi_h_ = NULL;

    // Setup image transport (publishing) and camera info topics
    image_transport::ImageTransport it(this->shared_from_this());
    this->cam_pub_ = it.advertise("image_raw", 1);

    // only load and publish calib file if it isn't empty
    // assume camera info is not loaded
    // Setup camera info manager for calibration
    this->cam_info_loaded_ = false;

    this->cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
                    this, this->cam_name_); // this constructor is deprecated in ROS2 rolling
                    // this->get_node_base_interface(), 
                    // this->get_node_services_interface(),
                    // this->get_node_logging_interface(),
                    // this->cam_name_, "", rclcpp::SystemDefaultsQoS(), "~"); // preferred rolling constructor

    if (this->cam_info_manager_->loadCameraInfo(this->cam_calib_file_)) {
        this->cam_info_loaded_ = true;
    }
    // loaded camera info properly
    if (this->cam_info_loaded_) {
        // advertise
        this->cam_info_pub_ =
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info", 10);
    }

    // Enable auto bandwidth calculation to ensure bandwidth limiting and
    // framerate setting are supported
    xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);
}

void XimeaROSCam::openCam() {

    // Init variables
    XI_RETURN xi_stat;

    // leave if there isn't a valid handle
    if (this->xi_h_ == NULL) { return; }

    // set the ximea debug level
    set(XI_PRM_DEBUG_LEVEL, XI_DL_FATAL);//XI_DL_ERROR);//XI_DL_WARNING);//

    // wrangle the image format
    auto format_name = get_parameter("format").as_string();
    if(auto it = PixelFormatMap.find(format_name); it != PixelFormatMap.end() && 
            set(XI_PRM_IMAGE_DATA_FORMAT, it->second) == XI_OK) 
    {
        this->pixel_format_ = it->second;
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting image format to: \"" << format_name << "\" (" <<
                this->pixel_format_ << ")");
    }
    else 
    {
        get(XI_PRM_IMAGE_DATA_FORMAT, this->pixel_format_);
        RCLCPP_WARN_STREAM(this->get_logger(), "\"" << format_name << 
                "\" not supported. Using current format: " << this->pixel_format_);
    }

    this->image_encoding_ = getImageEncoding();
    RCLCPP_INFO_STREAM(this->get_logger(), "image encoding: " << this->image_encoding_);

    //get(XI_PRM_IMAGE_PAYLOAD_SIZE, this->image_size_in_bytes_);
    get(XI_PRM_IMAGE_DATA_BIT_DEPTH, this->image_bit_depth_);

    this->setWhiteBalance();
    this->setTrigger();
    this->setExposure();
    this->setRegion();
    this->setBandwidth();
    this->setFramerate();
    
    // attempt to sample the camera clock  
    this->sampleCameraTimestamp();
    if(camera_timestamp_supported_)
        RCLCPP_INFO(this->get_logger(), "Using camera timestamp.");
    else
        RCLCPP_WARN(this->get_logger(), "Camera timestamp not supported. Using time of arrival.");


    //      -- Optimize transport buffer commit/size based on payload  --
    // // For usb controllers that can handle it...
    // src: https://www.ximea.com/support/wiki/apis/Linux_USB30_Support
    // xiSetParamInt(handle, XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT, 32);
    // xiGetParamInt( handle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_MAX,
    //  &buffer_size);
    // xiSetParamInt(handle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, buffer_size);

    // // For high frame rate performance
    // src: https://www.ximea.com/support/wiki/usb3/...
    //      ...How_to_optimize_software_performance_on_high_frame_rates
    
    //      -- Start camera acquisition --      
    xi_stat = xiStartAcquisition(this->xi_h_);
    if(xi_stat != XI_OK) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to start acquisition. "
                "xiStartAcquisition returned " << xi_stat);          
        this->is_active_ = false; 
    } else {
        RCLCPP_INFO(this->get_logger(), "Starting Acquisition...");   
        this->is_active_ = true; 
    }
}



void XimeaROSCam::openDeviceCb() {
    XI_RETURN xi_stat;

    if(!this->cam_serialno_.empty()) 
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Opening camera by serial number: " << this->cam_serialno_);
        xi_stat = xiOpenDeviceBy(XI_OPEN_BY_SN,
                this->cam_serialno_.c_str(),
                &this->xi_h_);
    }
    else if(!this->cam_user_id_.empty()) 
    {   // user id currenty supported on xiB, xiC, xiT & xiX
        RCLCPP_INFO_STREAM(this->get_logger(), "Opening camera by user id: " << this->cam_user_id_);
        xi_stat = xiOpenDeviceBy(XI_OPEN_BY_USER_ID,
                this->cam_user_id_.c_str(),
                &this->xi_h_);
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "No serial number or user id provided. Opening first camera on bus.");
        xi_stat = xiOpenDevice(0, &this->xi_h_);
    }
    

    if (xi_stat == XI_OK && this->xi_h_ != NULL) 
    {  
        if(this->cam_serialno_.empty()) 
            get(XI_PRM_DEVICE_SN, this->cam_serialno_);
        // if(this->user_id_.empty())  
        //     get(XI_PRM_DEVICE_USER_ID, this->user_id_);  
        RCLCPP_INFO_STREAM(this->get_logger(), "Successfully opened camera. Serial number: "
                        << this->cam_serialno_);
        this->xi_open_device_cb_->cancel(); 

        XimeaROSCam::openCam();
    }   
}

// Start aquiring data
void XimeaROSCam::frameCaptureCb()
{
    if (!this->is_active_)
        return;

    XI_IMG xi_img = {};
    std::string time_str;

    xi_img.size = sizeof(XI_IMG);
    xi_img.bp = NULL;
    xi_img.bp_size = 0;

    // Acquire image
    XI_RETURN xi_stat = xiGetImage(this->xi_h_, this->cam_img_cap_timeout_, &xi_img);
        
    // Add timestamp
    rclcpp::Time timestamp = iterpolateTimestamp(xi_img);

    // Was the image retrieval successful?
    if (xi_stat == XI_OK) 
    {   
        RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Capturing image from Ximea camera serial no: "
                << this->cam_serialno_
                << ". WxH: "
                << xi_img.width
                << " x "
                << xi_img.height << ".");

        // Setup image
        auto img_buffer = static_cast<char *>(xi_img.bp);

        sensor_msgs::msg::Image img;
        img.header.frame_id = cam_frameid_;
        img.header.stamp = timestamp;
        sensor_msgs::fillImage(img,
                image_encoding_,
                xi_img.height,
                xi_img.width,
                xi_img.width * (image_bit_depth_ / 8u) + xi_img.padding_x,
                img_buffer);

        // Publish image
        cam_pub_.publish(img);

        if (enable_diagnostics_) {
            cam_pub_diag_->tick(timestamp);
            //diag_updater.update(); // DISABLED WHILE UPGRADING TO ROS2
        }

        // Publish camera calibration info if camera info is loaded
        if (cam_info_loaded_) 
        {
            sensor_msgs::msg::CameraInfo cam_info = cam_info_manager_->getCameraInfo();
                // reset frame id
            cam_info.header.frame_id = cam_frameid_;
            cam_info.header.stamp = timestamp;
            cam_info_pub_->publish(cam_info);
        }

        // Publish image counter
        // Note that header.seq does this, but it is depreciated and
        // will be removed in ROS 2. Therefore here we did this instead.
        std_msgs::msg::UInt32 icount;
        img_count_++;                 // increment
        icount.data = img_count_;
        cam_img_counter_pub_->publish(icount);
        

        // Compress and save images if triggered and in calibration mode
        if (this->save_trigger_ && this->calib_mode_) 
        {
            this->save_trigger_ = false;

            time_str = this->formatTimeString(timestamp);

            if (this->image_directory_ != std::string("NO_PATH")) {
                std::string png_path = this->png_path_ + time_str + "_" +
                                        this->cam_name_ + ".png";

                if (this->saveOnTrigger(img_buffer,
                                        xi_img.height,
                                        xi_img.width,
                                        png_path)) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Saved image to: " << png_path);
                }
                else {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Failed to save image: " << png_path);
                }
            }
            else {
                RCLCPP_INFO_STREAM(this->get_logger(), "Directory path not set!");
            }
        }
    }
    else 
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "xiGetImage failed with error: " << xi_stat);
    }
    

    // If active, publish xiGetImage info to ROS message
    if(this->publish_xi_image_info_) 
    {
        ximea_camera_interfaces::msg::XiImageInfo xiImageInfoMsg;
        xiImageInfoMsg.header.frame_id = this->cam_frameid_;
        xiImageInfoMsg.header.stamp = timestamp;
        xiImageInfoMsg.size = xi_img.size;
        xiImageInfoMsg.bp_size = xi_img.bp_size;
        xiImageInfoMsg.frm = xi_img.frm;
        xiImageInfoMsg.width = xi_img.width;
        xiImageInfoMsg.height = xi_img.height;
        xiImageInfoMsg.nframe = xi_img.nframe;
        xiImageInfoMsg.ts_sec = xi_img.tsSec;
        xiImageInfoMsg.ts_usec = xi_img.tsUSec;
        xiImageInfoMsg.gpi_level = xi_img.GPI_level;
        xiImageInfoMsg.black_level = xi_img.black_level;
        xiImageInfoMsg.padding_x = xi_img.padding_x;
        xiImageInfoMsg.absolute_offset_x = xi_img.AbsoluteOffsetX;
        xiImageInfoMsg.absolute_offset_y = xi_img.AbsoluteOffsetY;
        xiImageInfoMsg.exposure_time_us = xi_img.exposure_time_us;
        xiImageInfoMsg.gain_db = xi_img.gain_db;
        xiImageInfoMsg.acq_nframe = xi_img.acq_nframe;
        xiImageInfoMsg.image_user_data = xi_img.image_user_data;
        // xiGetImageMsg.exposure_sub_times_us = (unsigned int) xi_img.exposure_sub_times_us;
        this->cam_xi_image_info_pub_->publish(xiImageInfoMsg);
    }
    

    // // To avoid warnings
    // (void)xi_stat;
}


// Save images on trigger with PNG compression
bool XimeaROSCam::saveOnTrigger(char *img_buffer,
                                        int img_h,
                                        int img_w,
                                        std::string filename) {
    cv::Mat cv_mat = cv::Mat(img_h, img_w, CV_8UC3, img_buffer);
    // Use PNG compression and least amount of compression possible
    // PNG compression scales from 0 (least) to 9 (most)
    std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 0};

    if (!cv::imwrite(filename, cv_mat, compression_params)) {
        return false;
    }
    return true;
}

// Format ROS timestamp into desired string format
// Format is YYYYMMDD_HHMMSS_uS
std::string XimeaROSCam::formatTimeString(rclcpp::Time timestamp) {
// DISABLED WHILE UPGRADING TO ROS2
    // boost::posix_time::time_facet *facet = new boost::posix_time::time_facet();
    // // Format is YYYYMMDD_HHMMSS_fractionalSeconds
    // facet->format("%Y%m%d_%H%M%S_%f");

    // std::stringstream stream;
    // stream.imbue(std::locale(std::locale::classic(), facet));
    // stream << timestamp;

    // std::string formatted_time = stream.str();
    // boost::erase_all(formatted_time, "."); // remove decimal

    //return formatted_time;

    std::stringstream ss;
    ss << timestamp.nanoseconds();
    return ss.str();
}

// Set save_trigger_ flag
void XimeaROSCam::triggerCb(const std_msgs::msg::Empty::SharedPtr msg) {
    this->save_trigger_ = true;

    // To avoid warnings
    (void)msg;
}

void XimeaROSCam::setWhiteBalance(){
    //      -- White balance mode --
    // Note: Setting XI_PRM_MANUAL right before or after setting coeffs
    // actually overrides the coefficients! This is because calculating
    // the manual coeffs takes time, so when the coefficients are set,
    // they will be overwritten once the manual coeff values are calculated.
    // This also is the same when XI_PRM_MANUAL is set to 0 as well.
    if(this->cam_white_balance_mode_ == 2)
    {
        if(set(XI_PRM_AUTO_WB, 1)) 
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to enable auto white balance.");
        else    
            RCLCPP_INFO_STREAM(this->get_logger(), "Auto white balance enabled.");
           
    }
    else if(this->cam_white_balance_mode_ == 1)
    {
        if(     set(XI_PRM_AUTO_WB, 0) |
                set(XI_PRM_WB_KR, this->cam_white_balance_coef_r_) |
                set(XI_PRM_WB_KG, this->cam_white_balance_coef_g_) |
                set(XI_PRM_WB_KB, this->cam_white_balance_coef_b_))
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set white balance user coefficients.");
        else    
            RCLCPP_INFO_STREAM(this->get_logger(), "White balance set to use user coefficients.");
    }
    else //if(this->cam_white_balance_mode_ == 0)
    {
        if(this->cam_white_balance_mode_ != 0)
            RCLCPP_WARN_STREAM(this->get_logger(), "Unknown white balance mode. Defaulting to disabled.");
        if(set(XI_PRM_AUTO_WB, 0)) 
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to disable white balance.");
        else    
            RCLCPP_INFO_STREAM(this->get_logger(), "White balance disabled.");
    }
}

void XimeaROSCam::setTrigger(){
    if (this->cam_trigger_mode_ == 2) // hardware trigger
    {
        int trigger_edge = (this->cam_hw_trigger_edge_ == 0)?
                XI_TRG_EDGE_RISING : XI_TRG_EDGE_FALLING;

        if(     set( XI_PRM_TRG_SOURCE, trigger_edge) | 
                set(XI_PRM_GPI_SELECTOR, 1) |
                set(XI_PRM_GPI_MODE, XI_GPI_TRIGGER))
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set hardware trigger.");
        else
            RCLCPP_INFO_STREAM(this->get_logger(), "Hardware trigger enabled.");            
    } 
    else if (this->cam_trigger_mode_ == 1) // software trigger
    { 
        if(set(XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE))
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set software trigger."); 
        else    
            RCLCPP_WARN_STREAM(this->get_logger(), "WARNING Software trigger enabled but not fully implemented."); 
    } 
    else // disable triggering  
    { 
        if(set( XI_PRM_TRG_SOURCE, XI_TRG_OFF))
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to disable triggering."); 
        else    
            RCLCPP_INFO_STREAM(this->get_logger(), "Triggering disabled."); 
    }    
}
void XimeaROSCam::setExposure(){
    if (this->cam_autoexposure_) { // set auto exposure
        if(     set( XI_PRM_AEAG, 1) |
                set( XI_PRM_EXP_PRIORITY, this->cam_autoexposure_priority_) |
                set( XI_PRM_AE_MAX_LIMIT, this->cam_autotime_limit_) |
                set( XI_PRM_AG_MAX_LIMIT, this->cam_autogain_limit_))                
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set exposure to auto.");                   
        else
            RCLCPP_INFO_STREAM(this->get_logger(), "Exposure set to auto." 
                    " Exposure priority: " << this->cam_autoexposure_priority_ << 
                    "  Max exposure: " << this->cam_autotime_limit_ << 
                    "  Max gain: " << this->cam_autogain_limit_); 
    } else { // set manual exposure       
        if(     set( XI_PRM_AEAG, 0) |
                set( XI_PRM_EXPOSURE, this->cam_exposure_time_) |
                set( XI_PRM_GAIN, this->cam_manualgain_))
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set exposure to manual.");
        else
            RCLCPP_INFO_STREAM(this->get_logger(), "Exposure set to manual." 
                    " Exposure: " << this->cam_exposure_time_ << 
                    "  Gain: " << this->cam_manualgain_); 
    }

}

void XimeaROSCam::setRegion(void){
    // alias the region parameters for brevity
    int& x = this->cam_roi_left_;
    int& y = this->cam_roi_left_;
    int& width = this->cam_roi_width_;
    int& height = this->cam_roi_height_;

    // check region mode? 
    //(xiGetParamInt(this->xi_h_, XI_PRM_REGION_SELECTOR, &region_selector) != XI_OK) | // no short circuit! 
    // (xiGetParamInt(this->xi_h_, XI_PRM_REGION_MODE, &region_mode) != XI_OK) | 

    // get the min max bounds and increment for each region parameters
    int min_x = 0, max_x = 0, inc_x = 0;
    int min_y = 0, max_y = 0, inc_y = 0;
    int min_width = 0, max_width = 0, inc_width = 0;
    int min_height = 0, max_height = 0, inc_height = 0;
    if(     get(XI_PRM_OFFSET_X XI_PRM_INFO_MIN, min_x) | // no short circuit! 
            get(XI_PRM_OFFSET_X XI_PRM_INFO_MAX, max_x) |
            get(XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, inc_x) |
            get(XI_PRM_OFFSET_Y XI_PRM_INFO_MIN, min_y) |
            get(XI_PRM_OFFSET_Y XI_PRM_INFO_MAX, max_y) |
            get(XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, inc_y) |
            get(XI_PRM_WIDTH XI_PRM_INFO_MIN, min_width) |
            get(XI_PRM_WIDTH XI_PRM_INFO_MAX, max_width) |
            get(XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, inc_width) |
            get(XI_PRM_HEIGHT XI_PRM_INFO_MIN, min_height) |
            get(XI_PRM_HEIGHT XI_PRM_INFO_MAX, max_height) |
            get(XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, inc_height))
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to get one or more region of interest parameters");
    } 
    else  // clamp to region bounds setting any negative parameters to default values
    {   
        x = std::clamp(roundDown((x<0? min_x:x), inc_x), min_x, max_x);
        y =  std::clamp(roundDown((y<0? min_y:y), inc_y), min_y, max_y);
        width = std::clamp(roundUp((width<0? max_width:width), inc_width), min_width, max_width - x);
        height = std::clamp(roundUp((height<0? max_height:height), inc_height), min_height, max_height - y);
    }

    // attempt to set 
    if(     set( XI_PRM_OFFSET_X, 0) | // not sure if required
            set( XI_PRM_OFFSET_Y, 0) |  
            set( XI_PRM_WIDTH, width) | // no short circuit!  
            set( XI_PRM_HEIGHT, height) |
            set( XI_PRM_OFFSET_X, x) |  
            set( XI_PRM_OFFSET_Y, y))          
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to set one or more region of interest parameters");
    }    
    // attempt to get 
    if(     get( XI_PRM_OFFSET_X, x) | // no short circuit! 
            get( XI_PRM_OFFSET_Y, y) |
            get( XI_PRM_WIDTH, width) | 
            get( XI_PRM_HEIGHT, height))          
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to get one or more region of interest parameters");
    }   
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera region: {" << x << ", " << y << "} - {" << width << ", " << height << "}");   
}

void XimeaROSCam::setBandwidth(void){
    // Compute available bandwidth for this camera
    int available_bandwidth = 0;           // Mbits per second
    if(get(XI_PRM_AVAILABLE_BANDWIDTH, available_bandwidth)){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Available bandwidth not supported.");
        return;
    }

    // If we have more than one camera per bus/controller, we divide the
    // available bw to accommodate for the same amount of cameras
    if(this->cam_num_in_bus_ > 1) 
       available_bandwidth /= this->cam_num_in_bus_;

    // apply the safety ratio
    if(this->cam_bw_safetyratio_ > 0.0)
       available_bandwidth = int(available_bandwidth * this->cam_bw_safetyratio_);

    // clamp to reported limits 
    int min_bandwidth=0, max_bandwidth=0;
    if(get( XI_PRM_LIMIT_BANDWIDTH XI_PRM_INFO_MIN, min_bandwidth) == XI_OK &&
                get( XI_PRM_LIMIT_BANDWIDTH XI_PRM_INFO_MAX, max_bandwidth) == XI_OK)
    {                
        available_bandwidth = std::clamp(available_bandwidth, min_bandwidth, max_bandwidth);
    }

    // Set bandwidth limit 
    if(set( XI_PRM_LIMIT_BANDWIDTH, available_bandwidth)){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to limit bandwidth to " <<  
                available_bandwidth << " Mbits/sec.");
    } else {        
        set( XI_PRM_LIMIT_BANDWIDTH_MODE, XI_ON, true); // not supported on MQ, MU, MD and MR models
        get( XI_PRM_LIMIT_BANDWIDTH, available_bandwidth ); // read back         
        RCLCPP_INFO_STREAM(this->get_logger(), "Bandwidth limited to " << 
                available_bandwidth << " Mbits/sec.");
    }
}

void XimeaROSCam::setFramerate(void){
    //      -- Framerate control  --
    // For information purposes, obtain min and max calculated possible fps
    float framerate = this->cam_framerate_set_; // as float
    if(     get( XI_PRM_FRAMERATE XI_PRM_INFO_MIN, this->min_fps_) == XI_OK ||
            get( XI_PRM_FRAMERATE XI_PRM_INFO_MAX, this->max_fps_) == XI_OK ) // max fps often returns as 0
        RCLCPP_INFO_STREAM(this->get_logger(), "Reported framerate limits. Min: " << this->min_fps_ << "  Max: " << this->max_fps_ ); 

    // If we are not in trigger mode, determine if we want to limit fps
    if (this->cam_trigger_mode_ == 0) {
        if (this->cam_framerate_control_) {
            // frame limit mode settings differ over camera models 
            // XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT is supported on CB, MC, MT, MX 
            // XI_ACQ_TIMING_MODE_FRAME_RATE is supported on MQ, MD
            // to cover for future models rather than check try both 
            if( set( XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT, true)  && 
                    set( XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE ) ) { 
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set frame rate limit mode.");
            } else if(set( XI_PRM_FRAMERATE, framerate )) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set frame rate of " 
                        << framerate << "Hz.");
            } else {
                get( XI_PRM_FRAMERATE, framerate );
                RCLCPP_INFO_STREAM(this->get_logger(), "Frame rate set to "
                         << framerate << "Hz.");                           
            }
        } else { // default to free run
            if(set( XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN))
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set frame rate to free run.");
            else  
                RCLCPP_INFO_STREAM(this->get_logger(), "Frame rate set to free run.");  
        }
    }

}

void XimeaROSCam::sampleCameraTimestamp(void){
    uint64_t nanoseconds;
    rclcpp::Time pre_stamp = this->get_clock()->now();
    camera_timestamp_supported_ = get(XI_PRM_TIMESTAMP, nanoseconds, true) == XI_OK;
    rclcpp::Time post_stamp = this->get_clock()->now();

    if(!camera_timestamp_supported_)        
        return;       

    
    timestamp_queue_.emplace_back(std::make_pair( pre_stamp, std::chrono::nanoseconds(nanoseconds) ));

    // RCLCPP_INFO_STREAM(this->get_logger(), "Timestamps:" << 
    //         " System: " << timestamp_queue_.back().first << 
    //         " Camera: " << timestamp_queue_.back().second );

    // monitor round trip speed
    rclcpp::Duration round_trip_duration(post_stamp - pre_stamp);
    if(round_trip_duration > rclcpp::Duration(std::chrono::microseconds{2500})) // arbitary 2.5 ms 
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Camera timestamp query took " << round_trip_duration.seconds() << " seconds." );    
}

rclcpp::Time XimeaROSCam::iterpolateTimestamp(const XI_IMG& frame){
    if(!camera_timestamp_supported_)  
        return this->get_clock()->now();

    sampleCameraTimestamp();

    std::deque< std::pair<rclcpp::Time,rclcpp::Duration> >& queue = timestamp_queue_; // alias 
    if(queue.size() < 2)
    {  
        RCLCPP_ERROR(this->get_logger(),"Not enough timestamp samples to interpolate.");
        return this->get_clock()->now();
    }

    rclcpp::Duration camera_stamp(frame.tsSec, frame.tsUSec * 1000);
    // TODO adjust camera stamp to be the start of exposure (model dependant)
    // consider using a precalculated bool to determin if exposure time should
    // be deducted  
    
    bool clock_wrap = queue.front().second > queue.back().second; 
    if(clock_wrap)
    {
        RCLCPP_WARN(this->get_logger(),"Clock wrap detected. Timestamp handling experimetal!");
        // xiQ, xiD 
        // 40 bit microsecond number - (overlaps after 305 hours)
        // TimeStamp is recorded at the start of Data Readout 

        // xiC, xiB, xiX, xiT 
        // 64 bit 4-nanoseconds number (overlaps after 2339 years) (sic)
        // TimeStamp is recorded at the start of Exposure.
        // It is assumed that these cameras use the full tsSec value 

        // untested experimental handling!
        rclcpp::Duration wrap_period{ std::chrono::nanoseconds{}};
        rclcpp::Duration rough_wrap_period = 
                (queue.front().second - queue.back().second) + 
                (queue.back().first - queue.front().first);

        const uint64_t million = 1000000;
        const uint64_t forty_bits = uint64_t(1) << 40;
        const rclcpp::Duration forty_bit_period(forty_bits / million, forty_bits % million);
        const rclcpp::Duration ros_duration_period(1<<31,0); // ros duration overflow

        if(std::fabs((rough_wrap_period - forty_bit_period).seconds()) < 1.0) 
            wrap_period = forty_bit_period; // ~305 hours
        else if(std::fabs((rough_wrap_period - ros_duration_period).seconds()) < 1.0)  
            wrap_period = rclcpp::Duration(1<<31,0); // ~68 years (worlds longest running camera)
        else
            wrap_period = rough_wrap_period;
         
        RCLCPP_WARN_STREAM(this->get_logger(), 
                "Adjusting timestamps using a period of " << wrap_period.seconds() << " seconds." );
        using iterator = std::deque< std::pair<rclcpp::Time,rclcpp::Duration> >::iterator;
        for(iterator it = queue.begin(); it != queue.end(); ++it)
            if(it->second > queue.back().second)
                it->second = it->second - wrap_period;
        if(camera_stamp > queue.back().second)
            camera_stamp = camera_stamp - wrap_period;    
    }    

    while(queue.size() > 2 && queue[1].second < camera_stamp)
        queue.pop_front();

    // interpolate stamp
    double scalar = (camera_stamp - queue[0].second).seconds() /
            (queue[1].second - queue[0].second).seconds();
    rclcpp::Time system_stamp = queue[0].first + 
            (queue[1].first - queue[1].first) * scalar;
   
    return system_stamp;
}
    

} // NAMESPACE ximea_camera

// Register the component with rclcpp
RCLCPP_COMPONENTS_REGISTER_NODE(ximea_camera::XimeaROSCam)
