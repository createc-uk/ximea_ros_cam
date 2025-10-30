#ifndef XIMEA_CAMERA_XIMEA_CAMERA_HPP
#define XIMEA_CAMERA_XIMEA_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <ximea_camera_interfaces/msg/xi_image_info.hpp>

#include <m3api/xiApi.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <deque>
#include <map>


namespace ximea_camera {

class XimeaROSCam : public rclcpp::Node // , std::enable_shared_from_this<XimeaROSCam>
{
public:
    // Node Constructor
    explicit XimeaROSCam(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Node Destructor
    ~XimeaROSCam();  // destructor

private:
    /**
     * @brief Initialize the camera node
     *
     * Initialize the camera node. Contains all initialization functions
     * to define parameters, subscriptions, publishers, etc...
     *
     */
    void initialize();

    /**
     * @brief  Initialize diagnostics.
     *
     * Initialize diagnostics and its parameters.
     */
    void initDiagnostics();

    /**
     * @brief  Initialize all publishers.
     *
     * Initialize all publishers.
     */
    void initPubs();

    /**
     * @brief  Initialize all timers.
     *
     * Initialize all timers.
     */
    void initTimers();

    /**
     * @brief  Initialize image storage parameters.
     *
     * Initialize image storage parameters.
     * Includes directory and compression parameters.
     */
    void initStorage();

    // Camera params and Functions
    void initCam();
    void openCam();
    static std::map<std::string, int> PixelFormatMap;
    std::string_view getImageEncoding();

    // // Get camera lists
    // std::vector<std::string> getCamConfigFiles(std::string cam_list);

    // Capture the camera(s) frames
    std::string cam_config_file_;    // camera config file
    // Camera variable list
    // Inactive variables
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr cam_img_counter_pub_;     // Image counter
    uint32_t img_count_ = 0;                     // Image count
    bool cam_info_loaded_ = false;                    // is camera info loaded?
    std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;   // Cam info manager handle
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;             // Cam info publisher handle
    rclcpp::Publisher<ximea_camera_interfaces::msg::XiImageInfo>::SharedPtr cam_xi_image_info_pub_; // xiGetImage info publisher handle
    // image_transport::ImageTransport cam_it_; // Image transport handle
    image_transport::Publisher cam_pub_;     // Image publisher handle
    // camera
    std::string cam_name_ = "camera";        // Main topic name for cam
    int pixel_format_ = -1;                  // Camera image format int val
    std::string image_encoding_ = {};        // Camera image encoding
    //int image_size_in_bytes_ = {};           // Camera image size in bytes
    int image_bit_depth_ = {};               // Camera image bit depth
    std::string cam_serialno_;               // Camera serial no
    std::string cam_user_id_;
    std::string cam_frameid_;
    float poll_time_;			     // For launching cameras in succession
    float poll_time_frame_;                  // For each image buffer check
    int cam_model_;
    std::string cam_calib_file_;
    int cam_trigger_mode_ = 0;
    int cam_hw_trigger_edge_;
    bool cam_autoexposure_;
    int cam_exposure_time_;
    float cam_manualgain_;
    int cam_autotime_limit_;
    float cam_autoexposure_priority_;
    float cam_autogain_limit_;
    bool cam_binning_en_;
    int cam_downsample_factor_;
    int cam_roi_left_ = 0;
    int cam_roi_top_ = 0;
    int cam_roi_width_ = 0;
    int cam_roi_height_ = 0;
    bool cam_framerate_control_ = false;   // framerate control - enable or disable
    int cam_framerate_set_ = 0;      // framerate control - setting fps
    int cam_img_cap_timeout_ = 0;       // max time to wait for img
    // white balance mode: 0 - none, 1 - use coeffs, 2 = auto
    int cam_white_balance_mode_ = 0;
    float cam_white_balance_coef_r_ = 1.0; // white balance coefficient (rgb)
    float cam_white_balance_coef_g_ = 1.0;
    float cam_white_balance_coef_b_ = 1.0;

    // Diagnostics
    bool enable_diagnostics_ = {};
    std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> cam_pub_diag_;
    double pub_frequency_tolerance_ = {};
    double pub_frequency_ = {};
    double frequency_min_ = {};
    double frequency_max_ = {};
    double age_min_ = {};
    double age_max_ = {};

    // Bandwidth Limiting
    int cam_num_in_bus_ = 0;            // # cameras in a single bus
    float cam_bw_safetyratio_ = 0.8;        // ratio used based on a camera avail bw

    // Active variables
    bool is_active_ = false;                // camera actively acquiring images?
    HANDLE xi_h_ = {};                   // camera xiAPI handle
    float min_fps_ = {};                 // camera calculated min fps
    float max_fps_ = {};                 // camera calculated max fps

    // Output Messages
    bool publish_xi_image_info_;     // publish xiGetImage handle?

    // Callback function for Camera Frame
    rclcpp::TimerBase::SharedPtr xi_open_device_cb_;
    void openDeviceCb();

    rclcpp::TimerBase::SharedPtr t_frame_cb_;
    void frameCaptureCb();

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    void triggerCb(const std_msgs::msg::Empty::SharedPtr msg);
    std::string formatTimeString(rclcpp::Time timestamp);
    bool saveOnTrigger( char *img_buffer, int img_h, int img_w, std::string filename);
    bool save_trigger_;
    bool calib_mode_;
    std::string image_directory_;
    std::string png_path_;
    std::string bin_path_;

    // convenience wrappers of the common param functions
    XI_RETURN get(const char* prm, int& value, bool suppress_warn = false);
    XI_RETURN get(const char* prm, uint64_t& value, bool suppress_warn = false);
    XI_RETURN get(const char* prm, float& value, bool suppress_warn = false);
    XI_RETURN get(const char* prm, std::string& value, bool suppress_warn = false);
    XI_RETURN set(const char* prm, int value, bool suppress_warn = false);
    XI_RETURN set(const char* prm, float value, bool suppress_warn = false);
    XI_RETURN set(const char* prm, const std::string& value, bool suppress_warn = false);

    void setWhiteBalance(void);
    void setTrigger(void);
    void setExposure(void);
    void setRegion(void);
    void setBandwidth(void);
    void setFramerate(void);

    bool camera_timestamp_supported_;
    void sampleCameraTimestamp(void);
    rclcpp::Time iterpolateTimestamp(const XI_IMG& frame);
    std::deque< std::pair<rclcpp::Time,rclcpp::Duration> > timestamp_queue_; 

};  // class XimeaROSCam

}  // namespace ximea_camera

#endif  // XIMEA_CAMERA_XIMEA_CAMERA_HPP
