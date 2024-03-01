#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/point_types.h> 
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <omp.h>

class LidarProjection : public rclcpp::Node
{
private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void load_camera_yaml(const std::string& file_path);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       image_sub_;
    std::string lidar_topic_;
    std::string output_image_topic_;
    std::string input_image_topic_;
    std::string camera_yaml_file_;
    std::string parent_tf_;
    std::string child_tf_;
    bool use_tf_;

    cv_bridge::CvImagePtr camera_image_;
    Eigen::Matrix4d camera_extrinsic_mat_;
    Eigen::Matrix3d camera_intrinsic_mat_;
    Eigen::Vector3d camera_translation_mat_;
    Eigen::Matrix3d camera_rotation_mat_;


    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
public:
    LidarProjection();
    ~LidarProjection();
};

