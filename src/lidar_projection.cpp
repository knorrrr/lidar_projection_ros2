#include <lidar_projection_ros2/lidar_projection.hpp>
#include <omp.h>
typedef Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> PointMatrix;

LidarProjection::LidarProjection() : Node("lidar_projection"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  this->declare_parameter("lidar_topic",        "/vlp32c/velodyne_points");
  this->declare_parameter("input_image_topic",  "/davis/image_raw");
  this->declare_parameter("output_image_topic", "/davis/lidar_projection_image");
  this->declare_parameter("camera_yaml_file",  "/home/hermes-22/git_open/lidar_projection_ros2/src/lidar_projection_ros2/params/0126_davis_manual_no_rotation.yaml");
  this->declare_parameter("use_tf", true);
  
  use_tf_            = this->get_parameter("use_tf").as_bool();
  lidar_topic_       = this->get_parameter("lidar_topic").as_string();
  output_image_topic_= this->get_parameter("output_image_topic").as_string();
  input_image_topic_ = this->get_parameter("input_image_topic").as_string();
  camera_yaml_file_  = this->get_parameter("camera_yaml_file").as_string();

  lidar_sub_       = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_, 10, std::bind(&LidarProjection::lidar_callback, this, std::placeholders::_1));
  image_sub_       = this->create_subscription<sensor_msgs::msg::Image>(input_image_topic_, rclcpp::SensorDataQoS() ,std::bind(&LidarProjection::image_callback, this, std::placeholders::_1));
  image_pub_       = this->create_publisher   <sensor_msgs::msg::Image>(output_image_topic_, 10);

  load_camera_yaml(camera_yaml_file_);
}

LidarProjection::~LidarProjection() {}

void LidarProjection::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Lidar callback");
  Eigen::Matrix4d ext_mat;
  if (use_tf_ == true)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try{
      transform_stamped = tf_buffer_.lookupTransform("davis", "vlp32c", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
      return;
    }
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);
    Eigen::Matrix3d rot_mat;
    rot_mat = q.toRotationMatrix();
    Eigen::Vector4d tran_vec(transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z, 1.0f);

    ext_mat.block<3, 3>(0, 0) = rot_mat;
    ext_mat.block<3, 1>(0, 3) = tran_vec.head(3);
    ext_mat.row(3) << 0, 0, 0, 1;
  }

  if (camera_image_ == nullptr)
    return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  PointMatrix cloud_mat(cloud->points.size(), 4);


  RCLCPP_INFO(this->get_logger(), "cloud size: %d", cloud->points.size()); 
  #pragma omp parallel for
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    //Remove invalid points and points behind the camera
    if (cloud->points[i].x < 0 || std::isnan(cloud->points[i].x) || std::isnan(cloud->points[i].y) || std::isnan(cloud->points[i].z))
    {
      continue;
    }
    //World to Camera
    Eigen::Vector4d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1.0f);
    Eigen::Vector4d camera_point = ext_mat * point;
    Eigen::Vector3d image_point = camera_intrinsic_mat_ * camera_point.head(3);


    //Camera to Image
    int u = image_point[0] / image_point[2];
    int v = image_point[1] / image_point[2];
    //row 180, col 240
    if ( (u >= 0 && u < camera_image_->image.cols) && (v >= 0 && v < camera_image_->image.rows))
    {
      // RCLCPP_INFO(this->get_logger(), "image_point[2]: %f", image_point[2]);
      camera_image_->image.at<cv::Vec3b>(v, u) = cv::Vec3b((int)image_point[2], 100, 100);
    }
  }

  sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", camera_image_->image).toImageMsg();
  image_pub_->publish(*image_msg);
  RCLCPP_INFO(this->get_logger(), "end_lidar_callback");
}

void LidarProjection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Image received");
  camera_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  RCLCPP_INFO(this->get_logger(), "Image received end");
}

void LidarProjection::load_camera_yaml(const std::string& file_path)
{
  RCLCPP_INFO(this->get_logger(), "Loading camera yaml file: %s", file_path.c_str());
  cv::FileStorage fs(file_path, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s", file_path.c_str());
    return;
  }
  cv::cv2eigen(fs["CameraMat"].mat(), camera_intrinsic_mat_);
  // cv::cv2eigen(fs["DistCoeff"], camera_distortion_mat_);
  if(use_tf_ == false){
    cv::cv2eigen(fs["CameraExtrinsicMat"].mat(), camera_extrinsic_mat_);
  }
  fs.release();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProjection>());
  rclcpp::shutdown();
  return 0;
}