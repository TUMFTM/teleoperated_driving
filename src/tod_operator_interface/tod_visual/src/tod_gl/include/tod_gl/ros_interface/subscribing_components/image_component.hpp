/**
 * @file image_component.hpp
 * @brief Image component that manages the subscription and the data for image topics.
 *        Provides camera intrinsics extraction and undistortion functionality.
 * @copyright TUMFTM 2024
 **/

 #pragma once

 #include <mutex>

 #include "tod_gl/utils/utils.hpp"
 #include <cv_bridge/cv_bridge.h>
 #include <glm/glm.hpp>
 #include <opencv2/opencv.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/camera_info.hpp>
 #include <sensor_msgs/msg/image.hpp>
 
 namespace tod_gl {
 
 /**
  * @struct CameraIntrinsics
  * @brief Structure holding intrinsic camera parameters.
  */
 struct CameraIntrinsics {
     float fx;
     float fy;
     float cx;
     float cy;
     float k1;
     float k2;
     float k3;
     float t1;
     float t2;
 };
 
 /**
  * @class ImageComponent
  * @brief Base class for image components that subscribes to image and camera info topics.
  */
 class ImageComponent {
   public:
     /**
      * @brief Default constructor.
      */
     ImageComponent();
 
     /**
      * @brief Default destructor.
      */
     virtual ~ImageComponent() = default;
 
     /**
      * @brief Extracts the camera intrinsic parameters from the received CameraInfo.
      * @return A CameraIntrinsics structure with the current intrinsic parameters.
      */
     CameraIntrinsics getCameraIntrinsics();

     sensor_msgs::msg::Image::SharedPtr get_image() const;
 
     // Public members (for simplicity – in a real design können diese encapsuliert werden)
     sensor_msgs::msg::CameraInfo camInfo;
     sensor_msgs::msg::Image undistortedRosImage;
     bool isCamInfoSet;
 
   protected:
     /**
      * @brief Returns the topic address for image subscription.
      * @return The image topic address as a string.
      */
     virtual std::string get_topic_address() const = 0;
 
     /**
      * @brief Returns the topic address for camera info subscription.
      * @return The camera info topic address as a string.
      */
     virtual std::string get_topic_address_camera_info() const = 0;
 
     /**
      * @brief Initializes subscriptions for image and camera info.
      * @param subNode Shared pointer to the ROS node.
      */
     void initialize_subscriptions(std::shared_ptr<rclcpp::Node> subNode);

     bool rotate_clockwise_{false};
 
   private:
     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
     rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _subscription_cam_info;
     std::shared_ptr<std::mutex> image_mutex_{std::make_shared<std::mutex>()};
     sensor_msgs::msg::Image::SharedPtr latest_image_;
 
     /**
      * @brief Callback for receiving image messages.
      * @param msg Shared pointer to the received image message.
      */
     void cb_message_received(const std::shared_ptr<const sensor_msgs::msg::Image> msg);
 
     /**
      * @brief Converts the image encoding from YUV422 to RGB8.
      */
     void convert_encoding_to_rgb8(sensor_msgs::msg::Image& image);

     void rotate_image_clockwise(sensor_msgs::msg::Image& image);
 
     /**
      * @brief Callback for receiving camera info messages.
      * @param msg Shared pointer to the received camera info message.
      */
     void cb_message_received_cam_info(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg);
 };
 
 /**
  * @class ImageComponentFrontCenter
  * @brief Image component for the front center camera.
  */
 class ImageComponentFrontCenter : public ImageComponent {
   public:
     ImageComponentFrontCenter();
     explicit ImageComponentFrontCenter(std::shared_ptr<rclcpp::Node> subNode);
 
   protected:
     std::string get_topic_address() const override;
     std::string get_topic_address_camera_info() const override;
 };
 
 /**
  * @class ImageComponentFrontRight
  * @brief Image component for the front right camera.
  */
 class ImageComponentFrontRight : public ImageComponent {
   public:
     ImageComponentFrontRight();
     explicit ImageComponentFrontRight(std::shared_ptr<rclcpp::Node> subNode);
 
   protected:
     std::string get_topic_address() const override;
     std::string get_topic_address_camera_info() const override;
 };
 
 /**
  * @class ImageComponentFrontLeft
  * @brief Image component for the front left camera.
  */
 class ImageComponentFrontLeft : public ImageComponent {
   public:
     ImageComponentFrontLeft();
     explicit ImageComponentFrontLeft(std::shared_ptr<rclcpp::Node> subNode);
 
   protected:
     std::string get_topic_address() const override;
     std::string get_topic_address_camera_info() const override;
 };
 
 /**
  * @class ImageComponentRearCenter
  * @brief Image component for the rear center camera.
  */
 class ImageComponentRearCenter : public ImageComponent {
   public:
     ImageComponentRearCenter();
     explicit ImageComponentRearCenter(std::shared_ptr<rclcpp::Node> subNode);
 
   protected:
     std::string get_topic_address() const override;
     std::string get_topic_address_camera_info() const override;
 };
 
 /**
  * @class ImageComponentRearRight
  * @brief Image component for the rear right camera.
  */
 class ImageComponentRearRight : public ImageComponent {
   public:
     ImageComponentRearRight();
     explicit ImageComponentRearRight(std::shared_ptr<rclcpp::Node> subNode);
 
   protected:
     std::string get_topic_address() const override;
     std::string get_topic_address_camera_info() const override;
 };
 
 /**
  * @class ImageComponentRearLeft
  * @brief Image component for the rear left camera.
  */
 class ImageComponentRearLeft : public ImageComponent {
   public:
     ImageComponentRearLeft();
     explicit ImageComponentRearLeft(std::shared_ptr<rclcpp::Node> subNode);
 
   protected:
     std::string get_topic_address() const override;
     std::string get_topic_address_camera_info() const override;
 };
 
 }  // namespace tod_gl
