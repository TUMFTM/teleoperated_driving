/**
 * @file image_component.cpp
 * @brief Implementation of the ImageComponent class and its derivatives.
 *        Manages subscriptions to image and camera info topics and performs necessary encoding conversions.
 * @copyright TUMFTM 2024
 */

 #include "tod_gl/ros_interface/subscribing_components/image_component.hpp"
 #include <iostream>
 #include <sensor_msgs/image_encodings.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <opencv2/opencv.hpp>
 
 namespace tod_gl {
 
 // -------------------- ImageComponent Methods --------------------
 
 ImageComponent::ImageComponent() : isCamInfoSet(false) {}

 CameraIntrinsics ImageComponent::getCameraIntrinsics() {
     CameraIntrinsics intrinsics;
     intrinsics.fx = camInfo.k[0];
     intrinsics.fy = camInfo.k[4];
     intrinsics.cx = camInfo.k[2];
     intrinsics.cy = camInfo.k[5];
 
     intrinsics.k1 = camInfo.d[0];
     intrinsics.k2 = camInfo.d[1];
     intrinsics.k3 = camInfo.d[4];
     intrinsics.t1 = camInfo.d[2];
     intrinsics.t2 = camInfo.d[3];
 
     return intrinsics;
 }
 
 void ImageComponent::initialize_subscriptions(std::shared_ptr<rclcpp::Node> subNode) {
     rclcpp::QoS qs = rclcpp::SensorDataQoS();
 
     std::string topic = get_topic_address();
     if (topic.empty()) {
         std::cout << "No topic specified for CameraImage." << std::endl;
         return;
     }
 
     subscription_ = subNode->create_subscription<sensor_msgs::msg::Image>(
         get_topic_address(), qs,
         [this](const sensor_msgs::msg::Image::SharedPtr msg) {
             this->cb_message_received(msg);
         });
 
     _subscription_cam_info = subNode->create_subscription<sensor_msgs::msg::CameraInfo>(
         get_topic_address_camera_info(), 1,
         [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
             this->cb_message_received_cam_info(msg);
         });
 }
 
 void ImageComponent::cb_message_received(const std::shared_ptr<const sensor_msgs::msg::Image> msg) {
     image = *msg;
     // If image encoding is YUV422, convert it to RGB8.
     if (image.encoding == sensor_msgs::image_encodings::YUV422) {
         convert_encoding_to_rgb8();
     }
 }
 
 void ImageComponent::convert_encoding_to_rgb8() {
     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::YUV422);
     cv::Mat bgr_image;
     cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_YUV2RGB_Y422);
     image.width = bgr_image.cols;
     image.height = bgr_image.rows;
     image.step = bgr_image.step;
     image.data.assign(bgr_image.datastart, bgr_image.dataend);
 }
 
 void ImageComponent::cb_message_received_cam_info(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg) {
     camInfo = *msg;
     isCamInfoSet = true;
 }
 
 // -------------------- Derived Classes --------------------
 
 ImageComponentFrontCenter::ImageComponentFrontCenter() : ImageComponent() {}
 
 ImageComponentFrontCenter::ImageComponentFrontCenter(std::shared_ptr<rclcpp::Node> subNode) {
    initialize_subscriptions(subNode);
 }
 
 std::string ImageComponentFrontCenter::get_topic_address() const {
     return "input/front_center/image";
 }
 
 std::string ImageComponentFrontCenter::get_topic_address_camera_info() const {
     return "input/front_center/cam_info";
 }
 
 ImageComponentFrontRight::ImageComponentFrontRight() : ImageComponent() {}
 
 ImageComponentFrontRight::ImageComponentFrontRight(std::shared_ptr<rclcpp::Node> subNode) {
     initialize_subscriptions(subNode);
 }
 
 std::string ImageComponentFrontRight::get_topic_address() const {
     return "input/front_right/image";
 }
 
 std::string ImageComponentFrontRight::get_topic_address_camera_info() const {
     return "input/front_right/cam_info";
 }
 
 ImageComponentFrontLeft::ImageComponentFrontLeft() : ImageComponent() {}
 
 ImageComponentFrontLeft::ImageComponentFrontLeft(std::shared_ptr<rclcpp::Node> subNode) {
    initialize_subscriptions(subNode);
 }
 
 std::string ImageComponentFrontLeft::get_topic_address() const {
     return "input/front_left/image";
 }
 
 std::string ImageComponentFrontLeft::get_topic_address_camera_info() const {
     return "input/front_left/cam_info";
 }
 
 ImageComponentRearCenter::ImageComponentRearCenter() : ImageComponent() {}
 
 ImageComponentRearCenter::ImageComponentRearCenter(std::shared_ptr<rclcpp::Node> subNode) {
    initialize_subscriptions(subNode);
 }
 
 std::string ImageComponentRearCenter::get_topic_address() const {
     return "input/rear_center/image";
 }
 
 std::string ImageComponentRearCenter::get_topic_address_camera_info() const {
     return "input/rear_center/cam_info";
 }
 
 ImageComponentRearRight::ImageComponentRearRight() : ImageComponent() {}
 
 ImageComponentRearRight::ImageComponentRearRight(std::shared_ptr<rclcpp::Node> subNode) {
    initialize_subscriptions(subNode);
 }
 
 std::string ImageComponentRearRight::get_topic_address() const {
     return "input/rear_right/image";
 }
 
 std::string ImageComponentRearRight::get_topic_address_camera_info() const {
     return "input/rear_right/cam_info";
 }
 
 ImageComponentRearLeft::ImageComponentRearLeft() : ImageComponent() {}
 
 ImageComponentRearLeft::ImageComponentRearLeft(std::shared_ptr<rclcpp::Node> subNode) {
    initialize_subscriptions(subNode);
 }
 
 std::string ImageComponentRearLeft::get_topic_address() const {
     return "input/rear_left/image";
 }
 
 std::string ImageComponentRearLeft::get_topic_address_camera_info() const {
     return "input/rear_left/cam_info";
 }
 
 }  // namespace tod_gl