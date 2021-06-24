#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/PointStamped.h>
//#include "../../../devel/include/stag_ros/STagMarker.h"
#include <stag_ros/STagMarkerArray.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <cmath>
#include <string>
#include <tf/transform_listener.h>

cv::Mat image;
cv::Mat tracking_image;
cv::Mat first_tracking_image;

cv::Rect2i rectangle_value;
bool is_click;
bool is_tracking;
bool init_flag = true;
constexpr int error_buf = 5;

int ar_look_timer = 0;

enum MarkerType {
  AR = 1,
  STAG,
};

MarkerType marker_type = AR;

image_transport::Publisher image_pub;
ros::Publisher look_at_point_pub;

std::array<cv::Scalar, error_buf> colors = {
  cv::Scalar(255, 0, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 0, 255),
  cv::Scalar(255, 255, 0),
  cv::Scalar(0, 255, 255)
  };

void mouse_callback(int event, int x, int y, int flags, void *userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    is_click = true;
    is_tracking = false;
    std::cout << "select" << std::endl;
    rectangle_value = cv::Rect2i(x, y, 0, 0);
  }
  if (event == cv::EVENT_LBUTTONUP) {
    is_click = false;
    is_tracking = true;
    std::cout << "end select" << std::endl;
    rectangle_value.width = x - rectangle_value.x;
    rectangle_value.height = y - rectangle_value.y;
    tracking_image = cv::Mat(image, rectangle_value).clone();
    first_tracking_image = cv::Mat(image, rectangle_value).clone();
  }
  if (event == cv::EVENT_MOUSEMOVE) {
    if (is_click) {
      rectangle_value.width = x - rectangle_value.x;
      rectangle_value.height = y - rectangle_value.y;
    }
  }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  int divide = 4;
  std::array<cv::Point2i, error_buf> min_error_point;
  std::array<double, error_buf> min_error;

  try {
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  //std::cout << "size: " << image.cols << ", " << image.rows << std::endl;
  cv::resize(image, image, cv::Size(), 0.25, 0.25);
  //cv::cvtColor(image, image, CV_RGB2GRAY);
  cv::Mat dest_image = image.clone();
  //for(int i = 0; i < image.cols; i++){
  //  for(int j = 0; j < image.rows; j++){
  //    image.at<cv::Vec3b>(j, i) /= 2;
  //  }
  //}
  
  if (is_click) {
    cv::rectangle(dest_image, rectangle_value, cv::Scalar(0, 0, 0), 3);
  }
  
  
  if (is_tracking){
    cv::Mat mat = (cv::Mat_<double>(2,3)<<1.0, 0.0, 0, 0, 1, 0);
    cv::warpAffine(tracking_image, dest_image, mat, dest_image.size(), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);

    //cv::cvtColor(dest_image, dest_image, CV_GRAY2RGB);

    cv::Mat graymap = (cv::Mat_<cv::Vec3d>(divide, divide));
    for (int i = 0; i < divide; i++){
      for (int j = 0; j < divide; j++){
        cv::Scalar hoge = cv::mean(cv::Mat(tracking_image, cv::Rect2i(tracking_image.cols*i/divide, tracking_image.rows*j/divide, tracking_image.cols*(i+1)/divide - tracking_image.cols*i/divide, tracking_image.rows*(j+1)/divide - tracking_image.rows*j/divide)));
        graymap.at<cv::Vec3d>(j,i) = cv::Vec3d(hoge[0], hoge[1], hoge[2]);
      }
    }

    //cv::Rect2i new_rectangle(0, 0, rectangle_value.width, rectangle_value.height);
    for (int i = 0; i < error_buf; i++) {
      min_error_point[i].x = 0;
      min_error_point[i].y = 0;
      min_error[i] = -1;
    }
    double max_error = 0;

    for (int i = std::max(rectangle_value.x-30, 0); i < std::min(rectangle_value.x+30,image.cols - rectangle_value.width); i++){
      for (int j = std::max(rectangle_value.y-30, 0); j < std::min(rectangle_value.y+30,image.rows - rectangle_value.height); j++){
        cv::Mat test_image = cv::Mat(image, cv::Rect2i(i, j, rectangle_value.width, rectangle_value.height)).clone();

        int error = 0;
        for (int p = 0; p < divide; p++){
          for (int q = 0; q < divide; q++){
            cv::Scalar hoge = cv::mean(cv::Mat(test_image, cv::Rect2i(test_image.cols*p/divide, test_image.rows*q/divide, test_image.cols*(p+1)/divide - test_image.cols*p/divide, test_image.rows*(q+1)/divide - test_image.rows*q/divide)));
            cv::Vec3d fuga = cv::Vec3d(hoge[0], hoge[1], hoge[2]) - graymap.at<cv::Vec3d>(q, p);
            error += fuga.dot(fuga);
          }
        }

        //dest_image.at<cv::Vec3b>(j + image.rows / 2, i + image.cols / 2)[0] = 255 / std::log(error+1);
        dest_image.at<cv::Vec3b>(j + rectangle_value.height / 2, i + rectangle_value.width / 2)[0] = std::max(std::min((int)(30 * (log(error + 1) - log(20))), 255), 0);
        dest_image.at<cv::Vec3b>(j + rectangle_value.height / 2, i + rectangle_value.width / 2)[1] = std::max(std::min((int)(30 * (log(error + 1) - log(20))), 255), 0);
        if (ar_look_timer > 0) {
          dest_image.at<cv::Vec3b>(j + rectangle_value.height / 2, i + rectangle_value.width / 2)[2] = std::max(std::min((int)(30 * (log(error + 1) - log(20))), 255), 0);
        } else {
          dest_image.at<cv::Vec3b>(j + rectangle_value.height / 2, i + rectangle_value.width / 2)[2] = std::max(std::min((int)(0 * (log(error + 1) - log(20))), 255), 0);
        }

        for (int k = 0; k < error_buf; k++) {
          if (min_error[k] < 0 || error < min_error[k]) {
            for (int l = error_buf-1; l > 0; l--) {
              min_error_point[l].x = min_error_point[l-1].x;
              min_error_point[l].y = min_error_point[l-1].y;
              min_error[l] = min_error[l-1];
            }
            min_error[k] = error;
            min_error_point[k].x = i;
            min_error_point[k].y = j;
            break;
          }
        }

        if (error > max_error) { 
          max_error = error;
        }
      }
    }
    if (ar_look_timer > 0) {
      ar_look_timer--;
    }
    std::cout << min_error[0] << " : " << max_error << std::endl;
    rectangle_value = cv::Rect2i(min_error_point[0].x, min_error_point[0].y, rectangle_value.width, rectangle_value.height);
    tracking_image = cv::Mat(image, rectangle_value) * 0.5 + tracking_image * 0.5 + first_tracking_image * 0.0;

    for (int i = 0; i < error_buf; i++){
      cv::rectangle(dest_image, cv::Rect2i(min_error_point[i].x, min_error_point[i].y, rectangle_value.width, rectangle_value.height), colors[i], 1);
    }
  }

  //cvtColor(image, image, CV_RGB2GRAY);
  //cv::Canny(image, image, 50, 100);

  //cv::imshow("image", dest_image);
  image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, dest_image).toImageMsg());

  if (is_tracking) {
    geometry_msgs::PointStamped look_at_point;
    look_at_point.header = msg->header;
    look_at_point.point.x = -std::atan2((rectangle_value.x + rectangle_value.width/2.0 - image.cols/2.0), image.cols/2.0*std::tan((90.0-(70/2.0))/360.0*2*M_PI)); //FOV 70x43(degree)
    look_at_point.point.y = std::atan2(rectangle_value.y + rectangle_value.height/2.0 - image.rows/2.0, image.rows/2.0*std::tan((90.0-(43/2.0))/360.0*2*M_PI));
    look_at_point.point.z = 1.0;
    look_at_point_pub.publish(look_at_point);
  }


  if (init_flag) {
    is_click = false;
    //cv::setMouseCallback("image", mouse_callback);
    init_flag = false;
  }
  cv::waitKey(1);
}
void imageCallback2(const sensor_msgs::ImageConstPtr& msg) {
  try {
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  //cv::imshow("image2", image);
  cv::waitKey(1);
}

void arCallback(const ar_track_alvar_msgs::AlvarMarkers msg) {
  if (!init_flag) {
    for (int i = 0; i < msg.markers.size(); i++) {
      if (msg.markers[i].id == 12) {
        double x = msg.markers[i].pose.pose.position.x;
        double y = msg.markers[i].pose.pose.position.y;
        double z = msg.markers[i].pose.pose.position.z;
        int px = (int)(image.cols/2.0*std::tan((90.0-(70/2.0))/360.0*2*M_PI) * x/z) + image.cols/2.0 - 10;
        int py = (int)(image.rows/2.0*std::tan((90.0-(43/2.0))/360.0*2*M_PI) * y/z) + image.rows/2.0 - 10;
        //std::cout << x << " " << y << " " << z << " " << px << " " << py << " " << image.cols << " " << image.rows << std::endl;
        rectangle_value = cv::Rect2i(px, py, 20, 20);
        tracking_image = cv::Mat(image, rectangle_value).clone();
        first_tracking_image = cv::Mat(image, rectangle_value).clone();
        is_tracking = true;
        ar_look_timer = 1;
      }
    }
  }
}

void stagCallback(const stag_ros::STagMarkerArray msg) {
  std::cout << "stag now" << std::endl;
  if (!init_flag) {
    for (int i = 0; i < msg.stag_array.size(); i++) {
      if (msg.stag_array[i].id.data == 0 && msg.stag_array[i].pose.position.z != 0) {
        double x = msg.stag_array[i].pose.position.x;
        double y = msg.stag_array[i].pose.position.y;
        double z = msg.stag_array[i].pose.position.z;
        int px = (int)(image.cols/2.0*std::tan((90.0-(70/2.0))/360.0*2*M_PI) * x/z) + image.cols/2.0 - 10;
        int py = (int)(image.rows/2.0*std::tan((90.0-(43/2.0))/360.0*2*M_PI) * y/z) + image.rows/2.0 - 10;
        std::cout << x << " " << y << " " << z << " " << px << " " << py << " " << image.cols << " " << image.rows << std::endl;
        rectangle_value = cv::Rect2i(px, py, 20, 20);
        tracking_image = cv::Mat(image, rectangle_value).clone();
        first_tracking_image = cv::Mat(image, rectangle_value).clone();
        is_tracking = true;
        ar_look_timer = 1;
      }
    }
  }
}
//void stagCallback(const ) {
//}

int main(int argc, char** argv) {
  bool isClick = false;
  rectangle_value = cv::Rect2i(0, 0, 0, 0);
  is_tracking = false;
  ros::Subscriber ar_sub;

  ros::init (argc, argv, "object_tracker");
  ros::NodeHandle nh("~");

  std::string marker_type_str = "ar";
  nh.getParam("/object_tracker/marker_type", marker_type_str);
  if (marker_type_str == "ar") {
    marker_type = MarkerType::AR;
  } else if (marker_type_str == "stag") {
    marker_type = MarkerType::STAG;
  }

  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("dest_image", 10);
  look_at_point_pub = nh.advertise<geometry_msgs::PointStamped>("/look_at_point", 10);
  //image_transport::Subscriber image_sub = it.subscribe("/camera/color/image_rect_color", 10, imageCallback);
  //image_transport::Subscriber image_sub2 = it.subscribe("/camera/color/image_rect_color", 10, imageCallback2);
    //image_transport::Subscriber image_sub = it.subscribe("/my_camera/image_rect_color", 10, imageCallback);
  image_transport::Subscriber image_sub = it.subscribe("/rs_l515/color/image_raw", 10, imageCallback);
  //image_transport::Subscriber image_sub2 = it.subscribe("/rs_l515/color/image_raw", 10, imageCallback2);

  if (marker_type == MarkerType::AR) {
    ar_sub = nh.subscribe("/ar_pose_marker", 10, arCallback);
  } else if (marker_type == MarkerType::STAG){
    ar_sub = nh.subscribe("/stag_ros/markers", 10, stagCallback);
  }
  std::cout << "start" << std::endl;
  ros::spin();
  return 0;
}
