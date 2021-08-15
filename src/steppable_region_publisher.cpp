#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/PointStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <list>
#include "polypartition.h"
#include <safe_footstep_planner/PolygonArray.h>

ros::Publisher pub;
ros::Publisher polygon_pub;
ros::Publisher meshed_polygons_pub;

bool condition(pcl::Normal& n1, pcl::Normal& n2, pcl::PointXYZ& p1, pcl::PointXYZ& p2, float thr1, float thr2) {
  //return pcl::isFinite(n1) && pcl::isFinite(n2) && pcl::isFinite(p1) && pcl::isFinite(p2) && std::abs(n1.normal_x * n2.normal_x + n1.normal_y * n2.normal_y + n1.normal_z * n2.normal_z) > thr1
  //  && std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z)) < thr2;
  return pcl::isFinite(p1) && pcl::isFinite(p2) && std::abs(p1.z-p2.z) < thr2;
}

float calc_tilt(cv::Vec3f center, cv::Vec3f tmpvec) {
  //return std::abs(std::atan2(tmpvec[2] - center[2], std::sqrt((tmpvec[0]-center[0])*(tmpvec[0]-center[0]) + (tmpvec[1]-center[1])*(tmpvec[1]-center[1]))));
  return std::abs(std::atan2(center[2] - tmpvec[2], 0.06));
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ros::Time begin_time = ros::Time::now();
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::PointCloud<pcl::Normal> cloud;
  pcl::fromROSMsg (*input, *cloud);

  //std::cout << "pc size : " << cloud->points.size() << std::endl;
  //std::cout << "pc width : " << cloud->width << std::endl;
  //std::cout << "pc height : " << cloud->height << std::endl;
  //std::cout << cloud->points[0].x << " " << cloud->points[0].y << " " << cloud->points[0].z << std::endl;
  //std::cout << cloud->points[1].x << " " << cloud->points[1].y << " " << cloud->points[1].z << std::endl;
  //std::cout << cloud->points[500].x << " " << cloud->points[500].y << " " << cloud->points[500].z << std::endl;
  //int i = 500 * 100 + 300;
  //std::cout << cloud.points[i].normal_x << " " << cloud.points[i].normal_y << " " << cloud.points[i].normal_z << " " << cloud.points[i].curvature << std::endl;


  float x_diff = 0;
  float y_diff = 0;
  for (int y = 0; y < 500; y+=2) {
    for (int x = 2; x < 500; x+=2) {
      if (pcl::isFinite(cloud->points[y*500+x])) {
        if (x_diff == 0 && pcl::isFinite(cloud->points[y*500+x-2])) { //連続してFiniteな部分を探し、diffを計算
          x_diff = cloud->points[y*500+x].x - cloud->points[y*500+x-2].x;
          y_diff = cloud->points[y*500+x].y - cloud->points[y*500+x-2].y;
        }
      } else {
        //std::cout << "nan value: " << j << " " << i << " " << cloud->points[i*500+j].x << " " << cloud->points[i*500+j].y << " " << cloud->points[i*500+j].z << std::endl;
        if (x_diff != 0 && pcl::isFinite(cloud->points[y*500+x-2])) { //inFiniteな部分には隣のzをコピー
          cloud->points[y*500+x].x = cloud->points[y*500+x-2].x + x_diff;
          cloud->points[y*500+x].y = cloud->points[y*500+x-2].y + y_diff;
          cloud->points[y*500+x].z = cloud->points[y*500+x-2].z;
        }
      }
    }
  }
  for (int i = 498; i >= 0; i-=2) {//逆順
    for (int j = 496; j >= 0; j-=2) {
      if ((!pcl::isFinite(cloud->points[i*500+j])) && pcl::isFinite(cloud->points[i*500+j+2])) { //inFiniteな部分には隣のzをコピー
        cloud->points[i*500+j].x = cloud->points[i*500+j+2].x - x_diff;
        cloud->points[i*500+j].y = cloud->points[i*500+j+2].y - y_diff;
        cloud->points[i*500+j].z = cloud->points[i*500+j+2].z;
      }
    }
  }
  x_diff = 0;
  y_diff = 0;
  for (int i = 2; i < 500; i+=2) {
    for (int j = 0; j < 500; j+=2) {
      if (pcl::isFinite(cloud->points[i*500+j])) {
        if (x_diff == 0 && pcl::isFinite(cloud->points[(i-2)*500+j])) { //連続してFiniteな部分を探し、diffを計算
          x_diff = cloud->points[i*500+j].x - cloud->points[(i-2)*500+j].x;
          y_diff = cloud->points[i*500+j].y - cloud->points[(i-2)*500+j].y;
        }
      } else {
        if (x_diff != 0 && pcl::isFinite(cloud->points[(i-2)*500+j])) { //inFiniteな部分には隣のzをコピー
          cloud->points[i*500+j].x = cloud->points[(i-2)*500+j].x + x_diff;
          cloud->points[i*500+j].y = cloud->points[(i-2)*500+j].y + y_diff;
          cloud->points[i*500+j].z = cloud->points[(i-2)*500+j].z;
        }
      }
    }
  }
  for (int i = 496; i >= 0; i-=2) {//逆順
    for (int j = 498; j >= 0; j-=2) {
      if ((!pcl::isFinite(cloud->points[i*500+j])) && pcl::isFinite(cloud->points[(i+2)*500+j])) { //inFiniteな部分には隣のzをコピー
        cloud->points[i*500+j].x = cloud->points[(i+2)*500+j].x - x_diff;
        cloud->points[i*500+j].y = cloud->points[(i+2)*500+j].y - y_diff;
        cloud->points[i*500+j].z = cloud->points[(i+2)*500+j].z;
      }
    }
  }

  ros::Time a_time = ros::Time::now();

  cv::Mat average_image = cv::Mat::zeros(250, 250, CV_32FC3);
  for (int y = 0; y < 250; y++) {
    for (int x = 0; x < 250; x++) {
      if (!pcl::isFinite(cloud->points[(y*2)*500+x*2])) {
        std::cout << "infinite aruyo " << x << " " << y << std::endl;
      } else {
        average_image.at<cv::Vec3f>(y, x)[0] = cloud->points[y*2*500+x*2].x;
        average_image.at<cv::Vec3f>(y, x)[1] = cloud->points[y*2*500+x*2].y;
        average_image.at<cv::Vec3f>(y, x)[2] = cloud->points[y*2*500+x*2].z;
      }
    }
  }

  ros::Time b_time = ros::Time::now();

  //cv::medianBlur(average_image, average_image, 3); //中央値を取る(x,y座標は3x3の中心になってしまう)
  cv::blur(average_image, average_image, cv::Size(3, 3));

  ros::Time c_time = ros::Time::now();

  cv::Mat binarized_image = cv::Mat::zeros(250, 250, CV_8UC1);
  cv::Mat image = cv::Mat::zeros(250, 250, CV_8UC3);
  int steppable_range = 3;
  float steppable_edge_height = steppable_range*0.02*std::tan(0.3);
  float steppable_corner_height = steppable_range*0.02*std::sqrt(2)*std::tan(0.3);
  float steppable_around_edge_range = 12/2;//[cm]/[cm]
  float steppable_around_corner_range = (int)(12/std::sqrt(8));//[cm]/[cm]
  float steppable_around_height_diff = 0.025;//[m]
  for (int x = 4; x < (250-4); x++) {
    for (int y = 4; y < (250-4); y++) {
      cv::Vec3f center = average_image.at<cv::Vec3f>(y, x);
      
      if (
        std::abs(center[2] - average_image.at<cv::Vec3f>(y+steppable_range, x+steppable_range)[2]) > steppable_corner_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y+steppable_range, x-steppable_range)[2]) > steppable_corner_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y-steppable_range, x+steppable_range)[2]) > steppable_corner_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y-steppable_range, x-steppable_range)[2]) > steppable_corner_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y+steppable_range, x+0)[2]) > steppable_edge_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y-steppable_range, x+0)[2]) > steppable_edge_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y+0, x+steppable_range)[2]) > steppable_edge_height ||
        std::abs(center[2] - average_image.at<cv::Vec3f>(y+0, x-steppable_range)[2]) > steppable_edge_height) {
        continue;
      }

      image.at<cv::Vec3b>(y, x)[0] = 100;
      image.at<cv::Vec3b>(y, x)[1] = 100;
      image.at<cv::Vec3b>(y, x)[2] = 100;
      if (
        average_image.at<cv::Vec3f>(y+(int)(steppable_around_edge_range), x)[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y, x+(int)(steppable_around_edge_range))[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y-(int)(steppable_around_edge_range), x)[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y, x-(int)(steppable_around_edge_range))[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y+(int)(steppable_around_corner_range), x+(int)(steppable_around_corner_range))[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y+(int)(steppable_around_corner_range), x-(int)(steppable_around_corner_range))[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y-(int)(steppable_around_corner_range), x+(int)(steppable_around_corner_range))[2] - center[2] > steppable_around_height_diff ||
        average_image.at<cv::Vec3f>(y-(int)(steppable_around_corner_range), x-(int)(steppable_around_corner_range))[2] - center[2] > steppable_around_height_diff) {
        continue;
      }
      binarized_image.at<uchar>(y, x) = 255;
      image.at<cv::Vec3b>(y, x)[0] = 200;
      image.at<cv::Vec3b>(y, x)[1] = 200;
      image.at<cv::Vec3b>(y, x)[2] = 200;
    }
  }

  ros::Time d_time = ros::Time::now();

  std::vector<std::vector<cv::Point>> approx_vector;
  std::list<TPPLPoly> polys, result;

  cv::morphologyEx(binarized_image, binarized_image, CV_MOP_CLOSE, cv::noArray(), cv::Point(-1, -1), 2);
  cv::morphologyEx(binarized_image, binarized_image, CV_MOP_OPEN, cv::noArray(), cv::Point(-1, -1), 2);
  cv::erode(binarized_image, binarized_image, cv::noArray(), cv::Point(-1, -1), 2);
  cv::morphologyEx(binarized_image, binarized_image, CV_MOP_OPEN, cv::noArray(), cv::Point(-1, -1), 2);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binarized_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

  ros::Time e_time = ros::Time::now();

  int size_threshold = 100;
  for (int j = 0; j < contours.size(); j++) {
    if (hierarchy[j][3] == -1) { //外側
      if (cv::contourArea(contours[j]) > size_threshold) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[j], approx, 2.5, true);
        if (approx.size() >= 3) {
          approx_vector.push_back(approx);
          TPPLPoly poly;
          poly.Init(approx.size());
          for (int k = 0; k < approx.size(); k++) {
            poly[k].x = approx[k].x;
            poly[k].y = -approx[k].y;
          }
          polys.push_back(poly);
        }
      }
    } else { //穴
      if (cv::contourArea(contours[j]) > size_threshold) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[j], approx, 2.0, true);
        if (approx.size() >= 3) {
          approx_vector.push_back(approx);
          TPPLPoly poly;
          poly.Init(approx.size());
          for (int k = 0; k < approx.size(); k++) {
            poly[k].x = approx[k].x;
            poly[k].y = -approx[k].y;
          }
          poly.SetHole(true);
          polys.push_back(poly);
        }
      }
    }
  }

  TPPLPartition pp;
  pp.Triangulate_EC(&polys, &result);
  //result = polys;

  //std::cout << "label num " << sub_new_label << " " << new_label << std::endl;

  cv::drawContours(image, approx_vector, -1, cv::Scalar(255, 0, 0));
  ros::Time f_time = ros::Time::now();

  jsk_recognition_msgs::PolygonArray polygon_msg;
  polygon_msg.header = std_msgs::Header();
  polygon_msg.header.frame_id = input->header.frame_id;
  safe_footstep_planner::PolygonArray meshed_polygons_msg;

  int i;
  std::list<TPPLPoly>::iterator iter;
  for (iter = result.begin(), i = 0; iter != result.end(); iter++, i++) {
    geometry_msgs::PolygonStamped ps;
    //for (int j = 0; j < iter->GetNumPoints(); j++) {
    for (int j = iter->GetNumPoints() - 1; j >= 0; j--) {
      image.at<cv::Vec3b>(-iter->GetPoint(j).y, iter->GetPoint(j).x)[2] = 255;
      int p1 = 500 * (-iter->GetPoint(j).y*2) + (iter->GetPoint(j).x*2);
      if (pcl::isFinite(cloud->points[p1])) {
        geometry_msgs::Point32 p;
        p.x = cloud->points[p1].x;
        p.y = cloud->points[p1].y;
        p.z = 0;
        ps.polygon.points.push_back(p);
      } else {
        std::cout << "infinite!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111i " << std::endl;
      }
    }
    ps.header = std_msgs::Header();
    ps.header.frame_id = input->header.frame_id;
    polygon_msg.polygons.push_back(ps);
    meshed_polygons_msg.polygons.push_back(ps.polygon);
  }

  polygon_pub.publish(polygon_msg);
  meshed_polygons_pub.publish(meshed_polygons_msg);
  pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());

  ros::Time end_time = ros::Time::now();
  std::cout << "all_time " << (end_time - begin_time).sec << "s " << (int)((end_time - begin_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "begin_a  " << (a_time - begin_time).sec << "s " << (int)((a_time - begin_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "a_b  " << (b_time - a_time).sec << "s " << (int)((b_time - a_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "b_c  " << (c_time - b_time).sec << "s " << (int)((c_time - b_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "c_d  " << (d_time - c_time).sec << "s " << (int)((d_time - c_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "d_e  " << (e_time - d_time).sec << "s " << (int)((e_time - d_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "e_f  " << (f_time - e_time).sec << "s " << (int)((f_time - e_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "f_end  " << (end_time - f_time).sec << "s " << (int)((end_time - f_time).nsec / 1000000) << "ms" << std::endl;
}

//void 
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
//{
//  // Container for original & filtered data
//  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//  pcl::PCLPointCloud2 cloud_filtered;
//
//  // Convert to PCL data type
//  pcl_conversions::toPCL(*cloud_msg, *cloud);
//
//  // Perform the actual filtering
//  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  sor.setInputCloud (cloudPtr);
//  sor.setLeafSize (0.1, 0.1, 0.1);
//  sor.filter (cloud_filtered);
//
//  // Convert to ROS data type
//  sensor_msgs::PointCloud2 output;
//  pcl_conversions::fromPCL(cloud_filtered, output);
//
//  // Publish the data
//  pub.publish (output);
//}

void img_cb (const sensor_msgs::Image msg) {
  std::cout << "cb" << std::endl;
  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC2)->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  std::cout << image.size().width << " " << image.size().height << std::endl;

  int max = 100;
  int min = 100;
  cv::Mat pub_image = cv::Mat::zeros(500, 1000, CV_8UC1);
  for (int y = 0; y < 1000; y++) {
    for (int x = 0; x < 500; x++) {
      int tmp = (int)(image.at<float>(x,y)*1000 + 150);
      if (tmp > max) {
        max = tmp;
      } else if (tmp < min) {
        min = tmp;
      }
      if (0 < tmp && tmp < 255) {
        pub_image.at<uchar>(x, y) = tmp;
      } else {
        pub_image.at<uchar>(x, y) = 0;
      }
    }
  }
  std::cout << "max: " << max << " min: " << min << std::endl;

  pub.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, pub_image).toImageMsg());
}

void polygon_cb (const jsk_recognition_msgs::PolygonArray& msg) {
  cv::Mat pub_image = cv::Mat::zeros(500, 500, CV_8UC3);
  for (int h = 0; h < msg.polygons.size(); h++) {
    for (int i = 0; i < msg.polygons[h].polygon.points.size() - 1; i++) {
      cv::line(pub_image, cv::Point((int)(msg.polygons[h].polygon.points[i].x * 50) + 250, (int)(msg.polygons[h].polygon.points[i].y*50)+250), cv::Point((int)(msg.polygons[h].polygon.points[i+1].x * 50) + 250, (int)(msg.polygons[h].polygon.points[i+1].y*50)+250), cv::Scalar(0,0,200), 3, 4);
    }
    cv::line(pub_image, cv::Point((int)(msg.polygons[h].polygon.points[0].x * 50) + 250, (int)(msg.polygons[h].polygon.points[0].y*50)+250), cv::Point((int)(msg.polygons[h].polygon.points[msg.polygons[h].polygon.points.size()-1].x * 50) + 250, (int)(msg.polygons[h].polygon.points[msg.polygons[h].polygon.points.size()-1].y*50)+250), cv::Scalar(0,0,200), 3, 4);
  }
  std::cout << "pub" << std::endl;
  pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg());
  polygon_pub.publish(msg);
}

void polytest() {
  std::cout << "hello" << std::endl;
  std::list<TPPLPoly> polys, result;
  TPPLPoly *poly;
  poly = new TPPLPoly();
  poly->Init(4);
  (*poly)[0].x = 0.0;
  (*poly)[0].y = 0.0;
  (*poly)[1].x = 1.0;
  (*poly)[1].y = 0.0;
  (*poly)[2].x = 1.0;
  (*poly)[2].y = 1.0;
  (*poly)[3].x = 0.0;
  (*poly)[3].y = 1.0;
  polys.push_back(*poly);


  TPPLPoly *hole;
  hole = new TPPLPoly();
  hole->Init(4);
  (*hole)[0].x = 0.1;
  (*hole)[0].y = 0.1;
  (*hole)[1].x = 0.1;
  (*hole)[1].y = 0.4;
  (*hole)[2].x = 0.4;
  (*hole)[2].y = 0.4;
  (*hole)[3].x = 0.4;
  (*hole)[3].y = 0.1;
  hole->SetHole(true);
  polys.push_back(*hole);

  TPPLPoly *hole2;
  hole2 = new TPPLPoly();
  hole2->Init(4);
  (*hole2)[0].x = 0.6;
  (*hole2)[0].y = 0.6;
  (*hole2)[1].x = 0.6;
  (*hole2)[1].y = 0.9;
  (*hole2)[2].x = 0.9;
  (*hole2)[2].y = 0.9;
  (*hole2)[3].x = 0.9;
  (*hole2)[3].y = 0.6;
  hole2->SetHole(true);
  polys.push_back(*hole2);

  std::cout << "hello2" << std::endl;

  TPPLPartition pp;
  pp.Triangulate_EC(&polys, &result);

  std::list<TPPLPoly>::iterator iter;
  for (iter = result.begin(); iter != result.end(); iter++) {
    std::cout << "hoge" << std::endl;
    for (int i = 0; i < iter->GetNumPoints(); i++) {
      std::cout << iter->GetPoint(i).x << ", " << iter->GetPoint(i).y << std::endl;
    }
  }
  std::cout << "hello3" << std::endl;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  //ros::Subscriber sub = nh.subscribe ("input", 1, img_cb);
  //ros::Subscriber sub = nh.subscribe ("input", 1, polygon_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);
  polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray> ("output_polygon", 1);
  meshed_polygons_pub = nh.advertise<safe_footstep_planner::PolygonArray> ("meshed_polygons", 1);

  std::cout << "hello" << std::endl;

  // Spin
  ros::spin();

  //polytest();
}
