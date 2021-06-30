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

ros::Publisher pub;
ros::Publisher polygon_pub;

bool condition(pcl::Normal& n1, pcl::Normal& n2, pcl::PointXYZ& p1, pcl::PointXYZ& p2, float thr1, float thr2) {
  //return pcl::isFinite(n1) && pcl::isFinite(n2) && pcl::isFinite(p1) && pcl::isFinite(p2) && std::abs(n1.normal_x * n2.normal_x + n1.normal_y * n2.normal_y + n1.normal_z * n2.normal_z) > thr1
  //  && std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z)) < thr2;
  return pcl::isFinite(p1) && pcl::isFinite(p2) && std::abs(p1.z-p2.z) < thr2;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "cb" << std::endl;
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

  //float x_offset = cloud->points[0].x;
  //float y_offset = cloud->points[0].y;
  //float before_z = cloud->points[0].z;
  //for (int i = 0; i < 500; i++) {
  //  for (int j = 0; j < 500; j++) {
  //    if (!pcl::isFinite(cloud->points[i*500+j])) {
  //      std::cout << "nan value: " << j << " " << i << " " << cloud->points[i*500+j].x << " " << cloud->points[i*500+j].y << " " << cloud->points[i*500+j].z << std::endl;
  //      cloud->points[i*500+j].z = before_z;
  //    } else {
  //      before_z = cloud->points[i*500+j].z;
  //    }
  //    cloud->points[i*500+j].x = x_offset + 0.01*j;
  //    cloud->points[i*500+j].y = y_offset + 0.01*i;
  //  }
  //}


  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	// Other estimation methods: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT.
	// They determine the smoothness of the result, and the running time.
    // 別な推定手法: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT.
    // 結果の滑らかさと、実行時間とを返す
	normalEstimation.setNormalEstimationMethod(normalEstimation.AVERAGE_3D_GRADIENT);
	// Depth threshold for computing object borders based on depth changes, in meters.
    // 深さの変化に基づきオブジェクトの境界を計算するための深さの閾値、単位はメートル
	normalEstimation.setMaxDepthChangeFactor(1.00f);
	// Factor that influences the size of the area used to smooth the normals.
    // 法線を滑らかにするのに使われる領域サイズの係数
	normalEstimation.setNormalSmoothingSize(7.0f);

	// Calculate the normals.　法線の計算
	normalEstimation.compute(*normals);

  ros::Time a_time = ros::Time::now();

  pcl::PointCloud<std::uint32_t>::Ptr labels (new pcl::PointCloud<std::uint32_t>());
  labels->width = cloud->width;
  labels->height = cloud->height;
  labels->is_dense = false;
  labels->points.resize(cloud->width * cloud->height);

  std::vector<std::uint32_t> label_table;
  label_table.push_back(0);
  std::uint32_t new_label = 1;// label0はnan

  std::vector<std::uint32_t> label_point_sum;
  label_point_sum.push_back(0);

  float angle_threshold = std::cos(0.05);
  float distance_threshold = 0.005;

  //conditionを基準にラベリング
  for (int y = 0; y < 500; y+=2) {
    for (int x = 0; x < 500; x+=2) {
      int p1 = y*500+x;
      int p2 = y*500+x-2;
      int p3 = (y-2)*500+x;
      if (/*pcl::isFinite(normals->points[p1]) &&*/ pcl::isFinite(cloud->points[p1])) {
        if (x != 0 && condition(normals->points[p1], normals->points[p2], cloud->points[p1], cloud->points[p2], angle_threshold, distance_threshold)) {
          if (y != 0 && condition(normals->points[p1], normals->points[p3], cloud->points[p1], cloud->points[p3], angle_threshold, distance_threshold)) {
            if (label_table[labels->points[p2]] != label_table[labels->points[p3]]) {
              std:uint32_t nl = label_table[labels->points[p3]];
              while (nl != label_table[nl]) {
                nl = label_table[nl];
                //std::cout << "while " << nl << std::endl;
              }
              if (label_table[labels->points[p2]] == labels->points[p2]) {
                label_table[labels->points[p2]] = nl;
              } else if (label_table[labels->points[p2]] == label_table[label_table[labels->points[p2]]]) {
                label_table[label_table[labels->points[p2]]] = nl;
                label_table[labels->points[p2]] = nl;
              } else {
                std::uint32_t l = label_table[labels->points[p2]];
                while (l != label_table[l]) {
                  l = label_table[l];
                  //std::cout << "while " << l << std::endl;
                }
                label_table[l] = nl;
                label_table[label_table[labels->points[p2]]] = nl;
                label_table[labels->points[p2]] = nl;
              }
              labels->points[p1] = nl;
              label_point_sum[nl]++;
            } else {
              labels->points[p1] = label_table[labels->points[p2]];
              label_point_sum[label_table[labels->points[p2]]]++;
            }
          } else {
            labels->points[p1] = label_table[labels->points[p2]];
            label_point_sum[label_table[labels->points[p2]]]++;
          }
        } else if (y != 0 && condition(normals->points[p1], normals->points[p3], cloud->points[p1], cloud->points[p3], angle_threshold, distance_threshold)) {
          labels->points[p1] = label_table[labels->points[p3]];
          label_point_sum[label_table[labels->points[p3]]]++;
        } else {
          labels->points[p1] = new_label;
          label_table.push_back(new_label);
          label_point_sum.push_back(1);
          new_label++;
        }
      } else {
        labels->points[p1] = 0;
        label_point_sum[0]++;
      }
    }
  }

  //ラベルを統合
  for (int i = 1; i < new_label; i++) {
    std::uint32_t l = i;
    while (l != label_table[l]) {
      l = label_table[l];
    }
    int tmp = label_point_sum[i];
    label_point_sum[i] -= tmp;
    label_point_sum[l] += tmp;
    label_table[i] = l;
  }

  //label_tableを整理
  std::vector<std::uint32_t> sub_label_table(new_label, 0);
  std::vector<std::uint32_t> sub_label_point_sum(new_label, 0);
  std::uint32_t sub_new_label = 1;
  sub_label_point_sum[0] = label_point_sum[0];
  int size_threshold = 100;
  for (int i = 1; i < new_label; i++) {
    if (sub_label_table[label_table[i]] == 0) {
      if (label_point_sum[label_table[i]] > size_threshold) {
        sub_label_table[label_table[i]] = sub_new_label;
        sub_label_point_sum[sub_new_label] = label_point_sum[label_table[i]];
        sub_new_label++;
      } else {
        sub_label_table[label_table[i]] = 0;
        sub_label_point_sum[0] += label_point_sum[label_table[i]];
      }
    }
    label_table[i] = sub_label_table[label_table[i]];
  }

  //for (int i = 1; i < new_label; i++) {
  //  std::cout << i << " : " << label_table[i] << std::endl;
  //}

  //int sum = 0;
  //for (int i = 0; i < sub_new_label; i++) {
  //  std::cout << i << " : " << sub_label_point_sum[i] << std::endl;
  //  sum += sub_label_point_sum[i];
  //}

  //std::cout << sum << std::endl;

  std::vector<cv::Mat> binarized_image;
  for (int i = 0; i < sub_new_label; i++) {
    binarized_image.push_back(cv::Mat::zeros(250, 250, CV_8UC1));
  }

  for (int y = 2; y < 498; y+=2) {
    for (int x = 2; x < 498; x+=2) {
      labels->points[y*500+x] = label_table[labels->points[y*500+x]];
      binarized_image[labels->points[y*500+x]].at<uchar>(x/2, y/2) = 255;
    }
  }

  std::vector<std::vector<cv::Point>> approx_vector;

  for (int i = 1; i < sub_new_label; i++) {
    cv::erode(binarized_image[i], binarized_image[i], cv::noArray(), cv::Point(-1, -1), 2);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binarized_image[i], contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    for (int j = 0; j < contours.size(); j++) {
      if (hierarchy[j][3] == -1) {
        if (cv::contourArea(contours[j]) > size_threshold / 2) {
          std::vector<cv::Point> approx;
          cv::approxPolyDP(contours[j], approx, 2.0, true);
          if (approx.size() >= 3) {
            approx_vector.push_back(approx);
          }
        }
      } else {
        if (cv::contourArea(contours[j]) > size_threshold) {
          std::vector<cv::Point> approx;
          cv::approxPolyDP(contours[j], approx, 2.0, true);
          if (approx.size() >= 3) {
            approx_vector.push_back(approx);
          }
        }
      }
    }
  }

  //std::cout << "label num " << sub_new_label << " " << new_label << std::endl;

  cv::Mat image = cv::Mat::zeros(250, 250, CV_8UC3);
  for (int y = 0; y < 500; y+=2) {
    for (int x = 0; x < 500; x+=2) {
      image.at<cv::Vec3b>(x/2, y/2)[0] = 30 + labels->points[y*500+x] * 29 % 226;
      image.at<cv::Vec3b>(x/2, y/2)[1] = 30 + labels->points[y*500+x] * 67 % 226;
      image.at<cv::Vec3b>(x/2, y/2)[2] = 30 + labels->points[y*500+x] * 47 % 226;
    }
  }
  cv::drawContours(image, approx_vector, -1, cv::Scalar(255, 0, 0));

  //cv::imshow("title", image);
  //cv::waitKey(0);

	//// Visualize them.　視覚化
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//// Display one normal out of 20, as a line of length 3cm.　
  //// 20個中1個の割合で法線を表示、3cmの長さとする
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.03, "normals");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
  //  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

  //pcl::ModelCoefficients coefficients;
  //pcl::PointIndices inliers;
  //// Create the segmentation object
  //pcl::SACSegmentation<pcl::PointXYZ> seg;
  //// Optional
  //seg.setOptimizeCoefficients (true);
  //// Mandatory
  //seg.setModelType (pcl::SACMODEL_PLANE);
  //seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setDistanceThreshold (0.03);

  //seg.setInputCloud (cloud.makeShared ());
  //seg.segment (inliers, coefficients);

  //if (inliers.indices.size () == 0)  
  //  {  
  //    PCL_ERROR ("Could not estimate a planar model for the given dataset.");  
  //    return;  
  //  }

  //std::cerr << "Model coefficients: " << coefficients.values[0] << " "  
  // << coefficients.values[1] << " "  
  // << coefficients.values[2] << " "  
  // << coefficients.values[3] << std::endl;

  //for (size_t i = 0; i < inliers.indices.size (); ++i) {  
  //  cloud.points[inliers.indices[i]].x = 0;  
  //  cloud.points[inliers.indices[i]].y = 0;  
  //  cloud.points[inliers.indices[i]].z = 0;  
  //}

  //pcl::visualization::CloudViewer viewer("Cloud Viewer");  
  //
  //viewer.showCloud(cloud.makeShared());  
  //
  //while (!viewer.wasStopped ())  
  //  {  
  //     
  //  }

  // Publish the model coefficients
  //sensor_msgs::PointCloud2 pub_msg;
  //pcl::toROSMsg(*cloud, pub_msg);
  //pub.publish (pub_msg);

  ros::Time b_time = ros::Time::now();

  //pub.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, image).toImageMsg());
  //pub.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, binarized_image[1]).toImageMsg());
  pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());

  jsk_recognition_msgs::PolygonArray polygon_msg;
  polygon_msg.header = std_msgs::Header();
  polygon_msg.header.frame_id = input->header.frame_id;
  //for (int i = 0; i < approx_vector.size(); i++) {
  for (int i = 0; i < 1; i++) {
    if (approx_vector[i].size() < 3) {
      continue;
    }
    geometry_msgs::PolygonStamped ps;
    //polygon_msg.polygons[i].polygon.points.resize(approx_vector[i].size());
    for (int j = 0; j < approx_vector[i].size(); j++) {
      int p1 = 500 * (approx_vector[i][j].x*2) + (approx_vector[i][j].y*2);
      if (pcl::isFinite(cloud->points[p1])) {
        geometry_msgs::Point32 p;
        p.x = cloud->points[p1].x;
        p.y = cloud->points[p1].y;
        p.z = cloud->points[p1].z;
        ps.polygon.points.push_back(p);
        //polygon_msg.polygons[i].polygon.points[j].x = cloud->points[p1].x;
        //polygon_msg.polygons[i].polygon.points[j].y = cloud->points[p1].y;
        //polygon_msg.polygons[i].polygon.points[j].z = cloud->points[p1].z;
      } else {
        std::cout << "infinite!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111i " << std::endl;
      }
    }
    ps.header = std_msgs::Header();
    ps.header.frame_id = input->header.frame_id;
    polygon_msg.polygons.push_back(ps);
  }
  polygon_pub.publish(polygon_msg);

  ros::Time end_time = ros::Time::now();
  std::cout << "all_time " << (end_time - begin_time).sec << "s " << (int)((end_time - begin_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "begin_a  " << (a_time - begin_time).sec << "s " << (int)((a_time - begin_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "a_b  " << (b_time - a_time).sec << "s " << (int)((b_time - a_time).nsec / 1000000) << "ms" << std::endl;
  std::cout << "b_end  " << (end_time - b_time).sec << "s " << (int)((end_time - b_time).nsec / 1000000) << "ms" << std::endl;
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

int
main (int argc, char** argv)
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

  // Spin
  ros::spin();
}
