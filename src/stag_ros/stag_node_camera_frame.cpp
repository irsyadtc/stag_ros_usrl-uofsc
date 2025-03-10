/**
MIT License

Copyright (c) 2023 Muhammad Irsyad Sahalan, CAREM, Universiti Teknologi Petronas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Project includes
#ifndef NDEBUG
#include "stag_ros/instrument.hpp"
#endif

// STag marker handle
#include "stag/Marker.h"

// STag ROS
#include "stag_ros/stag_node.h"
#include "stag_ros/load_yaml_tags.h"
#include "stag_ros/utility.hpp"

// ROS includes
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <stag_ros/STagMarker.h>
#include <stag_ros/STagMarkerArray.h>

//edit
#include <tf2_ros/transform_broadcaster.h>

#include <stdexcept>
#include <iostream>
#include <stag_ros/common.hpp>

namespace stag_ros {

StagNode::StagNode(ros::NodeHandle &nh,
                   image_transport::ImageTransport &imageT) {
  // Load Parameters
  loadParameters();

  // Initialize Stag
  try {
    stag = new Stag(stag_library, error_correction, false);
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
    exit(-1);
  }

  // Set Subscribers
  imageSub = imageT.subscribe(
      image_topic, 1, &StagNode::imageCallback, this,
      image_transport::TransportHints(is_compressed ? "compressed" : "raw"));
  cameraInfoSub =
      nh.subscribe(camera_info_topic, 1, &StagNode::cameraInfoCallback, this);

  // Set Publishers
  if (debug_images)
    imageDebugPub = imageT.advertise("stag_ros/image_markers", 1);
  bundlePub = nh.advertise<stag_ros::STagMarkerArray>("stag_ros/bundles", 1);
  markersPub = nh.advertise<stag_ros::STagMarkerArray>("stag_ros/markers", 1);

  // Initialize camera info
  got_camera_info = false;
  cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distortionMat = cv::Mat::zeros(1, 5, CV_64F);
  rectificationMat = cv::Mat::zeros(3, 3, CV_64F);
  projectionMat = cv::Mat::zeros(3, 4, CV_64F);
}

StagNode::~StagNode() { delete stag; }

void StagNode::loadParameters() {
  // Create private nodeHandle to load parameters
  ros::NodeHandle nh_lcl("~");

  nh_lcl.param("libraryHD", stag_library, 15);
  nh_lcl.param("errorCorrection", error_correction, 7);
  nh_lcl.param("raw_image_topic", image_topic, std::string("image_raw"));
  nh_lcl.param("camera_info_topic", camera_info_topic,
               std::string("camera_info"));
  nh_lcl.param("is_compressed", is_compressed, false);
  nh_lcl.param("show_markers", debug_images, false);
  nh_lcl.param("publish_tf", publish_tf, false);
  nh_lcl.param("tag_tf_prefix", tag_tf_prefix, std::string("STag_"));

  loadTagsBundles(nh_lcl, "tags", "bundles", tags, bundles);
}

bool StagNode::getTagIndex(const int id, int &tag_index) {
  for (int i = 0; i < tags.size(); ++i) {
    if (tags[i].id == id) {
      tag_index = i;
      return true;
    }
  }
  return false;  // not found
}

bool StagNode::getBundleIndex(const int id, int &bundle_index, int &tag_index) {
  for (int bi = 0; bi < bundles.size(); ++bi) {
    for (int ti = 0; ti < bundles[bi].tags.size(); ++ti) {
      if (bundles[bi].tags[ti].id == id) {
        bundle_index = bi;
        tag_index = ti;
        return true;
      }
    }
  }
  return false;  // not found
}

void StagNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
#ifndef NDEBUG
  INSTRUMENT;
#endif
  if (got_camera_info) {
    cv::Mat gray;
    msgToGray(msg, gray);

    // Process the image to find the markers
    stag->detectMarkers(gray);
    std::vector<Marker> markers = stag->getMarkerList();

    // Publish debug image
    if (debug_images) {
      cv_bridge::CvImage rosMat;
      rosMat.header = msg->header;
      rosMat.encoding = "bgr8";
      rosMat.image = stag->drawMarkers();

      sensor_msgs::Image rosImage;
      rosMat.toImageMsg(rosImage);

      imageDebugPub.publish(rosImage);
    }

    // For each marker in the list
    if (markers.size() > 0) {
      // Create markers msg
      std::vector<cv::Mat> tag_pose(tags.size());
      std::vector<cv::Mat> bundle_pose(bundles.size());
      std::vector<std::vector<cv::Point2d>> bundle_image(bundles.size());
      std::vector<std::vector<cv::Point3d>> bundle_world(bundles.size());

      for (int i = 0; i < markers.size(); i++) {
        // Create marker msg
        int tag_index, bundle_index;

        // if tag is a single tag, push back
        if (getTagIndex(markers[i].id, tag_index)) {
          std::vector<cv::Point2d> tag_image(5);
          std::vector<cv::Point3d> tag_world(5);

          tag_image[0] = markers[i].center;
          tag_world[0] = tags[tag_index].center;

          for (size_t ci = 0; ci < 4; ++ci) {
            tag_image[ci + 1] = markers[i].corners[ci];
            tag_world[ci + 1] = tags[tag_index].corners[ci];
          }

          cv::Mat marker_pose = cv::Mat::zeros(3, 4, CV_64F);
          Common::solvePnpSingle(tag_image, tag_world, marker_pose,
                                 cameraMatrix, distortionMat);
          tag_pose[tag_index] = marker_pose;

        } else if (getBundleIndex(markers[i].id, bundle_index, tag_index)) {
          bundle_image[bundle_index].push_back(markers[i].center);
          bundle_world[bundle_index].push_back(
              bundles[bundle_index].tags[tag_index].center);

          for (size_t ci = 0; ci < 4; ++ci) {
            bundle_image[bundle_index].push_back(markers[i].corners[ci]);
            bundle_world[bundle_index].push_back(
                bundles[bundle_index].tags[tag_index].corners[ci]);
          }
        }
      }

      for (size_t bi = 0; bi < bundles.size(); ++bi) {
        if (bundle_image[bi].size() > 0) {
          cv::Mat b_pose = cv::Mat::zeros(3, 4, CV_64F);
          //SOLVEPNP HERE
          Common::solvePnpBundle(bundle_image[bi], bundle_world[bi], b_pose,
                                 cameraMatrix, distortionMat);
          bundle_pose[bi] = b_pose;
        }
      }

      // Bundles
      std::vector<tf::Transform> bundle_transforms;
      std::vector<string> bundle_frame_ids;
      std::vector<int> bundle_ids;
      for (size_t bi = 0; bi < bundles.size(); ++bi) {
        if (bundle_pose[bi].empty()) continue;

				//NEED TO INVERT THESE VALUES
        tf::Matrix3x3 rotMat(
            bundle_pose[bi].at<double>(0, 0), bundle_pose[bi].at<double>(0, 1),
            bundle_pose[bi].at<double>(0, 2), bundle_pose[bi].at<double>(1, 0),
            bundle_pose[bi].at<double>(1, 1), bundle_pose[bi].at<double>(1, 2),
            bundle_pose[bi].at<double>(2, 0), bundle_pose[bi].at<double>(2, 1),
            bundle_pose[bi].at<double>(2, 2));
        tf::Quaternion rotQ;
        rotMat.getRotation(rotQ);

        tf::Vector3 tfVec(bundle_pose[bi].at<double>(0, 3),
                          bundle_pose[bi].at<double>(1, 3),
                          bundle_pose[bi].at<double>(2, 3));
        auto bundle_tf = tf::Transform(rotQ, tfVec);

        bundle_transforms.push_back(bundle_tf);
        bundle_frame_ids.push_back(bundles[bi].frame_id);
        bundle_ids.push_back(-1);
      }

      if (bundle_ids.size() > 0)
      //NEED TO SWAP MSG-> HEADER FRAME ID WITH BUNDLE_FRAME_IDS
      //NEED TO INVERT BUNDLE_TRANSFORMS, BUNDLE_FRAME_IDS, BUNDLE_IDS
      {
      	ROS_WARN("Bundle_publish");
      	Common::publishTransformInverse(bundle_transforms, bundlePub, msg->header,
                                 tag_tf_prefix, bundle_frame_ids, bundle_ids,
                                 publish_tf);
      	
        //Common::publishTransform(bundle_transforms, bundlePub, msg->header,
        //                         tag_tf_prefix, bundle_frame_ids, bundle_ids,
        //                         publish_tf);
      }

      // Markers
      std::vector<tf::Transform> marker_transforms;
      std::vector<string> marker_frame_ids;
      std::vector<int> marker_ids;
      for (size_t ti = 0; ti < tags.size(); ++ti) {
        if (tag_pose[ti].empty()) continue;
				//NEED TO INVERT THESE VALUES
        tf::Matrix3x3 rotMat(
            tag_pose[ti].at<double>(0, 0), tag_pose[ti].at<double>(0, 1),
            tag_pose[ti].at<double>(0, 2), tag_pose[ti].at<double>(1, 0),
            tag_pose[ti].at<double>(1, 1), tag_pose[ti].at<double>(1, 2),
            tag_pose[ti].at<double>(2, 0), tag_pose[ti].at<double>(2, 1),
            tag_pose[ti].at<double>(2, 2));
        tf::Quaternion rotQ;
        rotMat.getRotation(rotQ);

        tf::Vector3 tfVec(tag_pose[ti].at<double>(0, 3),
                          tag_pose[ti].at<double>(1, 3),
                          tag_pose[ti].at<double>(2, 3));
        auto marker_tf = tf::Transform(rotQ, tfVec);

        marker_transforms.push_back(marker_tf);
        marker_frame_ids.push_back(tags[ti].frame_id);
        marker_ids.push_back(tags[ti].id);
      }

      if (marker_ids.size() > 0)
      {
      	//NEED TO SWAP MSG-> HEADER FRAME ID WITH BUNDLE_FRAME_IDS
      	//NEED TO INVERT BUNDLE_TRANSFORMS, BUNDLE_FRAME_IDS, BUNDLE_IDS
      	ROS_WARN("Marker_ids_publish");
      	Common::publishTransformInverse(marker_transforms, markersPub, msg->header,
                                 tag_tf_prefix, marker_frame_ids, marker_ids,
                                 publish_tf);
        //Common::publishTransform(marker_transforms, markersPub, msg->header,
        //                         tag_tf_prefix, marker_frame_ids, marker_ids,
        //                         publish_tf);
        }

    } else
      ROS_WARN("No markers detected");
  }
}

void StagNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!got_camera_info) {
    // Get camera Matrix
    cameraMatrix.at<double>(0, 0) = msg->K[0];
    cameraMatrix.at<double>(0, 1) = msg->K[1];
    cameraMatrix.at<double>(0, 2) = msg->K[2];
    cameraMatrix.at<double>(1, 0) = msg->K[3];
    cameraMatrix.at<double>(1, 1) = msg->K[4];
    cameraMatrix.at<double>(1, 2) = msg->K[5];
    cameraMatrix.at<double>(2, 0) = msg->K[6];
    cameraMatrix.at<double>(2, 1) = msg->K[7];
    cameraMatrix.at<double>(2, 2) = msg->K[8];

    // Get distortion Matrix
    distortionMat = cv::Mat::zeros(1, msg->D.size(), CV_64F);
    for (size_t i = 0; i < msg->D.size(); i++)
      distortionMat.at<double>(0, i) = msg->D[i];

    // Get rectification Matrix
    rectificationMat.at<double>(0, 0) = msg->R[0];
    rectificationMat.at<double>(0, 1) = msg->R[1];
    rectificationMat.at<double>(0, 2) = msg->R[2];
    rectificationMat.at<double>(1, 0) = msg->R[3];
    rectificationMat.at<double>(1, 1) = msg->R[4];
    rectificationMat.at<double>(1, 2) = msg->R[5];
    rectificationMat.at<double>(2, 0) = msg->R[6];
    rectificationMat.at<double>(2, 1) = msg->R[7];
    rectificationMat.at<double>(2, 2) = msg->R[8];

    // Get projection Matrix
    projectionMat.at<double>(0, 0) = msg->P[0];
    projectionMat.at<double>(0, 1) = msg->P[1];
    projectionMat.at<double>(0, 2) = msg->P[2];
    projectionMat.at<double>(1, 0) = msg->P[3];
    projectionMat.at<double>(1, 1) = msg->P[4];
    projectionMat.at<double>(1, 2) = msg->P[5];
    projectionMat.at<double>(2, 0) = msg->P[6];
    projectionMat.at<double>(2, 1) = msg->P[7];
    projectionMat.at<double>(2, 2) = msg->P[8];
    projectionMat.at<double>(2, 0) = msg->P[9];
    projectionMat.at<double>(2, 1) = msg->P[10];
    projectionMat.at<double>(2, 2) = msg->P[11];

    got_camera_info = true;
  }
}
}  // namespace stag_ros

int main(int argc, char **argv) {
  ROS_INFO("stag_node_camera_frame starts");
  ros::init(argc, argv, "stag_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport imageT(nh);

  stag_ros::StagNode stagN(nh, imageT);

  ros::spin();

  return 0;
}
