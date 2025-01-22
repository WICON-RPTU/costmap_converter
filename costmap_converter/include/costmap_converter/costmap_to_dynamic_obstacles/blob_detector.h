/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following code makes use of the OpenCV library.
 * OpenCV is licensed under the terms of the 3-clause BSD License.
 *
 * Contributions:
 * Modified and extended by Riyan Cyriac Jose at the Department of Wireless Communication and Navigation (WICON), RPTU Kaiserslautern.
 * New features added to support dynamic parameterization for ROS2.
 * Introduced parameter structures for modularity and dynamic updates.
 *
 * Authors: Franz Albers, Christoph Rösmann, Riyan Cyriac Jose
 *********************************************************************/

#ifndef BLOBDETECTOR_H_
#define BLOBDETECTOR_H_

// Basically the OpenCV SimpleBlobDetector, extended with getContours()

#include <opencv2/features2d/features2d.hpp>

/**
 * @class BlobDetector
 * @brief Detect blobs in image (specialized for dynamic obstacles in the costmap)
 *
 * This class is based on OpenCV's blob detector cv::SimpleBlobDetector.
 * It has been modified and specialized for dynamic obstacle tracking in the costmap:
 * -> The modified version also returns contours of the blob.
 *
 * See http://docs.opencv.org/trunk/d0/d7a/classcv_1_1SimpleBlobDetector.html for the original class.
 */
class BlobDetector : public cv::SimpleBlobDetector
{
public:
  struct Params
  {
    // Blob Detector Parameters
    bool filterByColor;         //! Filter blobs by color/intensity
    int blobColor;              //! Color of the blob (e.g., 255 for light blobs)
    double thresholdStep;       //! Distance between neighboring thresholds for binary image conversion
    double minThreshold;        //! Minimum threshold for binary image conversion
    double maxThreshold;        //! Maximum threshold for binary image conversion
    size_t minRepeatability;    //! Minimum detections required for a blob to be considered real
    double minDistBetweenBlobs; //! Minimum distance between blob centers
    bool filterByArea;          //! Filter blobs based on the number of pixels
    double minArea;             //! Minimum area (in pixels) of a blob
    double maxArea;             //! Maximum area (in pixels) of a blob
    bool filterByCircularity;   //! Filter blobs based on circularity
    double minCircularity;      //! Minimum circularity value (0 for a line)
    double maxCircularity;      //! Maximum circularity value (1 for a circle)
    bool filterByConvexity;     //! Filter blobs based on convexity
    double minConvexity;        //! Minimum convexity ratio
    double maxConvexity;        //! Maximum convexity ratio
    bool filterByInertia;       //! Filter blobs based on inertia ratio
    double minInertiaRatio;     //! Minimum inertia ratio
    double maxInertiaRatio;     //! Maximum inertia ratio
  };
  Params blob_det_params;

  //! Default constructor which optionally accepts custom parameters
  BlobDetector(const BlobDetector::Params& parameters);

  //! Create shared instance of the blob detector with given parameters
  static cv::Ptr<BlobDetector> create(const BlobDetector::Params& params);

  /**
   * @brief Detects keypoints in an image and extracts contours
   *
   * In contrast to the original detect method, this extended version
   * also extracts contours. Contours can be accessed by getContours()
   * after invoking this method.
   *
   * @todo The mask option is currently not implemented.
   *
   * @param image     image
   * @param keypoints The detected keypoints.
   * @param mask      Mask specifying where to look for keypoints (optional). It must be a 8-bit integer
   *                  matrix with non-zero values in the region of interest.
   */
  virtual void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                      const cv::Mat& mask = cv::Mat());

  /**
   * @brief Access contours extracted during detection stage
   * @return Read-only reference to the contours set of the previous detect() run
   */
  const std::vector<std::vector<cv::Point>>& getContours() { return contours_; }

  //! Update internal parameters
  void updateParameters(const BlobDetector::Params& parameters);

protected:
  struct Center
  {
    cv::Point2d location;
    double radius;
    double confidence;
  };

  virtual void findBlobs(const cv::Mat& image, const cv::Mat& binary_image, std::vector<Center>& centers,
                         std::vector<std::vector<cv::Point>>& cur_contours) const;

  std::vector<std::vector<cv::Point>> contours_;

  Params params_;
};

#endif // BLOBDETECTOR_H_
