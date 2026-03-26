/**
 * @file InversePerspectiveMapping.cpp
 * @author Mohamed Aly <malaa@caltech.edu>
 * @date 11/29/2006
 * @brief Implements inverse perspective mapping functionality for lane detection
 */

#include "InversePerspectiveMapping.hpp"

#include "CameraInfoOpt.h"

#include <iostream>
#include <math.h>
#include <assert.h>
#include <list>

using namespace std;



namespace perception
{

#define VP_PORTION 0.05

/**
 * @brief Coordinate system description
 * 
 * We are assuming the world coordinate frame center is at the camera,
 * the ground plane is at height -h, the X-axis is going right,
 * the Y-axis is going forward, the Z-axis is going up. The
 * camera is looking forward with optical axis in direction of
 * Y-axis, with possible pitch angle (above or below the Y-axis)
 * and yaw angle (left or right).
 * The camera coordinates have the same center as the world, but the Xc-axis goes right,
 * the  Yc-axis goes down, and the Zc-axis (optical cxis) goes forward. The
 * uv-plane of the image is such that u is horizontal going right, v is
 * vertical going down.
 * The image coordinates uv are such that the pixels are at half coordinates
 * i.e. first pixel is (.5,.5) ...etc where the top-left point is (0,0) i.e.
 * the tip of the first pixel is (0,0)
 */

/**
 * @brief Generates Inverse Perspective Mapping of input image
 * @param inImage Input image to transform
 * @param outImage Output IPM transformed image
 * @param ipmInfo IPM parameters and info for the transformation
 * @param cameraInfo Camera calibration parameters
 * @param outPoints Optional list to store indices of points outside image bounds
 * @details Performs IPM transform assuming flat ground plane using camera parameters
 */
void imGetIPM(const cv::Mat& inImage, cv::Mat& outImage,
               IPMInfo* ipmInfo, const CameraInfo* cameraInfo,
               std::list<cv::Point>* outPoints) {
    // Check input images types
    if (!(inImage.type() == outImage.type() &&
        (inImage.type() == CV_32F || inImage.type() == CV_8U))) {
        std::cerr << "Unsupported image types in imGetIPM";
        exit(1);
    }

    // Get size of input image
    float v = inImage.rows;
    float u = inImage.cols;

    // Get vanishing point
    cv::Point2f vp = imGetVanishingPoint(cameraInfo);
    vp.y = std::max(0.0f, vp.y);

    // Get extent of the image in the xfyf plane
    float eps = ipmInfo->vpPortion * v;
    ipmInfo->ipmLeft = std::max(0.0f, ipmInfo->ipmLeft);
    ipmInfo->ipmRight = std::min(u - 1.0f, ipmInfo->ipmRight);
    ipmInfo->ipmTop = std::max(vp.y + eps, ipmInfo->ipmTop);
    ipmInfo->ipmBottom = std::min(v - 1.0f, ipmInfo->ipmBottom);

    // Create points matrix for transformation
    cv::Mat uvLimits = (cv::Mat_<float>(2, 4) <<
        vp.x, ipmInfo->ipmRight, ipmInfo->ipmLeft, vp.x,
        ipmInfo->ipmTop, ipmInfo->ipmTop, ipmInfo->ipmTop, ipmInfo->ipmBottom);

    // Transform points to ground plane
    cv::Mat xyLimits;
    imTransformImage2Ground(uvLimits, xyLimits, cameraInfo);

    // Get extent on ground plane
    double xfMin, xfMax, yfMin, yfMax;
    cv::minMaxLoc(xyLimits.row(0), &xfMin, &xfMax);
    cv::minMaxLoc(xyLimits.row(1), &yfMin, &yfMax);

    // Create output grid
    int outRow = outImage.rows;
    int outCol = outImage.cols;
    float stepRow = (yfMax - yfMin) / outRow;
    float stepCol = (xfMax - xfMin) / outCol;

    // Create transformation grid
    cv::Mat xyGrid(2, outRow * outCol, CV_32F);
    for (int i = 0, idx = 0; i < outRow; i++) {
        float y = yfMax - 0.5f * stepRow - i * stepRow;
        for (int j = 0; j < outCol; j++, idx++) {
            xyGrid.at<float>(0, idx) = xfMin + 0.5f * stepCol + j * stepCol;
            xyGrid.at<float>(1, idx) = y;
        }
    }

    // Transform ground coordinates to image
    cv::Mat uvGrid;
    imTransformGround2Image(xyGrid, uvGrid, cameraInfo);

    // Interpolate values
    float mean = cv::mean(inImage)[0];

    if (inImage.type() == CV_32F) {
        for (int i = 0; i < outRow; i++) {
            for (int j = 0; j < outCol; j++) {
                int idx = i * outCol + j;
                float ui = uvGrid.at<float>(0, idx);
                float vi = uvGrid.at<float>(1, idx);

                if (ui < ipmInfo->ipmLeft || ui > ipmInfo->ipmRight ||
                    vi < ipmInfo->ipmTop || vi > ipmInfo->ipmBottom) {
                    outImage.at<float>(i, j) = mean;
                } else {
                    if (ipmInfo->ipmInterpolation == 0) {
                        outImage.at<float>(i, j) = inImage.at<float>(int(vi), int(ui));
                        // outImage.at<float>(i, j) = static_cast<float>(
                        //     cv::getRectSubPix(inImage, cv::Size(1,1), cv::Point2f(ui, vi)));
                    } else {
                        outImage.at<float>(i, j) = inImage.at<float>(int(vi + 0.5f), int(ui + 0.5f));
                    }
                }

                if (outPoints && (ui < ipmInfo->ipmLeft + 10 || ui > ipmInfo->ipmRight - 10 ||
                    vi < ipmInfo->ipmTop || vi > ipmInfo->ipmBottom - 2)) {
                    outPoints->push_back(cv::Point(j, i));
                }
            }
        }
    } else {
        // Similar implementation for CV_8U type
        // ... (implement similar logic for uchar type)
    }

    // Update IPM info
    ipmInfo->xLimits[0] = xyGrid.at<float>(0, 0);
    ipmInfo->xLimits[1] = xyGrid.at<float>(0, outRow * outCol - 1);
    ipmInfo->yLimits[1] = xyGrid.at<float>(1, 0);
    ipmInfo->yLimits[0] = xyGrid.at<float>(1, outRow * outCol - 1);
    ipmInfo->xScale = 1.0f / stepCol;
    ipmInfo->yScale = 1.0f / stepRow;
    ipmInfo->width = outCol;
    ipmInfo->height = outRow;
}

/**
 * @brief Transforms points from image frame to ground plane
 * @param inPoints Input points in image coordinates (2xN matrix)
 * @param outPoints Output points in world coordinates on ground plane (2xN matrix)
 * @param cameraInfo Camera calibration parameters
 * @details Projects points from image UV coordinates to XY coordinates on ground plane Z=-height
 */
void imTransformImage2Ground(const cv::Mat& inPoints, cv::Mat& outPoints, 
                            const CameraInfo* cameraInfo) {
    // Create matrix with two additional rows
    cv::Mat inPoints4(inPoints.rows + 2, inPoints.cols, inPoints.type());
    
    // Copy input points to first two rows
    inPoints.copyTo(inPoints4(cv::Rect(0, 0, inPoints.cols, inPoints.rows)));
    inPoints4.row(2).setTo(1.0f);

    // Create transformation matrix
    float c1 = cos(cameraInfo->pitch);
    float s1 = sin(cameraInfo->pitch);
    float c2 = cos(cameraInfo->yaw);
    float s2 = sin(cameraInfo->yaw);
    
    cv::Mat transform = (cv::Mat_<float>(4, 3) <<
        -cameraInfo->cameraHeight * c2 / cameraInfo->focalLength.x,
        cameraInfo->cameraHeight * s1 * s2 / cameraInfo->focalLength.y,
        (cameraInfo->cameraHeight * c2 * cameraInfo->opticalCenter.x / cameraInfo->focalLength.x) -
        (cameraInfo->cameraHeight * s1 * s2 * cameraInfo->opticalCenter.y / cameraInfo->focalLength.y) -
        cameraInfo->cameraHeight * c1 * s2,

        cameraInfo->cameraHeight * s2 / cameraInfo->focalLength.x,
        cameraInfo->cameraHeight * s1 * c2 / cameraInfo->focalLength.y,
        (-cameraInfo->cameraHeight * s2 * cameraInfo->opticalCenter.x / cameraInfo->focalLength.x) -
        (cameraInfo->cameraHeight * s1 * c2 * cameraInfo->opticalCenter.y / cameraInfo->focalLength.y) -
        cameraInfo->cameraHeight * c1 * c2,

        0,
        cameraInfo->cameraHeight * c1 / cameraInfo->focalLength.y,
        (-cameraInfo->cameraHeight * c1 * cameraInfo->opticalCenter.y / cameraInfo->focalLength.y) +
        cameraInfo->cameraHeight * s1,

        0,
        -c1 / cameraInfo->focalLength.y,
        (c1 * cameraInfo->opticalCenter.y / cameraInfo->focalLength.y) - s1);

    // Multiply matrices
    cv::Mat result = transform * inPoints4(cv::Rect(0, 0, inPoints.cols, 3));

    // Divide by last row
    for (int i = 0; i < inPoints.cols; i++) {
        float div = result.at<float>(3, i);
        result.at<float>(0, i) /= div;
        result.at<float>(1, i) /= div;
    }

    // Copy result to output
    outPoints = result(cv::Rect(0, 0, inPoints.cols, 2)).clone();
}

/**
 * @brief Transforms points from ground plane to image frame
 * @param inPoints Input points in world coordinates on ground plane (2xN matrix)
 * @param outPoints Output points in image coordinates (2xN matrix)
 * @param cameraInfo Camera calibration parameters
 * @details Projects points from XY coordinates on ground plane Z=-height to image UV coordinates
 */
void imTransformGround2Image(const cv::Mat& inPoints, cv::Mat& outPoints,
                            const CameraInfo* cameraInfo) {
    // Add one row to input points
    cv::Mat inPoints3(inPoints.rows + 1, inPoints.cols, inPoints.type());
    inPoints.copyTo(inPoints3(cv::Rect(0, 0, inPoints.cols, inPoints.rows)));
    inPoints3.row(2).setTo(-cameraInfo->cameraHeight);

    // Create transformation matrix
    float c1 = cos(cameraInfo->pitch);
    float s1 = sin(cameraInfo->pitch);
    float c2 = cos(cameraInfo->yaw);
    float s2 = sin(cameraInfo->yaw);

    cv::Mat transform = (cv::Mat_<float>(3, 3) <<
        cameraInfo->focalLength.x * c2 + c1 * s2 * cameraInfo->opticalCenter.x,
        -cameraInfo->focalLength.x * s2 + c1 * c2 * cameraInfo->opticalCenter.x,
        -s1 * cameraInfo->opticalCenter.x,

        s2 * (-cameraInfo->focalLength.y * s1 + c1 * cameraInfo->opticalCenter.y),
        c2 * (-cameraInfo->focalLength.y * s1 + c1 * cameraInfo->opticalCenter.y),
        -cameraInfo->focalLength.y * c1 - s1 * cameraInfo->opticalCenter.y,

        c1 * s2,
        c1 * c2,
        -s1);

    // Multiply matrices
    cv::Mat result = transform * inPoints3;

    // Divide by last row
    for (int i = 0; i < inPoints.cols; i++) {
        float div = result.at<float>(2, i);
        result.at<float>(0, i) /= div;
        result.at<float>(1, i) /= div;
    }

    // Copy result to output
    outPoints = result(cv::Rect(0, 0, inPoints.cols, 2)).clone();
}

/**
 * @brief Computes vanishing point in image plane
 * @param cameraInfo Camera calibration parameters
 * @return Vanishing point coordinates in image frame
 * @details Finds intersection of image plane with line in XY-plane making yaw angle with Y-axis
 */
cv::Point2f imGetVanishingPoint(const CameraInfo *cameraInfo)
{
  // Get the vp in world coordinates
    cv::Mat vp = (cv::Mat_<float>(3, 1) << 
        sin(cameraInfo->yaw)/cos(cameraInfo->pitch),
        cos(cameraInfo->yaw)/cos(cameraInfo->pitch),
        0);

    // Transform from world to camera coordinates
    // Rotation matrix for yaw
    cv::Mat tyaw = (cv::Mat_<float>(3, 3) << 
        cos(cameraInfo->yaw), -sin(cameraInfo->yaw), 0,
        sin(cameraInfo->yaw), cos(cameraInfo->yaw), 0,
        0, 0, 1);

    // Rotation matrix for pitch
    cv::Mat transform = (cv::Mat_<float>(3, 3) << 
        1, 0, 0,
        0, -sin(cameraInfo->pitch), -cos(cameraInfo->pitch),
        0, cos(cameraInfo->pitch), -sin(cameraInfo->pitch));

    // Combined transform
    transform = transform * tyaw;

    // Transformation from (xc, yc) in camera coordinates to (u,v) in image frame
    // Matrix to shift optical center and focal length
    cv::Mat t1 = (cv::Mat_<float>(3, 3) << 
        cameraInfo->focalLength.x, 0, cameraInfo->opticalCenter.x,
        0, cameraInfo->focalLength.y, cameraInfo->opticalCenter.y,
        0, 0, 1);

    // Combine transform and apply to vp
    transform = t1 * transform;
    vp = transform * vp;

    // Return result as Point2f
    return cv::Point2f(vp.at<float>(0), vp.at<float>(1));
}

/**
 * @brief Converts point from IPM pixel coordinates to world coordinates
 * @param point Point to transform (in/out parameter)
 * @param ipmInfo IPM parameters from imGetIPM
 */
void imPointImIPM2World(cv::Point2f& point, const IPMInfo* ipmInfo) {
    point.x /= ipmInfo->xScale;
    point.x += ipmInfo->xLimits[0];
    point.y /= ipmInfo->yScale;
    point.y = ipmInfo->yLimits[1] - point.y;
}

/**
 * @brief Converts points from IPM pixel coordinates to world coordinates
 * @param inMat Input matrix of points (2xN)
 * @param outMat Output matrix of transformed points (2xN)
 * @param ipmInfo IPM parameters from imGetIPM
 */
void imTransformImIPM2Ground(const cv::Mat& inMat, cv::Mat& outMat, const IPMInfo* ipmInfo) {
    if (&inMat != &outMat) {
        inMat.copyTo(outMat);
    }

    // Work on x-direction (first row)
    outMat.row(0) = outMat.row(0) / ipmInfo->xScale + ipmInfo->xLimits[0];
    
    // Work on y-direction (second row)
    outMat.row(1) = -outMat.row(1) / ipmInfo->yScale + ipmInfo->yLimits[1];
}

/**
 * @brief Converts points from IPM pixel coordinates to image coordinates
 * @param inMat Input matrix of points (2xN)
 * @param outMat Output matrix of transformed points (2xN)
 * @param ipmInfo IPM parameters from imGetIPM
 * @param cameraInfo Camera calibration parameters
 */
void imTransformImIPM2Im(const cv::Mat& inMat, cv::Mat& outMat, 
                         const IPMInfo* ipmInfo, const CameraInfo* cameraInfo) {
    imTransformImIPM2Ground(inMat, outMat, ipmInfo);
    imTransformGround2Image(outMat, outMat, cameraInfo);
}

/**
 * @brief Initializes camera info structure from config file
 * @param fileName Path to camera configuration file
 * @param cameraInfo Camera parameters structure to initialize
 */
void imInitCameraInfo (char * const fileName, CameraInfo *cameraInfo)
{
  //parsed camera data
  CameraInfoParserInfo camInfo;
  //read the data
  assert(cameraInfoParser_configfile(fileName, &camInfo, 0, 1, 1)==0);
  //init the strucure
  cameraInfo->focalLength.x = camInfo.focalLengthX_arg;
  cameraInfo->focalLength.y = camInfo.focalLengthY_arg;
  cameraInfo->opticalCenter.x = camInfo.opticalCenterX_arg;
  cameraInfo->opticalCenter.y = camInfo.opticalCenterY_arg;
  cameraInfo->cameraHeight = camInfo.cameraHeight_arg;
  cameraInfo->pitch = camInfo.pitch_arg * CV_PI/180;
  cameraInfo->yaw = camInfo.yaw_arg * CV_PI/180;
  cameraInfo->imageWidth = camInfo.imageWidth_arg;
  cameraInfo->imageHeight = camInfo.imageHeight_arg;
}

/**
 * @brief Scales camera parameters according to input image size
 * @param cameraInfo Camera parameters to scale (in/out)
 * @param size Target image dimensions
 */
 void imScaleCameraInfo (CameraInfo *cameraInfo, cv::Size size)
 {
  //compute the scale factor
  double scaleX = size.width/cameraInfo->imageWidth;
  double scaleY = size.height/cameraInfo->imageHeight;
  //scale
  cameraInfo->imageWidth = size.width;
  cameraInfo->imageHeight = size.height;
  cameraInfo->focalLength.x *= scaleX;
  cameraInfo->focalLength.y *= scaleY;
  cameraInfo->opticalCenter.x *= scaleX;
  cameraInfo->opticalCenter.y *= scaleY;
 }

/**
 * @brief Calculates extent of image on ground plane
 * @param cameraInfo Camera calibration parameters
 * @param ipmInfo IPM parameters structure (xLimits and yLimits are updated)
 * @details Computes the boundaries of the image when projected onto ground plane
 */
void imGetIPMExtent(const CameraInfo *cameraInfo, IPMInfo *ipmInfo )
{
// Get size of input image
    float v = cameraInfo->imageHeight;
    float u = cameraInfo->imageWidth;

    // Get the vanishing point
    cv::Point2f vp = imGetVanishingPoint(cameraInfo);
    vp.y = std::max(0.0f, vp.y);

    // Get extent of the image in the xfyf plane
    float eps = VP_PORTION * v;
    
    // Create matrix for points
    cv::Mat uvLimits = (cv::Mat_<float>(2, 4) << 
        vp.x, u, 0, vp.x,
        vp.y + eps, vp.y + eps, vp.y + eps, v);

    // Get these points on the ground plane
    cv::Mat xyLimits;
    imTransformImage2Ground(uvLimits, xyLimits, cameraInfo);

    // Get extent on the ground plane
    double xfMin, xfMax, yfMin, yfMax;
    cv::minMaxLoc(xyLimits.row(0), &xfMin, &xfMax);
    cv::minMaxLoc(xyLimits.row(1), &yfMin, &yfMax);

    // Store results
    ipmInfo->xLimits[0] = xfMin;
    ipmInfo->xLimits[1] = xfMax;
    ipmInfo->yLimits[1] = yfMax;
    ipmInfo->yLimits[0] = yfMin;
}

} // namespace LaneDetector