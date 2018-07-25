/* User code: This file will not be overwritten by TASTE. */

#include "aruco_marker_detector.h"
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "Context-aruco-marker-detector.h"
#include <base/samples/RigidBodyState.hpp>
#include <base_support/Base-samples-RigidBodyStateConvert.hpp>

using namespace cv;
aruco::MarkerDetector  detector;
VideoCapture cap; // open the default camera
aruco::CameraParameters calib;
void aruco_marker_detector_startup()
{
    /* Write your initialization code here,
       but do not make any call to a required interface. */
	detector.setThresholdMethod(detector.ADPT_THRES);
    detector.setThresholdParams(7, 7);
    detector.setCornerRefinementMethod(detector.SUBPIX);
    detector.setMinMaxSize(0.04, 0.5);
    detector.enableErosion(false);
    detector.setWarpSize(56);
    detector.pyrDown(0);
    if(aruco_marker_detector_ctxt.draw_augmented_image){
    	namedWindow("markers", 0);
    }
    cap = VideoCapture(aruco_marker_detector_ctxt.capture_device_id);
    if(!cap.isOpened()){  // check if we succeeded
    	std::cerr << "Failed to open camera" << std::endl;
    	return;
    }
    int width = 640; 
    int height =  360; 

    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);

    double fx = 387.12433;
    double fy = 387.56096;
    double cx = 327.92965;
    double cy = 182.68857;

    double d0 = 0.05945;
    double d1 =  -0.11467;
    double d2 =  -0.00396;
    double d3 = -0.00003;
  
    Mat distortionCoefficients = (Mat1d(1, 4) << d0, d1, d2, d3);
    Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    calib.setParams(cameraMatrix, distortionCoefficients, Size(width, height));
}

void aruco_marker_detector_PI_trigger()
{
    if(!cap.isOpened()){  // check if we succeeded
    	std::cout << "Camera was not opened" << std::endl;
    	return;
    }
       
    Mat frame;
    cap >> frame; // get a new frame from camera
    std::vector<aruco::Marker> detected_markers;
    detector.detect(frame, detected_markers, calib, 
    	            aruco_marker_detector_ctxt.marker_size);
    for(size_t i=0; i<detected_markers.size(); i++){
        detected_markers[i].draw(frame, cv::Scalar(255,0,0));
        aruco::CvDrawingUtils::draw3dCube(frame, detected_markers[i], calib);
        aruco::CvDrawingUtils::draw3dAxis(frame, detected_markers[i], calib);
    }

    if(detected_markers.size() > 0){
        double position[3];
        double orientation[4];
        detected_markers[0].OgreGetPoseParameters(position, orientation);
        asn1SccBase_samples_RigidBodyState rbs_a;

        base::samples::RigidBodyState rbs;
        rbs.time = base::Time::now();
        rbs.sourceFrame = "marker";
        rbs.targetFrame = "camera";
        rbs.position.x() = position[0];
        rbs.position.y() = position[1];
        rbs.position.z() = position[2];
        rbs.orientation.w() = orientation[0];
    	rbs.orientation.x() = orientation[1];
    	rbs.orientation.y() = orientation[2];
    	rbs.orientation.z() = orientation[3];
    	asn1SccBase_samples_RigidBodyState_toAsn1(rbs_a, rbs);

    	aruco_marker_detector_RI_pose(&rbs_a);
    }

    if(aruco_marker_detector_ctxt.draw_augmented_image){
    	imshow("markers", frame);
    	waitKey(10);
    }
}

