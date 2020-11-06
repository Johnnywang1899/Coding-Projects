#include <imagePipeline.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
//#define IMAGE_TOPIC "camera/image" //Webcam

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
	printf("Shoot!");
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
	//ros::spinOnce();
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates

    // Store the image taken by the camera as img_scene
	Mat img_scene = img;

   	cv::imshow("view", img);

    // Declear two identification criteria parameters
	vector<double> stdDerivative;
	vector<int> numberOfGoodMatches;
	
    // Compare img_scene with three templates one by one (knnMatches, filter, processing)
	for (int j=0;  j<3; j++){
		Mat img_object = boxes.templates[j];

        // Threshold value for defining keypoints
		int minHessian = 400;
		Ptr<SURF> detector = SURF::create(minHessian);

		vector<KeyPoint> keypoints_object, keypoints_scene;
		Mat descriptors_object, descriptors_scene;

        // Compute keypoints and their corresponding descriptors for both img_object and img_scene
        detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
		detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);

        // knnMatch
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		std::vector< std::vector<DMatch> > knn_matches;
		matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
		
		// Filter matches using the Lowe's ratio test
		const float ratio_thresh = 0.7f;
		std::vector<DMatch> good_matches;
		for (size_t i = 0; i < knn_matches.size(); i++)
		{
			if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
			{
				good_matches.push_back(knn_matches[i][0]);
			}
		}

        // numberOfGoodMatches and stdDerivative parameters extraction from good_matches
		double sum_coorDist = 0;
		vector<double> coorDistVect;

        // Find keypoint pixel coordinates
		for (DMatch m : good_matches){
			int img1_idx = m.queryIdx;
			int img2_idx = m.trainIdx;

			float x1, y1, x2, y2;
			x1 = keypoints_object[img1_idx].pt.x;
			y1 = keypoints_object[img1_idx].pt.y;
			x2 = keypoints_scene[img2_idx].pt.x;
			y2 = keypoints_scene[img2_idx].pt.y;

			double coorDist = sqrt(pow((x1-x2),2) + pow((y1-y2),2));

			sum_coorDist += coorDist;

			coorDistVect.push_back(coorDist);
		}

		//printf("Sum distance: %f \n", sum_coorDist);

		double mean_dist = sum_coorDist/coorDistVect.size();

		double std_dev_dist = 0;

		for (double x : coorDistVect){
			std_dev_dist += pow((x - mean_dist),2);
		}

		std_dev_dist = sqrt(std_dev_dist/(coorDistVect.size() - 1));

		printf("STD DEV dist : %f \n", std_dev_dist);
		printf("Number of Matches : %d \n", coorDistVect.size());

		stdDerivative.push_back(std_dev_dist);
		numberOfGoodMatches.push_back(coorDistVect.size());

        // Draw good matches
		Mat img_matches;
		drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        imshow("Good Matches & Object detection", img_matches);
        
		ros::Duration(1).sleep();

		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for(int i = 0; i<good_matches.size(); i++){
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

        // Draw outlines for potential object location in the scene
        if (numberOfGoodMatches > 10){
		    Mat H = findHomography(obj, scene, RANSAC);

		    std::vector<Point2f> obj_corners(4);
		    obj_corners[0] = cvPoint(0,0);
		    obj_corners[1] = cvPoint(img_object.cols, 0);
		    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
		    obj_corners[3] =  cvPoint(0, img_object.rows);
		    std::vector<Point2f> scene_corners(4);

		    perspectiveTransform(obj_corners, scene_corners, H);

		    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1]+Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
		    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2]+Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
		    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3]+Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
		    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0]+Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        }

		//waitKey(0);

	}

    // Finding matching template
    for (int i = 0; i < 3; i++){
        if (stdDerivative[i] < 125 && numberOfGoodMatches[i] > 30){
		    if (template_id == -1)
            	template_id = i;
		    else if (stdDerivative[i] < stdDerivative[template_id])
		        template_id = i;
	    }
    }

	if (template_id == -1 && numberOfGoodMatches[1] > 60)
		template_id = 1;


    cv::waitKey(1);
    }
    return template_id;
}
