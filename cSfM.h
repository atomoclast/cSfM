//
// Created by Andrew Dahdouh
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <opencv/cv.hpp>


#ifndef CSFM_CSFM_H
#define CSFM_CSFM_H

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;


class ImagePose
{
public:
    Mat img;
    Mat descriptor;
    vector<KeyPoint> keypoint;
    Mat T; //pose tf matrix
    Mat P; // proejction matrix
    using num_keypoints = size_t;
    using num_landmark = size_t;
    using num_images = size_t;
    map<num_keypoints, map<num_images, num_keypoints>> keypoint_matches;
    map<num_keypoints, num_landmark> keypoint_commonpoint;

    num_keypoints& keypoint_match_idx(size_t keypoint_idx, size_t img_idx)
    {
        return keypoint_matches[keypoint_idx][img_idx];
    }
    bool kp_match_exist(size_t keypoint_idx, size_t img_idx)
    {
        return keypoint_matches[keypoint_idx].count(img_idx) > 0;
    };



};

class CommonPoint
{
public:
    Point3f point;
    int num_found; // How many times has this particular point been seen between images.
};


class SFM_Tracker
{
public:
    vector<ImagePose> vImgPose;
    vector<CommonPoint> vGlobalPoint;

};


void findMatches(vector<Mat> &images, SFM_Tracker &track)
{

    // Go through and find strong features in each image.
    // https://docs.opencv.org/3.4.1/db/d70/tutorial_akaze_matching.html

    Ptr<AKAZE> feature = AKAZE::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    cout << "Finding Keypoints."<<endl;
    for(int img =0; img < images.size(); img++)
    {
        ImagePose imPose;
        imPose.img = images[img];
        feature->detect(images[img], imPose.keypoint);
        feature->compute(images[img], imPose.keypoint, imPose.descriptor);

        track.vImgPose.emplace_back(imPose);
    }

    cout<<"Found Keypoints. Matching them now...."<<endl;

    //match keypoints...

    cout<<"Number of keypoints/poses: " << track.vImgPose.size() << endl;

    for(size_t m=0; m < track.vImgPose.size()-1; m++)
    {
        auto &img_pose_m = track.vImgPose[m];
        for (size_t n=m+1; n < track.vImgPose.size(); n++)
        {
            auto &img_pose_n = track.vImgPose[n];
            vector<vector<DMatch>> matches;
            vector<Point2f> source, destination;
            vector<uchar> mask;
            vector<int> m_keypoint, n_keypoint;

            // match keypoints within range.
            matcher -> knnMatch(img_pose_m.descriptor, img_pose_n.descriptor, matches, 2);
            for (auto &match : matches)
            {
                if(match[0].distance < 0.99*match[1].distance)
                {
                    source.push_back(img_pose_m.keypoint[match[0].queryIdx].pt);
                    destination.push_back(img_pose_n.keypoint[match[0].trainIdx].pt);

                    m_keypoint.push_back(match[0].queryIdx);
                    n_keypoint.push_back(match[0].trainIdx);
                }
            }
            cout << "find Fund Mat" << endl;
            // toss bad matches and keep good ones.
            findFundamentalMat(source, destination, FM_RANSAC, 3.0, 0.99, mask);

            Mat img_matches;
//            canvas.push_back(img_pose_n.img.clone());
//
            cout<< "about to match the images? " <<endl;

            drawMatches(img_pose_m.img, img_pose_m.keypoint, img_pose_n.img, img_pose_n.keypoint, matches, img_matches);

            resize(img_matches, img_matches, img_matches.size()/2);
            imshow("img", img_matches);
            waitKey(0);

        }

    }
}



void findKeypoints(Mat& img1, vector<KeyPoint>& keypoints_1,  Mat& img2, vector<KeyPoint>& keypoints_2,
                   vector<DMatch>& valid_matches )
{

    int minHessian = 600;
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(minHessian);

//    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    detector->detectAndCompute( img1, Mat(), keypoints_1, descriptors_1 );
    detector->detectAndCompute( img2, Mat(), keypoints_2, descriptors_2 );

    //-- Step 2: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );
    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );
    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
        { valid_matches.push_back( matches[i]); }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img1, keypoints_1, img2, keypoints_2,
                 valid_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
//    imshow( "Good Matches", img_matches );
//
//    waitKey(0);

}

//void reconstruct_image_pair(vector<Mat>& points2d, vector<Mat>& Rs_est, vector<Mat>& ts_est, Matx33d& K, vector<Mat>& points3d_estimated)
//{
//    cout << "Reconstructing...." << endl;
//}

#endif //CSFM_CSFM_H
