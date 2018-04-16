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
// Can't have this....build errors due to ambiguous naming of variables...
//using namespace gtsam;


class ImagePose
{
public:
    Mat img;
    Mat descriptor;
    vector<KeyPoint> keypoints;
    Mat T; //pose tf matrix
    Mat P; // projection matrix
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

    num_landmark& keypoint_3d(size_t kp_idx)
    {
        return keypoint_commonpoint[kp_idx];
    }
    bool keypoint_3d_exist(size_t kp_idx) {
        return keypoint_commonpoint.count(kp_idx) > 0;
    }
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
        feature->detect(images[img], imPose.keypoints);
        feature->compute(images[img], imPose.keypoints, imPose.descriptor);

        //Place image into vector of images.
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
                    source.push_back(img_pose_m.keypoints[match[0].queryIdx].pt);
                    destination.push_back(img_pose_n.keypoints[match[0].trainIdx].pt);

                    m_keypoint.push_back(match[0].queryIdx);
                    n_keypoint.push_back(match[0].trainIdx);
                }
            }
            cout << "find Fund Mat" << endl;
            // toss bad matches and keep good ones.
            findFundamentalMat(source, destination, FM_RANSAC, 3.0, 0.99, mask);
            cout << "Pushing Keypoints into vectors. Size of mask: " << mask.size() << endl;
//            cout <<"mask[0]: " << mask[0]<<endl;
            for(size_t i=0; i < mask.size(); i++)
            {
                if(mask[i])
                {
                    img_pose_m.keypoint_match_idx(m_keypoint[i], n) = n_keypoint[i];
                    img_pose_n.keypoint_match_idx(n_keypoint[i], m) = m_keypoint[i];

//                    cout<< n_keypoint[i] << " " << m_keypoint[i]<<endl;

                }
            }

            // Match images and visualize it:
//            Mat img_matches;
//            cout<< "about to match the images? " <<endl;
//            drawMatches(img_pose_m.img, img_pose_m.keypoints, img_pose_n.img, img_pose_n.keypoints, matches, img_matches);
//            resize(img_matches, img_matches, img_matches.size()/2);
//            imshow("img", img_matches);
//            waitKey(0);

        }
    }
}


void triangulateSFMPoints(SFM_Tracker &track, Mat K)
{
    Point2d pp(K.at<double>(0,2), K.at<double>(1,2)); //cx and cy
    cout << endl << "Initial Camera Matrix K:" << endl << K << endl << endl;

    track.vImgPose[0].T = Mat::eye(4, 4, CV_64F);
    track.vImgPose[0].P = K*Mat::eye(3, 4, CV_64F);

    int w = track.vImgPose[0].img.size().width;
    int h = track.vImgPose[0].img.size().height;

    cout <<"Image width and height: " << w << ", " << h << endl;
    cout<< "Size of keypoint matches: " << track.vImgPose[0].keypoint_matches.size() << endl;
    cout << "Iterating through images now..." << endl;
    for (size_t i=0; i < track.vImgPose.size() - 1; i++) {
        auto &prev = track.vImgPose[i];
        auto &cur = track.vImgPose[i+1];

        vector<Point2f> source, destination;
        vector<size_t> keypoint_used;

        cout<<"iterating through keypoints." << endl;
        for (size_t k=0; k < prev.keypoints.size(); k++) {
            if (prev.kp_match_exist(k, i+1)) {
                size_t match_idx = prev.keypoint_match_idx(k, i+1);

                source.push_back(prev.keypoints[k].pt);
                destination.push_back(cur.keypoints[match_idx].pt);

                keypoint_used.push_back(k);
            }
        }


//        for( unsigned int i = 0; i < matches.size(); i++)
//        {
//            //queryidx -> left image
//            imgPts1.push_back(imgKeyPts[0][matches[i].queryIdx].pt);
//            //trainidx -> right image
//            imgPts2.push_back(imgKeyPts[1][matches[i].trainIdx].pt);
//        }
//
//
//        Mat F = findFundamentalMat(imgPts1, imgPts2, FM_RANSAC, 0.1, 0.99);
//
//        cout<<"F type " << CV_MAT_TYPE(F.type())<<endl;
//        cout<<"K type " << K.type()<<endl;
//        cout<<"K inv type " << Kinv.type()<<endl;
//        cout<<"K t type "<< (K.t()).type()<<endl;

        Mat status;
        // NOTE: pose from destination to source
        cout << "Finding essential Matrix." << endl;
        cout << "Size of destination: " << destination.size() << ", Size of source: " << source.size() << endl;
        Mat E = findEssentialMat(destination, source, K.at<double>(0,0), pp, RANSAC, 0.999, 1.0, status);
        Mat local_R, local_t;

        cout<< "Recovering Pose." << endl;
        // https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        recoverPose(E, destination, source, local_R, local_t, K.at<double>(0,0), pp, status);

        // local tansform
        Mat T = Mat::eye(4, 4, CV_64F);
        local_R.copyTo(T(Range(0, 3), Range(0, 3)));
        local_t.copyTo(T(Range(0, 3), Range(3, 4)));

        // accumulate transform
        cur.T = prev.T*T;

        // make projection matrix
        Mat R = cur.T(Range(0, 3), Range(0, 3));
        Mat t = cur.T(Range(0, 3), Range(3, 4));

        Mat P(3, 4, CV_64F);

        P(Range(0, 3), Range(0, 3)) = R.t();
        P(Range(0, 3), Range(3, 4)) = -R.t()*t;
        P = K*P;

        cur.P = P;

        Mat points4D;
        triangulatePoints(prev.P, cur.P, source, destination, points4D);

        // Scale the new 3d points to be similar to the existing 3d points (landmark)
        // Use ratio of distance between pairing 3d points
        if (i > 0) {
            double scale = 0;
            int count = 0;

            Point3f prev_camera;

            prev_camera.x = prev.T.at<double>(0, 3);
            prev_camera.y = prev.T.at<double>(1, 3);
            prev_camera.z = prev.T.at<double>(2, 3);

            vector<Point3f> new_pts;
            vector<Point3f> existing_pts;

            for (size_t j=0; j < keypoint_used.size(); j++) {
                size_t k = keypoint_used[j];
                if (status.at<uchar>(j) && prev.kp_match_exist(k, i+1) && prev.keypoint_3d_exist(k)) {
                    Point3f pt3d;

                    pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                    pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
                    pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);

                    size_t idx = prev.keypoint_3d(k);
                    Point3f avg_landmark = track.vGlobalPoint[idx].point / (track.vGlobalPoint[idx].num_found - 1);

                    new_pts.push_back(pt3d);
                    existing_pts.push_back(avg_landmark);
                }
            }

            // ratio of distance for all possible point pairing
            // probably an over kill! can probably just pick N random pairs
            for (size_t j=0; j < new_pts.size()-1; j++) {
                for (size_t k=j+1; k< new_pts.size(); k++) {
                    double s = norm(existing_pts[j] - existing_pts[k]) / norm(new_pts[j] - new_pts[k]);

                    scale += s;
                    count++;
                }
            }

            assert(count > 0);

            scale /= count;

            cout << "image " << (i+1) << " ==> " << i << " scale=" << scale << " count=" << count <<  endl;

            // apply scale and re-calculate T and P matrix
            local_t *= scale;

            // local tansform
            Mat T = Mat::eye(4, 4, CV_64F);
            local_R.copyTo(T(Range(0, 3), Range(0, 3)));
            local_t.copyTo(T(Range(0, 3), Range(3, 4)));

            // accumulate transform
            cur.T = prev.T*T;

            // make projection ,matrix
            R = cur.T(Range(0, 3), Range(0, 3));
            t = cur.T(Range(0, 3), Range(3, 4));

            Mat P(3, 4, CV_64F);
            P(Range(0, 3), Range(0, 3)) = R.t();
            P(Range(0, 3), Range(3, 4)) = -R.t()*t;
            P = K*P;

            cur.P = P;

            triangulatePoints(prev.P, cur.P, source, destination, points4D);
        }

        // Find good triangulated points
        for (size_t j=0; j < keypoint_used.size(); j++) {
            if (status.at<uchar>(j)) {
                size_t k = keypoint_used[j];
                size_t match_idx = prev.keypoint_match_idx(k, i+1);

                Point3f pt3d;

                pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
                pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);

                if (prev.keypoint_3d_exist(k)) {
                    // Found a match with an existing landmark
                    cur.keypoint_3d(match_idx) = prev.keypoint_3d(k);

                    track.vGlobalPoint[prev.keypoint_3d(k)].point += pt3d;
                    track.vGlobalPoint[cur.keypoint_3d(match_idx)].num_found++;
                } else {
                    // Add new 3d point
                    CommonPoint commonPoint;

                    commonPoint.point = pt3d;
                    commonPoint.num_found = 2;

                    track.vGlobalPoint.push_back(commonPoint);

                    prev.keypoint_3d(k) = track.vGlobalPoint.size() - 1;
                    cur.keypoint_3d(match_idx) = track.vGlobalPoint.size() - 1;
                }
            }
        }
    }

    // Average out the landmark 3d position
    for (auto &l : track.vGlobalPoint) {
        if (l.num_found >= 3) {
            l.point /= (l.num_found - 1);
        }
    }

}

//void bundleAdjustment(SFM_Tracker &track, Mat K, gtsam::Values& result)
//{
//    using namespace gtsam;
//    cout << "Running Bundle Adjustment now." << endl;
//    double f = K.at<double>(0,0);
//    double cx = K.at<double>(0,2);
//    double cy = K.at<double>(1,2);
//
//    // Define the camera observation noise model.
//    Cal3_S2::shared_ptr Kay(f, f, 0, cx, cy); //Skew matrix
//
//    // Create the camera observation noise model.
////    noiseModel::Isotropic:shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
//    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 2.0);
//
//    // Create a factor graph
//    NonlinearFactorGraph graph;
//    Values initial;
//
//}


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
