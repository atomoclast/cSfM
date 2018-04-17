#include "cSfM.h"

//#include <iostream>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/core.hpp>
//#include <fstream>
//
//using namespace std;
//using namespace cv;
//using namespace cv::sfm;

#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

static void help() {
    cout
            << "\n------------------------------------------------------------------------------------\n"
            << " This program leverages the multiview reconstruction capabilities utilizing \n"
            << " GT SAM.\n"
            << " It reconstruct a scene from a set of 2D images \n"
            << " Usage:\n"
            << "         cSfM <path_to_file> <f> <cx> <cy>\n"
            << " where: path_to_file is the file absolute path into your system which contains\n"
            << "        the list of images to use for reconstruction. \n"
            << "        f  is the focal lenght in pixels. \n"
            << "        cx is the image principal point x coordinates in pixels. \n"
            << "        cy is the image principal point y coordinates in pixels. \n"
            << "------------------------------------------------------------------------------------\n\n"
            << endl;
}

static int getdir(const string _filename, vector<String> &files)
{
    ifstream myfile(_filename.c_str());
    if (!myfile.is_open()) {
        cout << "Unable to read file: " << _filename << endl;
        exit(0);
    } else {;
        size_t found = _filename.find_last_of("/\\");
        string line_str, path_to_file = _filename.substr(0, found);
        while ( getline(myfile, line_str) )
            files.push_back(path_to_file+string("/")+line_str);
    }
    return 1;
}

int main(int argc, char** argv ) {

    if(argc!=5)
    {
        help();
        exit(0);

    }

    // Open the Image Paths
    SFM_Tracker tracker;

    vector<String> images_paths;
    vector<Mat> images;

    getdir( argv[1], images_paths );

    // Build intrinsics
    double f  = atof(argv[2]),
            cx = atof(argv[3]), cy = atof(argv[4]);

    Mat cvK = Mat::eye(3, 3, CV_64F);
    cvK.at<double>(0,0) = f;
    cvK.at<double>(1,1) = f;
    cvK.at<double>(0,2) = cx;
    cvK.at<double>(1,2) = cy;

    cout <<"Opening Images..."<<endl;

    for(int i = 0; i<images_paths.size(); i++)
    {
        images.push_back(imread(images_paths[i], IMREAD_COLOR));
    }

    cout <<"Opened Images..."<<endl;

    cout << "Finding Matches..."<<endl;
    findMatches(images, tracker);
    cout << "Found Matches..."<<endl;

//    imshow("Test?", tracker.vImgPose[0].img);
//    waitKey(0);

    // Triangulate Points in space.
    triangulateSFMPoints(tracker, cvK);

//    double f = kMat.at<double>(0,0);
//    double cx = kMat.at<double>(0,2);
//    double cy = kMat.at<double>(1,2);

    gtsam::Values result;
    gtsam::Cal3_S2 K(f, f, 0, cx, cy); //Skew matrix
    bundleAdjustment(tracker, K,result);

//    generateOutputs(tracker, K, result, images_paths);

    {
        using namespace gtsam;

        Matrix3 K_refined = result.at<Cal3_S2>(Symbol('K', 0)).K();
        cout << "Resultant Camera Matrix K: " << endl << K_refined << endl;

        system("mkdir -p output/visualize");
        system("mkdir -p output/txt");
        system("mkdir -p output/models");

        ofstream option("output/options.txt");

        // Should be size()-1? Or just size()????
        option << "timages -1 " << 0 << " " << (tracker.vImgPose.size()) << endl;
        option << "oimages 0" << endl;
        option << "level 1" << endl;

        option.close();

        for (size_t i = 0; i < tracker.vImgPose.size(); i++) {
            Eigen::Matrix<double, 3, 3> R;
            Eigen::Matrix<double, 3, 1> t;
            Eigen::Matrix<double, 3, 4> P;
            char str[256];

            R = result.at<Pose3>(Symbol('x', i)).rotation().matrix();
            t = result.at<Pose3>(Symbol('x', i)).translation().vector();

            P.block(0, 0, 3, 3) = R.transpose();
            P.col(3) = -R.transpose() * t;
            P = K_refined * P;

            cout <<"Copying image."<<endl;

            sprintf(str, "cp -f %s output/visualize/%04d.jpg", images_paths[i].c_str(), (int)i);
            system(str);

            cout <<"Copying image."<<endl;

            sprintf(str, "output/txt/%04d.txt", (int) i);
            ofstream out(str);

            out << "CONTOUR" << endl;

            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 4; k++) {
                    out << P(j, k) << " ";
                }
                out << endl;
            }
        }
    }

    return 0;
}
