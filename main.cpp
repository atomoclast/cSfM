#include <iostream>
#include "cSfM.h"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

using namespace cv;
using namespace std;

static void help() {
    cout
            << "\n------------------------------------------------------------------------------------\n"
            << " This program generates 3D pointclouds generated from images sets. \n"
            << " It reconstruct a scene from a set of 2D images \n"
            << " Usage:\n"
            << "        cSfM <path_to_file>\n"
            << " where: path_to_file is the file absolute path into your system which contains\n"
            << "        the list of images to use for reconstruction. \n"
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

    if(argc!=2)
    {
        help();
        exit(0);

    }

    // Open the Image Paths
    vector<String> images_paths;
    vector<Mat> images;
    getdir( argv[1], images_paths );

    cout <<"Opening Images..."<<endl;

    for(int i = 0; i<images_paths.size(); i++)
    {
        images.push_back(imread(images_paths[i], IMREAD_GRAYSCALE));
    }

    cout <<"Opened Images..."<<endl;

//    imshow("Opened first image:", images[0]);
//    waitKey(0);


    for(int i = 0; i < images.size()-1; i++) {

        cout <<"\n\n>>>Matching images " << i << " and " << i+1 << endl;

        vector<DMatch> valid_matches;
        vector<KeyPoint> keypoints_1, keypoints_2;

        cout << "Finding Keypoints..." << endl;
        findKeypoints(images[i], keypoints_1, images[i+1], keypoints_2, valid_matches);

        Mat img_matches;
        drawMatches(images[i], keypoints_1, images[i+1], keypoints_2, valid_matches, img_matches);

        for (int i = 0; i < (int) valid_matches.size(); i++) {
            printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, valid_matches[i].queryIdx,
                   valid_matches[i].trainIdx);
        }
        imshow("good matches", img_matches);
        waitKey(0);
    }



    return 0;
}
