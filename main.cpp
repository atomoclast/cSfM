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
    float f  = atof(argv[2]),
            cx = atof(argv[3]), cy = atof(argv[4]);

    Matx33d K = Matx33d( f, 0, cx,
                         0, f, cy,
                         0, 0,  1);

    cout <<"Opening Images..."<<endl;

    for(int i = 0; i<images_paths.size(); i++)
    {
        images.push_back(imread(images_paths[i], IMREAD_COLOR));
    }

    cout <<"Opened Images..."<<endl;

    cout << "Finding Matches..."<<endl;
    findMatches(images, tracker);
    cout << "Found Matches..."<<endl;







    return 0;
}
