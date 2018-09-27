
/*---------------------------- INCLUDES/MACROS -------------------------------*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#include <boost/filesystem.hpp>

#include <Eigen/StdVector>
#include <Eigen/Dense>

#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"


/*-------------------------------- CLASSES -----------------------------------*/

typedef struct stereo_extrinsics{
    // cv::Mat R;
    // cv::Mat t;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
} camera_extrinsics;

class camera{
    public:
        int rate;
        Eigen::Vector2d resolution;
        Eigen::Vector4d distortion;
        Eigen::Matrix3d intrinsics;
        Eigen::Matrix4d extrinsics;
        camera();
        camera(std::string &filename);
        void print();
};

class camera_stereo{
    public:
        camera cam0;
        camera cam1;
        stereo_extrinsics extrinsics;
        camera_stereo();
        camera_stereo(camera &s0, camera &s1);
        void initialize(camera &s0, camera &s1);
        void print();
};

/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

/* File I/O */
int parse_input(int argc, char *argv[], std::string &dataset);
int parse_directory(std::string &folder, std::vector<std::vector<std::string>> &filenames);

template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);

/* Stereo rectification */
void stereo_rectify(camera_stereo &cam_stereo,
                    cv::Mat &img0, cv::Mat &img1,
                    cv::Mat &imgU0, cv::Mat &imgU1);

void imshow_rectified(camera_stereo &cam_stereo,
                      cv::Mat &img0, cv::Mat &img1,
                      cv::Mat &imgU0, cv::Mat &imgU1);
