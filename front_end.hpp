
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

class camera{
    public:
        int rate;
        Eigen::Vector2d resolution;
        Eigen::Vector4d distortion;
        Eigen::Matrix3d intrinsics;
        Eigen::Matrix4d extrinsics;
        void print();
};

/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

/* File I/O */
int parse_directory(std::string &folder, std::vector<std::vector<std::string>> &filenames);

template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);
