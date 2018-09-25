
/*---------------------------- INCLUDES/MACROS -------------------------------*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"

#include <boost/filesystem.hpp>

/* extra includes from 'inertial_alignment' (class 'camera') */
#include <math.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>


/*-------------------------------- CLASSES -----------------------------------*/

class camera{
    public:
        int rate;
        Eigen::Vector2d resolution;
        Eigen::Vector4d distortion;
        Eigen::Vector4d intrinsics;
        Eigen::Matrix4d extrinsics;
        void print();
};

/*------------------------ FUNCTION PROTOTYPES -------------------------------*/

/* File I/O */
template<class sensor> int loadYAML(std::string &filename, sensor &s);
template<> int loadYAML<camera>(std::string &filename, camera &s);
