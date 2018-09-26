
#include "front_end.hpp"
#include "front_end_functions.cpp"

int main (int argc, char *argv[])
{
    // Check input arguments
    std::string folder;
    if(argc==1){
        folder = "../01 Datasets/The EuRoc MAV Dataset/raw/V1_01_easy/mav0";
    }
    else if(argc==2){
        folder = argv[1];
    }
    else{
        std::cout << "Error: wrong number of arguments. "
                  << "Use: './front_end <path_to_data_folder>' \n";
        return 0;
    }

    // Parse data directories
    std::vector<std::vector<std::string>> filenames(2);
    parse_directory(folder, filenames);

    // Load calibration
    camera cam0, cam1;
    std::string cam0YAML = folder+"/cam0/sensor.yaml";
    std::string cam1YAML = folder+"/cam1/sensor.yaml";
    if(loadYAML<camera>(cam0YAML, cam0) && loadYAML<camera>(cam1YAML, cam1)) {
        cam0.print();
        cam1.print();
    }

    // Convert camera object to OpenCV calibration matrices
    cv::Mat distortion_0, distortion_1,
            intrinsics_0, intrinsics_1;
    cv::eigen2cv(cam0.distortion, distortion_0);
    cv::eigen2cv(cam0.intrinsics, intrinsics_0);
    cv::eigen2cv(cam1.distortion, distortion_1);
    cv::eigen2cv(cam1.intrinsics, intrinsics_1);
    cv::Size resolution;
    resolution.width = cam0.resolution(0);
    resolution.height  = cam0.resolution(1);
    // Compute camera extrinsics (c1_R_c0, c1_t_c0)
    Eigen::Matrix3d b_R_c0 = cam0.extrinsics.block(0,0,3,3);
    Eigen::Vector3d b_t_c0 = cam0.extrinsics.block(0,3,3,1);
    Eigen::Matrix3d b_R_c1 = cam1.extrinsics.block(0,0,3,3);
    Eigen::Vector3d b_t_c1 = cam1.extrinsics.block(0,3,3,1);

    cv::Mat R, t;
    Eigen::Matrix3d c1_R_c0 = b_R_c1.transpose() * b_R_c0;
    Eigen::Vector3d c1_t_c0 = b_R_c1.transpose() * (b_t_c1 - b_t_c0);
    cv::eigen2cv(c1_R_c0, R);
    cv::eigen2cv(c1_t_c0, t);

    // Compute stereo rectification transforms
    cv::Mat R0, R1, P0, P1, Q;
    cv::stereoRectify(intrinsics_0, distortion_0, intrinsics_1, distortion_0,
                      resolution, R, t, R0, R1, P0, P1, Q);

    // Rectify stereo image pair
    cv::Mat map0x, map0y, map1x, map1y;
    cv::Mat img0, img1, imgU0, imgU1;

    img0 = cv::imread(folder + "/cam0/data/1403715274112143104.png"); // Test images, delete
    img1 = cv::imread(folder + "/cam1/data/1403715274112143104.png"); // Test images, delete

    cv::initUndistortRectifyMap(intrinsics_0, distortion_0, R0, P0, resolution, CV_32FC1, map0x, map0y);
    cv::initUndistortRectifyMap(intrinsics_1, distortion_1, R1, P1, resolution, CV_32FC1, map1x, map1y);

    cv::remap(img0, imgU0, map0x, map0y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(img1, imgU1, map1x, map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    // Draw horizontal epipolar lines
    cv::Point ptLeft, ptRight;
    ptLeft.x = 0;
    ptRight.x = resolution.width;
    int lines = 20;
    for(int i=0; i<=lines; i++){
        ptLeft.y = i * resolution.height/lines;
        ptRight.y = i * resolution.height/lines;
        cv::line(imgU0, ptLeft, ptRight, cv::Scalar(0, 255, 0));
        cv::line(imgU1, ptLeft, ptRight, cv::Scalar(0, 255, 0));
    }

    // Show original and rectified stereo pair with superimposed epipolars
    cv::Mat img, imgU, img_final;
    cv::hconcat(img0, img1, img);
    cv::hconcat(imgU0, imgU1, imgU);
    cv::vconcat(img, imgU, img_final);
    cv::imshow("Original and rectified stereo pair", img_final);
    cv::waitKey(0);

    // // Load images, rectify, detect features, extract descriptors, match keypoints
    // cv::Mat img_0, img_1;
    // for(int i=0; i<filenames[0].size(); i++){
    //     img_0 = cv::imread(folder + "/cam0/data/" + filenames[0][i]);
    //     img_1 = cv::imread(folder + "/cam0/data/" + filenames[1][i]);
    //     if( !img_0.data || !img_1.data )
    //     { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    //
    //     //-- Detect keypoints
    //     cv::Ptr<cv::ORB> detector = cv::ORB::create();
    //     std::vector<cv::KeyPoint> keypoints_0, keypoints_1;
    //     detector->detect(img_0, keypoints_0);
    //     detector->detect(img_1, keypoints_1);
    //
    //     //-- Compute descriptors
    //     cv::Mat descriptors_0, descriptors_1;
    //     detector->compute(img_0, keypoints_0, descriptors_0);
    //     detector->compute(img_1, keypoints_1, descriptors_1);
    //
    //     //-- Match keypoints (NORM_HAMMING = 6; crossCheck = true)
    //     cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    //     std::vector<cv::DMatch> matches;
    //     matcher.match(descriptors_0, descriptors_1, matches);
    //     //-- Remove outlier matches
    //     std::vector<int> delete_list;
    //     for(int i=0; i<matches.size(); i++){
    //         int index_0 = matches[i].queryIdx;
    //         int index_1 = matches[i].trainIdx;
    //         float y_0 = keypoints_0[index_0].pt.y;
    //         float y_1 = keypoints_1[index_1].pt.y;
    //         int tolerance = 10;
    //         if ((y_0-y_1)*(y_0-y_1) > tolerance*tolerance){
    //             delete_list.push_back(i);
    //         }
    //     }
    //     // std::cout << "pre-removal matches: " << matches.size() << '\n';
    //     for(int i=0; i<delete_list.size(); i++){
    //         int index = delete_list[i]-i;
    //         matches.erase(matches.begin()+index);
    //     }
    //     // std::cout << "post-removal matches: " << matches.size() << '\n';
    //
    //     //-- Draw keypoints and matches
    //     cv::Mat img_keypoints_0, img_keypoints_1, img_matches;
    //     // cv::drawKeypoints(img_0, keypoints_0, img_keypoints_0);
    //     // cv::drawKeypoints(img_1, keypoints_1, img_keypoints_1);
    //     cv::drawMatches(img_0, keypoints_0, img_1, keypoints_1, matches, img_matches);
    //
    //     //-- Show detected (drawn) keypoints
    //     // cv::Mat img_pair, img_final;
    //     // cv::hconcat(img_keypoints_0, img_keypoints_1, img_pair);
    //     // cv::vconcat(img_pair, img_matches, img_final);
    //     // cv::imshow("Keypoints 0", img_keypoints_0);
    //     // cv::imshow("Keypoints 1", img_keypoints_1);
    //     cv::imshow("Keypoint Matches", img_matches);
    //     // cv::imshow("Wassup", img_final);
    //     cv::waitKey(0);
    //
    //     cv::Mat img_0_prev, img_1_prev;
    //     img_0_prev = img_0;
    //     img_1_prev = img_1;
    // }
    return 0;
}
