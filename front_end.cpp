
#include "front_end.hpp"
#include "front_end_functions.cpp"

int main (int argc, char *argv[])
{
    // Check input arguments
    std::string dataset = "../01 Datasets/The EuRoc MAV Dataset/raw/";
    parse_input(argc, argv, dataset);

    // Parse data directories
    std::vector<std::vector<std::string>> filenames(2);
    parse_directory(dataset, filenames);

    // Create camera objects / load calibration data
    std::string cam0YAML = dataset+"/cam0/sensor.yaml";
    std::string cam1YAML = dataset+"/cam1/sensor.yaml";
    camera cam0(cam0YAML);
    camera cam1(cam1YAML);

    // Create stereo camera object
    camera_stereo cam_stereo(cam0, cam1);
    cam_stereo.print();

    // Stereo rectify test image pair
    cv::Mat img0, img1, imgU0, imgU1;
    img0 = cv::imread(dataset + "/cam0/data/1403715274112143104.png"); // Test images, delete
    img1 = cv::imread(dataset + "/cam1/data/1403715274112143104.png"); // Test images, delete
    stereo_rectify(cam_stereo, img0, img1, imgU0, imgU1);

    // Show original and rectified stereo pair with superimposed epipolars
    imshow_rectified(cam_stereo, img0, img1, imgU0, imgU1);

    // ---------------------------------------------------------------------- //
    //                          CONTINUAR AQUI AMANHÃ!                        //
    // ---------------------------------------------------------------------- //

    // // Load images, rectify, detect features, extract descriptors, match keypoints
    // cv::Mat img_0, img_1;
    // for(int i=0; i<filenames[0].size(); i++){
    //     img_0 = cv::imread(dataset + "/cam0/data/" + filenames[0][i]);
    //     img_1 = cv::imread(dataset + "/cam0/data/" + filenames[1][i]);
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
