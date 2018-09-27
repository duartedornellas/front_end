
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

    // Load images, rectify, detect features, extract descriptors, match keypoints
    cv::Mat img0, img1, imgU0, imgU1;
    for(int i=0; i<filenames[0].size(); i++){
        img0 = cv::imread(dataset + "/cam0/data/" + filenames[0][i]);
        img1 = cv::imread(dataset + "/cam1/data/" + filenames[1][i]);
        if( !img0.data || !img1.data )
        { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

        // Rectify stereo image pair
        stereo_rectify(cam_stereo, img0, img1, imgU0, imgU1);

        //-- Detect keypoints
        cv::Ptr<cv::ORB> detector = cv::ORB::create();
        detector->setFastThreshold(15);
        detector->setMaxFeatures(150);
        std::vector<cv::KeyPoint> keypoints_0, keypoints_1;
        detector->detect(imgU0, keypoints_0);
        detector->detect(imgU1, keypoints_1);

        //-- Compute descriptors
        cv::Mat descriptors_0, descriptors_1;
        detector->compute(imgU0, keypoints_0, descriptors_0);
        detector->compute(imgU1, keypoints_1, descriptors_1);

        //-- Match keypoints (NORM_HAMMING = 6; crossCheck = true)
        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors_0, descriptors_1, matches);
        epipolar_check(matches, keypoints_0, keypoints_1);

        //-- Draw keypoint matches. Uncomment below for horizontal epipolar grid
        cv::Mat img_keypoints_0, img_keypoints_1, img_matches;
        cv::drawMatches(imgU0, keypoints_0, imgU1, keypoints_1, matches, img_matches);
        // draw_lines(img_matches);

        //-- Show matched keypoints / get keyboard input for program execution
        cv::imshow("Isochronous Keypoint Matches", img_matches);
        if (imshow_quit()){
            break;
        }

        //--- Update image buffers
        // cv::Mat img0_prev, img1_prev;
        // img0_prev = img0;
        // img1_prev = img1;
    }
    return 0;
}
