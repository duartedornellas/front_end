
/*---------------------------- CLASS METHODS ---------------------------------*/

/* Camera */

camera::camera(){}
camera::camera(std::string &filename){
    loadYAML<camera>(filename, *this);
}

void camera::print(){
    std::cout << "Rate: \n" << this->rate << '\n'
              << "Resolution: \n" << this->resolution.transpose() << '\n'
              << "Distortion parameters: \n" << this->distortion.transpose() << '\n'
              << "Intrinsics: \n"   << this->intrinsics << '\n'
              << "Extrinsics (camera to IMU): \n" << this->extrinsics << "\n\n";
}

/* Stereo camera pair */
camera_stereo::camera_stereo(){}
camera_stereo::camera_stereo(camera &s0, camera &s1){
    this->initialize(s0, s1);
}

void camera_stereo::print(){
    std::cout << "Extrinsics (camera to camera): \n"
    << "t: \n" << this->extrinsics.t.transpose() << '\n'
    << "R: \n" << this->extrinsics.R << "\n\n";
    std::cout << "Camera 0: \n";
    this->cam0.print();
    std::cout << "Camera 1: \n";
    this->cam1.print();
}

void camera_stereo::initialize(camera &s0, camera &s1){

    this->cam0 = s0;
    this->cam1 = s1;

    Eigen::Matrix3d b_R_c0 = this->cam0.extrinsics.block(0,0,3,3);
    Eigen::Vector3d b_t_c0 = this->cam0.extrinsics.block(0,3,3,1);
    Eigen::Matrix3d b_R_c1 = this->cam1.extrinsics.block(0,0,3,3);
    Eigen::Vector3d b_t_c1 = this->cam1.extrinsics.block(0,3,3,1);

    Eigen::Matrix3d c1_R_c0 = b_R_c1.transpose() * b_R_c0;
    Eigen::Vector3d c1_t_c0 = b_R_c1.transpose() * (b_t_c1 - b_t_c0);

    // To use with 'extrinsics' struct with 'cv::Mat' elements
    // cv::Mat R, t;
    // cv::eigen2cv(c1_R_c0, R);
    // cv::eigen2cv(c1_t_c0, t);
    //
    // this->extrinsics.R = R;
    // this->extrinsics.t = t;

    // To use with 'extrinsics' struct with 'Eigen::Matrix' elements
    this->extrinsics.R = c1_R_c0;
    this->extrinsics.t = c1_t_c0;
}

/*------------------------------ FUNCTIONS -----------------------------------*/

/* File I/O */
int parse_input(int argc, char *argv[], std::string &dataset){
    if(argc==1){
        dataset += "V1_01_easy";
    }
    else if(argc==2){
        dataset += argv[1];
    }
    else{
        std::cout << "Error: wrong number of arguments. "
                  << "Use: './front_end <path_to_data_c>' \n";
        return 0;
    }
    dataset += "/mav0";
    return 1;
}

int parse_directory(std::string &folder, std::vector<std::vector<std::string>> &filenames){
    boost::filesystem::path p0(folder + "/cam0/data");
    boost::filesystem::path p1(folder + "/cam1/data");
    for (auto i  = boost::filesystem::directory_iterator(p0);
              i != boost::filesystem::directory_iterator(); i++){
        if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
        {filenames[0].push_back(i->path().filename().string());}
    }
    std::sort(filenames[0].begin(),filenames[0].end());

    for (auto i  = boost::filesystem::directory_iterator(p1);
              i != boost::filesystem::directory_iterator(); i++){
        if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
        {filenames[1].push_back(i->path().filename().string());}
    }
    std::sort(filenames[1].begin(),filenames[1].end());

    // Check parsing went ok / dataset is synchronized
    if(filenames[0]==filenames[1]){
        return 1;
    }
    else{
        std::cout << "Error: Unsynchronized image pair.\n";
        return 0;
    }
}

template<> int loadYAML<camera>(std::string &filename, camera &s){
    std::string line;
    std::ifstream filestream (filename.c_str());
    if (filestream.is_open()){
        std::cout << "Success opening file '" << filename << "'.";
        while(getline(filestream, line)){
            std::stringstream lineStream(line);
            lineStream >> std::ws;
            std::string field, cell;
            std::getline(lineStream, field, ' ');
            while(std::getline(lineStream, cell, ' ')){
                if(field!="sensor_type:" &&
                   field!="data:" &&
                   field!="rate_hz:" &&
                   field!="resolution:" &&
                   field!="intrinsics:" &&
                   field!="distortion_coefficients:"){
                    break;
                }

                if(field=="sensor_type:"){
                    std::getline(lineStream, cell, ' ');
                    if(cell!="camera"){
                        std::cout << "Error: Incompatible yaml file/argument. "
                                  << "'sensor_type': " << cell << "; "
                                  << "'input_type': 'camera' \n";
                        filestream.close();
                        return 0;
                    }
                    break;
                }
                else if(field=="data:"){
                    field.clear();
                    int i=0, j=0;
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    s.extrinsics(i,j) = atof(cell.c_str());
                    j++;
                    while(std::getline(lineStream, cell, ',')){
                        s.extrinsics(i,j) = atof(cell.c_str());
                        j++;
                    }
                    j=0;
                    i++;
                    std::cout << '\n';
                    while(i<4){
                        getline(filestream, line);
                        std::stringstream lineStream(line);
                        lineStream >> std::ws;
                        std::string cell;
                        while(std::getline(lineStream, cell, ',')){
                            s.extrinsics(i,j) = atof(cell.c_str());
                            j++;
                        }
                        j=0;
                        i++;
                    }
                    break;
                }
                else if(field=="rate_hz:"){
                    s.rate = atof(cell.c_str());
                }
                else if(field=="resolution:"){
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    s.resolution(i) = atof(cell.c_str());
                    i++;
                    while(std::getline(lineStream, cell, ',')){
                        s.resolution(i) = atof(cell.c_str());
                        i++;
                    }
                    break;
                }
                else if(field=="intrinsics:"){
                    Eigen::Vector4d buffer;
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    buffer(i) = atof(cell.c_str());
                    i++;
                    while(i<4){
                        std::getline(lineStream, cell, ',');
                        buffer(i) = atof(cell.c_str());
                        i++;
                    }
                    s.intrinsics << buffer(0),         0, buffer(2),
                                            0, buffer(1), buffer(3),
                                            0,         0,         1;
                    break;
                }
                else if(field=="distortion_coefficients:"){
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    s.distortion(i) = atof(cell.c_str());
                    i++;
                    while(i<4){
                        std::getline(lineStream, cell, ',');
                        s.distortion(i) = atof(cell.c_str());
                        i++;
                    }
                    break;
                }
            }
        }

        filestream.close();
        return 1;
    }
    else{
        std::cout << "Error opening file '" << filename << "'\n";
        return 0;
    }
}

/* Stereo rectification */
void stereo_rectify(camera_stereo &cam_stereo,
    cv::Mat &img0, cv::Mat &img1,
    cv::Mat &imgU0, cv::Mat &imgU1){
        // Convert camera object to OpenCV calibration matrices
        cv::Mat intrinsics_0, distortion_0;
        cv::eigen2cv(cam_stereo.cam0.intrinsics, intrinsics_0);
        cv::eigen2cv(cam_stereo.cam0.distortion, distortion_0);

        cv::Mat intrinsics_1, distortion_1;
        cv::eigen2cv(cam_stereo.cam1.intrinsics, intrinsics_1);
        cv::eigen2cv(cam_stereo.cam1.distortion, distortion_1);

        cv::Size resolution(cam_stereo.cam0.resolution(0),
        cam_stereo.cam0.resolution(1));

        cv::Mat R, t;
        cv::eigen2cv(cam_stereo.extrinsics.R, R);
        cv::eigen2cv(cam_stereo.extrinsics.t, t);

        // Compute stereo rectification transforms
        cv::Mat R0, R1, P0, P1, Q;
        cv::stereoRectify(intrinsics_0, distortion_0, intrinsics_1, distortion_1,
            resolution, R, t, R0, R1, P0, P1, Q);

            // Rectify stereo image pair
            cv::Mat map0x, map0y, map1x, map1y;
            cv::initUndistortRectifyMap(intrinsics_0, distortion_0, R0, P0, resolution, CV_32FC1, map0x, map0y);
            cv::initUndistortRectifyMap(intrinsics_1, distortion_1, R1, P1, resolution, CV_32FC1, map1x, map1y);
            cv::remap(img0, imgU0, map0x, map0y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
            cv::remap(img1, imgU1, map1x, map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        }

/* Feature matching */
void epipolar_check(std::vector<cv::DMatch> &matches,
                    std::vector<cv::KeyPoint> &keypoints_0,
                    std::vector<cv::KeyPoint> &keypoints_1){
    std::vector<int> delete_list;
    for(int i=0; i<matches.size(); i++){
        int index_0 = matches[i].queryIdx;
        int index_1 = matches[i].trainIdx;
        float x_0 = keypoints_0[index_0].pt.x;
        float x_1 = keypoints_1[index_1].pt.x;
        float y_0 = keypoints_0[index_0].pt.y;
        float y_1 = keypoints_1[index_1].pt.y;
        int tolerance = 1;
        if ((y_0-y_1)*(y_0-y_1) > tolerance*tolerance ||   // outside epipolars
             x_0 < x_1){                                   // negative disparity
            delete_list.push_back(i);
        }
    }
    // std::cout << "pre-removal matches: " << matches.size() << '\n';
    for(int i=0; i<delete_list.size(); i++){
        int index = delete_list[i]-i;
        matches.erase(matches.begin()+index);
    }
    // std::cout << "post-removal matches: " << matches.size() << '\n';
}

/* User interface */

void draw_lines(cv::Mat &img){
    cv::Point ptLeft, ptRight;
    ptLeft.x = 0;
    ptRight.x = img.cols;
    int lines = 20;
    for(int i=0; i<=lines; i++){
        ptLeft.y = i * img.rows/lines;
        ptRight.y = ptLeft.y;
        cv::line(img, ptLeft, ptRight, cv::Scalar(0, 255, 0));
    }
}

void imshow_rectified(camera_stereo &cam_stereo,
                      cv::Mat &img0, cv::Mat &img1,
                      cv::Mat &imgU0, cv::Mat &imgU1){
        // Draw horizontal epipolar lines on rectified image pair
        cv::Point ptLeft, ptRight;
        ptLeft.x = 0;
        ptRight.x = cam_stereo.cam0.resolution(0);
        int lines = 20;
        for(int i=0; i<=lines; i++){
            ptLeft.y = i * cam_stereo.cam0.resolution(1)/lines;
            ptRight.y = ptLeft.y;
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
    }

void imshow_matches(cv::Mat &imgU0, std::vector<cv::KeyPoint> &keypoints_0,
                   cv::Mat &imgU1, std::vector<cv::KeyPoint> &keypoints_1,
                   std::vector<cv::DMatch> &matches){
    cv::Mat imgU01;
    cv::addWeighted(imgU0, 0.5, imgU1, 0.5, 0.0, imgU01);

    std::vector<cv::Point2f> matches_00, matches_01;
    for(std::vector<cv::DMatch>::iterator it = matches.begin(); it != matches.end(); ++it) {
        // Check the correspondence of 'matches_01' and 'keypoints_0[it->queryIdx]
        //
        // 'keypoints_0' was input as the query image,
        // 'keypoints_1' was input as the train image  in:
        // 'matcher.match(descriptors_0, descriptors_1, matches)';
        //
        // 'query' means known beforehand (order is switched since matching...)
        //
        matches_00.push_back(keypoints_1[it->trainIdx].pt);
        matches_01.push_back(keypoints_0[it->queryIdx].pt);
        cv::rectangle(imgU01, cv::Point(matches_00.back().x-2, matches_00.back().y-2),
                              cv::Point(matches_00.back().x+2, matches_00.back().y+2),
                              cv::Scalar(255,0,0), 1, 8, 0);
        cv::rectangle(imgU01, cv::Point(matches_01.back().x-2, matches_01.back().y-2),
                              cv::Point(matches_01.back().x+2, matches_01.back().y+2),
                              cv::Scalar(0,0,255), 1, 8, 0);
        cv::arrowedLine(imgU01, matches_00.back(), matches_01.back(), cv::Scalar(0,255,0), 1, 8, 0, 0.125);
    }
    cv::imshow("Keypoint matches", imgU01);
}

int  imshow_quit(){
    int key = cv::waitKey(30);
    if (key == 32){                         // 'spacebar' to pause
        key = -1;
        while(key != 32){
            key = cv::waitKey(0);
            if (key == 27 || key == 113){   // 'q' or 'ESC' to quit (paused)
                return 1;
            }
        }
    }
    if (key == 27 || key == 113){           // 'q' or 'ESC' to quit
        return 1;
    }
    return 0;
}
