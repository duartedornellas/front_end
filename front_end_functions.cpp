
/*---------------------------- CLASS METHODS ---------------------------------*/

/* Camera */
void camera::print(){
    std::cout << '\n'
              << "Rate: \n" << this->rate << '\n'
              << "Resolution: \n" << this->resolution.transpose() << '\n'
              << "Distortion parameters: \n" << this->distortion.transpose() << '\n'
              << "Intrinsics: \n"   << this->intrinsics.transpose() << '\n'
              << "Extrinsics: \n" << this->extrinsics << "\n\n";
}

/*------------------------------ FUNCTIONS -----------------------------------*/

/* File I/O */
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
                    field.clear();
                    cell.erase(cell.begin()); // delete '[' on 1st element
                    cell.pop_back();          // delete ',' on 1st element
                    int i=0;
                    s.intrinsics(i) = atof(cell.c_str());
                    i++;
                    while(i<4){
                        std::getline(lineStream, cell, ',');
                        s.intrinsics(i) = atof(cell.c_str());
                        i++;
                    }
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
