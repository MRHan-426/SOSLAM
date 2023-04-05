//
// Created by ziqihan on 4/4/23.
//
#include "HandMadeData.h"

namespace gtsam_soslam{

HandMadeData::HandMadeData(const std::string& img_path, const std::string& xml_path, const std::string& calib_file, const std::string& odom_file)
            : img_path_(img_path),xml_path_(xml_path),calib_file_(calib_file)
{
    i = 0;
    if (file_num(img_path) == 0 || file_num(xml_path) == 0){
        std::cerr << "WARN: there be no imgs or xmls in the folder path, please check." << std::endl;
    }
    else if (file_num(img_path) != file_num(xml_path)){
        std::cerr << "WARN: number of images and xmls does not match, please check." << std::endl;
    }
    else{
        num_img_ = file_num(img_path);
    }

    std::ifstream infile(odom_file);

    if (infile.is_open()) {
        std::string line;
        while (std::getline(infile, line)) {
            odom_data_.push_back(line);
        }
        infile.close();
        if (odom_data_.size() != static_cast<size_t>(num_img_)){
            std::cerr << "length of image and odom does not match, please check." << std::endl;
        }
    } else {
        std::cerr << "WARN: cannot open odom.txt, please check." << std::endl;
    }

}

int HandMadeData::file_num(const std::string& folder_path){
    DIR *dir;
    struct dirent *ent;
    struct stat buf;
    int file_count = 0;
    if ((dir = opendir(folder_path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string full_path = folder_path + "/" + ent->d_name;
            if (stat(full_path.c_str(), &buf) == 0 && S_ISREG(buf.st_mode)) {
                file_count++;
            }
        }
        closedir(dir);
        return file_count;
    }
    else{
        return int(0);
    }
}
gtsam::Vector5 HandMadeData::calib_rgb() {
    try {
        std::ifstream infile(calib_file_);
        if (!infile.is_open()) {
            throw std::runtime_error("Failed to open file!");
        }
        std::string line;
        gtsam::Vector5 result;
        // Just read the first line
        getline(infile, line);
        std::istringstream iss(line);
        double num;
        for (int j = 0; j < 5 && iss >> num; j++) {
            result(j) = num;
        }
        infile.close();
        return result;
    }
    catch (std::exception& e) {
       std::cout << "Use backup calibration parameters" << std::endl;
        gtsam::Vector5 result;
        result << 525.0, 525.0, 0.0, 160.0, 120.0;
        return result;
    }
}

std::tuple<gtsam::Pose3, gtsam::Matrix3 , cv::Mat> HandMadeData::next(SoSlamState& state)
{
    std::vector<std::string> parts;
    std::stringstream ss(odom_data_[i]); //start from 0
    std::string token;
    while (ss >> token) {
    parts.push_back(token);
    }
    auto tx = std::stod(parts[1]);
    auto ty = std::stod(parts[2]);
    auto tz = std::stod(parts[3]);
    auto qx = std::stod(parts[4]);
    auto qy = std::stod(parts[5]);
    auto qz = std::stod(parts[6]);
    auto qw = std::stod(parts[7]);
    gtsam::Pose3 pose(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));

    ++i;

    std::string img_name = img_path_ + std::to_string(i) + ".png"; //start from 1.png
    gtsam::Matrix3 mat = gtsam::Matrix3::Zero();
    cv::Mat img = cv::imread(img_name, cv::IMREAD_COLOR); //BGR
    return std::make_tuple(pose, mat, img);
}


std::vector<Detection> HandMadeDetector::detect(SoSlamState& state)
{
    ++i;
    std::vector<Detection> detections;

    std::string xml_name = xml_path_ + std::to_string(i) + ".xml"; //start from 1.xml

    pugi::xml_document doc;
    if (!doc.load_file(xml_name.c_str())) {
        std::cerr << "WARN: cannot open xml file, please check." << std::endl;
    }
    pugi::xml_node root = doc.child("annotation");

    for (pugi::xml_node object = root.child("object"); object; object = object.next_sibling("object")) {

        pugi::xml_node bndbox = object.child("bndbox");
        pugi::xml_node name = object.child("name");
        pugi::xml_node objectkey = object.child("objectkey");

        auto key = std::stoull(objectkey.text().get());
        gtsam::Key quadrickey = gtsam::Symbol('q', key);

        std::string label = name.text().get();
        gtsam::Vector4 bounds;
        bounds << bndbox.child("xmin").text().as_int(), bndbox.child("ymin").text().as_int(), bndbox.child("xmax").text().as_int(), bndbox.child("ymax").text().as_int();

        detections.emplace_back(
                label, bounds, state.this_step.pose_key, quadrickey
        );
    }
    return detections;
}

std::vector<Detection> HandMadeAssociator::flat(const std::vector<std::vector<Detection>>& list_of_lists)
{
    std::vector<Detection> result;
    for (const auto& sublist : list_of_lists) {
        result.insert(result.end(), sublist.begin(), sublist.end());
    }
    return result;
}

std::tuple<std::vector<Detection>, std::vector<Detection>, std::vector<Detection>> HandMadeAssociator::associate(SoSlamState& state)
{
    // Our dataset has been associated by hand
    // In fact the associator here is fake
    auto& s = state;
    auto& n = state.this_step;
    std::vector<Detection> new_ds = (!n.isValid()) ? std::vector<Detection>{} : n.detections;
    std::vector<Detection> newly_associated = new_ds;

    std::vector<Detection> associated = s.associated_;
    associated.insert(associated.end(), newly_associated.begin(), newly_associated.end());

    // In fact, we make sure there are no unassocaited objects in the dataset.
    std::vector<Detection> unassociated;
    unassociated = s.unassociated_;

    return std::make_tuple(newly_associated, associated, unassociated);
}

} // namespace gtsam_soslam