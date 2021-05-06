//
// Created by gxfordream on 2021/5/6.
//
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace std;
using namespace cv;

int get_intrinsics_distortion(string &yaml_path, cv::Mat &camera_matrix, cv::Mat &distortion_coefficients,
                              cv::Size img_size){
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_path);
    } catch (YAML::BadFile &e) {
        cout << "read error!" << endl;
        return -1;
    }
    string camera_model = config["cam0"]["camera_model"].as<string>();
    string distortion_model = config["cam0"]["distortion_model"].as<string>();
    vector<double> distortion_coeffs = config["cam0"]["distortion_coeffs"].as<vector<double>>();
    vector<double> intrinsics = config["cam0"]["intrinsics"].as<vector<double>>();
    vector<int> resolution = config["cam0"]["resolution"].as<vector<int>>();
    string rostopic = config["cam0"]["rostopic"].as<string>();

    // get parameters
    double fx, fy, cx, cy;
    fx = intrinsics[0];
    fy = intrinsics[1];
    cx = intrinsics[2];
    cy = intrinsics[3];
    double k1, k2, k3, k4;
    k1 = distortion_coeffs[0];
    k2 = distortion_coeffs[1];
    k3 = distortion_coeffs[2];
    k4 = distortion_coeffs[3];
    cout << k1 << k2 << k3 << k4 << endl;
    camera_matrix = (Mat_<double>(3, 3) << fx, .0, cx, .0, fy, cy, .0, .0, 1.0);
    distortion_coefficients = (Mat_<double>(1, 4) << k1, k2, k3, k4);
    img_size.width = resolution[0];
    img_size.height = resolution[1];

    return 0;
}

int main(int argc, char** argv)
{
    // load yaml information
    string equi_yaml = "/home/gxfordream/Desktop/project/kalibr/WN-SHD1080P-FV2/camchain.yaml";
    string radtan_yaml = "/home/gxfordream/Desktop/project/kalibr/WN-SHD1080P-FV2/pinhole-radtan.yaml";
    Mat equi_camera_matrix, equi_distortion_coefficients, radtan_camera_matrix, radtan_distortin_coefficients;
    cv::Size img_size;
    get_intrinsics_distortion(equi_yaml, equi_camera_matrix, equi_distortion_coefficients, img_size);
    get_intrinsics_distortion(radtan_yaml, radtan_camera_matrix, radtan_distortin_coefficients, img_size);


    // load video
    VideoCapture capture;
    capture.open("/home/gxfordream/Desktop/project/kalibr/distort/calibration.mp4");
    if(!capture.isOpened()){
        cout << "could not open the video" << endl;
        return -1;
    }


    while(1)
    {
        Mat frame;
        capture >> frame;
        flip(frame, frame, 1);
        Mat distortion = frame.clone();
        Mat radtan_distortion = frame.clone();

        undistort(frame, radtan_distortion, radtan_camera_matrix, radtan_distortin_coefficients);
        cv::fisheye::undistortImage(frame, distortion, equi_camera_matrix, equi_distortion_coefficients, equi_camera_matrix, img_size);
        //第一个参数src，输入参数，代表畸变的原始图像；
        //第二个参数dst，矫正后的输出图像，跟输入图像具有相同的类型和大小；
        //第三个参数cameraMatrix为之前求得的相机的内参矩阵；
        //第四个参数distCoeffs为之前求得的相机畸变矩阵；
        //第五个参数newCameraMatrix，默认跟cameraMatrix保持一致；
        imshow("原图", frame);
        imshow("fisheye校正图", distortion);
        imshow("radtan校正图", radtan_distortion);
        if (waitKey(30) > 0) break;
    }

    return 0;

}


