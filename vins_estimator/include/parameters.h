#pragma once
#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <queue>
#include <iomanip>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 2;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Matrix3d>>> GroundTruth;
extern std::queue<double> GROUND_BUF;
extern Eigen::Vector3d GROUNDT;
extern Eigen::Quaterniond GROUNDR;
extern Eigen::Matrix3d ROTATION_MATRIX;
extern double TIMESTAMP_t;
extern double BASELINE;
extern std::string GROUND_PATH;

extern Eigen::Matrix4d T_C1C2;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;
extern Eigen::Matrix4d T_0_0;
extern Eigen::Matrix4d T_1_0;
extern Eigen::Matrix4d T_0_1;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

extern int INITIAL_PATTERN1;
extern int INITIAL_PATTERN2;

extern int OPTIMIZATION_PATTERN;

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
