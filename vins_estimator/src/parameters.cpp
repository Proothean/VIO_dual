#include "../include/parameters.h"
#include <sensor_msgs/PointCloud.h>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};
Eigen::Matrix4d T_C1C2;



std::string GROUND_PATH;
std::queue<double> GROUND_BUF;
Eigen::Vector3d GROUNDT;
Eigen::Quaterniond GROUNDR;
Eigen::Matrix3d ROTATION_MATRIX;
Eigen::Matrix4d T_1_0;
Eigen::Matrix4d T_0_0;
Eigen::Matrix4d T_0_1;
//Eigen::Transform<>S
double TIMESTAMP_t;
sensor_msgs::PointCloud TIMESTAMP;
double BASELINE;
std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Matrix3d>>> GroundTruth;


double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

int INITIAL_PATTERN1;
int INITIAL_PATTERN2;

int OPTIMIZATION_PATTERN;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}



void getGroundTruth()
{ 
  std::ifstream fin (GROUND_PATH);
  std::string line;
  std::string tmpline;
  std::string tmpline1;
  std::string sub_line;
  

  T_0_0<<1.0,0,0,0,
	 0,1.0,0,0,
	 0,0,1.0,0,
	 0,0,0,1.0;
  T_1_0<<0.9999967147276215, 0.0022276888117144476, -0.0012680443676252341, -0.11008051627857199,
	-0.0022095208901783663, 0.9998974001748293, 0.014152990533781513, 0.000502098531178961,
	0.001299442725159215, -0.014150142266832727, 0.9998990373644874, -0.0007541728860674408,
	0.0,                   0.0,                  0.0,                1.0;
  T_0_1=T_1_0.inverse();
  ROS_INFO_STREAM("C0-C1 :"<< std::endl << T_0_1 );

  if ( ! fin)
  {
    ROS_INFO_STREAM("didn't find data.csv : " );
    }
 
 while (std::getline(fin, line))
  {
    tmpline=line;
    tmpline1=line;
    
  /*  
    for(int i = 0; i<15;i++)
    {
      sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
      groundR.y()=std::stof(sub_line.c_str());
      //ground_buf.push(std::stof(sub_line.c_str()));
      tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
      sub_line = tmpline1.substr(tmpline.find_last_of(",")+1);
      //ground_buf.push(std::stof(sub_line.c_str()));
      groundR.x()=std::stof(sub_line.c_str());
      tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    }
    */
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //groundR.z() = std::stof(sub_line.c_str());
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//--------------------------------------------1
    tmpline = line.substr(0,line.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //groundR.z() = std::stof(sub_line.c_str());
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//--------------------------------------------2
    tmpline = line.substr(0,line.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//--------------------------------------------3
    //groundR.y() = std::stof(sub_line.c_str());
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//-------------------------------------------4
    //groundR.x() = std::stof(sub_line.c_str());
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//--------------------------------------------5
    //groundR.w() = std::stof(sub_line.c_str());
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//--------------------------------------------6
    //groundT.z() = std::stof(sub_line.c_str());
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//--------------------------------------------7
    //groundT.y() = std::stof(sub_line.c_str());
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------8
    //groundT.x() = std::stof(sub_line.c_str());
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //timeStamp = std::stod(sub_line.c_str());
   // GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------9
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------10
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------11-------------
    GROUNDR.z() = std::stof(sub_line.c_str());
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------12
    GROUNDR.y() = std::stof(sub_line.c_str());
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------13
    GROUNDR.x() = std::stof(sub_line.c_str());
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------14
    GROUNDR.w() = std::stof(sub_line.c_str());
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------15
    GROUNDT.z() = std::stof(sub_line.c_str());
    tmpline1= tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------16
    GROUNDT.y() = std::stof(sub_line.c_str());
    tmpline = tmpline1.substr(0,tmpline1.size()- sub_line.size()-1);
    sub_line = tmpline.substr(tmpline.find_last_of(",")+1);
    //GROUND_BUF.push(std::stof(sub_line.c_str()));//---------------------------------------------17
    GROUNDT.x() = std::stof(sub_line.c_str());
    tmpline1 = tmpline.substr(0,tmpline.size()- sub_line.size()-1);
    sub_line = tmpline1.substr(tmpline1.find_last_of(",")+1);
    //GROUND_BUF.push(std::stod(sub_line.c_str()));//---------------------------------------------18
    TIMESTAMP_t = std::stod(sub_line.c_str())/1000000000;
    
    
    //ROS_INFO_STREAM("data.csv sub_line : " << std::endl << ground_buf.back());
    ROTATION_MATRIX = GROUNDR.matrix();
    GroundTruth.push_back(std::make_pair(TIMESTAMP_t,std::make_pair(GROUNDT,ROTATION_MATRIX)));
    
   // ROS_INFO("timestamp = %d ", GroundTruth.size());
    //std::cout << "[COUT_INFO] GroundTruth : " << std::setprecision(20) << GroundTruth[0].first << std::endl;
    //ROS_INFO_STREAM("ROTATION_MATRIX : " << std::endl << ROTATION_MATRIX);
    TIMESTAMP_t=0;
    
    }
/*
   for(auto &it : GroundTruth)
   {
      TIMESTAMP.header.stamp = ros::Time::fromSec(it.first);
     //ROS_INFO("timestamp = %d ", it.first);
     std::cout << "[COUT_INFO] TimeStamp: " << std::setprecision(19) << it.first<< std::endl;
     std::cout << "[COUT_INFO] GROUNDT.x: " << std::setprecision(7) << it.second.first.x()<< std::endl;
     //ros::Time::fromSec(TIMESTAMP) 
  }
*/
    
   
}


void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;
    BASELINE = fsSettings["baseline"];
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    fsSettings["6Dground_truth"] >> GROUND_PATH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {	
      /*
       * ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
	*/
	;
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
	  /*
	   * 
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        */
	  ;
	  
	}
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation0"] >> cv_R;
        fsSettings["extrinsicTranslation0"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R0 : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T0 : " << std::endl << TIC[0].transpose());

        fsSettings["extrinsicRotation1"] >> cv_R;
        fsSettings["extrinsicTranslation1"] >> cv_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q1(eigen_R);
        eigen_R = Q1.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R1 : " << std::endl << RIC[1]);
        ROS_INFO_STREAM("Extrinsic_T1 : " << std::endl << TIC[1].transpose());
	getGroundTruth();
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
        TR = 0;

    INITIAL_PATTERN1 = fsSettings["initial_pattern1"];
    INITIAL_PATTERN2 = fsSettings["initial_pattern2"];

    OPTIMIZATION_PATTERN = fsSettings["optimization_pattern"];
    
    fsSettings.release();
}
