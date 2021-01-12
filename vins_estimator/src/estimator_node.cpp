#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "../include/estimator.h"
#include "../include/parameters.h"
#include "../include/utility/visualization.h"
#include "../../camera_model/include/camodocal/camera_models/PinholeCamera.h"


// @param main vio operator
Estimator estimator;

// @param buffer
std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
//queue<sensor_msgs::PointCloudConstPtr> keypoint_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;
//@param keypoint from left and right

double division_x=0;
double division_y=0;
int err_cout1=0;
int err_cout2=0;
int err_cout3=0;



vector<std::pair<int, Eigen::Vector3d>> Point_in_world;
vector<std::pair<int, Eigen::Vector3d>> Point_in_world_prev;
vector<double> dep;
vector<double> dep1;
Eigen::Vector3d imgpointL;
Eigen::Vector3d imgpointR;
Eigen::Vector3d imgpointpixR;
Eigen::Vector3d imgpointpixL;
vector<Eigen::Vector3d> tmp_pixR;
vector<cv::Point2d> tmp_cvpixR;
vector<cv::Point2d> tmp_cvpixR_prev;
vector<cv::Point3f> Pc_prev;
cv::Point2d cvPointR;
vector<Eigen::Vector3d> tmp_pixL;
vector<cv::Point2d> tmp_cvpixL;
vector<cv::Point2d> tmp_cvpixL_prev;
cv::Point2d cvPointL;
cv::Point3d cvPoint3D;
vector<Eigen::Vector3d> tmp_imgpointL;
vector<Eigen::Vector3d> tmp_left_porcessed;
vector<Eigen::Vector3d> tmp_right_porcessed;
vector<Eigen::Vector3d> tmp_imgpointR;
vector<Eigen::Vector3d> imgpointL_prev;
std::vector<int> feature_id_prev;
Eigen::Matrix3d Rotation_prev;
Eigen::Vector3d Translation_prev;
std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Matrix3d>>> GroundTruth_prev;
std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Matrix3d>>> GroundTruth_curr;
// @param mutex for buf, status value and vio processing
std::mutex m_buf;
std::mutex m_state; 
std::mutex m_estimator;
std::mutex m_feature;

// @param temp status values
double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

// @param flags
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

// @brief predict status values: Ps/Vs/Rs
//bk=>bk+1
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //获取当前时间
    double t = imu_msg->header.stamp.toSec();
    //首帧判断
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    //获取dt并传递时间
    double dt = t - latest_time;
    latest_time = t;
    
    //获取当前时刻的IMU采样值
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};    
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
/*    
    ROS_INFO_STREAM("time : "<< std::endl << t );
    ROS_INFO_STREAM("IMU acc : "<< std::endl << linear_acceleration );
    ROS_INFO_STREAM("IMU ang : "<< std::endl << angular_velocity );
    */
    //世界坐标系下
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    
}

// @brief update status values: Ps/Vs/Rs
//非线性优化时被调用
void update()
{
    //从估计器中得到滑动窗口中最后一个图像帧的imu更新项【P，Q，V，ba，bg，a，g】
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    //对imu_buf中剩余的imu_msg进行PVQ递推
    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}


cv::Point3d uv2xyz(cv::Point2d uvLeft,cv::Point2d uvRight){
  using namespace cv;
  //     [u1]      [xw]                      [u2]      [xw]
    //zc1 *|v1| = Pl*[yw]                  zc2*|v2| = P2*[yw]
    //     [ 1]      [zw]                      [ 1]      [zw]
    //               [1 ]                                [1 ]
/*  
  double leftIntrinsic[3][3] = {461.15862106007575 , 0, 362.65929181685937 ,
			       0, 459.75286598073296 , 248.52105668448124 ,
			       0,                  0,                  1};
  double rightIntrinsic[3][3] = {460.09781682258682 , 0, 373.14916359808268 ,
			        0, 458.90983492218902 , 254.40734973672119 ,
				0,                  0,                  1};*/
  double leftIntrinsic[3][3] = {FOCAL_LENGTH, 0, (COL / 2) ,
			        0, FOCAL_LENGTH, (ROW / 2) ,
			        0,            0,        1};
  double rightIntrinsic[3][3] = {FOCAL_LENGTH, 0, (COL / 2) ,
			        0, FOCAL_LENGTH, (ROW / 2) ,
			        0,            0,        1};
				
  double leftRotation[3][3] = {0.0148655429818, -0.999880929698, 0.00414029679422,
			       0.999557249008, 0.0149672133247, 0.025715529948, 
			      -0.0257744366974, 0.00375618835797, 0.999660727178};
  double leftTranslation[1][3] = {-0.0216401454975,-0.064676986768, 0.00981073058949};
/*
  double leftRotation[3][3] = {1.0, 0.0, 0.0,
			       0.0, 1.0, 0.0, 
			       0.0, 0.0, 1.0 };
  double leftTranslation[1][3] = {0.0, 0.0, 0.0};*/

/*
  double rightRotation[3][3] = {0.999997, -0.00220952, 0.00129944,
			       0.00222769, 0.999897, -0.0141501,
			      -0.00126804, 0.014153, 0.999899};

  double rightTranslation[1][3] = {0.110082, -0.000267494, 0.000607404};
  */
  double rightRotation[3][3] = {0.0125552670891, -0.999755099723, 0.0182237714554,
				0.999598781151, 0.0130119051815, 0.0251588363115,
				-0.0253898008918, 0.0179005838253, 0.999517347078};

  double rightTranslation[1][3] = {-0.0198435579556,0.0453689425024,0.00786212447038};
  
    Mat mLeftRotation = Mat(3,3,CV_64F,leftRotation);
    Mat mLeftTranslation = Mat(3,1,CV_64F,leftTranslation);
    Mat mLeftRT = Mat(3,4,CV_64F);//左相机RT矩阵
    hconcat(mLeftRotation,mLeftTranslation,mLeftRT);
    Mat mLeftIntrinsic = Mat(3,3,CV_64F,leftIntrinsic);
    Mat mLeftP = mLeftIntrinsic * mLeftRT;
//     ROS_INFO_STREAM("left camera P : "<< std::endl << mLeftP );
//     ROS_INFO_STREAM("left camera point : "<< std::endl << uvLeft );
    Mat mRightRotation = Mat(3,3,CV_64F,rightRotation);
    Mat mRightTranslation = Mat(3,1,CV_64F,rightTranslation);
    Mat mRightRT = Mat(3,4,CV_64F);//右相机RT矩阵
    hconcat(mRightRotation,mRightTranslation,mRightRT);
    Mat mRightIntrinsic = Mat(3,3,CV_64F,rightIntrinsic);
    Mat mRightP = mRightIntrinsic * mRightRT;
//     ROS_INFO_STREAM("right camera P : "<< std::endl << mRightP );
//     ROS_INFO_STREAM("left camera point : "<< std::endl << uvRight );
    Mat A = Mat(4,3,CV_64F);
    A.at<double>(0,0) = uvLeft.x * mLeftP.at<double>(2,0) - mLeftP.at<double>(0,0);
    A.at<double>(0,1) = uvLeft.x * mLeftP.at<double>(2,1) - mLeftP.at<double>(0,1);
    A.at<double>(0,2) = uvLeft.x * mLeftP.at<double>(2,2) - mLeftP.at<double>(0,2);
 
    A.at<double>(1,0) = uvLeft.y * mLeftP.at<double>(2,0) - mLeftP.at<double>(1,0);
    A.at<double>(1,1) = uvLeft.y * mLeftP.at<double>(2,1) - mLeftP.at<double>(1,1);
    A.at<double>(1,2) = uvLeft.y * mLeftP.at<double>(2,2) - mLeftP.at<double>(1,2);
 
    A.at<double>(2,0) = uvRight.x * mRightP.at<double>(2,0) - mRightP.at<double>(0,0);
    A.at<double>(2,1) = uvRight.x * mRightP.at<double>(2,1) - mRightP.at<double>(0,1);
    A.at<double>(2,2) = uvRight.x * mRightP.at<double>(2,2) - mRightP.at<double>(0,2);
 
    A.at<double>(3,0) = uvRight.y * mRightP.at<double>(2,0) - mRightP.at<double>(1,0);
    A.at<double>(3,1) = uvRight.y * mRightP.at<double>(2,1) - mRightP.at<double>(1,1);
    A.at<double>(3,2) = uvRight.y * mRightP.at<double>(2,2) - mRightP.at<double>(1,2);
    
    Mat B = Mat(4,1,CV_64F);
    B.at<double>(0,0) = mLeftP.at<double>(0,3) - uvLeft.x * mLeftP.at<double>(2,3);
    B.at<double>(1,0) = mLeftP.at<double>(1,3) - uvLeft.y * mLeftP.at<double>(2,3);
    B.at<double>(2,0) = mRightP.at<double>(0,3) - uvRight.x * mRightP.at<double>(2,3);
    B.at<double>(3,0) = mRightP.at<double>(1,3) - uvRight.y * mRightP.at<double>(2,3);
    
    Mat XYZ = Mat(3,1,CV_64F);
    solve(A,B,XYZ,DECOMP_SVD);

    Point3f world;
    world.x = XYZ.at<double>(0,0);
    world.y = XYZ.at<double>(1,0);
    world.z = XYZ.at<double>(2,0);
//     ROS_INFO_STREAM("world point : "<< std::endl << world );
    return world;
  
}



bool uv2xyz2(vector<cv::Point2d>& p1,vector<int>& id, vector<cv::Point2d>& p2,vector<int>& idp,
	     Eigen::Matrix3d Rota_prev,Eigen::Vector3d Tran_prev,Eigen::Matrix3d Rota,Eigen::Vector3d Tran,
	      vector<cv::Point3f>& pt_3d){//d->f
  using namespace cv;
  int id_bias;
  id_bias=id.size()/2;
  int id_biasp;
  id_biasp=idp.size()/2;
  Mat R, T, mask;
  vector<cv::Point2f> p11;
  vector<cv::Point2f> p22;
  vector<cv::Point3f> p3d;
  cv::Mat distCoeffs(4, 1, cv::DataType<float>::type);   // Distortion vector

    distCoeffs.at<float>(0) = 0.0;
    distCoeffs.at<float>(1) = 0.0;
    distCoeffs.at<float>(2) = 0.0;
    distCoeffs.at<float>(3) = 0.0;
    
    vector<Point2d> reproj_error;

    for(int i=0;i<id_bias;i++){
      vector<int>::iterator it = find(idp.begin(), idp.end(), id[i]);
      if(it!=idp.end()){
	int idx=distance(idp.begin(), it);
	p11.push_back(p1[i]);
	p3d.push_back(pt_3d[i]);
	p22.push_back(p2[idx]);
      }
    }
//     ROS_INFO_STREAM("p11=:"<<std::endl<<p11);
//     ROS_INFO_STREAM("p22=:"<<std::endl<<p22);
//     ROS_INFO_STREAM("p3d=:"<<std::endl<<p3d);
//     ROS_INFO_STREAM("Rota_prev=:"<<std::endl<<Rota_prev);
//     ROS_INFO_STREAM("Tran_prev=:"<<std::endl<<Tran_prev);
//     ROS_INFO_STREAM("Tran_prev=:"<<std::endl<<Rota);
//     ROS_INFO_STREAM("Tran_prev=:"<<std::endl<<Tran);
    
         
  Mat KK(3, 3, CV_32FC1);

  KK = (Mat_<float>(3, 3) << 1.0, 0.0, 0.0, 
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0);
  //根据内参矩阵获取相机的焦距和光心坐标（主点坐标）
    double focal_length = FOCAL_LENGTH;
    Point2d principle_point(COL/2,ROW/2);
    unsigned int pts_size = p11.size();
     
    Mat_<double> R1;
    Eigen::Matrix3d tmpR;
    Eigen::Vector3d tmpT;
    int countt;
//     tmpR=RIC[0].inverse()*Rota.inverse()*Rota_prev*RIC[0];
    tmpR=RIC[0].inverse()*Rota_prev.inverse()*Rota*RIC[0];
    eigen2cv<double>(tmpR,R1);
//     ROS_INFO_STREAM("R1=:"<<std::endl<<R1);       
      Vec3d rvec; Rodrigues(R1 ,rvec);
//       ROS_INFO_STREAM("rvec=:"<<std::endl<<rvec); 
//       tmpT=RIC[0].inverse()*Rota.inverse()*Rota_prev*TIC[0]
// 	      +RIC[0].inverse()*Rota.inverse()*Tran_prev
// 	      -RIC[0].inverse()*Rota.inverse()*Tran
//            -RIC[0].inverse()*TIC[0];
      tmpT=RIC[0].inverse()*Rota_prev.inverse()*Rota*TIC[0]
          +RIC[0].inverse()*Rota_prev.inverse()*Tran
          -RIC[0].inverse()*Rota_prev.inverse()*Tran_prev
	  -RIC[0].inverse()*TIC[0];
//     ROS_INFO_STREAM("tmpT=:"<<std::endl<<tmpT);
      
      Vec3d tvec(tmpT[0],tmpT[1],tmpT[2]);
//       ROS_INFO_STREAM("tvec=:"<<std::endl<<tvec);
      
      vector<Point2f> reprojected_pt_set1;
        projectPoints(pt_3d,rvec,tvec,KK,distCoeffs,reprojected_pt_set1);//----------------------------------
      
       for (unsigned int i=0; i<pts_size; i++) {
	 
 		reproj_error.push_back(460*(p22[i] - reprojected_pt_set1[i]));
		double error=norm(460*(p22[i] - reprojected_pt_set1[i]));
  		ROS_INFO_STREAM("reprojection_error=:"<<std::endl<<reproj_error.back());	 
//  		if(error>20.0||error<-20.0)									    
// 		{countt++;ROS_INFO_STREAM("false distacne");}
      }  		
//       ROS_INFO_STREAM("pts_size=:"<<std::endl<<pts_size);
//       ROS_INFO_STREAM("coun=:"<<std::endl<<countt);
      countt=0;
	
}
 
bool uv2xyz1(vector<cv::Point2d>& p1, vector<cv::Point2d>& p2,
	      vector<cv::Point3f>& pt_3d){//d->f
  using namespace cv;
  Mat R, T, mask;
  vector<cv::Point2f> p11;
  vector<cv::Point2f> p22;
  Mat proj1(3, 4, CV_32FC1);
  Mat proj2(3, 4, CV_32FC1);
  
  proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
  proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

  cv::Mat distCoeffs(4, 1, cv::DataType<float>::type);   // Distortion vector

    distCoeffs.at<float>(0) = 0.0;
    distCoeffs.at<float>(1) = 0.0;
    distCoeffs.at<float>(2) = 0.0;
    distCoeffs.at<float>(3) = 0.0;
    
    vector<Point2d> reproj_error;


    
   
  Mat extrinsic= (Mat_<float>(3, 4) << 0.9999967147276215, 0.0022276888117144476, -0.0012680443676252341, -0.11008051627857199,
				       -0.0022095208901783663, 0.9998974001748293, 0.014152990533781513, 0.000502098531178961,
				       0.001299442725159215, -0.014150142266832727, 0.9998990373644874, -0.0007541728860674408);

  //根据内参矩阵获取相机的焦距和光心坐标（主点坐标）
    double focal_length = FOCAL_LENGTH;
    Point2d principle_point(COL/2,ROW/2);
    unsigned int pts_size = p1.size();
    
     
    //根据匹配点求取本征矩阵，使用RANSAC，进一步排除失配点
      Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);
    if (E.empty()) return false;

    double feasible_count = countNonZero(mask);
     cout << (int)feasible_count << " -in- " << p1.size() << endl;
    //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
    if (feasible_count <= 15 && (feasible_count / p1.size()) < 0.6)
      return false;
    
      int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);
    //分解本征矩阵，获取相对变换 
    //同时位于两个相机前方的点的数量要足够大
//     if (((double)pass_count) / feasible_count < 0.7)
//       return false;

    
    
    //两个相机的投影矩阵[R T]，triangulatePoints只支持float型



//     R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
//     T.convertTo(proj2.col(3), CV_32FC1);
//     ROS_INFO_STREAM("R=:"<<std::endl<<R);
//     ROS_INFO_STREAM("T=:"<<std::endl<<T);
//     ROS_INFO_STREAM("proj2=:"<<std::endl<<proj2);
//     
    proj2 = extrinsic;
    
    Mat pt_set1_pt,pt_set2_pt;
    Mat pt_3d_h(1,pts_size,CV_32FC4);

    for(int i=0;i<p1.size();i++){
        p11.push_back(p1[i]); 
        p22.push_back(p2[i]); }    
    

    Mat pt_set1_pt_2r = Mat(2,p11.size(),CV_32F);
    Mat pt_set2_pt_2r = Mat(2,p22.size(),CV_32F);
    for(int i=0;i<p11.size() ;i++){
      pt_set1_pt_2r.at<float>(0,i)=p11[i].x;
      pt_set1_pt_2r.at<float>(1,i)=p11[i].y;
      pt_set2_pt_2r.at<float>(0,i)=p22[i].x;
      pt_set2_pt_2r.at<float>(1,i)=p22[i].y;         
    }
    
    
    
//        ROS_INFO_STREAM("pt_set1_pt_2r=:"<<std::endl<<pt_set1_pt_2r.size);
//        ROS_INFO_STREAM("p11=:"<<std::endl<<p11.size());    
//      ROS_INFO_STREAM("p1=:"<<std::endl<<pt_set1_pt_2r.size);
//      ROS_INFO_STREAM("p2=:"<<std::endl<<pt_set2_pt_2r.size);
        //三角化重建
/*    
       ROS_INFO_STREAM("pt_set1_pt_2r=:"<<std::endl<<pt_set1_pt_2r);
       ROS_INFO_STREAM("p1.data=:"<<std::endl<<p11);*/
        ROS_INFO_STREAM("proj2=:"<<std::endl<<proj2);

       triangulatePoints(proj1, proj2, pt_set1_pt_2r, pt_set2_pt_2r, pt_3d_h);//-------------------------------
       
       for(int i=0;i<p11.size();i++){
	 Point3f p; 
	 p.x =pt_3d_h.at<float>(0,i);
	 p.y =pt_3d_h.at<float>(1,i);
	 p.z =pt_3d_h.at<float>(2,i);
 	 p = p/pt_3d_h.at<float>(3,i);
	 pt_3d.push_back(p);
      }
       
      
//        ROS_INFO_STREAM("p11=:"<<std::endl<<p11);
//        ROS_INFO_STREAM("p22=:"<<std::endl<<p22);
//        ROS_INFO_STREAM("pt_3d=:"<<std::endl<<pt_3d);
     


}


void getKeypoints(sensor_msgs::PointCloudConstPtr& img_msg )
{ int count=0;
  //sensor_msgs::PointCloudConstPtr img_msg = keypoint_buf.back();
  int Bias = img_msg->points.size();
  int halfBias = img_msg->points.size()/2;
  Eigen::Matrix3d Rotation;
  Eigen::Vector3d Translation;
  Eigen::Matrix3f T_BS ;
  Eigen::Matrix3f Intrsic ;
  std::vector<int> feature_id;
  Eigen::Matrix3d Intrinsic;
  Eigen::Matrix3d Intrinsic1;
  Eigen::Matrix3d Intrinsic2;
  Eigen::Vector3d Point;
  
  Intrinsic2<<FOCAL_LENGTH,0, (COL / 2),
	      0,   FOCAL_LENGTH,(ROW /2),
	      0,0,1;
  
  
  Intrinsic<<461.15862106007575 , 0, 362.65929181685937 ,
	     0, 459.75286598073296 , 248.52105668448124 ,
	     0,                  0,                  1;
  Intrinsic1<<460.09781682258682 , 0, 373.14916359808268 ,
	      0, 458.90983492218902 , 254.40734973672119 ,
	      0,                  0,                  1;
	      
//iterate groundtruth per frame
 for(auto &it : GroundTruth)
 {  
    // measurements.header.stamp = ros::Time::fromNSec(it.first);
     
     //ROS_INFO("keypoint timestamp = %f ", keypoint_buf.front()->header.stamp.toSec());
   if((it.first-img_msg->header.stamp.toSec())>-0.000005 && (it.first-img_msg->header.stamp.toSec()) <0.000005 )
   {
      Rotation = it.second.second;
      Translation = it.second.first;
       
       ROS_INFO("GroundTruth timestamp = %f ", it.first);

      
     for (unsigned int i = 0; i < Bias; i++)
     {
        
	feature_id.push_back(img_msg->channels[0].values[i]);
        int camera_id = img_msg->channels[5].values[i];
	
	if (camera_id==0)
	{        
	  imgpointL[0]=img_msg->points[i].x;//undistortion
	  imgpointL[1]=img_msg->points[i].y;
	  imgpointL[2]=img_msg->points[i].z;
/*
	  imgpointpixL[0] =imgpointL[0]*FOCAL_LENGTH + (COL / 2);
	  imgpointpixL[1] =imgpointL[1]*FOCAL_LENGTH + (ROW / 2);
	  imgpointpixL[2] =1.0;
	  */
	  imgpointpixL[0] =img_msg->channels[1].values[i];
	  imgpointpixL[1] =img_msg->channels[2].values[i];
	  imgpointpixL[2] =1.0;
	  
/*	  
	  cvPointL.x=imgpointpixL[0];
	  cvPointL.y=imgpointpixL[1];
	  */
	  cvPointL.x=imgpointL[0];
	  cvPointL.y=imgpointL[1];
	  tmp_cvpixL.push_back(cvPointL);
	  
	  
	  tmp_imgpointL.push_back(imgpointL); 
	  tmp_pixL.push_back(imgpointpixL);
	  
	}
	
	if (camera_id==1)
	{ 
	  
	  imgpointR[0]=img_msg->points[i].x;
	  imgpointR[1]=img_msg->points[i].y;
	  imgpointR[2]=img_msg->points[i].z;
/*
	  imgpointpixR[0]=imgpointR[0] * FOCAL_LENGTH + (COL / 2);
	  imgpointpixR[1]=imgpointR[1] * FOCAL_LENGTH + (ROW / 2);
	  imgpointpixR[2]=1.0;
*/
	  imgpointpixR[0] =img_msg->channels[1].values[i];
	  imgpointpixR[1] =img_msg->channels[2].values[i];
	  imgpointpixR[2] =1.0;	  
	  
	  tmp_pixR.push_back(imgpointpixR);
	  tmp_imgpointR.push_back(imgpointR); 

	  cvPointR.x=imgpointR[0];
	  cvPointR.y=imgpointR[1];
	  tmp_cvpixR.push_back(cvPointR);

	}         
   }  
   vector<cv::Point3f> Pc; 

   if(!tmp_cvpixL_prev.empty() && uv2xyz1(tmp_cvpixL,tmp_cvpixR,Pc)){
   uv2xyz2(tmp_cvpixL_prev,feature_id_prev,tmp_cvpixL,feature_id,
	   Rotation_prev,Translation_prev,Rotation,Translation,
	   Pc);
      
      Rotation_prev=Rotation;
      Translation_prev=Translation;
   }   
   

   
     break;
  }	      
 }	  tmp_cvpixL_prev.clear();
	  tmp_cvpixL_prev=tmp_cvpixL;
	  tmp_cvpixL.clear();
	  tmp_cvpixR_prev.clear();
	  tmp_cvpixR_prev=tmp_cvpixR;
	  tmp_cvpixR.clear();
   	  feature_id_prev.clear();
	  feature_id_prev = feature_id;
	  feature_id.clear();
	  imgpointL_prev.clear();
	  imgpointL_prev = tmp_imgpointL;
	  tmp_imgpointL.clear();
	  tmp_imgpointR.clear();
}



// @brief take and align measurement from feature frames and IMU measurement
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {//边界判定：数据取完，说明配对完成
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;
    //边界判定：imu buf里所有的数据时间都比img_buf 第一个帧时间戳要早，说明缺乏imu数据，需要等待IMU数据
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }
    //边界判断：IMU第一个数据的时间要大于第一个图像特征数据的时间(说明图像帧有多的)
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        //核心操作：装入视觉帧信息
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();
	   //核心操作：装入IMU信息
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
	    //最后一帧imu被相邻两个视觉帧共用，因此被放回到imu_buf中
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

// @brief put IMU measurement in buffer and publish status values: Ps/Vs/Rs
//往IMU_buf里赛数据
//预积分获得Ps/Vs/Rs
//如果处于非线性优化阶段，吧预积分的的PVQ发布到RVIZ(visualization.cpp/publatestOdometry())
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);// TODO useless?
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

// @brief put feature measurement in buffer
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    //m_feature.lock();
   // keypoint_buf.push(feature_msg);
    //m_feature.unlock();
    
    
    //ROS_INFO("keypoint timestamp = %f ", feature_buf.front()->header.stamp.toSec());
    con.notify_one();
    //con1.notify_one();
}

// @brief restart
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// @brief put relocalization flag in buffer
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// @brief get Rs[0] in world coordinate system
void initFirstIMUPose(const vector<sensor_msgs::ImuConstPtr> &imu_msgs,
                      const sensor_msgs::PointCloudConstPtr &img_msg)
{
    vector<pair<double, Eigen::Vector3d>> accVector;
    for (auto &imu_msg : imu_msgs)
    {
        double dx = 0, dy = 0, dz = 0;
        double t = imu_msg->header.stamp.toSec();
        double img_t = img_msg->header.stamp.toSec() + estimator.td;
        if (t <= img_t)
        { 
            dx = imu_msg->linear_acceleration.x;
            dy = imu_msg->linear_acceleration.y;
            dz = imu_msg->linear_acceleration.z;
            Eigen::Vector3d acc(dx,dy,dz);
            accVector.emplace_back(make_pair(t,acc));
        }
    }
    estimator.initFirstIMUPose(accVector);
}

// @brief IMU pre-integration and get pre-optimized status values Ps/Vs/Rs
void processIMU(sensor_msgs::ImuConstPtr &imu_msg,
                sensor_msgs::PointCloudConstPtr &img_msg)
{
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
    double t = imu_msg->header.stamp.toSec();
    double img_t = img_msg->header.stamp.toSec() + estimator.td;
    if (t <= img_t)
    { 
        if (current_time < 0)
            current_time = t;
        double dt = t - current_time;
        ROS_ASSERT(dt >= 0);
        current_time = t;
        dx = imu_msg->linear_acceleration.x;
        dy = imu_msg->linear_acceleration.y;
        dz = imu_msg->linear_acceleration.z;
        rx = imu_msg->angular_velocity.x;
        ry = imu_msg->angular_velocity.y;
        rz = imu_msg->angular_velocity.z;
        //粗略预积分，值传给IntegrationBase对象
        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));                    
        //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
    }
    else//处于边界位置的imu数据，被相邻两帧共享，而且对前一帧的影响更大一些，对数据进行现行分配
	//每个大于图像帧时间戳的第一个imu_msg是被两个图像帧公用的（出现次数少）
    {
        double dt_1 = img_t - current_time;
        double dt_2 = t - img_t;
        current_time = img_t;
        ROS_ASSERT(dt_1 >= 0);
        ROS_ASSERT(dt_2 >= 0);
        ROS_ASSERT(dt_1 + dt_2 > 0);
        double w1 = dt_2 / (dt_1 + dt_2);
        double w2 = dt_1 / (dt_1 + dt_2);
        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
        estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
        //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
    }
}

// @brief setReloFrame
void setReloFrame(sensor_msgs::PointCloudConstPtr &relo_msg)
{
    //sensor_msgs::PointCloudConstPtr relo_msg = NULL;
    while (!relo_buf.empty())
    {
        relo_msg = relo_buf.front();
        relo_buf.pop();
    }
    if (relo_msg != NULL)
    {
        vector<Vector3d> match_points;
        double frame_stamp = relo_msg->header.stamp.toSec();
        for (unsigned int i = 0; i < relo_msg->points.size(); i++)
        {
            Vector3d u_v_id;
            u_v_id.x() = relo_msg->points[i].x;
            u_v_id.y() = relo_msg->points[i].y;
            u_v_id.z() = relo_msg->points[i].z;
            match_points.push_back(u_v_id);
        }
        Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
        Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
        Matrix3d relo_r = relo_q.toRotationMatrix();
        int frame_index;
        frame_index = relo_msg->channels[0].values[7];
        estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
    }    
}

// @brief main vio function, including initialization and optimization
void processVIO(sensor_msgs::PointCloudConstPtr& img_msg)
{
    int count_left = 0;
    int count_right = 0;
    /*image
     * 存放特征点信息
     * 索引值为feature_id
     * value值是一个vector，那么同一个特征点在不同摄像头下会有不同的观测信息
     */
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
    //ROS_INFO("img_msg->points.size(): %d",img_msg->points.size());
    for (unsigned int i = 0; i < img_msg->points.size(); i++)
    {
        int feature_id = img_msg->channels[0].values[i];
        int camera_id = img_msg->channels[5].values[i]; 

        if (camera_id == 0)
            count_left ++;
        if (camera_id == 1)
            count_right ++;

        double x = img_msg->points[i].x;
        double y = img_msg->points[i].y;
        double z = img_msg->points[i].z;
        double p_u = img_msg->channels[1].values[i];
        double p_v = img_msg->channels[2].values[i];
        double velocity_x = img_msg->channels[3].values[i];
        double velocity_y = img_msg->channels[4].values[i];
//         double p_u = img_msg->channels[2].values[i];
//         double p_v = img_msg->channels[3].values[i];
//         double velocity_x = img_msg->channels[4].values[i];
//         double velocity_y = img_msg->channels[5].values[i];
        
        
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
	
	
    }
        for (auto it:image)
            assert(it.second.size() == 2);
            

    
    estimator.processImage(image, img_msg->header);
}

// @brief visualization
void visualize(sensor_msgs::PointCloudConstPtr &relo_msg,std_msgs::Header &header)
{
    pubOdometry(estimator, header);
    pubKeyPoses(estimator, header);
    pubCameraPose(estimator, header);
//     pubPointCloud(estimator, header);
    pubTF(estimator, header);
    pubKeyframe(estimator);
//     if (relo_msg != NULL)
    if(0)
        pubRelocalization(estimator);
   //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
}


// @brief main vio function, including initialization and optimization
void processMeasurement(vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>& measurements)
{
    for (auto &measurement : measurements)//对measurements 中的每个measurement(IMUs,IMG)组合进行操作
    {
        auto img_msg = measurement.second;

        //if use dual-IMU initial-pattern, then we need intial gauss of Rs
        if (!estimator.initFirstPoseFlag && INITIAL_PATTERN2)//??
            initFirstIMUPose(measurement.first,measurement.second);

        // get status value:Rs/Vs/Ps
        for (auto &imu_msg : measurement.first)
            processIMU(imu_msg,img_msg);

        // set relocalization frame
         sensor_msgs::PointCloudConstPtr relo_msg = NULL;
//         setReloFrame(relo_msg);//-----------------------------------
        ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());
//     	getKeypoints(img_msg);//--------------------------------------------------------------------
        // main function for vio
        TicToc t_s;
        processVIO(img_msg);

        double whole_t = t_s.toc();
        printStatistics(estimator, whole_t);
        std_msgs::Header header = img_msg->header;
        header.frame_id = "world";

        // show in rviz
        visualize(relo_msg, header);
    }
}

// @brief main vio function, including initialization and optimization
// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        TicToc t_e;
        // get measurement in buf and make them aligned
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
	    std::unique_lock<std::mutex> fl(m_feature);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        // main function of vio
        m_estimator.lock();
        processMeasurement(measurements);
        m_estimator.unlock();

        // update status value Rs/Ps/Vs
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
        //ROS_WARN("whole vio processing costs: %f", t_e.toc());
	
	
    }
}

// @brief main function 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);
    
    
    
    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
