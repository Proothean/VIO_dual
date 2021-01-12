#include "../../include/initial/initial_sfm.h"
#include <ros/ros.h>

GlobalSFM::GlobalSFM(){}

void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
						Vector2d &point0, Vector2d &point1, Vector2d &pointR0, Vector2d &pointR1, Vector3d &point_3d)
{	/*
  	Matrix4d T_1_0;
	T_1_0<<0.9999967147276215, 0.0022276888117144476, -0.0012680443676252341, -0.11008051627857199,
		     -0.0022095208901783663, 0.9998974001748293, 0.014152990533781513, 0.000502098531178961,
		      0.001299442725159215, -0.014150142266832727, 0.9998990373644874, -0.0007541728860674408,
		      0.0, 0.0, 0.0, 1.0;
	T_1_0=T_1_0.inverse();
	*/
 	Vector3d point_in_3D;
	Matrix4d design_matrix = Matrix4d::Zero();
 	Matrix4d design_matrix1 = Matrix4d::Zero();
	Vector3d point_tmp;
	
	
	point_tmp[0]=point1[0];
	point_tmp[1]=point1[1];
	point_tmp[2]=1.0;
	design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
	design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
	design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
	design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
	
	/*
	design_matrix1.row(0) = T_0_0.row(0)  - point0[0] * T_0_0.row(2);
	design_matrix1.row(1) = point0[1] * T_0_0.row(2) - T_0_0.row(1);
	design_matrix1.row(2) = T_0_1.row(0) - pointR0[0] * T_0_1.row(2);
	design_matrix1.row(3) = pointR0[1] * T_0_1.row(2) - T_0_1.row(1);
	*/
/*	 
	design_matrix1.row(0) = point0[0] * Pose01.row(2) - Pose01.row(0);
	design_matrix1.row(1) = point0[1] * Pose01.row(2) - Pose01.row(1);
	design_matrix1.row(2) = pointR0[0] * Pose11.row(2) - Pose11.row(0);
	design_matrix1.row(3) = pointR0[1] * Pose11.row(2) - Pose11.row(1);
	*/
	Vector4d triangulated_point,triangulated_point1;
	triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
//  	triangulated_point1 = design_matrix1.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	point_3d(0) = triangulated_point(0) / triangulated_point(3);
	point_3d(1) = triangulated_point(1) / triangulated_point(3);
	point_3d(2) = triangulated_point(2) / triangulated_point(3);
	
	
/*	
	point_in_3D(0) = triangulated_point1(0) / triangulated_point1(3);
	point_in_3D(1) = triangulated_point1(1) / triangulated_point1(3);
	point_in_3D(2) = triangulated_point1(2) / triangulated_point1(3);
	*/
//	point_in_3D=point_in_3D/point_in_3D(2);
// 	point_tmp = Pose0*Pose1.inverse()*point_tmp;
//  	ROS_INFO_STREAM("ll: "<<std::endl<<point0);
//  	ROS_INFO_STREAM("rr: "<<std::endl<<pointR0);
// 	ROS_INFO_STREAM("point_3d: "<<std::endl<<point_3d/point_3d(2));
//  	ROS_INFO_STREAM("point_3d1: "<<std::endl<<point_in_3D);
// 	ROS_INFO_STREAM("Pose0: "<<std::endl<<Pose0);
// 	ROS_INFO_STREAM("Pose1: "<<std::endl<<Pose1);
// 	

	
}


bool GlobalSFM::solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
								vector<SFMFeature> &sfm_f)
{
	vector<cv::Point2f> pts_2_vector;
// 	vector<cv::Point2f> pts_2_vector1;
	vector<cv::Point3f> pts_3_vector;
// 	vector<cv::Point3f> pts_3_vector1;
	for (int j = 0; j < feature_num; j++)
	{
		if (sfm_f[j].state != true)
			continue;
		Vector2d point2d;
		for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
		{
			if (sfm_f[j].observation[k].first == i)
			{       
				Vector2d img_pts = sfm_f[j].observation[k].second;
				Vector2d img_pts1 = sfm_f[j].observation1[k].second;
				cv::Point2f pts_2(img_pts(0), img_pts(1));
// 				cv::Point2f pts_21(img_pts1(0), img_pts1(1));
				pts_2_vector.push_back(pts_2);
// 				pts_2_vector1.push_back(pts_21);
				cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2]);
// 				cv::Point3f pts_31(sfm_f[j].position1[0], sfm_f[j].position1[1], sfm_f[j].position1[2]);
				pts_3_vector.push_back(pts_3);
// 				pts_3_vector1.push_back(pts_31);
				break;
			}
		}
	}
	if (int(pts_2_vector.size()) < 15)
	{
		printf("unstable features tracking, please slowly move you device!\n");
		if (int(pts_2_vector.size()) < 10)
			return false;
	}
// 	cv::Mat r, rvec, t, D, tmp_r, r1, rvec1, t1, D1, tmp_r1;
	cv::Mat r, rvec, t, D, tmp_r;
	cv::eigen2cv(R_initial, tmp_r);
// 	cv::eigen2cv(R_initial1, tmp_r1);
	cv::Rodrigues(tmp_r, rvec);
// 	cv::Rodrigues(tmp_r1, rvec1);
	cv::eigen2cv(P_initial, t);
// 	cv::eigen2cv(P_initial1, t1);
	cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	bool pnp_succ,pnp_succ1;
	pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
//  	pnp_succ1 = cv::solvePnP(pts_3_vector1, pts_2_vector1, K, D1, rvec1, t1, 1);
	if(!(pnp_succ))
	{
		return false;
	}
	cv::Rodrigues(rvec, r);
// 	cv::Rodrigues(rvec1, r1);
	//cout << "r " << endl << r << endl;
	MatrixXd R_pnp;
// 	MatrixXd R_pnp1;
// 	cv::cv2eigen(r1, R_pnp1);
	cv::cv2eigen(r, R_pnp);
	MatrixXd T_pnp;
// 	MatrixXd T_pnp1;
	cv::cv2eigen(t, T_pnp);
// 	cv::cv2eigen(t1, T_pnp1);
	R_initial = R_pnp;
	P_initial = T_pnp;
// 	R_initial1 = R_pnp1;
// 	P_initial1 = T_pnp1;
	return true;

}

void GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
			int frame1,Eigen::Matrix<double, 3, 4> &Pose1, vector<SFMFeature> &sfm_f)
{
	assert(frame0 != frame1);
	for (int j = 0; j < feature_num; j++)//在所有特征里依次寻找
	{
		if (sfm_f[j].state == true)//如已三角化，则跳过
			continue;
		bool has_0 = false, has_1 = false;
		Vector2d point0;
		Vector2d point1;
		Vector2d pointR0;
		Vector2d pointR1;
		for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)//如果这个特征在frame0出现过
		{
			if (sfm_f[j].observation[k].first == frame0)
			{
				point0 = sfm_f[j].observation[k].second;//把他的归一化坐标提取出来
				pointR0 = sfm_f[j].observation1[k].second;
				has_0 = true;
			}
			if (sfm_f[j].observation[k].first == frame1)//如果在frame1出现过
			{
				point1 = sfm_f[j].observation[k].second;//提取归一化坐标
				pointR1 = sfm_f[j].observation1[k].second;
				has_1 = true;
			}
		}
		if (has_0 && has_1)//两个归一化坐标都存在
		{
			Vector3d point_3d;
			triangulatePoint(Pose0,Pose1,point0, point1,pointR0,pointR1, point_3d);//根据他们的位姿和归一化坐标，输出在参考系l下的的空间坐标
			sfm_f[j].state = true;//已完成三角化
			sfm_f[j].position[0] = point_3d(0);//把参考系l下的空间坐标赋值给这个特征点的对象
			sfm_f[j].position[1] = point_3d(1);
			sfm_f[j].position[2] = point_3d(2);
			//cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
		}							  
	}
}

// 	 q w_R_cam t w_R_cam
//  c_rotation cam_R_w 
//  c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)
bool GlobalSFM::construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points)
{	
//(1)把第l帧作为参考坐标系，获得最新一帧在参考坐标系下的位姿
	feature_num = sfm_f.size();
	//cout << "set 0 and " << l << " as known " << endl;
	// have relative_r relative_t
	// intial two view
/*	
	Quaterniond* q1;
	Vector3d* T1;
	q1[l].w() = 1;
	q1[l].x() = 0;
	q1[l].y() = 0;
	q1[l].z() = 0; 
	T1[l].setZero();	
	*/
	q[l].w() = 1;
	q[l].x() = 0;
	q[l].y() = 0;
	q[l].z() = 0;
	T[l].setZero();//这里把l帧看作参考坐标系，根据当前帧到l帧的relative_R，relative_T，
	//得到当前帧在参考坐标系下的位姿，之后的pose[i]表示第l帧到第i帧的变换矩阵[R/T]
	q[frame_num - 1] = q[l] * Quaterniond(relative_R);//frame_num-1表示当前帧* relative c0_->ck
	T[frame_num - 1] = relative_T;
	//cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
	//cout << "init t_l " << T[l].transpose() << endl;

	//rotate to cam frame
//（2）构造容器，存储滑窗内 第L帧 相对于其他帧 和最新一帧 的位姿
	/*容器内存储的是相对运动，大写容器对应的是第l帧变换到各个帧
	 * 小写容器是用于全局BA时使用的，也同样是l帧变换到各个帧。三角化深度需要相反旋转
	 */
	Matrix3d c_Rotation[frame_num];
// 	Matrix3d c_Rotation1[frame_num];	
	Vector3d c_Translation[frame_num];
// 	Vector3d c_Translation1[frame_num];	
	Quaterniond c_Quat[frame_num];
// 	Quaterniond c_Quat1[frame_num];
	double c_rotation[frame_num][4];
// 	double c_rotation1[frame_num][4];
	double c_translation[frame_num][3];
// 	double c_translation1[frame_num][3];
	Eigen::Matrix<double, 3, 4> Pose[frame_num];
// 	Eigen::Matrix<double, 3, 4> Pose1[frame_num];

//（3）对于第l帧和最新一帧，它们的相对运动是已知的，可以直接放入容器
//从l帧旋转到各个帧的旋转平移
	c_Quat[l] = q[l].inverse();
// 	c_Quat1[l] = q1[l].inverse();
	c_Rotation[l] = c_Quat[l].toRotationMatrix();
// 	c_Rotation1[l] = c_Quat1[l].toRotationMatrix();
	c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
// 	c_Translation1[l] = -1 * (c_Rotation1[l] * T1[l]);
	Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
	Pose[l].block<3, 1>(0, 3) = c_Translation[l];
// 	Pose1[l].block<3, 3>(0, 0) = c_Rotation1[l];
// 	Pose1[l].block<3, 1>(0, 3) = c_Translation1[l];

	c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
// 	c_Quat1[frame_num - 1] = q1[frame_num - 1].inverse();
	c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
// 	c_Rotation1[frame_num - 1] = c_Quat1[frame_num - 1].toRotationMatrix();
	c_Translation[frame_num - 1] = -1 * (c_Rotation[frame_num - 1] * T[frame_num - 1]);
// 	c_Translation1[frame_num - 1] = -1 * (c_Rotation1[frame_num - 1] * T1[frame_num - 1]);
	Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
	Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];
// 	Pose1[frame_num - 1].block<3, 3>(0, 0) = c_Rotation1[frame_num - 1];
// 	Pose1[frame_num - 1].block<3, 1>(0, 3) = c_Translation1[frame_num - 1];
//（4）三角化l帧和最新帧，获得他们的共视点在l帧上的空间坐标
// 三角化前提条件：两帧的（相对）位姿已知。这样才能把他们的共视点的三维坐标还原出来。
	//1: trangulate between l ----- frame_num - 1
	//2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1; 
	for (int i = l; i < frame_num - 1 ; i++)
	{
		// solve pnp
		if (i > l)
		{
			Matrix3d R_initial = c_Rotation[i - 1];
			Vector3d P_initial = c_Translation[i - 1];
// 			Matrix3d R_initial1 = c_Rotation1[i - 1];
// 			Vector3d P_initial1 = c_Translation1[i - 1];
			if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
				return false;
			c_Rotation[i] = R_initial;
			c_Translation[i] = P_initial;
// 			c_Rotation1[i] = R_initial1;
// 			c_Translation1[i] = P_initial1;
			c_Quat[i] = c_Rotation[i];
// 			c_Quat1[i] = c_Rotation1[i];
			Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
			Pose[i].block<3, 1>(0, 3) = c_Translation[i];
// 			Pose1[i].block<3, 3>(0, 0) = c_Rotation1[i];
// 			Pose1[i].block<3, 1>(0, 3) = c_Translation1[i];
		}

		// triangulate point based on the solve pnp result
		triangulateTwoFrames(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f);
	}
	//3: triangulate l-----l+1 l+2 ... frame_num -2
	for (int i = l + 1; i < frame_num - 1; i++)
		triangulateTwoFrames(l, Pose[l], i,Pose[i], sfm_f);
	//4: solve pnp l-1; triangulate l-1 ----- l
	//             l-2              l-2 ----- l
	for (int i = l - 1; i >= 0; i--)
	{
		//solve pnp
		Matrix3d R_initial = c_Rotation[i + 1];
		Vector3d P_initial = c_Translation[i + 1];
// 		Matrix3d R_initial1 = c_Rotation1[i + 1];
// 		Vector3d P_initial1 = c_Translation1[i + 1];
		if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
			return false;
		c_Rotation[i] = R_initial;
		c_Translation[i] = P_initial;
// 		c_Rotation1[i] = R_initial1;
// 		c_Translation1[i] = P_initial1;
		c_Quat[i] = c_Rotation[i];
// 		c_Quat1[i] = c_Rotation1[i];
		Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
		Pose[i].block<3, 1>(0, 3) = c_Translation[i];
// 		Pose1[i].block<3, 3>(0, 0) = c_Rotation1[i];
// 		Pose1[i].block<3, 1>(0, 3) = c_Translation1[i];
		//triangulate
		triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
	}
	//5: triangulate all other points
	for (int j = 0; j < feature_num; j++)
	{
		if (sfm_f[j].state == true)
			continue;
		if ((int)sfm_f[j].observation.size() >= 2)
		{
			Vector2d point0, point1,pointR0,pointR1;
			int frame_0 = sfm_f[j].observation[0].first;
			point0 = sfm_f[j].observation[0].second;
			pointR0 = sfm_f[j].observation1[0].second;
			int frame_1 = sfm_f[j].observation.back().first;
			point1 = sfm_f[j].observation.back().second;
			pointR1 = sfm_f[j].observation.back().second;
			Vector3d point_3d;
			triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1,pointR0,pointR1, point_3d);
			sfm_f[j].state = true;
			sfm_f[j].position[0] = point_3d(0);
			sfm_f[j].position[1] = point_3d(1);
			sfm_f[j].position[2] = point_3d(2);
			//cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
		}		
	}

/*corres
	for (int i = 0; i < frame_num; i++)
	{
		q[i] = c_Rotation[i].transpose(); 
		cout << "solvePnP  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
	}
	for (int i = 0; i < frame_num; i++)
	{
		Vector3d t_tmp;
		t_tmp = -1 * (q[i] * c_Translation[i]);
		cout << "solvePnP  t" << " i " << i <<"  " << t_tmp.x() <<"  "<< t_tmp.y() <<"  "<< t_tmp.z() << endl;
	}
*/
	//full BA
	ceres::Problem problem;
	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
	//cout << " begin full BA " << endl;
	for (int i = 0; i < frame_num; i++)
	{
		//double array for ceres
		c_translation[i][0] = c_Translation[i].x();
		c_translation[i][1] = c_Translation[i].y();
		c_translation[i][2] = c_Translation[i].z();
		c_rotation[i][0] = c_Quat[i].w();
		c_rotation[i][1] = c_Quat[i].x();
		c_rotation[i][2] = c_Quat[i].y();
		c_rotation[i][3] = c_Quat[i].z();
		problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
		problem.AddParameterBlock(c_translation[i], 3);
		if (i == l)
		{
			problem.SetParameterBlockConstant(c_rotation[i]);
		}
		if (i == l || i == frame_num - 1)
		{
			problem.SetParameterBlockConstant(c_translation[i]);
		}
	}

	for (int i = 0; i < feature_num; i++)
	{
		if (sfm_f[i].state != true)
			continue;
		for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
		{
			int l = sfm_f[i].observation[j].first;
			ceres::CostFunction* cost_function = ReprojectionError3D::Create(
												sfm_f[i].observation[j].second.x(),
												sfm_f[i].observation[j].second.y());

    		problem.AddResidualBlock(cost_function, NULL, c_rotation[l], c_translation[l], 
    								sfm_f[i].position);	 
		}

	}
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	//options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = 0.2;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
	{
		//cout << "vision only BA converge" << endl;
	}
	else
	{
		//cout << "vision only BA not converge " << endl;
		return false;
	}
	for (int i = 0; i < frame_num; i++)
	{
		q[i].w() = c_rotation[i][0]; 
		q[i].x() = c_rotation[i][1]; 
		q[i].y() = c_rotation[i][2]; 
		q[i].z() = c_rotation[i][3]; 
		q[i] = q[i].inverse();
		//cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
	}
	for (int i = 0; i < frame_num; i++)
	{

		T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
		//cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
	}
	for (int i = 0; i < (int)sfm_f.size(); i++)
	{
		if(sfm_f[i].state)
			sfm_tracked_points[sfm_f[i].id] = Vector3d(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
	}
	return true;

}

