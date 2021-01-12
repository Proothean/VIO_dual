//#include "backend.h"
#include "../include/estimator.h" 

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    //TODO
    g = G;
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    //sum_of_back = 0;
    //sum_of_front = 0;
    frame_count = 0;
//    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;


    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
//    if (last_marginalization_info != nullptr)
//        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
//    last_marginalization_info = nullptr;
//    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();
    initFirstPoseFlag = false;

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{   
  /*初始化
   * pre_integration[frame_count] integration_base 实例 factor/integration_base.h
   * dt_buf[frame_count] 帧数值缓存 对齐
   * linear_acceleration_buf[frame_count]IMU测量值 对齐
   * angular_velocity_buf[frame_count]IMU测量值 对齐
   * Rs[frame_count]从body系到world系（left camera?), 数据是由IMU预积分得到，目前在这里存放的是没有用bias修整过的值
   * Ps[frame_count]
   * Vs[frame_count]
   * frame_count更新？
   * https://blog.csdn.net/weixin_43991178/article/details/100763656
   * 
   */
  
    //边界判断：如果当前帧不是第一帧IMU，那么就把它看成第一个IMU，而且把他的值取出来作为初始值
  //force to first imu??
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    //边界判断：如果当前IMU帧没有构造IntegrationBase，那就构造一个，后续会用上
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    //核心操作！！
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

//         if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
	//j-1时刻？？
    //ignore noise
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{//                               feature id        camera id              xyz uv vxvy
    // 1. decide marg_old or marg new and add features to f_manager
    //ROS_INFO("new image coming ------------------------------------------");
    //ROS_INFO("Adding feature points %lu", image.size());
    //ROS_INFO("the frame count is: %d",frame_count);
  /*true: 上一帧是关键帧，marg_old; false: 上一帧不是关键帧marg_second_new
   * f_manager可以看作一个存放着滑窗内所有特征点信息的容器，其中最关键的部分是list<FeaturePerId> feature.
   * 其中每一个特征点，可以看作是一个FeaturePerId的对象，它存放着一个特征点在滑窗中的所有信息，
   * 其中最关键的部分是vector<FeaturePerFrame> feature_per_frame.其中一个特征点在一个帧中的信息，
   * 可以看作是一个FeaturePerFrame的对象，它存放那个这一个特征点在滑窗里一个帧里面的信息，
   * 包括归一化坐标，像素坐标，像素速度
   */
   
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    // 2. build all_image_frame for initialization
    //ROS_INFO("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    //ROS_INFO("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    //ROS_INFO("Solving %d", frame_count);
    //ROS_INFO("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    if (solver_flag != NON_LINEAR) 
    {
        ImageFrame imageframe(image, header.stamp.toSec());//包含某一阵的全部信息：位姿，特征点信息，预积分信息，是否是关键帧
        
        imageframe.pre_integration = tmp_pre_integration;
        all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    // 3. calibrate the rotation and translation from camera to IMU------------useless
    //calibrationExRotation();

    //4. initialize！！！！！！！！！！！！！！！！！！！！！！！
    if (solver_flag == INITIAL)
	    initial(header);

    //5. optimization！！！！！！！！！！！！！！！！！！！！！！
    else
        backend.backend(this);
}
/*
void Estimator::calibrationExRotation()
{
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
}
*/
void Estimator::initial(const std_msgs::Header &header)
{
    if (frame_count == WINDOW_SIZE)
    {
        bool result = false;
        if(  (header.stamp.toSec() - initial_timestamp) > 0.1)
        {//确保有足够的frame参与初始化，有外参，且当前帧时间戳大于初始化时间戳+0.1秒
            result = intializer.initialStructure(this);//视觉惯性联合初始化
            initial_timestamp = header.stamp.toSec();//更新初始化时间戳
        }
        if(result)//初始化成功则进行一次非线性优化
        {
            solver_flag = NON_LINEAR;//非线性优化flag
            backend.solveOdometry(this);//执行非线性优化具体函数solverOdometry()
            backend.slideWindow(this);//进行滑窗
            f_manager.removeFailures();
            ROS_INFO("Initialization finish!");
            last_R = Rs[WINDOW_SIZE];//得到当前帧与第一帧的位姿
            last_P = Ps[WINDOW_SIZE];
            last_R0 = Rs[0];
            last_P0 = Ps[0];               
        }
        else
            backend.slideWindow(this);//初始化不成功则进行滑窗操作
    }
    else
        frame_count++;//图像帧+1
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = backend.para_Pose[i][j];
        }
    }
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    //printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}
