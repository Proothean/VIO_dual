#include "../include/feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


// the following function belongs to FeatureTracker Class
FeatureTracker::FeatureTracker() {}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void FeatureTracker::equalize(const cv::Mat &_img,cv::Mat &img)
{
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;
}

void FeatureTracker::flowTrack()
{
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status,status1;
        vector<float> err;
        //track previous left
	
	/*void cv::calcOpticalFlowPyrLK	(	
	  InputArray 	prevImg,
	  InputArray 	nextImg,
	  InputArray 	prevPts,
	  InputOutputArray 	nextPts,
	  OutputArray 	status, 输出状态向量；如果找到相应特征的流，则元素设置为1,否则设置为0.
	  OutputArray 	err, 输出错误的矢量；向量的每个元素都设置为相应特征的错误
	  Size 	winSize = Size(21, 21), 每个金字塔等级的搜索窗口大小
	  int 	maxLevel = 3, 基于0的最大金字塔等级数；如果设置为0,则不是用金字塔；如果为1 则使用两个级别
	  TermCriteria 	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),制定迭代搜索算法的终止条件，
					在指定的最大迭代次数criteria.maxCount之后或当搜索窗口移动小于criteria.epsilon时）
	  int 	flags = 0,
	  double 	minEigThreshold = 1e-4 
)		
	 * 
	 * 
	 * 
	 */
    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
	if(1){
	  vector<uchar> reverse_status;
	  vector<cv::Point2f> reverse_pts = cur_pts;
	  cv::calcOpticalFlowPyrLK(forw_img, cur_img, forw_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
	  for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(cur_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
	}
	
	cv::Mat img_show = forw_img.clone();	
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
	

	/*
	for(int i=0;i<cur_pts.size();i++)
	{
	  ROS_INFO_STREAM("cur_pts : "<<std::endl<<cur_pts[i]);
	  ROS_INFO("i=%d",i);
	}	    
	for(int i=0;i<forw_pts.size();i++)
	{
	  ROS_INFO_STREAM("forw_pts : "<<std::endl<<forw_pts[i]);
	  ROS_INFO("i=%d",i);
	}
	*/

        reduceVector(cur_pts, status);
//         if (cur_pts1.size() > 0)
//             reduceVector(cur_pts1, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        //reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);

	/*
	for ( auto kp:forw_pts )
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);    
	
	cv::imshow("corners", img_show);
	cv::waitKey(1);
	*/
        //track left to right
	

//        if(1){
 	if(!forw_pts.empty()){
	
	  vector<cv::Point2f> reverseLeftPts;
	  vector<uchar> statusRightLeft;
	  vector<float> err; 
	  cv::calcOpticalFlowPyrLK(forw_img, forw_img1, forw_pts, forw_pts1, status1, err, cv::Size(21, 21), 3);
	  if(1){
	  cv::calcOpticalFlowPyrLK(forw_img1, forw_img, forw_pts1, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
	   for(size_t i = 0; i < status1.size(); i++)
                {
                    if(status1[i] && statusRightLeft[i] && inBorder(forw_pts1[i]) && distance(forw_pts[i], reverseLeftPts[i]) <= 0.5)
                        status1[i] = 1;
                    else
                        status1[i] = 0;
                }
	  }
	}
	
	
	/*    
	for(int i=0;i<forw_pts.size();i++)
	{
	  ROS_INFO_STREAM("forw_pts1 : "<<std::endl<<forw_pts1[i]);
	  ROS_INFO("i=%d",i);
	}	
	*/
	cv::Mat img_show1 = forw_img1.clone();

	

        reduceVector(cur_pts, status1);
        if (cur_pts1.size() > 0)
            reduceVector(cur_pts1, status1);
        reduceVector(forw_pts, status1);
        reduceVector(forw_pts1, status1);
        reduceVector(ids, status1);
        //reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status1);
	
/*
	ids1=ids;
	reduceVector(forw_pts1, status1);
	reduceVector(ids1, status1);
*/
/*
	for ( auto kp:forw_pts1 )
	  cv::circle(img_show1, kp, 10, cv::Scalar(0, 120, 0), 1);        
	cv::imshow("corners1", img_show1);
	cv::waitKey(1);
			
	for ( auto kp:forw_pts )
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);    
	
	cv::imshow("corners", img_show);
	cv::waitKey(1);
*/	
	/*
	for (int i =0; i<forw_pts.size();i++)
	{
	  ROS_INFO_STREAM("feature point id: " << std::endl << ids[i]);
	  ROS_INFO_STREAM("current point : " << std::endl << cur_pts[i]);
	  ROS_INFO_STREAM("following point : " << std::endl << forw_pts[i]);
	  ROS_INFO_STREAM("following point : " << std::endl << forw_pts1[i]);

	}
	*/
	
	
	
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());

      
    }
}

void FeatureTracker::trackNew()
{
    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;

        //the order of ids, forw_pts and track_cnt will change
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);

        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
}

void FeatureTracker::setMask1()
{
    //if(FISHEYE)
        //mask = fisheye_mask.clone();
   // else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // prefer to keep features that are tracked for long time in left
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
//      vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id1;//---------------

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
    }

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });
/*
    for (unsigned int i = 0; i < forw_pts1.size(); i++)//------------------------------
        cnt_pts_id1.push_back(make_pair(track_cnt[i], make_pair(forw_pts1[i], ids[i])));

    sort(cnt_pts_id1.begin(), cnt_pts_id1.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });*/

    cur_pts.clear();
//      forw_pts1.clear();//---------------
    ids.clear();
//      ids1.clear();//----------------------------
    track_cnt.clear();

//      auto it1 = cnt_pts_id1.begin();//-----------
    for (auto &it : cnt_pts_id)
    {//当前特征点位置对应的mask值为255,则保留当前特征值，将对应的特征点位置pts，id，被追踪次数cnt分别存入
        if (mask.at<uchar>(it.second.first) == 255)// this place no features
        {
            cur_pts.push_back(it.second.first);
//              forw_pts1.push_back((*it1).second.first);//-------------------
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }//在mask中将当前特征点卓为半径为MIN_DIST的区域设置为0,后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
//          it1 ++;//-------------------
    }
//      ids1 = ids;//------------------
}


void FeatureTracker::setMask()
{
    //if(FISHEYE)
        //mask = fisheye_mask.clone();
   // else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // prefer to keep features that are tracked for long time in left
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
     vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id1;//---------------

    for (unsigned int i = 0; i < forw_pts.size(); i++)
    {cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
    }

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    for (unsigned int i = 0; i < forw_pts1.size(); i++)//------------------------------
        cnt_pts_id1.push_back(make_pair(track_cnt[i], make_pair(forw_pts1[i], ids[i])));

    sort(cnt_pts_id1.begin(), cnt_pts_id1.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
     forw_pts1.clear();//---------------
    ids.clear();
     ids1.clear();//----------------------------
    track_cnt.clear();

     auto it1 = cnt_pts_id1.begin();//-----------
    for (auto &it : cnt_pts_id)
    {//当前特征点位置对应的mask值为255,则保留当前特征值，将对应的特征点位置pts，id，被追踪次数cnt分别存入
        if (mask.at<uchar>(it.second.first) == 255)// this place no features
        {
            forw_pts.push_back(it.second.first);
             forw_pts1.push_back((*it1).second.first);//-------------------
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }//在mask中将当前特征点卓为半径为MIN_DIST的区域设置为0,后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
         it1 ++;//-------------------
    }
     ids1 = ids;//------------------
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
	
    }
}

void FeatureTracker::readImage(const cv::Mat &_img,const cv::Mat &_img1, double _cur_time)
{
    cv::Mat img;
    cv::Mat img1;
    TicToc t_r;
    cur_time = _cur_time;

    equalize(_img, img);//---------------------------------------------------
    equalize(_img1, img1);
     
    if (forw_img.empty() || forw_img1.empty())
    {
        cur_img = forw_img = img;
        cur_img1 = forw_img1 = img1;
    }
    else
    {
        forw_img = img;
        forw_img1 = img1;
    }

    forw_pts.clear();
    forw_pts1.clear();
    ids1.clear();

    flowTrack();//LK optical---------------------------------------------------------------

    for (auto &n : track_cnt)
        n++;

    trackNew();//rejectWithF--------------------------------------------------------------
	       //setMask
	       //cv::goodFeaturesToTrack
	       //addPoints
    cur_img = forw_img;
    cur_img1 = forw_img1;//cur_img1 seems useless.

    //cur_pts and ids are BIGGER than cur_pts1, because of NEW FEATURES!
    cur_pts.clear();
    cur_pts1.clear();
    cur_pts = forw_pts;
    cur_pts1 = forw_pts1;
    /*
    for (int i=0;i<cur_pts.size();i++){
    ROS_INFO_STREAM("cur_pts : "<<std::endl<<cur_pts[i]);
    if(!cur_pts1.empty())
    ROS_INFO_STREAM("cur_pts1 : "<<std::endl<<cur_pts1[i]);
    }
    */
    undistortedPoints();//--------------------------------------------------------------------

    drawTrack(cur_img, cur_img1, ids, cur_pts, cur_pts1, prevLeftPtsMap);//show keypoint on R&L image
    //vector<cv::DMatch> matches;
    
    //cv::drawKeypoints(cur_img,cur_pts, cur_img1,cur_pts1);
    
    
    
    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    prev_time = cur_time;
}

// map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
 void FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
//     ROW = cur_img.rows;
//     COL = cur_img.cols;
    cur_img1 = _img1;

    cur_pts.clear();

//         if (forw_img.empty() || forw_img1.empty())
//     {
//         cur_img = forw_img = _img;
//         cur_img1 = forw_img1 = _img1;
//     }
//     else
//     {
//         forw_img = _img;
//         forw_img1 = _img1;
//     }
    
//     forw_pts.clear();
//     forw_pts1.clear();
//     ids1.clear();
    
    
    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
//         ROS_INFO("prev->cur");
        // reverse check
        if(1)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
//             ROS_INFO("cur->prev");
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
//         ROS_INFO("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        //rejectWithF();
//         ROS_INFO("set mask begins");
        TicToc t_m;
        setMask1();
//         ROS_INFO("set mask costs %fms", t_m.toc());

//         ROS_INFO("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
//             ROS_INFO("goodFeatureToTrack");
        }
        else
            n_pts.clear();
//         ROS_INFO("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)
        {
//             ROS_INFO("1");
            cur_pts.push_back(p);
            ids.push_back(n_id++);
//             ids.push_back(-1);
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }
//          ROS_INFO("2");


    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
   ROS_INFO_STREAM("cur_un_pts=:"<<std::endl<<cur_un_pts);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);
    if(!_img1.empty() )
    {
        
        ids1.clear();
        cur_pts1.clear();
        cur_un_pts1.clear();
        pts_velocity1.clear();
        cur_un_pts_map1.clear();
       
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, cur_img1, cur_pts, cur_pts1, status, err, cv::Size(21, 21), 3);
//             ROS_INFO("forw->cur1");
            // reverse check cur right ---- cur left
            if(1)
            {
                cv::calcOpticalFlowPyrLK(cur_img1, cur_img, cur_pts1, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
//                 ROS_INFO("cur1->frow");
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_pts1[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids1 = ids;
            reduceVector(cur_pts1, status);
            reduceVector(ids1, status);
            // only keep left-right pts
/*           
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(cur_un_pts1, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_pts1 = undistortedPts(cur_pts1, m_camera[1]);
            ROS_INFO_STREAM("cur_un_pts1=:"<<std::endl<<cur_un_pts1);
            pts_velocity1 = ptsVelocity(ids1, cur_un_pts1, cur_un_pts_map1, prev_un_pts_map1);
        }
        prev_un_pts_map1 = cur_un_pts_map1;
    }
     if(SHOW_TRACK)
        drawTrack(cur_img, cur_img1, ids, cur_pts, cur_pts1, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
   

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];
//--------------------------------------------------------------------------------------
     
    for (size_t i = 0; i < ids.size(); i++)
    {
        
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = forw_pts[i].x;
        p_v = forw_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() )
    {
        for (size_t i = 0; i < ids1.size(); i++)
        {
            int feature_id = ids1[i];
            double x, y ,z;
            x = cur_un_pts1[i].x;
            y = cur_un_pts1[i].y;
            z = 1;
            double p_u, p_v;
            p_u = forw_pts1[i].x;
            p_v = forw_pts1[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = pts_velocity1[i].x;
            velocity_y = pts_velocity1[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    printf("feature track whole time %f\n", t_r.toc());
    //------------------------------------------------------------------
}











void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        //ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts1(forw_pts1.size());

        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[1]->liftProjective(Eigen::Vector2d(forw_pts1[i].x, forw_pts1[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts1[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts1, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        //reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        if (cur_pts1.size() > 0)
            reduceVector(cur_pts1, status);
        reduceVector(forw_pts, status);
        reduceVector(forw_pts1, status);
        //reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(ids1, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts1.size(), 1.0 * forw_pts1.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());

    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
            
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    //if (calib_file.size() == 2)
    //    stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        }

    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;

        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
//         ROS_INFO_STREAM("un_pts : "<<std::endl<<un_pts);
//         ROS_INFO("2.9");
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    
    for (unsigned int i = 0; i < ids.size(); i++)
    {   
//         ROS_INFO("3.1.1");
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }
    
    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {   
        for (unsigned int i = 0; i < forw_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    
    return pts_velocity;
}






void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts1.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera[0]->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
	//ROS_INFO("0 publish %f, [x]: %f, [y]: %f", cur_time, b.x(), b.y());                     2d point
	//ROS_INFO_STREAM("cur_pts : "<<std::endl<<cur_pts[i]);
    }

    if (cur_pts1.size() > 0)
    {
        //put them out here!!
        //assert(ids1.size() == cur_pts1.size());

        // for (unsigned int i = 0; i < ids1.size(); i++)
        //     assert(ids1[i] == ids[i]);

        for (unsigned int i = 0; i < cur_pts1.size(); i++)
        {
            Eigen::Vector2d a1(cur_pts1[i].x, cur_pts1[i].y);
            Eigen::Vector3d b1;
            m_camera[1]->liftProjective(a1, b1);
            cur_un_pts1.push_back(cv::Point2f(b1.x() / b1.z(), b1.y() / b1.z()));
	    //ROS_INFO("1 publish %f, [x]: %f, [y]: %f", cur_time, b1.x(), b1.y());              2d point
	    //ROS_INFO_STREAM("cur_pts1 : "<<std::endl<<cur_pts1[i]);
        }
    }
    
    
    
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty())
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
    }
}

cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}

