/**
* This file is part of ORB-SLAM2.
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "MapTracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>

#include <mutex>
#include <chrono>
#include "Converter.h"

#define DEBUG_TRACKING


using namespace std;

namespace ORB_SLAM2
{

MapTracking::MapTracking (System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer,
										MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB,
										const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
		mLocalMapper(NULL), mpReferenceKF(NULL)

{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST); // For initializer

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    global_pub = pSys->monoNode.advertise<pcl::PointCloud<pcl::PointXYZ>>("global_points", 1, true);
    local_pub = pSys->monoNode.advertise<pcl::PointCloud<pcl::PointXYZ>>("local_points", 1, true);
    orb_pose_pub = pSys->monoNode.advertise<geometry_msgs::PoseStamped>("/orb_pose", 10);
}


void MapTracking::setMapLoaded()
{
    mState = MAP_OPEN;
    mvpLocalMapPoints = mpMap->GetReferenceMapPoints();
}

void MapTracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void MapTracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void MapTracking::SetMapPublisher(MapPublisher *pViewer)
{
    mpMapPublisher = pViewer;
}

cv::Mat MapTracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    double sum_time = 0;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    std::cout << "Convert gray Image " << ttrack << "\n";
		sum_time += ttrack;

    t1 = std::chrono::steady_clock::now();

    if(mState==NOT_INITIALIZED or mState==NO_IMAGES_YET or mState==LOST) // diff
        mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth);
    else
        mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth);

    t2 = std::chrono::steady_clock::now();
    ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    std::cout << "Orb Image " << ttrack << "\n";
		sum_time += ttrack;

    t1 = std::chrono::steady_clock::now();
    Track();
    t2 = std::chrono::steady_clock::now();
    ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    std::cout << "Tracking Image " << ttrack << "\n";
		sum_time += ttrack;

		std::cout << "Sum of basic module time: " << sum_time << "\n";

		// t1 = std::chrono::steady_clock::now();
		// mpLocalMapper->RunOnce();
		// t2 = std::chrono::steady_clock::now();
    // ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    // std::cout << "local mapping " << ttrack << "\n";

    return mCurrentFrame.mTcw.clone();
}

void MapTracking::Track()
{
    if(mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    std::cout << "Number of local map KF: " << mvpLocalKeyFrames.size() << '\n';

    mLastProcessedState = mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState == NOT_INITIALIZED)
    {
        MonocularInitialization();

        if (mpFrameDrawer != NULL)
        	mpFrameDrawer->Update(this);

        if(mState != OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;
				std::chrono::steady_clock::time_point t1;
				std::chrono::steady_clock::time_point t2;
				double ttrack;
        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // Local Mapping is activated. This is the normal behaviour, unless
        // you explicitly activate the "only tracking" mode.
        if(mState == OK)
				{
		        // Local Mapping might have changed some MapPoints tracked in last frame
		        CheckReplacedInLastFrame();

		        if(mVelocity.empty()) // || mCurrentFrame.mnId<mnLastRelocFrameId+2)
		        {
		            bOK = TrackReferenceKeyFrame();
		        }
		        else
		        {
							  t1 = std::chrono::steady_clock::now();
		            bOK = TrackWithMotionModel(); // matching with previous frame
								t2 = std::chrono::steady_clock::now();
								ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
								std::cout << "TrackWithMotionModel " << ttrack << "\n";

		            if(!bOK) {
								    t1 = std::chrono::steady_clock::now();
									  bOK = TrackReferenceKeyFrame();
										t2 = std::chrono::steady_clock::now();
										ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
										std::cout << "TrackReferenceKeyFrame " << ttrack << "\n";
								}
		        }
				}

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
				t1 = std::chrono::steady_clock::now();
        if(bOK)
            bOK = TrackLocalMap();
				t2 = std::chrono::steady_clock::now();
				ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
				std::cout << "TrackLocalMap " << ttrack << "\n";

        // mCurrentFrame.mTcwを使って位置調整？
        pcl::PointCloud<pcl::PointXYZ>::Ptr priorMap = mpMap->GetPriorMapPoints();
        if (priorMap) {
            t1 = std::chrono::steady_clock::now();
            ScanWithNDT(mCurrentFrame.mTcw);
            // ScanWithNDT(mCurrentFrame.mpReferenceKF->GetPoseInverse());
            // mCurrentFrame.mpReferenceKF->GetPoseInverse().t();
            t2 = std::chrono::steady_clock::now();
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << "ScanWithNDT " << ttrack << "\n";
        }

        if(bOK){
            mState = OK;
        }else{
            mState = LOST;
            std::cout << "Lost tracking\n";
            exit(0);
        }

        // Update drawer
        if (mpFrameDrawer != NULL)
            mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        // Update motion model
        if(!mLastFrame.mTcw.empty())
        {
            cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
            mVelocity = mCurrentFrame.mTcw*LastTwc;
        }
        else
            mVelocity = cv::Mat();

        if (mpMapDrawer != NULL)
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        // Clean temporal point matches
        for(int i=0; i<mCurrentFrame.N; ++i)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
                if(pMP->Observations() < 1)
                {
                    mCurrentFrame.mvbOutlier[i] = false;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }
        }

        // Check if we need to insert a new keyframe
        if(NeedNewKeyFrame()) {
            CreateNewKeyFrame();
						// mpLocalMapper->RunOnce();
        }

        // We allow points with high innovation (considered outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for(int i=0; i<mCurrentFrame.N;++i)
        {
            if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        // std::cout << "Current Position\n" << mCurrentFrame.mTcw << "\n";
        std::cout << "Current inverse Position\n" << mCurrentFrame.mTcw.inv() << "\n";
        // XXX: We found some occurences of empty pose from mpReferenceKF
        if (mCurrentFrame.mpReferenceKF->GetPose().empty()==true)
            cout << "XXX: KF pose is empty" << endl;

        cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        //  cout << mCurrentFrame.mpReferenceKF << endl;
        mlRelativeFramePoses.push_back(Tcr);
        mlAbsoluteFramePoses.push_back(mCurrentFrame.mTcw);
        mlpReferences.push_back(mpReferenceKF);
    }
}

void MapTracking::SetSourceMap(pcl::PointCloud<pcl::PointXYZ>::Ptr priorMap) {
    icp_.setInputTarget(priorMap);
    return;
}

void MapTracking::ScanWithNDT(cv::Mat currAbsolutePos)
{
    // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    std::cout << "ScanWithNDT\n";
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    // Local Map
    pcl::PointCloud<pcl::PointXYZ>::Ptr l_points (new pcl::PointCloud<pcl::PointXYZ>);

    for (auto&& p: spRefMPs) {
        if (p == NULL || p->isBad())
            continue;
        cv::Mat pos = p->GetWorldPos();
        pcl::PointXYZ point;
        point.x = pos.at<float>(0);
        point.y = pos.at<float>(1);
        point.z = pos.at<float>(2);
        l_points->push_back(point);
    }

		std::cout << "Set ICP configuration\n";

    icp_.setInputSource(l_points);
    icp_.setMaximumIterations(10);
    icp_.setDistThreshold(1.5);
    pcl::PointCloud<pcl::PointXYZ> output;

		std::cout << "Start alignment\n";

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    icp_.align (output);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    std::cout << "[done, " << ttrack <<  " s : " << output.width * output.height << " points], has converged: ";
    std::cout << icp_.hasConverged() << " with score: " << icp_.getFitnessScore () << "\n";
    Eigen::Matrix4d transformation = icp_.getFinalTransformation ();
    std::cout << "Transformation\n";
    std::cout << transformation << "\n";

    // // currAbsolutePos.copyTo(Tcw);
    // cv::Mat Rcw = currAbsolutePos.rowRange(0,3).colRange(0,3);
    // cv::Mat tcw = currAbsolutePos.rowRange(0,3).col(3);
    // cv::Mat Rwc = Rcw.t();
    // cv::Mat Ow = -Rwc*tcw;
    //
    // cv::Mat Twc = cv::Mat::eye(4,4,currAbsolutePos.type());
    // Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    // Ow.copyTo(Twc.rowRange(0,3).col(3));
}


// void MapTracking::ScanWithNDT(cv::Mat currAbsolutePos)
// {
//     std::cout << "ScanWithNDT\n";
//     const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
//     const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
//
//     set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
// 		ros::Time current_scan_time = ros::Time::now();
//
//     if(vpMPs.empty())
//         return;
//
//     pcl::PointCloud<pcl::PointXYZ>::Ptr g_points (new pcl::PointCloud<pcl::PointXYZ>);
//
//     for (auto&& p: vpMPs) {
//       if (p == NULL || p->isBad() || spRefMPs.count(p))
//         continue;
//       cv::Mat pos = p->GetWorldPos();
//       pcl::PointXYZ point;
//       point.x = pos.at<float>(2) + 0.27;
//       point.y = -pos.at<float>(0) + 0.06;
//       point.z = -pos.at<float>(1) - 0.05;
//       g_points->push_back(point);
//     }
//     sensor_msgs::PointCloud2 pc2_g;
//
//     pc2_g.header.frame_id = "velodyne";
//     pc2_g.header.stamp = current_scan_time; //header->stamp;
//     // pc2_g.header.seq=header->seq;
//     g_points->header = pcl_conversions::toPCL(pc2_g.header);
//     global_pub.publish(g_points);
//
//     pcl::PointCloud<pcl::PointXYZ>::Ptr l_points (new pcl::PointCloud<pcl::PointXYZ>);
//
//     for (auto&& p: spRefMPs) {
//       if (p == NULL || p->isBad())
//         continue;
//       cv::Mat pos = p->GetWorldPos();
//       pcl::PointXYZ point;
//       point.x = pos.at<float>(2) + 0.27;
//       point.y = -pos.at<float>(0) + 0.06;
//       point.z = -pos.at<float>(1) - 0.05;
//       l_points->push_back(point);
//     }
//     sensor_msgs::PointCloud2 pc2_l;
//     pc2_l.header.frame_id= "velodyne";
// 		pc2_l.header.stamp = current_scan_time; //header->stamp;
//     l_points->header = pcl_conversions::toPCL(pc2_l.header);
//     local_pub.publish(l_points);
//
// 		// currAbsolutePos.copyTo(Tcw);
// 		cv::Mat Rcw = currAbsolutePos.rowRange(0,3).colRange(0,3);
// 		cv::Mat tcw = currAbsolutePos.rowRange(0,3).col(3);
// 		cv::Mat Rwc = Rcw.t();
// 		cv::Mat Ow = -Rwc*tcw;
//
// 		cv::Mat Twc = cv::Mat::eye(4,4,currAbsolutePos.type());
// 		Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
// 		Ow.copyTo(Twc.rowRange(0,3).col(3));
//
// 		static tf::TransformBroadcaster br;
// 		tf::Quaternion current_q;
// 		tf::Transform transform;
//
// 		Eigen::Quaterniond q = Converter::toQuaternion(Twc);
//
// 		orb_pose_msg.header.frame_id = "/orb_pose";
// 		orb_pose_msg.header.stamp = current_scan_time; //ros::Time::now();
// 		orb_pose_msg.pose.position.x = 0;
// 		orb_pose_msg.pose.position.y = 0;
// 		orb_pose_msg.pose.position.z = 0;
// 		orb_pose_msg.pose.orientation.x = q.z();
// 		orb_pose_msg.pose.orientation.y = -q.x();
// 		orb_pose_msg.pose.orientation.z = -q.y();
// 		orb_pose_msg.pose.orientation.w = q.w();
// 		orb_pose_pub.publish(orb_pose_msg);
//
// 		// current_q.setRPY(x, y, z);
// 		current_q.setRPY(0, 0, 0);
// 		transform.setOrigin(tf::Vector3(Twc.at<float>(2, 3) + 0.27,
// 		                                -Twc.at<float>(0, 3) + 0.06,
// 																		-Twc.at<float>(1, 3) - 0.05));
// 		transform.setRotation(current_q);
// 		// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "orb_pose"));
// 		br.sendTransform(tf::StampedTransform(transform, current_scan_time, "velodyne", "orb_pose"));
// 		current_q.setRPY(0, 0, 0);
// 		transform.setOrigin(tf::Vector3(0.81, 0, 1.73));
// 		transform.setRotation(current_q);
// 		br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "velodyne"));
//
// 		// static tf::TransformBroadcaster br;
// 		// tf::Quaternion current_q;
// 		// tf::Transform transform;
// 		// current_q.setRPY(0, 0, 0);
// 		// transform.setOrigin(tf::Vector3(1, 1, 1));
//     // transform.setRotation(current_q);
// }

void MapTracking::MapOpenMonocularInitialization ()
{
    mnLastKeyFrameId = mCurrentFrame.mnId;
}

void MapTracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size() <= 100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
        	cerr << "Not enough matches: " << nmatches << endl;
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            std::cout << "tcw: " << tcw << "\n";
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // TODO: 仮のTransformの値
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            // tcw *= 100;
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
	    std::cout << "##### Initialization Success #####\n";
        }
        else {
            cerr << "##### Initialization failed #####\n";
        }
    }
}

void MapTracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
				// worldPos *= ratio;

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

		// Bundle Adjustment
		cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

		// # TODO: Update temporary transformation value

		Optimizer::GlobalBundleAdjustemnt(mpMap,20);

		// Set median depth to 1
		float medianDepth = pKFini->ComputeSceneMedianDepth(2);
		float invMedianDepth = 1.0f/medianDepth;

		if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
		{
				cout << "Wrong initialization, reseting..." << endl;
				Reset();
				return;
		}

		// Scale initial baseline
		cv::Mat Tc2w = pKFcur->GetPose();

		std::cout << "Tc2w\n" << Tc2w << "\n";
		// TODO: Scale alignment
		invMedianDepth = 0.75;
		// invMedianDepth = -0.8586941 / Tc2w.at<float>(2, 3);
		std::cout << "invMedianDepth: " << invMedianDepth << "\n";

		Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
		pKFcur->SetPose(Tc2w);

		std::cout << "Initial Tc2w\n";
		std::cout << Tc2w << "\n";

		// Scale points
		vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
		for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
		{
				if(vpAllMapPoints[iMP])
				{
						MapPoint* pMP = vpAllMapPoints[iMP];
						pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
				}
		}


    // Scale points
    // vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    // for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    // {
    //     if(vpAllMapPoints[iMP])
    //     {
    //         MapPoint* pMP = vpAllMapPoints[iMP];
    //         pMP->SetWorldPos(pMP->GetWorldPos()*0.72); //ratio);
    //     }
    // }
    // // Bundle Adjustment
    // cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
    // Optimizer::InitialGlobalBundleAdjustemnt(mpMap, 20);
		//
    // // Set median depth to 1
    // float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    // //float invMedianDepth = 1.0f/medianDepth;
		//
    // if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    // {
    //     cout << "Wrong initialization, reseting..." << endl;
    //     Reset();
    //     return;
    // }

		// // Scale initial baseline
		// std::cout <<
		//
		// cv::Mat Tc2w = pKFcur->GetPose();
		// std::cout << "orig\n" << Tc2w << "\n";
		// cv::Mat initPose = (cv::Mat_<float>(4, 4) <<
		// 											 0.9999978, 5.272628/10000, -2.066935/1000, -4.690294/100,
		// 											 -5.296506/10000, 0.9999992, -1.154865/1000, -2.839928/100,
		// 											 2.066324/1000, 1.155958/1000, 0.9999971, 0.8586941,
		// 											 0, 0, 0, 1);
		// cv::Mat invPose = initPose.inv();
		// double ratio =  invPose.at<float>(2, 3) / Tc2w.at<float>(2, 3);
		// Tc2w = invPose;
		// std::cout << "InitPose\n" << initPose << "\n";
		// std::cout << "invPose\n" << Tc2w << "\n";
		//
		// //invMedianDepth = -0.8586941 / Tc2w.at<float>(2, 3);
		// std::cout << "ratio: " << ratio << "\n";
		// pKFcur->SetPose(Tc2w);
		// mCurrentFrame.SetPose(pKFcur->GetPose());

    // Scale initial baseline
    // TODO: Scale alignment
    //invMedianDepth = 0.72;
    //invMedianDepth = -0.8586941 / Tc2w.at<float>(2, 3);
    //std::cout << "invMedianDepth: " << invMedianDepth << "\n";

    //Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    //pKFcur->SetPose(Tc2w);

    // Scale points
    //vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    //for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    //{
    //    if(vpAllMapPoints[iMP])
    //    {
    //        MapPoint* pMP = vpAllMapPoints[iMP];
    //        pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
    //    }
    //}

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());
    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
}

void MapTracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; ++i)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool MapTracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if(nmatches < 15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                --nmatches;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                ++nmatchesMap;
        }
    }

    return nmatchesMap >= 10;
}

void MapTracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());
    return;
}

/*
Matching with previous frame using predicted motion model.
*/
bool MapTracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 15, mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search <th * 2>
    if(nmatches < 20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 30, mSensor==System::MONOCULAR);
    }

    if(nmatches < 20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; ++i)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                --nmatches;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                ++nmatchesMap;
        }
    }

    return nmatchesMap>=10;
}

bool MapTracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    mnMatchesInliers++;

            }
        }
    }

		std::cout << "Number of match in TrackLocalMap: " << mnMatchesInliers << "\n";
    // Decide if the tracking was successful
    if(mnMatchesInliers < 10) {
        return false;
    }
    else
        return true;
}

bool MapTracking::NeedNewKeyFrame()
{
    const int nKFs = mpMap->KeyFramesInMap();

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    const float ratioMap = 1.0;

    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*0.9f || ratioMap<thMapRatio) && mnMatchesInliers>15);

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void MapTracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;
    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void MapTracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch > 0)
    {
        ORBmatcher matcher(0.8);
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, 1);
    }
}

void MapTracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();

		std::cout << "mvpLocalMapPoints size: " << mvpLocalMapPoints.size() << "\n";
}

void MapTracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for (const auto& pKF : mvpLocalKeyFrames)
    {
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

				for (const auto& pMP : vpMPs)
        {
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void MapTracking::UpdateLocalKeyFrames()
{
	  std::cout << "Number of local map KF: Before Filtering: " << mvpLocalKeyFrames.size() << '\n';
		vector<KeyFrame*> vpLocalKeyFrames = mCurrentFrame.mpReferenceKF->GetVectorCovisibleKeyFrames();
		std::cout << "Number of local map KF: Before Filtering: " << vpLocalKeyFrames.size() << '\n';
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; ++i)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
								for (const auto& it : observations)
								    ++keyframeCounter[it.first];
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

		std::cout << "KeyFrameCounter size: " << keyframeCounter.size() << "\n";

    int max = 0;
    KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    //mvpLocalKeyFrames.reserve(3*keyframeCounter.size());
    mvpLocalKeyFrames.reserve(keyframeCounter.size());

    std::vector<int> sortIndex;
    sortIndex.reserve((int)keyframeCounter.size());

    std::vector<int> countSoter;
    countSoter.reserve((int)keyframeCounter.size());
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    int count = 0;
    for (const auto& it : keyframeCounter)
    {
        KeyFrame* pKF = it.first;

        if(pKF==NULL || pKF->isBad())
            continue;

        if(it.second > max)
        {
            max = it.second;
            pKFmax = pKF;
        }

        countSoter.push_back(it.second);
        sortIndex.push_back(count);
        mvpLocalKeyFrames.push_back(it.first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
				++count;
    }

    std::sort(sortIndex.begin(), sortIndex.end(), [&](int i1, int i2){return countSoter[i1] > countSoter[i2]; });

    int max_size = 10;

    //for (int i=0; i < (int)countSoter.size(); ++i)
    //    std::cout << "ID: " << mvpLocalKeyFrames[sortIndex[i]]->mnId << "\t" << countSoter[sortIndex[i]] << "\n";

    std::vector<KeyFrame*> filteredKeyFrames;
    filteredKeyFrames.reserve(max_size);

    int filter_size = max_size;
    if (filter_size > countSoter.size())
        filter_size = countSoter.size();
    for (int i=0; i < filter_size; ++i)
        filteredKeyFrames.push_back(mvpLocalKeyFrames[sortIndex[i]]);
        //std::cout << "ID: " << filteredKeyFrames[i]->mnId << "\t" << countSoter[sortIndex[i]] << "\n";

    mvpLocalKeyFrames = filteredKeyFrames;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    std::cout << "Create KeyFrame " << ttrack << "\n";
    std::cout << "Number of local map KF: " << countSoter.size() << '\n';
    std::cout << "Number of local map KF: After Filtering: " << mvpLocalKeyFrames.size() << '\n';

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}


bool MapTracking::Relocalization()
{
	  return true;
}

void MapTracking::Reset()
{
    // mpMapPublisher->RequestStop();

    cout << "System Reseting" << endl;
    // while(!mpMapPublisher->isStopped())
    //     usleep(3000);

		// Reset Local Mapping
		cout << "Reseting Local Mapper...";
		mpLocalMapper->RequestReset();
		cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
		mlAbsoluteFramePoses.clear();
    mlpReferences.clear();
    // mlFrameTimes.clear();
    // mlbLost.clear();

    // mpMapPublisher->Release();
}

void MapTracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}


void MapTracking::ChangeCalibration(const double fx, const double fy, const double cx, const double cy)
{
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);
    printf ("IntrinsicMatrix changed to %f,%f,%f,%f\n", fx, fy, cx, cy);
}


void MapTracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
