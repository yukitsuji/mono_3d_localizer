/**
* This file is part of ORB-SLAM2.
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


#ifndef MAPTRACKING_H
#define MAPTRACKING_H

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
//#include <pcl/console/print.h>
//#include <pcl/console/time.h>
//#include <pcl/console/parse.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_estimation_normal_shooting.h>
//#include <pcl/registration/transformation_estimation_lm.h>
//#include <pcl/registration/correspondence_rejection_distance.h>
//#include <pcl/registration/correspondence_rejection_one_to_one.h>
//#include <pcl/registration/correspondence_rejection_median_distance.h>
//#include <pcl/registration/correspondence_rejection_sample_consensus.h>
//#include <pcl/registration/correspondence_rejection_trimmed.h>
//#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <icp_7dof/icp_7dof.h>
//#include <icp_7dof/icp_7dof_transform_lm.h>

#include <Eigen/Geometry>

#include "MapPublisher.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

typedef Eigen::Transform<float,3,Eigen::Affine> Transform3;


namespace ORB_SLAM2
{

class MapPublisher;
class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class System;

//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;
//using namespace pcl::registration;

class MapTracking
{

public:
    MapTracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    Transform3 LocalizeImage (const cv::Mat &image, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetViewer(Viewer *pViewer);
    void SetMapPublisher(MapPublisher* pMapPublisher);

    // Load new settings
    // The focal length should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal length
    void ChangeCalibration(const string &strSettingPath);
    void ChangeCalibration(const double fx, const double fy, const double cx, const double cy);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void setMapLoaded ();

    // XXX: Stub
    KeyFrame* getNearestKeyFrame()
    { return mpLastKeyFrame; }

    inline void setFps (int f)
    {  mMaxFrames = f; }

    // Setup prior map
    // void SetSourceMap(pcl::PointCloud<pcl::PointXYZ>::Ptr priorMap);
    bool isUpdateMap = false;

    void SetICP(pcl::IterativeClosestPoint7dof &icp) {
        *icp_ = icp;
        use_icp_ = true;
    }

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3,
		MAP_OPEN=4
    };

    enum trackingMode {
    	RELOCALIZATION = 0,
		TRACK_WITH_MOTION_MODEL = 1,
		TRACK_REFERENCE_KEYFRAME = 2,
		TRACK_LOCAL_MAP = 3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<cv::Mat> mlAbsoluteFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    // used when offline
    LocalMapping *mLocalMapper;

    trackingMode lastTrackingMode;

    bool trackingIsGood() const
    {return mLastProcessedState==ORB_SLAM2::MapTracking::OK and !mCurrentFrame.mTcw.empty(); }

    ros::Publisher global_pub;
    ros::Publisher local_pub;
    ros::Publisher orb_pose_pub;
    geometry_msgs::PoseStamped orb_pose_msg;

    pcl::IterativeClosestPoint7dof *icp_;

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    void MapOpenMonocularInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    void ScanWithNDT(cv::Mat currAbsolutePos);

    enum RelocalizationMode {
    	SEARCH_DB = 1,
		SEARCH_MAPPING = 2,
		SEARCH_LOCAL_MAP = 3
    };
    bool Relocalization ();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    //Drawers
    MapPublisher* mpMapPublisher;
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    int
    // Working resolution
    imageWorkWidth,
    imageWorkHeight,
    // ROI
    ROIx0,
    ROIy0,
    ROIwidth,
    ROIheight;

private:
    bool use_icp_ = false;
};
} //namespace ORB_SLAM

#endif // MAPTRACKING_H
