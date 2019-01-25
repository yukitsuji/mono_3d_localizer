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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "MapTracking.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include <icp_7dof/icp_7dof.h>
#include <mutex>
#include <pcl/console/print.h>
#include <pcl/filters/passthrough.h>

namespace ORB_SLAM2
{

class Tracking;
class MapTracking;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const string &strSettingPath, const float bMonocular);

    void SetTracker(Tracking* pTracker);
    void SetTracker(MapTracking* pTracker);

    // Main function
    void Run();

    // This is used for single thread mode (aka. bag mapping)
    void RunOnce ();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset(const bool _offline=false);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    std::mutex localMappingRunMutex;

    void SetICP(pcl::IterativeClosestPoint7dof &icp) {
        icp_ = &icp;
        use_icp_ = true;
    }

    pcl::IterativeClosestPoint7dof *icp_;

    double dist_coeff_;
    int use_icp2_;

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();
    void KeyFrameCullingByNumber(int max_count);

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    Tracking* mpTracker;
    MapTracking* mpMapTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

private:
    bool use_icp_ = false;
};
} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
