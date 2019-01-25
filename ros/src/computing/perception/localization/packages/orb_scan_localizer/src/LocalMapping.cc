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

#include "LocalMapping.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include <mutex>
#include <chrono>


namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const string &strSettingPath, const float bMonocular):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  dist_coeff_ = fSettings["ORBextractor.dist_coeff"];
  use_icp2_ = fSettings["ORBextractor.use_icp2"];
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker = pTracker;
}

void LocalMapping::SetTracker(MapTracking *pTracker)
{
    mpMapTracker = pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap() > 2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap,
                                                     icp_, dist_coeff_, use_icp_, use_icp2_);

                // Check redundant local Keyframes
                KeyFrameCulling();

                // KeyFrameCullingByNumber(60);

                if (false)
                {
                  // if (mpMap->KeyFramesInMap() > 2)
                  // {
                  //   if (use_icp_ && use_icp2_)
                  //   {
                  //       std::cout << "ICP Matching\n";
                  //       // Collect local map point and KeyFrames
                  //       // Use GetBestCovisibilityKeyFrames or 10 keyframes
                  //       // TODO: Calculate number of map points
                  //       vector<KeyFrame*> mvpLocalKeyFrames; // = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
                  //       std::vector<MapPoint*> localMapPoints;
                  //       localMapPoints.clear();
                  //
                  //       vector<MapPoint*> mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
                  //
                  //       map<KeyFrame*,int> keyframeCounter;
                  //       for(int i=0; i<mpCurrentKeyFrame->N; i++)
                  //       {
                  //           if(mvpMapPoints[i])
                  //           {
                  //               MapPoint* pMP = mvpMapPoints[i];
                  //               if(!pMP->isBad())
                  //               {
                  //                   const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                  //                   for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                  //                       keyframeCounter[it->first]++;
                  //               }
                  //               else
                  //               {
                  //                   mvpMapPoints[i] = NULL;
                  //               }
                  //           }
                  //       }
                  //
                  //       if(keyframeCounter.empty())
                  //           return;
                  //
                  //       mvpLocalKeyFrames.clear();
                  //       mvpLocalKeyFrames.reserve(keyframeCounter.size());
                  //
                  //       // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
                  //       for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
                  //       {
                  //           KeyFrame* pKF = it->first;
                  //           if(pKF==NULL || pKF->isBad())
                  //               continue;
                  //           mvpLocalKeyFrames.push_back(it->first);
                  //       }
                  //
                  //
                  //       // 各Map PointにLocalMappingのKeyFrameのIDを格納する変数を作成
                  //       for (const auto& pKF : mvpLocalKeyFrames)
                  //       {
                  //           // std::cout << "KeyFrame id: " << pKF->mnId << "\n";
                  //           const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
                  //
                  //           for (const auto& pMP : vpMPs)
                  //           {
                  //               if(!pMP)
                  //                   continue;
                  //               if(pMP->mnLocalMappingForFrame == mpCurrentKeyFrame->mnId)
                  //                   continue;
                  //               if(!pMP->isBad())
                  //               {
                  //                   localMapPoints.push_back(pMP);
                  //                   pMP->mnLocalMappingForFrame = mpCurrentKeyFrame->mnId;
                  //               }
                  //           }
                  //       }
                  //
                  //       Eigen::Matrix4d toRef = Converter::toMatrix4d(mpCurrentKeyFrame->GetPose());
                  //
                  //       // icp matching using map point
                  //       pcl::PointCloud<pcl::PointXYZ>::Ptr l_points (new pcl::PointCloud<pcl::PointXYZ>);
                  //       for (auto&& p : localMapPoints)
                  //       {
                  //           cv::Mat pos = p->GetWorldPos();
                  //           pcl::PointXYZ point;
                  //           point.x = pos.at<float>(0);
                  //           point.y = pos.at<float>(1);
                  //           point.z = pos.at<float>(2);
                  //           l_points->push_back(point);
                  //       }
                  //
                  //       pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
                  //       *filtered_points = *l_points;
                  //
                  //       icp_->transformCloudPublic(*filtered_points, *filtered_points, toRef);
                  //
                  //       pcl::PassThrough<pcl::PointXYZ> pass;
                  //       // pass.setInputCloud (filtered_points);
                  //       // pass.setFilterFieldName ("x");
                  //       // pass.setFilterLimits (-10.0, 10.0);
                  //       // pass.filter (*filtered_points);
                  //       pass.setInputCloud (filtered_points);
                  //       pass.setFilterFieldName ("z");
                  //       pass.setFilterLimits (-500, 30.0);
                  //       pass.filter (*filtered_points);
                  //       pass.setInputCloud (filtered_points);
                  //       pass.setFilterFieldName ("y");
                  //       pass.setFilterLimits (-3, 2);
                  //       pass.filter (*filtered_points);
                  //
                  //       // double filter_res = 0.2f;
                  //       // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
                  //       // voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
                  //       // voxel_grid_filter.setInputCloud(filtered_points);
                  //       // voxel_grid_filter.filter(*filtered_points);
                  //
                  //       icp_->transformCloudPublic(*filtered_points, *filtered_points, toRef.inverse());
                  //
                  //       // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
                  //       icp_->setInputSource(filtered_points);
                  //       icp_->setMaximumIterations(1);
                  //       icp_->setDistThreshold(dist_coeff_);
                  //       pcl::PointCloud<pcl::PointXYZ> output;
                  //       Eigen::Matrix4d transformation;
                  //
                  //       icp_->align(output, Eigen::Matrix4d::Identity(), toRef);
                  //
                  //       transformation = icp_->getFinalTransformation();
                  //
                  //       // Update map point and keyframes
                  //       cv::Mat cv_transformation = Converter::toCvMat(transformation);
                  //       cv::Mat cv_toRef = Converter::toCvMat(toRef);
                  //       cv::Mat cv_invToRef = cv_toRef.inv();
                  //
                  //       // Get Map Mutex
                  //       {
                  //         unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
                  //
                  //         double scale = pow(cv::determinant(cv_transformation), 1.0/3.0);
                  //
                  //         for (auto&& pKF : mvpLocalKeyFrames)
                  //         {
                  //             if (pKF->mnId == mpCurrentKeyFrame->mnId)
                  //             {
                  //                 cv::Mat local_transform;
                  //                 cv_transformation.copyTo(local_transform);
                  //                 cv::Mat local_R = local_transform.rowRange(0,3).colRange(0,3);
                  //                 cv::Mat local_T = local_transform.rowRange(0,3).col(3);
                  //
                  //                 local_R /= scale;
                  //
                  //                 cv::Mat prev_R, prev_T, prev_pose, after_R, after_T, after_pose;
                  //
                  //                 // OK
                  //                 prev_pose = pKF->GetPoseInverse();
                  //                 prev_pose.rowRange(0, 3).colRange(0, 3).copyTo(prev_R);
                  //                 after_R = prev_R * local_R;
                  //
                  //                 // Not OK
                  //                 prev_pose.rowRange(0,3).col(3).copyTo(prev_T);
                  //                 after_T = prev_T + prev_R * local_T;
                  //                 // after_T = prev_T + local_T;
                  //
                  //                 prev_pose.copyTo(after_pose);
                  //                 after_R.copyTo(after_pose.rowRange(0, 3).colRange(0, 3));
                  //                 after_T.copyTo(after_pose.rowRange(0, 3).col(3));
                  //                 pKF->SetPose(after_pose.inv());
                  //             }
                  //         }
                  //
                  //         cv_invToRef = mpCurrentKeyFrame->GetPoseInverse();
                  //
                  //         // Update KeyFrames
                  //         for (auto&& pKF : mvpLocalKeyFrames)
                  //         {
                  //             if (pKF->mnId != mpCurrentKeyFrame->mnId) {
                  //                 cv::Mat prev_pose = pKF->GetPoseInverse(); // GetPose();
                  //                 cv::Mat relative_pose = cv_toRef * prev_pose;
                  //                 relative_pose.rowRange(0,3).col(3) *= scale; // multiply scale to translation
                  //                 cv::Mat after_pose = cv_invToRef * relative_pose;
                  //                 pKF->SetPose(after_pose.inv());
                  //             }
                  //         }
                  //
                  //         // Update points using PCL
                  //         pcl::PointCloud<pcl::PointXYZ>::Ptr converted_points (new pcl::PointCloud<pcl::PointXYZ>);
                  //         *converted_points = *l_points;
                  //         Eigen::Matrix4d updateToGlobal = Converter::toMatrix4d(mpCurrentKeyFrame->GetPoseInverse());
                  //         icp_->transformCloudPublic(*converted_points, *converted_points, toRef);
                  //         icp_->transformCloudPublic(*converted_points, *converted_points, transformation);
                  //         icp_->transformCloudPublic(*converted_points, *converted_points, toRef.inverse()); //updateToGlobal);
                  //
                  //         vector<MapPoint*> hoge;
                  //         hoge.clear();
                  //
                  //         // Update points
                  //         for(size_t i=0, iend=localMapPoints.size(); i<iend; ++i)
                  //         {
                  //             MapPoint* pMP = localMapPoints[i];
                  //             if(pMP)
                  //             {
                  //                 if(!pMP->isBad())
                  //                 {
                  //                     pcl::PointXYZ p = converted_points->points[i];
                  //                     cv::Mat pose = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
                  //                     pMP->SetWorldPos(pose);
                  //                     pMP->UpdateNormalAndDepth();
                  //                     hoge.push_back(pMP);
                  //                 }
                  //             }
                  //         }
                  //
                  //         // mpMap->SetLocalMappingPoint(hoge);
                  //       }
                  //   }
                  // }
                }
            }
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish()) {
                break;
            }
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}


void LocalMapping::RunOnce()
{
    std::cout << "RunOnce\n";
    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double ttrack;
    SetAcceptKeyFrames(false);
    std::cout << "mlNewKeyFrames.size: " << mlNewKeyFrames.size() << "\n";
    // Check if there are keyframes in the queue
    if(CheckNewKeyFrames())
    {
        t1 = std::chrono::steady_clock::now();
        // BoW conversion and insertion in Map
        ProcessNewKeyFrame();
        t2 = std::chrono::steady_clock::now();
        ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        std::cout << "ProcessNewKeyFrame " << ttrack << "\n";

        t1 = std::chrono::steady_clock::now();
        // Check recent MapPoints
        MapPointCulling();
        t2 = std::chrono::steady_clock::now();
        ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        std::cout << "MapPointCulling " << ttrack << "\n";


        t1 = std::chrono::steady_clock::now();
        // Triangulate new MapPoints
        CreateNewMapPoints();
        t2 = std::chrono::steady_clock::now();
        ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        std::cout << "CreateNewMapPoints " << ttrack << "\n";

        if(!CheckNewKeyFrames())
        {
            t1 = std::chrono::steady_clock::now();
            // Find more matches in neighbor keyframes and fuse point duplications
            SearchInNeighbors();
            t2 = std::chrono::steady_clock::now();
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << "SearchInNeighbors " << ttrack << "\n";
        }

        mbAbortBA = false;

        std::cout << "mlNewKeyFrames.size: " << mlNewKeyFrames.size() << "\n";

        if(!CheckNewKeyFrames())
    	  {
            t1 = std::chrono::steady_clock::now();
      	    // Local BA
            if(mpMap->KeyFramesInMap() > 2)
                Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap,
                                                 icp_, dist_coeff_, use_icp_, use_icp2_);
            t2 = std::chrono::steady_clock::now();
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << "LocalBundleAdjustment " << ttrack << "\n";


            t1 = std::chrono::steady_clock::now();
            // Check redundant local Keyframes
            KeyFrameCulling();
            t2 = std::chrono::steady_clock::now();
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << "KeyFrameCulling " << ttrack << "\n";

            if (false){
              // t1 = std::chrono::steady_clock::now();
              // KeyFrameCullingByNumber(60);
              // t2 = std::chrono::steady_clock::now();
              // ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
              // std::cout << "KeyFrameCullingByNumber " << ttrack << "\n";

              // if (mpMap->KeyFramesInMap() > 2)
              // {
              //
              // if (use_icp_ && use_icp2_)
              // {
              //     // Collect local map point and KeyFrames
              //     // Use GetBestCovisibilityKeyFrames or 10 keyframes
              //     // TODO: Calculate number of map points
              //     vector<KeyFrame*> mvpLocalKeyFrames; // = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
              //     std::vector<MapPoint*> localMapPoints;
              //     localMapPoints.clear();
              //
              //     vector<MapPoint*> mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
              //
              //     map<KeyFrame*,int> keyframeCounter;
              //     for(int i=0; i<mpCurrentKeyFrame->N; i++)
              //     {
              //         if(mvpMapPoints[i])
              //         {
              //             MapPoint* pMP = mvpMapPoints[i];
              //             if(!pMP->isBad())
              //             {
              //                 const map<KeyFrame*,size_t> observations = pMP->GetObservations();
              //                 for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
              //                     keyframeCounter[it->first]++;
              //             }
              //             else
              //             {
              //                 mvpMapPoints[i] = NULL;
              //             }
              //         }
              //     }
              //
              //     if(keyframeCounter.empty())
              //         return;
              //
              //     mvpLocalKeyFrames.clear();
              //     mvpLocalKeyFrames.reserve(keyframeCounter.size());
              //
              //     // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
              //     for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
              //     {
              //         KeyFrame* pKF = it->first;
              //         if(pKF==NULL || pKF->isBad())
              //             continue;
              //         mvpLocalKeyFrames.push_back(it->first);
              //     }
              //
              //     std::cout << "############## KeyFrame ICP: " << mvpLocalKeyFrames.size() << " ############\n";
              //     std::cout << "############## KeyFrame ICP: " << mpCurrentKeyFrame->GetVectorCovisibleKeyFrames().size() << " ############\n";
              //
              //     // 各Map PointにLocalMappingのKeyFrameのIDを格納する変数を作成
              //     for (const auto& pKF : mvpLocalKeyFrames)
              //     {
              //         // std::cout << "KeyFrame id: " << pKF->mnId << "\n";
              //         const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
              //
              //         for (const auto& pMP : vpMPs)
              //         {
              //             if(!pMP)
              //                 continue;
              //             if(pMP->mnLocalMappingForFrame == mpCurrentKeyFrame->mnId)
              //                 continue;
              //             if(!pMP->isBad())
              //             {
              //                 localMapPoints.push_back(pMP);
              //                 pMP->mnLocalMappingForFrame = mpCurrentKeyFrame->mnId;
              //             }
              //         }
              //     }
              //
              //     t1 = std::chrono::steady_clock::now();
              //
              //     Eigen::Matrix4d toRef = Converter::toMatrix4d(mpCurrentKeyFrame->GetPose());
              //
              //     // icp matching using map point
              //     pcl::PointCloud<pcl::PointXYZ>::Ptr l_points (new pcl::PointCloud<pcl::PointXYZ>);
              //     for (auto&& p : localMapPoints)
              //     {
              //         cv::Mat pos = p->GetWorldPos();
              //         pcl::PointXYZ point;
              //         point.x = pos.at<float>(0);
              //         point.y = pos.at<float>(1);
              //         point.z = pos.at<float>(2);
              //         l_points->push_back(point);
              //     }
              //
              //     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
              //     *filtered_points = *l_points;
              //
              //     icp_->transformCloudPublic(*filtered_points, *filtered_points, toRef);
              //
              //     pcl::PassThrough<pcl::PointXYZ> pass;
              //     // pass.setInputCloud (filtered_points);
              //     // pass.setFilterFieldName ("x");
              //     // pass.setFilterLimits (-40.0, 40.0);
              //     // pass.filter (*filtered_points);
              //     // pass.setInputCloud (filtered_points);
              //     // pass.setFilterFieldName ("z");
              //     // pass.setFilterLimits (-40.0, 40.0);
              //     // pass.filter (*filtered_points);
              //     pass.setInputCloud (filtered_points);
              //     pass.setFilterFieldName ("y");
              //     pass.setFilterLimits (-2, 5);
              //     pass.filter (*filtered_points);
              //
              //     // double filter_res = 0.2f;
              //     // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
              //     // voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
              //     // voxel_grid_filter.setInputCloud(filtered_points);
              //     // voxel_grid_filter.filter(*filtered_points);
              //
              //     icp_->transformCloudPublic(*filtered_points, *filtered_points, toRef.inverse());
              //
              //     // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
              //     icp_->setInputSource(filtered_points);
              //     icp_->setMaximumIterations(1);
              //     icp_->setDistThreshold(dist_coeff_);
              //     pcl::PointCloud<pcl::PointXYZ> output;
              //     Eigen::Matrix4d transformation;
              //
              //     t2 = std::chrono::steady_clock::now();
              //     ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
              //     std::cout << "Prepare icp " << ttrack << "\n";
              //
              //     t1 = std::chrono::steady_clock::now();
              //
              //     icp_->align(output, Eigen::Matrix4d::Identity(), toRef);
              //
              //     t2 = std::chrono::steady_clock::now();
              //     ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
              //     std::cout << "ICP " << ttrack << "\n";
              //
              //     transformation = icp_->getFinalTransformation();
              //
              //     // Update map point and keyframes
              //     cv::Mat cv_transformation = Converter::toCvMat(transformation);
              //     cv::Mat cv_toRef = Converter::toCvMat(toRef);
              //     cv::Mat cv_invToRef = cv_toRef.inv();
              //
              //     // Get Map Mutex
              //     {
              //     unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
              //
              //     t1 = std::chrono::steady_clock::now();
              //
              //     double scale = pow(cv::determinant(cv_transformation), 1.0/3.0);
              //
              //     for (auto&& pKF : mvpLocalKeyFrames)
              //     {
              //         if (pKF->mnId == mpCurrentKeyFrame->mnId)
              //         {
              //             cv::Mat local_transform;
              //             cv_transformation.copyTo(local_transform);
              //             cv::Mat local_R = local_transform.rowRange(0,3).colRange(0,3);
              //             cv::Mat local_T = local_transform.rowRange(0,3).col(3);
              //
              //             local_R /= scale;
              //
              //             cv::Mat prev_R, prev_T, prev_pose, after_R, after_T, after_pose;
              //
              //             prev_pose = pKF->GetPoseInverse();
              //             prev_pose.rowRange(0, 3).colRange(0, 3).copyTo(prev_R);
              //             after_R = prev_R * local_R;
              //
              //             prev_pose.rowRange(0,3).col(3).copyTo(prev_T);
              //             after_T = prev_T + prev_R * local_T;
              //
              //
              //             prev_pose.copyTo(after_pose);
              //             after_R.copyTo(after_pose.rowRange(0, 3).colRange(0, 3));
              //             after_T.copyTo(after_pose.rowRange(0, 3).col(3));
              //             pKF->SetPose(after_pose.inv());
              //             pKF->local_scale = scale;
              //             std::cout << "local_transform:\n " << local_transform << "\n";
              //             std::cout << "prev_pose:\n " << prev_pose << "\n";
              //             std::cout << "after_pose:\n " << after_pose << "\n";
              //             break;
              //         }
              //     }
              //
              //     cv_invToRef = mpCurrentKeyFrame->GetPoseInverse();
              //
              //     // Update KeyFrames
              //     for (auto&& pKF : mvpLocalKeyFrames)
              //     {
              //         if (pKF->mnId != mpCurrentKeyFrame->mnId) {
              //             cv::Mat prev_pose = pKF->GetPoseInverse(); // GetPose();
              //             cv::Mat relative_pose = cv_toRef * prev_pose;
              //             std::cout << "Relative pose: " << pKF->mnId << "\n" << relative_pose << "\n";
              //             relative_pose.rowRange(0,3).col(3) *= scale; // multiply scale to translation
              //             cv::Mat after_pose = cv_invToRef * relative_pose;
              //             pKF->SetPose(after_pose.inv());
              //             pKF->local_scale = scale;
              //         }
              //     }
              //
              //     t2 = std::chrono::steady_clock::now();
              //     ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
              //     std::cout << "Update keyframe " << ttrack << "\n";
              //
              //     t1 = std::chrono::steady_clock::now();
              //
              //     // Update points using PCL
              //     pcl::PointCloud<pcl::PointXYZ>::Ptr converted_points (new pcl::PointCloud<pcl::PointXYZ>);
              //     *converted_points = *l_points;
              //     Eigen::Matrix4d updateToGlobal = Converter::toMatrix4d(mpCurrentKeyFrame->GetPoseInverse());
              //     // icp_->transformCloudPublic(*converted_points, *converted_points, toRef);
              //     // icp_->transformCloudPublic(*converted_points, *converted_points, transformation);
              //     // icp_->transformCloudPublic(*converted_points, *converted_points, toRef.inverse()); //updateToGlobal);
              //     icp_->transformCloudPublic(*converted_points, *converted_points, toRef.inverse() * transformation * toRef);
              //
              //     vector<MapPoint*> hoge;
              //     hoge.clear();
              //
              //     // Update points
              //     for(size_t i=0, iend=localMapPoints.size(); i<iend; ++i)
              //     {
              //         MapPoint* pMP = localMapPoints[i];
              //         if(pMP)
              //         {
              //             if(!pMP->isBad())
              //             {
              //                 pcl::PointXYZ p = converted_points->points[i];
              //                 cv::Mat pose = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
              //                 pMP->SetWorldPos(pose);
              //                 pMP->UpdateNormalAndDepth();
              //                 hoge.push_back(pMP);
              //             }
              //         }
              //     }
              //
              //     mpMap->SetLocalMappingPoint(hoge);
              //
              //     t2 = std::chrono::steady_clock::now();
              //     ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
              //     std::cout << "Update Mappoint " << ttrack << "\n";
              //     }
              // }
              // }
            }
        }
    }
    if (!mlNewKeyFrames.size())
        SetAcceptKeyFrames(true);
}


void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs = 2;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio() < 0.25f)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth<0.01)
            continue;

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }
    sort(vpTargetKFs.begin(), vpTargetKFs.end());
    vpTargetKFs.erase(unique(vpTargetKFs.begin(), vpTargetKFs.end()), vpTargetKFs.end());

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi, vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);

    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; ++i)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames = flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs){
            pKF->SetBadFlag();
        }
    }
}

void LocalMapping::KeyFrameCullingByNumber(int max_count)
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    if (vpLocalKeyFrames.size() <= max_count) return;

    for (vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin()+(max_count-1), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if (pKF->mnId == 0) continue;
        pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}


void LocalMapping::RequestReset(const bool _offline)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    if (_offline==true)
    	return ResetIfRequested();

    else while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}


void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
