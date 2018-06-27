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

#include "MapPublisher.h"
#include <pangolin/pangolin.h>

#include <mutex>


namespace ORB_SLAM2
{

MapPublisher::MapPublisher(
	System* pSystem,
	FrameDrawer *pFrameDrawer,
	MapDrawer *pMapDrawer,
	MapTracking *pTracking,
	const cv::FileStorage &fSettings,
	int opMode):
		mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpMapTracker(pTracking),
		mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false),
		modeLocalization (opMode==System::LOCALIZATION ? true : false)
{
    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    int argc = 0;
    ros::init (argc, NULL, "orb_map");
    ros::NodeHandle node;
    map_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ>>("orb_points", 1, true);
}

void MapPublisher::Run()
{
    mbFinished = false;

    // pangolin::OpenGlMatrix Twc;
    // Twc.SetIdentity();
    while(1)
    {
        mpMapDrawer->PublishMapPoints(map_pub);

        // cv::Mat im = mpFrameDrawer->getLastFrame();
        // if (im.empty() == false) {
        	// cv::imshow(mapViewTitle, im);
        // }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void MapPublisher::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool MapPublisher::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void MapPublisher::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool MapPublisher::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void MapPublisher::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool MapPublisher::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool MapPublisher::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void MapPublisher::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
