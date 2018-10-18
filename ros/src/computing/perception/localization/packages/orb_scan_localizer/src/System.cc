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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <exception>
#include <chrono>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile,
               const eSensor sensor, const ros::NodeHandle &node,
               const bool bUseMapPublisher, const bool is_publish,
               const bool bUseViewer, const bool is_visualize,
               const string &mpMapFileName, const operationMode mode):
        mSensor(sensor),
        mapFileName(mpMapFileName),
        mbReset(false),
        mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false),
        opMode(mode),
        isUseViewer(bUseViewer),
        isUseMapPublisher(bUseMapPublisher),
        monoNode(node)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
    {
      cout << "Monocular" << endl;
    } else {
      cout << "Only Monocular mode is provided\n";
      exit(-1);
    }

    //Check settings file
    fsSettings = cv::FileStorage(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    cout << endl << "Loading ORB Vocabulary..." << endl;
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();
    try {
        std::cout << "Map File Name: " << mapFileName << "\n";
        if (!mapFileName.empty()) {
            mpMap->loadPCDFile(mapFileName);
            double resolution_ = 1.0;
            voxel_grid_.setLeafSize(resolution_, resolution_, resolution_);
            voxel_grid_.setMinimumPoints(10);
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            voxel_grid_.setInput(mpMap->GetPriorMapPoints());
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << "VoxelGrid: " << ttrack << "\n";
            std::cout << "Number of map points: " << mpMap->GetPriorMapPoints()->size() << "\n";
            t1 = std::chrono::steady_clock::now();
            //mpMap->SetPriorMapPoints(voxel_grid_.setPointsRaw(mpMap->GetPriorMapPoints()));
            t2 = std::chrono::steady_clock::now();
            ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << "SetPointsRaw: " << ttrack << "\n";
            std::cout << "Number of map points: " << mpMap->GetPriorMapPoints()->size() << "\n";
        }
    } catch (exception &e) {
        std::cout << e.what() << "\n";
    }

    //Create Drawers. These are used by the MapPublisher
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpMapTracker = new MapTracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                   mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    if (!mapFileName.empty()) {
        mpMapTracker->SetSourceMap(mpMap->GetPriorMapPoints());
        mpMap->VoxelGridFilter(2.0);
    }

    std::cout << "Launched tracker\n";
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    std::cout << "Launched local map\n";

    mpLocalMapper->SetTracker(mpMapTracker);
    std::cout << "Set up local map\n";

    // Initialize the MapPublisher thread and launch
    // mpMapPublisher = new MapPublisher(this, mpFrameDrawer, mpMapDrawer, mpMapTracker,
    //                                   fsSettings, opMode);
    // if (bUseMapPublisher) {
    //     if(is_publish)
    //         mptMapPublisher = new thread(&MapPublisher::Run, mpMapPublisher);
    //     mpMapTracker->SetMapPublisher(mpMapPublisher);
    //
    //     //Set pointers between threads
    //     mpMapTracker->SetLocalMapper(mpLocalMapper);
    // }

    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpMapTracker,
                          fsSettings, opMode);
    if (bUseViewer) {
        if(is_visualize)
            mptViewer = new thread(&Viewer::Run, mpViewer);
        mpMapTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpMapTracker->SetLocalMapper(mpLocalMapper);

    // if (!bUseMapPublisher && !bUseViewer) {
    //     mpMapTracker->SetMapPublisher(mpMapPublisher);
    //     mpMapTracker->SetLocalMapper(mpLocalMapper);
    // }
    std::cout << "Set up tracker\n";
    std::cout << "Constructor finished\n";
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                               const bool is_publish)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpMapTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat camPosOrb = mpMapTracker->GrabImageMonocular(im, timestamp);

    {
        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpMapTracker->mState;
        mTrackedMapPoints = mpMapTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpMapTracker->mCurrentFrame.mvKeysUn;
    }
    return camPosOrb;
}


System::System(const string &strVocFile, const string &strSettingsFile,
               const eSensor sensor, const bool bUseViewer,
               const string &mpMapFileName, const operationMode mode):
				mSensor(sensor),
				mapFileName(mpMapFileName),
				mbReset(false),
				mbActivateLocalizationMode(false),
				mbDeactivateLocalizationMode(false),
				opMode (mode)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
    {
      cout << "Monocular" << endl;
    } else {
      cout << "Only Monocular mode is provided\n";
      exit(-1);
    }

    //Check settings file
    fsSettings = cv::FileStorage(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  	cout << endl << "Loading ORB Vocabulary..." << endl;
		if(!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Failed to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();
    // try {
    // 	cout << "Loading map..." << endl;
    // 	mpMap->loadFromDisk (mapFileName, mpKeyFrameDatabase);
    // } catch (exception &e) {
    // 	cout << "Unable to load map " << mapFileName << endl;
    // }

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // if (mpMap->mbMapUpdated)
    	// mpTracker->setMapLoaded();

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    std::cout << "Launched tracker\n";
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
    std::cout << "Launched local map\n";

    mpLocalMapper->SetTracker(mpTracker);
    std::cout << "Set up local map\n";

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                          fsSettings, opMode);
    if(bUseViewer)
        mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    std::cout << "Set up tracker\n";
    std::cout << "Constructor finished\n";
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat camPosOrb = mpTracker->GrabImageMonocular(im, timestamp);

    {
        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    }
    return camPosOrb;
}


void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}


void System::Shutdown()
{

	// Wait until all thread have effectively stopped
	if (opMode==System::MAPPING) {
		mpViewer->RequestFinish();
    // mpMapPublisher->RequestFinish();
		mpLocalMapper->RequestFinish();

		while(!mpLocalMapper->isFinished() || !mpViewer->isFinished()) // || !mpMapPublisher->isFinished())
		{
			usleep(5000);
		}

		// try {
		// 	mpMap->saveToDisk(mapFileName, mpKeyFrameDatabase);
		// } catch (...) {}

		while (!mpViewer->isFinished())
			usleep (5000);

		// while (!mpMapPublisher->isFinished())
		// 	usleep (5000);

    if (isUseViewer) {
      pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }
	}

}


void System::SaveTrajectoryTUM(const string &filename)
{
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::LoadMap(const string &filename)
{
	// mpMap->loadFromDisk (filename, mpKeyFrameDatabase);
	mpTracker->setMapLoaded();
}


void System::SaveMap(const string &filename)
{
	// mpMap->saveToDisk(filename, mpKeyFrameDatabase);
}

} //namespace ORB_SLAM
