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

#include "Map.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "MapPoint.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include <mutex>
#include <cstdio>
#include <exception>
#include <string>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>
#include "MapObjectSerialization.h"


using std::string;


template<class T>
vector<T> set2vector (const set<T> &st)
{
	vector<T> rt;
	for (auto &smember: st) {
		rt.push_back(smember);
	}
	return rt;
}


namespace ORB_SLAM2
{

Map::Map():
	mnMaxKFid(0),
	mbMapUpdated(false)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


KeyFrame* Map::offsetKeyframe (KeyFrame* kfSrc, int offset)
{
	try {
		int p = kfMapSortedId.at(kfSrc);
		p -= offset;
		return kfListSorted.at(p);
	} catch (...) {return NULL;}
}

void Map::loadPCDFile(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filename.c_str(), *pcd) == -1) {
        std::cerr << "Load 3D Prior Map Failed " << filename << "\n";
    }
    std::cerr << "Load 3D Prior Map " << filename << '\n';
    _pcd = pcd;
    return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Map::GetPriorMapPoints() {
    return _pcd;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Map::SetPriorMapPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd) {
    _pcd = pcd;
}

void Map::VoxelGridFilter(double filter_res) {
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
	voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
	voxel_grid_filter.setInputCloud(_pcd);
	voxel_grid_filter.filter(*_pcd);
}

// we expect direction vector has been normalized,
// as returned by Frame::getDirectionVector()
//KeyFrame* Map::getNearestKeyFrame (const float &x, const float &y, const float &z,
//	const float fdir_x, const float fdir_y, const float fdir_z,
//	vector<KeyFrame*> *kfSelectors)
KeyFrame*
Map::getNearestKeyFrame (const Eigen::Vector3f &position,
	const Eigen::Quaternionf &orientation,
	vector<KeyFrame*> *kfSelectors
)
{
	KeyFramePt queryPoint;
	queryPoint.x = position.x(), queryPoint.y = position.y(), queryPoint.z = position.z();

	const int k = 15;
	vector<int> idcs;
	vector<float> sqrDist;
	idcs.resize(k);
	sqrDist.resize(k);

	int r = kfOctree->nearestKSearch(queryPoint, k, idcs, sqrDist);
	if (r==0) {
		cerr << "*\n";
		return NULL;
	}

	Eigen::Matrix3f mOrient = orientation.toRotationMatrix();
	float
		fdir_x = mOrient(0,2),
		fdir_y = mOrient(1,2),
		fdir_z = mOrient(2,2);

	float cosd = 0;
	KeyFrame *ckf = NULL;
	int i = 0;
	for (auto ip: idcs) {

		float dirx, diry, dirz, cosT;
		KeyFrame *checkKF = kfCloud->at(ip).kf;

		// get direction vector
		cv::Mat orient = checkKF->GetRotation().t();
		dirx = orient.at<float>(0,2);
		diry = orient.at<float>(1,2);
		dirz = orient.at<float>(2,2);
		float norm = sqrtf(dirx*dirx + diry*diry + dirz*dirz);
		dirx /= norm;
		diry /= norm;
		dirz /= norm;

		cosT = (dirx*fdir_x + diry*fdir_y + dirz*fdir_z) / (sqrtf(fdir_x*fdir_x + fdir_y*fdir_y + fdir_z*fdir_z) * sqrtf(dirx*dirx + diry*diry + dirz*dirz));

		if (cosT < 0)
			continue;
		else
			return checkKF;
		//		if (cosT > cosd) {
		//			cosd = cosT;
		//			ckf = checkKF;
		//		}
		//		if (kfSelectors!=NULL)
		//			kfSelectors->at(i) = checkKF;
		//		i+=1;
	}

	return ckf;
}


void Map::extractVocabulary (ORBVocabulary *mapVoc)
{
	vector<vector<DBoW2::FORB::TDescriptor> > keymapFeatures;
	keymapFeatures.reserve (mspKeyFrames.size());

	vector<KeyFrame*> allKeyFrames = GetAllKeyFrames();
	fprintf (stderr, "KF: %d\n", allKeyFrames.size());

	for (vector<KeyFrame*>::const_iterator it=allKeyFrames.begin(); it!=allKeyFrames.end(); it++) {

		KeyFrame *kf = *it;
		vector<cv::Mat> kfDescriptor;

		// take map points that are belong to this keyframe
		set<MapPoint*> mapSet = kf->GetMapPoints();
		vector<MapPoint*> mapPointList = set2vector(mapSet);

		// for each map points, pick best descriptor and add to descriptors of this keyframe
		for (auto &mpp: mapPointList) {
			cv::Mat mpDescriptor = mpp->GetDescriptor();
			kfDescriptor.push_back(mpDescriptor);
		}

		keymapFeatures.push_back (kfDescriptor);
	}

	mapVoc->create (keymapFeatures);
}


} //namespace ORB_SLAM
