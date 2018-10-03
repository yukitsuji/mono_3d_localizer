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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <string>
#include <opencv2/opencv.hpp>
#include <exception>
#include <vector>
#include <map>
#include <mutex>

// For Fast keyframe search
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_search.hpp>
#include <pcl/io/pcd_io.h>


#include <ORBVocabulary.h>

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;


/*
 * Warning: please do NOT modify this point structure
 * We have observed significant performance reduction.
 */
struct KeyFramePt {
	PCL_ADD_POINT4D;
	ORB_SLAM2::KeyFrame *kf;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16 ;


class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void extractVocabulary (ORBVocabulary *cvoc);

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;


		// Map Storage Handlers
    class BadMapFile : public std::exception {};
    class MapFileException : public std::exception {};

    // Load pcd file
    void loadPCDFile(const std::string &filename);
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetPriorMapPoints();
    pcl::PointCloud<pcl::PointXYZ>::Ptr SetPriorMapPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd); 

		struct MapFileHeader {
			char signature[7];
			long unsigned int
				numOfKeyFrame,
				numOfMapPoint,
				numOfReferencePoint;
		};

		KeyFrame* getNearestKeyFrame (
			const Eigen::Vector3f &position,
			const Eigen::Quaternionf &orientation,
			vector<KeyFrame*> *kfSelectors);

		KeyFrame* offsetKeyframe (KeyFrame* kfSrc, int offset);

		// These are used for augmented localization
		std::vector<KeyFrame*> kfListSorted;
		std::map<KeyFrame*, int> kfMapSortedId;

		pcl::PointCloud<pcl::PointXYZ>::Ptr _pcd;

		bool mbMapUpdated;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;

    pcl::PointCloud<KeyFramePt>::Ptr kfCloud;
    pcl::octree::OctreePointCloudSearch<KeyFramePt>::Ptr kfOctree;

    KeyFrameDatabase *mKeyFrameDb;

};

} //namespace ORB_SLAM

#endif // MAP_H
