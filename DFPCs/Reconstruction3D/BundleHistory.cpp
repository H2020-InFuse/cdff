#include "BundleHistory.hpp"

using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

BundleHistory::BundleHistory(int size) :
	size(size),
	leftImageList(size),
	rightImageList(size)
	{
	mostRecentEntryIndex = NO_RECENT_ENTRY;
	oldestEntryIndex = 0;
	for (int index = 0; index < size; index++)
		{
		leftImageList.at(index) = NULL;
		rightImageList.at(index) = NULL;
		}
	}

BundleHistory::~BundleHistory()
	{
	for(int index= 0; index < size; index++)
		{
		DeleteAllDataAt(index);
		}
	}

void BundleHistory::AddImages(const Frame& leftImage, const Frame& rightImage)
	{
	FramePtr leftCopy = NewFrame();
	FramePtr rightCopy = NewFrame();
	Copy(leftImage, *leftCopy);
	Copy(rightImage, *rightCopy);
	AddImages(leftCopy, rightCopy);
	}

void BundleHistory::AddFeatures(const VisualPointFeatureVector2D& featureVector, std::string featureCategory)
	{
	VisualPointFeatureVector2DPtr copy = NewVisualPointFeatureVector2D();
	Copy(featureVector, *copy);
	AddFeatures(copy, featureCategory);
	}

void BundleHistory::AddFeatures3d(const VisualPointFeatureVector3D& featureVector, std::string featureCategory)
	{
	VisualPointFeatureVector3DPtr copy = NewVisualPointFeatureVector3D();
	Copy(featureVector, *copy);
	AddFeatures3d(copy, featureCategory);
	}

void BundleHistory::AddMatches(const CorrespondenceMap2D& leftRightCorrespondenceMap, std::string correspondenceCategory)
	{
	CorrespondenceMap2DPtr copy = NewCorrespondenceMap2D();
	Copy(leftRightCorrespondenceMap, *copy);
	AddMatches(copy, correspondenceCategory);
	}

void BundleHistory::AddPointCloud(const PointCloud& pointCloud, std::string cloudCategory)
	{
	PointCloudPtr copy = NewPointCloud();
	Copy(pointCloud, *copy);
	AddPointCloud(copy, cloudCategory);
	}

void BundleHistory::AddImages(FrameConstPtr leftImage, FrameConstPtr rightImage)
	{
	if ( mostRecentEntryIndex != NO_RECENT_ENTRY && (mostRecentEntryIndex+1) % size == oldestEntryIndex)
		{
		DeleteAllDataAt(oldestEntryIndex);
		leftImageList.at(oldestEntryIndex) = leftImage;
		rightImageList.at(oldestEntryIndex) = rightImage;
		mostRecentEntryIndex = oldestEntryIndex;
		oldestEntryIndex = (oldestEntryIndex + 1 ) % size;
		}
	else
		{
		mostRecentEntryIndex = (mostRecentEntryIndex + 1) % size;
		leftImageList.at(mostRecentEntryIndex) = leftImage;
		rightImageList.at(mostRecentEntryIndex) = rightImage;		
		}
	}

void BundleHistory::AddFeatures(VisualPointFeatureVector2DConstPtr featureVector, std::string featureCategory)
	{
	std::map<std::string, FeatureVectorList>::iterator list = featureVectorList.find(featureCategory);
	
	if (list == featureVectorList.end())
		{
		featureVectorList[featureCategory] = FeatureVectorList(size);
		for (int index = 0; index < size; index++)
			{
			featureVectorList[featureCategory].at(index) = NULL;
			}
		}
	else
		{
		delete( featureVectorList[featureCategory].at(mostRecentEntryIndex) );
		}
	featureVectorList[featureCategory].at(mostRecentEntryIndex) = featureVector;
	}

void BundleHistory::AddFeatures3d(VisualPointFeatureVector3DConstPtr featureVector, std::string featureCategory)
	{
	std::map<std::string, FeatureVector3dList>::iterator list = featureVector3dList.find(featureCategory);
	
	if (list == featureVector3dList.end())
		{
		featureVector3dList[featureCategory] = FeatureVector3dList(size);
		for (int index = 0; index < size; index++)
			{
			featureVector3dList[featureCategory].at(index) = NULL;
			}
		}
	else
		{
		delete( featureVector3dList[featureCategory].at(mostRecentEntryIndex) );
		}
	featureVector3dList[featureCategory].at(mostRecentEntryIndex) = featureVector;
	}

void BundleHistory::AddMatches(CorrespondenceMap2DConstPtr leftRightCorrespondenceMap, std::string correspondenceCategory)
	{
	std::map<std::string, CorrespondenceMapList>::iterator list = leftRightCorrespondenceMapList.find(correspondenceCategory);
	if (list == leftRightCorrespondenceMapList.end())
		{
		leftRightCorrespondenceMapList[correspondenceCategory] = CorrespondenceMapList(size);
		for (int index = 0; index < size; index++)
			{
			leftRightCorrespondenceMapList[correspondenceCategory].at(index) = NULL;
			}
		}
	else
		{
		delete( leftRightCorrespondenceMapList[correspondenceCategory].at(mostRecentEntryIndex) );
		}
	leftRightCorrespondenceMapList[correspondenceCategory].at(mostRecentEntryIndex) = leftRightCorrespondenceMap;
	}

void BundleHistory::AddPointCloud(PointCloudConstPtr pointCloud, std::string cloudCategory)
	{
	std::map<std::string, PointCloudList>::iterator list = pointCloudList.find(cloudCategory);
	if (list == pointCloudList.end())
		{
		pointCloudList[cloudCategory] = PointCloudList(size);
		for (int index = 0; index < size; index++)
			{
			pointCloudList[cloudCategory].at(index) = NULL;
			}
		}
	else
		{
		delete( pointCloudList[cloudCategory].at(mostRecentEntryIndex) );
		}
	pointCloudList[cloudCategory].at(mostRecentEntryIndex) = pointCloud;
	}
		
FrameWrapper::FrameConstPtr BundleHistory::GetLeftImage(int backwardSteps)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index < 0)
		{
		return NULL;
		}

	return leftImageList.at(index);
	}

FrameWrapper::FrameConstPtr BundleHistory::GetRightImage(int backwardSteps)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index < 0)
		{
		return NULL;
		}

	return rightImageList.at(index);
	}

VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr BundleHistory::GetFeatures(int backwardSteps, std::string featureCategory)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index < 0)
		{
		return NULL;
		}
	std::map<std::string, FeatureVectorList>::iterator list = featureVectorList.find(featureCategory);	
	if (list == featureVectorList.end())
		{
		return NULL;
		}

	return featureVectorList[featureCategory].at(index);
	}

VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr BundleHistory::GetFeatures3d(int backwardSteps, std::string featureCategory)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index < 0)
		{
		return NULL;
		}
	std::map<std::string, FeatureVector3dList>::iterator list = featureVector3dList.find(featureCategory);	
	if (list == featureVector3dList.end())
		{
		return NULL;
		}

	return featureVector3dList[featureCategory].at(index);
	}

CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr BundleHistory::GetMatches(int backwardSteps, std::string correspondenceCategory)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index < 0)
		{
		return NULL;
		}
	std::map<std::string, CorrespondenceMapList>::iterator list = leftRightCorrespondenceMapList.find(correspondenceCategory);
	if (list == leftRightCorrespondenceMapList.end())
		{
		return NULL;
		}

	return leftRightCorrespondenceMapList[correspondenceCategory].at(index);
	}

PointCloudWrapper::PointCloudConstPtr BundleHistory::GetPointCloud(int backwardSteps, std::string cloudCategory)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index < 0)
		{
		return NULL;
		}
	std::map<std::string, PointCloudList>::iterator list = pointCloudList.find(cloudCategory);
	if (list == pointCloudList.end())
		{
		return NULL;
		}

	return pointCloudList[cloudCategory].at(index);
	}

void BundleHistory::RemoveEntry(int backwardSteps)
	{
	int index = BackwardStepsToIndex(backwardSteps);
	if (index == NO_RECENT_ENTRY)
		{
		return;
		}
	if (index == oldestEntryIndex)
		{
		RemoveOldestEntry();
		}

	DeleteAllDataAt(index);
	while (index != mostRecentEntryIndex)
		{
		int nextIndex = (index+1) % size;

		leftImageList.at(index) = leftImageList.at(nextIndex);
		rightImageList.at(index) = rightImageList.at(nextIndex);
		ReplaceIndexByIndexOnMap(featureVectorList, index, nextIndex);
		ReplaceIndexByIndexOnMap(leftRightCorrespondenceMapList, index, nextIndex);
		ReplaceIndexByIndexOnMap(pointCloudList, index, nextIndex);
		index = nextIndex;		
		}
	
	leftImageList.at(mostRecentEntryIndex) = NULL;
	rightImageList.at(mostRecentEntryIndex) = NULL;
	ReplaceIndexByNullOnMap(featureVectorList, mostRecentEntryIndex);
	ReplaceIndexByNullOnMap(leftRightCorrespondenceMapList, mostRecentEntryIndex);
	ReplaceIndexByNullOnMap(pointCloudList, mostRecentEntryIndex);

	if (mostRecentEntryIndex == oldestEntryIndex)
		{
		mostRecentEntryIndex = NO_RECENT_ENTRY;
		}
	else
		{
		mostRecentEntryIndex = (mostRecentEntryIndex + size -1) % size;
		}
	}

void BundleHistory::RemoveOldestEntry()
	{
	DeleteAllDataAt(oldestEntryIndex);
	if (mostRecentEntryIndex == NO_RECENT_ENTRY)
		{
		return;
		}
	if (oldestEntryIndex == mostRecentEntryIndex)
		{
		mostRecentEntryIndex = NO_RECENT_ENTRY;
		return;
		}
	oldestEntryIndex = (oldestEntryIndex + 1) % size;
	}

int BundleHistory::BackwardStepsToIndex(int backwardSteps)
	{
	if (backwardSteps >= size)
		{
		return NO_RECENT_ENTRY;
		}

	int candidateIndex = (mostRecentEntryIndex + size - backwardSteps) % size;

	if (oldestEntryIndex <= mostRecentEntryIndex)
		{
		if (oldestEntryIndex <= candidateIndex && candidateIndex <= mostRecentEntryIndex)
			{
			return candidateIndex;
			}
		}
	else
		{
		if (oldestEntryIndex <= candidateIndex || candidateIndex <= mostRecentEntryIndex)
			{
			return candidateIndex;
			}
		}

	return NO_RECENT_ENTRY;
	}

}
}
}
/** @} */

