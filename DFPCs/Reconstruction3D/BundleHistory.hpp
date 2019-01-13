#ifndef BUNDLEHISTORY_HPP
#define BUNDLEHISTORY_HPP

#include <Types/CPP/Frame.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>

#include <vector>
#include <map>

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{


/*This class keeps memory of the most recent computation history for a sequence of stereo image pairs. It allows storage of the latest N entries, composed by a pair of stereo images,
a set of 2D features, a set of 3D features, a set of 2D Matches, and a set of point clouds. Only the stereo images are a mondatory component of each entry, all other components are optional.
When new data is added beyond the N entries, the oldest entry is removed and forgotten. 
*/
class BundleHistory
	{
	public:
		typedef std::vector<FrameWrapper::FrameConstPtr> ImageList;
		typedef std::vector<VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr> FeatureVectorList;
		typedef std::vector<VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr> FeatureVector3dList;
		typedef std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> CorrespondenceMapList;
		typedef std::vector<PointCloudWrapper::PointCloudConstPtr> PointCloudList;

		BundleHistory() = delete;
		explicit BundleHistory(int size);
		~BundleHistory();

		/* @brief This method adds an new entry to the History. If the history contains already N entries, the oldest entry is removed to make space for the new one 
		*
		*  @param leftImage, the Frame of the left image associated to the entry;
		*  @param rightImage, the Frame of the right image associated to the entry;
 		*/
		void AddImages(const FrameWrapper::Frame& leftImage, const FrameWrapper::Frame& rightImage);

		/* @brief This method adds a set of 2D features associated to the current entry (i.e. associated to the latest image pair). 
		*
		*  @param featureVector, the input 2D features;
		*  @param featureCategory, the string identifier of the vector in the entry; This allow storage of multiple vectors under different identifiers.
 		*/		
		void AddFeatures(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& featureVector, std::string featureCategory = "DEFAULT");

		/* @brief This method adds a set of 3D features associated to the current entry (i.e. associated to the latest image pair). 
		*
		*  @param featureVector, the input 3D features;
		*  @param featureCategory, the string identifier of the vector in the entry; This allow storage of multiple vectors under different identifiers.
 		*/	
		void AddFeatures3d(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& featureVector, std::string featureCategory = "DEFAULT");

		/* @brief This method adds a set of 2D matches associated to the current entry (i.e. associated to the latest image pair). 
		*
		*  @param correspondenceMap, the input 2D matches;
		*  @param correspondenceCategory, the string identifier of the vector in the entry; This allow storage of multiple vectors under different identifiers.
 		*/	
		void AddMatches(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& correspondenceMap, std::string correspondenceCategory = "DEFAULT");

		/* @brief This method adds a point cloud associated to the current entry (i.e. associated to the latest image pair). 
		*
		*  @param cloud, the input point cloud;
		*  @param cloudCategory, the string identifier of the vector in the entry; This allow storage of multiple vectors under different identifiers.
 		*/	
		void AddPointCloud(const PointCloudWrapper::PointCloud& cloud, std::string cloudCategory = "DEFAULT");
		

		/* @brief This method retrieves the left image associate to an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one.
 		*/
		FrameWrapper::FrameConstPtr GetLeftImage(int backwardSteps);

		/* @brief This method retrieves the right image associate to an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one.
 		*/
		FrameWrapper::FrameConstPtr GetRightImage(int backwardSteps);

		/* @brief This method retrieves the 2D features associated to an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one;
		*  @param featureCategory, the identifier of the set of 2D features.
 		*/
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr GetFeatures(int backwardSteps, std::string featureCategory = "DEFAULT");

		/* @brief This method retrieves the 3D features associated to an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one;
		*  @param featureCategory, the identifier of the set of 3D features.
 		*/
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr GetFeatures3d(int backwardSteps, std::string featureCategory = "DEFAULT");

		/* @brief This method retrieves the 2D matches associated to an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one;
		*  @param correspondenceCategory, the identifier of the set of 2D matches.
 		*/
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr GetMatches(int backwardSteps, std::string correspondenceCategory = "DEFAULT");

		/* @brief This method retrieves the point cloud associated to an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one;
		*  @param cloudCategory, the identifier of the point cloud.
 		*/
		PointCloudWrapper::PointCloudConstPtr GetPointCloud(int backwardSteps, std::string cloudCategory = "DEFAULT");


		/* @brief This method removes an entry.
		*
		*  @param backwardSteps, the step time distance between the requested entry and the most recent one;
 		*/
		void RemoveEntry(int backwardSteps);

		/* @brief This method removes the oldest entry.
 		*/
		void RemoveOldestEntry();

	protected:

	private:
		// Internal variable holding the state of the circular queue.
		static const int NO_RECENT_ENTRY = -1;
		int size;
		int mostRecentEntryIndex;
		int oldestEntryIndex;		

		//Data Structure for the implementation of the circular queue.
		ImageList leftImageList;
		ImageList rightImageList;
		std::map<std::string, FeatureVectorList> featureVectorList;
		std::map<std::string, FeatureVector3dList> featureVector3dList;
		std::map<std::string, CorrespondenceMapList> leftRightCorrespondenceMapList;
		std::map<std::string, PointCloudList> pointCloudList;

		//Support methods for adding data to the queue
		void AddImages(FrameWrapper::FrameConstPtr leftImage, FrameWrapper::FrameConstPtr rightImage);
		void AddFeatures(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featureVector, std::string featureCategory = "DEFAULT");
		void AddFeatures3d(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr featureVector, std::string featureCategory = "DEFAULT");
		void AddMatches(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftRightCorrespondenceMap, std::string correspondenceCategory = "DEFAULT");
		void AddPointCloud(PointCloudWrapper::PointCloudConstPtr, std::string cloudCategory = "DEFAULT");

		// Conversion method between the queue index and the time step distance between an entry and the most recent one
		int BackwardStepsToIndex(int backwardSteps);

	/*
	* Inline Methods
	*
	*/
		template <typename Type>
		inline void DeleteIfNotNull(Type* &pointer)
			{
			if (pointer != NULL)
				{
				delete (pointer);
				pointer = NULL;
				}
			}

		template <typename Type>
		inline void DeleteMapEntry(std::map<std::string, Type>& vectorMap, int index)
			{
			for(typename std::map<std::string, Type>::iterator iterator = vectorMap.begin(); iterator != vectorMap.end(); ++iterator)
				{
				DeleteIfNotNull( iterator->second.at(index) );
				}
			}

		inline void DeleteAllDataAt(int index)
			{
			DeleteIfNotNull( leftImageList.at(index) );
			DeleteIfNotNull( rightImageList.at(index) );
			DeleteMapEntry( featureVectorList, index );
			DeleteMapEntry( leftRightCorrespondenceMapList, index );
			DeleteMapEntry( pointCloudList, index );
			}

		template <typename Type>
		inline void ReplaceIndexByIndexOnMap(std::map<std::string, Type>& vectorMap, int replacedIndex, int replacingIndex)
			{
			for(typename std::map<std::string, Type>::iterator iterator = vectorMap.begin(); iterator != vectorMap.end(); ++iterator)
				{
				iterator->second.at(replacedIndex) = iterator->second.at(replacingIndex);
				}
			}

		template <typename Type>
		inline void ReplaceIndexByNullOnMap(std::map<std::string, Type>& vectorMap, int replacedIndex)
			{
			for(typename std::map<std::string, Type>::iterator iterator = vectorMap.begin(); iterator != vectorMap.end(); ++iterator)
				{
				iterator->second.at(replacedIndex) = NULL;
				}
			}

	};

}
}
}
#endif
/* BundleHistory.hpp */
/** @} */
