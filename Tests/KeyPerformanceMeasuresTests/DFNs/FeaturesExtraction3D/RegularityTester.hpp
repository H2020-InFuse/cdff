/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RegularityTester.hpp
 * @date 02/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.7 of deliverable 5.5.
 * " Keypoints should be selected at regular locations throughout the point cloud as viewed by a human observer.  
 *   <Regular> refers to keypoints not exhibiting gaps or clustering exceeding 20% of the average separation between keypoints."
 *
 * @{
 */

#ifndef REGULARITY_TESTER_HPP
#define REGULARITY_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <VisualPointFeatureVector3D.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>

#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <boost/make_shared.hpp>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class RegularityTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		enum AverageSeparationType
			{
			POINTS_PAIR_DISTANCE,
			CLOSEST_NEIGHBOUR_DISTANCE
			};

		RegularityTester(std::string configurationFilePath, std::string pointCloudFilePath, AverageSeparationType averageSeparationType, dfn_ci::FeaturesExtraction3DInterface* dfn);
		~RegularityTester();

		void ExecuteDfn();
		bool IsOutputRegular(float regularity);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		typedef std::vector<int> Cluster;

		Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudWrapper::PointCloudConstPtr>* stubCloudCache;
		Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<PointCloudWrapper::PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr>* stubVector3dCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockVector3dConverter;

		std::string configurationFilePath;
		std::string pointCloudFilePath;
		PointCloudWrapper::PointCloudConstPtr inputCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr outputFeaturesVector;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		dfn_ci::FeaturesExtraction3DInterface* dfn;

		AverageSeparationType averageSeparationType;
		std::vector<Cluster> clustersList;
		float averageNumberOfPointsInCluster;

		void SetUpMocksAndStubs();
		void LoadPointCloud();
		void ConfigureDfn();

		float ComputeAverageSeparation();
		float ComputeAveragePointsPairDistance();
		float ComputeAverageClosestNeighbourDistance();
		void ComputeClustersList(float distanceThreshold);
		void CompleteCluster(Cluster& cluster, std::vector<bool>& pointsInCluster, float distanceThreshold);
		bool ThereAreNoGaps(float distanceThreshold);
		bool ThereAreNoClusters(float distanceThreshold);

		float ComputeDistance(int pointIndex1, int pointIndex2);
		float ComputeDistance(const Cluster& cluster1, const Cluster& cluster2);
		float ComputeClusterSize(const Cluster& cluster);
		float ComputeWouldBeSizeIfPointWasAdded(const Cluster& cluster, int pointIndex, float currentSize);
		float ComputeAverageNumberOfPointsInCluster();
	};

#endif

/* RegularityTester.hpp */
/** @} */
