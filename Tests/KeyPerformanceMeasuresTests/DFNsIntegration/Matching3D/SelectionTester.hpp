/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.hpp
 * @date 11/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.8 of deliverable 5.5.
 * "90% of manually defined matches on manually defined features should be detected correctly.  Manually defined matches are considered to be matches between features made by a human 
 * on close inspection of a set of features on a pair of images.  A human will inspect each set of matches and identify those that are incorrect."
 *
 * @{
 */

#ifndef SELECTION_TESTER_HPP
#define SELECTION_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/BaseTypes.hpp>
#include <Types/CPP/CorrespondenceMap3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/SupportTypes.hpp>

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
class SelectionTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		SelectionTester();
		~SelectionTester();

		void SetDfns(CDFF::DFN::FeaturesDescription3DInterface* descriptor, CDFF::DFN::FeaturesMatching3DInterface* matcher);
		void SetConfigurationFilePaths(const std::string& featuresDescriptorConfigurationFilePath, const std::string& featuresMatcherConfigurationFilePath);
		void SetInputFilesPaths(const std::string& sourceCloudFilePath, const std::string& sinkCloudFilePath, const std::string& correspondencesFilePath);
		void ExecuteDfns();
		bool AreCorrespondencesValid(float percentageThreshold);

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
		typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

		std::string featuresDescriptorConfigurationFilePath;
		std::string featuresMatcherConfigurationFilePath;
		std::string sourceCloudFilePath;
		std::string sinkCloudFilePath;
		std::string correspondencesFilePath;

		PointCloudWrapper::PointCloudConstPtr inputSourceCloud, inputSinkCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSourceKeypointsVector, inputSinkKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sourceFeaturesVector, sinkFeaturesVector;
		CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr outputCorrespondenceMap;
		CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr referenceCorrespondenceMap;
		PoseWrapper::Pose3DConstPtr sourcePoseInSink;
		bool matcherSuccess;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		CDFF::DFN::FeaturesDescription3DInterface* descriptor;
		CDFF::DFN::FeaturesMatching3DInterface* matcher;

		bool dfnsWereLoaded;
		bool inputCloudsWereLoaded;
		bool inputKeypointsWereLoaded;
		bool precisionReferenceWasLoaded;

		void LoadInputCloud(const std::string& cloudFilePath, PointCloudWrapper::PointCloudConstPtr& cloud);
		void LoadReferenceCorrespondenceMap();
		void ConfigureDfns();

		bool ValidateCorrespondences(float percentageThreshold);
		bool CorrespondencesAreTheSame(int referenceIndex, int outputIndex);

		void ComputeMatchesFromPose();
		BaseTypesWrapper::Point3D TransformPoint(const BaseTypesWrapper::Point3D& point, const AffineTransform& affineTransform);
		BaseTypesWrapper::Point3D FindClosestSinkPointTo(const BaseTypesWrapper::Point3D& sourcePoint, float& closestDistance);
	};

#endif

/* SelectionTester.hpp */
/** @} */
