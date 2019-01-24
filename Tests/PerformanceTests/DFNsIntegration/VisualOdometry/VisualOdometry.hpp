/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualOdometry.cpp
 * @date 23/01/2019
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * This class is the main class for the performance test of the integration pipeline of the
 * following three DFNs: FeatureExtraction2D, FeatureDescription2D and FeatureMatching2D
 *
 *
 * @{
 */

#ifndef VISUAL_ODOMETRY
#define VISUAL_ODOMETRY

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>

#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>

#include <Converters/MatToFrameConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>
#include <PerformanceTests/Aggregator.hpp>

#include <Eigen/Geometry>

#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/Matrix.hpp>
#include <Types/CPP/Pose.hpp>

class VisualOdometry : public PerformanceTestInterface
	{
	public:
		VisualOdometry(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, const std::string& performanceMeasuresFileName);
		~VisualOdometry();

		void SetDfns(CDFF::DFN::FeaturesExtraction2DInterface* extractor, CDFF::DFN::FeaturesDescription2DInterface* descriptor, 
			CDFF::DFN::FeaturesMatching2DInterface* matcher, CDFF::DFN::FundamentalMatrixComputationInterface* matrixComputer,
			CDFF::DFN::CamerasTransformEstimationInterface* poseEstimator);
		void SetInputFiles(const std::string& imageFileNamesFolder, const std::string& imagesFileContainer, const std::string& posesFileContainer);
		void LoadInputFiles();
		void SetImageLimit(unsigned imageLimit);
	protected:

	private:
		struct SingleCorrespondence
			{
			unsigned sourceX;
			unsigned sinkX;
			unsigned sourceY;
			unsigned sinkY;
			};

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;

		CDFF::DFN::FeaturesExtraction2DInterface* extractor;
		CDFF::DFN::FeaturesDescription2DInterface* descriptor;
		CDFF::DFN::FeaturesMatching2DInterface* matcher;
		CDFF::DFN::FundamentalMatrixComputationInterface* matrixComputer;
		CDFF::DFN::CamerasTransformEstimationInterface* poseEstimator;

		FrameWrapper::FrameConstPtr leftFrame;
		FrameWrapper::FrameConstPtr rightFrame;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr leftFeaturesVectorHolder;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr rightFeaturesVectorHolder;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr correspondenceMapHolder;
		bool decompositionSuccess;
		bool ransacSuccess;
		PoseWrapper::Pose3D pose;

		Aggregator groundPositionDistanceAggregator;
		Aggregator groundOrientationDistanceAggregator;
		Aggregator leftKeypointsAggregator;
		Aggregator rightKeypointsAggregator;
		Aggregator correspondencesAggregator;
		std::vector<std::string> imageFileNamesList;
		std::vector<PoseWrapper::Pose3D> posesList;
		std::vector<double> imageTimesList;
		std::vector<double> poseTimesList;
		std::vector<PoseWrapper::Pose3D> imagePosesList;

		void LoadImageFileNames();
		void LoadPoses();
		void ComputeImagePoses();

		float initialX, initialY, initialZ;
		float initialRoll, initialPitch, initialYaw;
		
		PoseWrapper::Pose3D InterpolatePose(unsigned beforePoseIndex, unsigned afterPoseIndex, double imageTime);
		void SetupMocksAndStubs();

		bool SetNextInputs() override;
		void ExecuteDfns() override;
		MeasuresMap ExtractMeasures() override;

		float ComputeMixedOrderPixelIndex();

		int long inputId;
		std::string imageFileNamesFolder;
		std::string imagesFileContainer;
		std::string posesFileContainer;
		unsigned imageLimit;

		static int SourceCompare(const void* c1, const void* c2);
		static int SourceCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2);
		static int SinkCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2);
	};
#endif

/* VisualOdometry.hpp */
/** @} */
