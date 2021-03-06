/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file QualityTester.hpp
 * @date 14/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.6 of deliverable 5.5.
 * "Resulting point cloud should be within the expected bounds of error described in D5.2", and
 * "Expected performance is no more than 10% outliers as estimated by a human inspecting the point cloud", and 
 * "position estimation less than 1% of R, where R is the maximum operational distance of the camera/sensor", and 
 * "90% similarity in shape to the object viewed with less than 10% error in dimensional analysis (only for components larger than 10% of the total size of the object)"
 *
 * @{
 */

#ifndef QUALITY_TESTER_HPP
#define QUALITY_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/Frame.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

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
class QualityTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		QualityTester();
		~QualityTester();

		void SetDfn(const std::string& configurationFilePath, CDFF::DFN::StereoReconstructionInterface* dfn);
		void SetInputFilesPaths(const std::string& inputLeftImageFilePath, const std::string& inputRightImageFilePath);
		void SetOutputFilePath(const std::string& outputPointCloudFilePath);
		void SetOutliersFilePath(const std::string& outliersReferenceFilePath);
		void SetMeasuresFilePath(const std::string& measuresReferenceFilePath);
		void ExecuteDfn();
		bool IsOutliersQualitySufficient(float outliersPercentageThreshold);
		bool IsCameraDistanceQualitySufficient(float cameraOperationDistance, float cameraDistanceErrorPercentage);
		bool IsDimensionsQualitySufficient(float shapeSimilarityPercentange, float dimensionalErrorPercentage, float componentSizeThresholdPercentage);

		void SaveOutputPointCloud(const std::string& outputPointCloudFilePath);

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
		struct Line
			{
			int32_t sourceIndex;
			int32_t sinkIndex;
			float length;
			};

		typedef std::vector<Line> Object;

		std::string configurationFilePath;
		std::string inputLeftImageFilePath;
		std::string inputRightImageFilePath;
		std::string outputPointCloudFilePath;
		std::string outliersReferenceFilePath;
		std::string measuresReferenceFilePath;

		FrameWrapper::FrameConstPtr inputLeftFrame;
		FrameWrapper::FrameConstPtr inputRightFrame;
		PointCloudWrapper::PointCloudConstPtr outputPointCloud;
		cv::Mat outliersMatrix;
		cv::Mat pointsToCameraMatrix;
		std::vector<Object> objectsList;

		Converters::MatToFrameConverter frameConverter;
		Converters::PointCloudToPclPointCloudConverter pointCloudConverter;
		Converters::PclPointCloudToPointCloudConverter inverseCloudConverter;
		CDFF::DFN::StereoReconstructionInterface* dfn;

		bool inputImagesWereLoaded;
		bool outputPointCloudWasLoaded;
		bool outliersReferenceWasLoaded;
		bool measuresReferenceWasLoaded;
		bool dfnExecuted;
		bool dfnWasLoaded;

		void LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame);
		void LoadOutputPointCloud();
		void LoadOutliersReference();
		void LoadMeasuresReference();
		void ConfigureDfn();

		float ComputeCameraDistanceError();
		float ComputeShapeSimilarity();
		bool EvaluateDimensionalError(float dimensionalErrorPercentage, float componentSizeThresholdPercentage);
		float ComputeObjectDimension(int objectIndex);
		float ComputeLineAbsoluteError(const Line& line);
		float ComputeObjectShapeSimilarity(int objectIndex);
	};

#endif

/* QualityTester.hpp */
/** @} */
