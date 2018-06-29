/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionExecutor.hpp
 * @date 16/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class is used for execution of the DFPC StereoReconstruction on a sequence of input images
 *
 * @{
 */

#ifndef RECONSTRUCTION_EXECUTOR_HPP
#define RECONSTRUCTION_EXECUTOR_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <SupportTypes.hpp>
#include <MatToFrameConverter.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>

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
class ReconstructionExecutor
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ReconstructionExecutor();
		~ReconstructionExecutor();

		void SetDfpc(std::string configurationFilePath, dfpc_ci::Reconstruction3DInterface* dfpc);
		void SetInputFilesPaths(std::string inputImagesFolder, std::string inputImagesListFileName);
		void SetOutputFilePath(std::string outputPointCloudFilePath);
		void SetOutliersFilePath(std::string outliersReferenceFilePath);
		void SetMeasuresFilePath(std::string measuresReferenceFilePath);
		void ExecuteDfpc();
		bool IsOutliersQualitySufficient(float outliersPercentageThreshold);
		bool IsCameraDistanceQualitySufficient(float cameraOperationDistance, float cameraDistanceErrorPercentage);
		bool IsDimensionsQualitySufficient(float shapeSimilarityPercentange, float dimensionalErrorPercentage, float componentSizeThresholdPercentage);

		void SaveOutputPointCloud(std::string outputPointCloudFilePath);

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
		std::string inputImagesFolder;
		std::string inputImagesListFileName;
		std::string outputPointCloudFilePath;
		std::string outliersReferenceFilePath;
		std::string measuresReferenceFilePath;
		std::vector<std::string> leftImageFileNamesList;
		std::vector<std::string> rightImageFileNamesList;

		FrameWrapper::FrameConstPtr inputLeftFrame;
		FrameWrapper::FrameConstPtr inputRightFrame;
		PointCloudWrapper::PointCloudConstPtr outputPointCloud;
		PoseWrapper::Pose3DConstPtr outputCameraPose;
		bool outputSuccess;
		cv::Mat outliersMatrix;
		cv::Mat pointsToCameraMatrix;
		std::vector<Object> objectsList;

		Converters::MatToFrameConverter frameConverter;
		Converters::PointCloudToPclPointCloudConverter pointCloudConverter;
		Converters::PclPointCloudToPointCloudConverter inverseCloudConverter;
		dfpc_ci::Reconstruction3DInterface* dfpc;

		bool inputImagesWereLoaded;
		bool outputPointCloudWasLoaded;
		bool outliersReferenceWasLoaded;
		bool measuresReferenceWasLoaded;
		bool dfpcExecuted;
		bool dfpcWasLoaded;

		void LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame);
		void LoadInputImagesList();
		void LoadOutputPointCloud();
		void LoadOutliersReference();
		void LoadMeasuresReference();
		void ConfigureDfpc();

		float ComputeCameraDistanceError();
		float ComputeShapeSimilarity();
		bool EvaluateDimensionalError(float dimensionalErrorPercentage, float componentSizeThresholdPercentage);
		float ComputeObjectDimension(int objectIndex);
		float ComputeLineAbsoluteError(const Line& line);
		float ComputeObjectShapeSimilarity(int objectIndex);
	};

#endif

/* ReconstructionExecutor.hpp */
/** @} */
