/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudModelLocalisation.hpp
 * @date 22/01/2019
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * This is the test interface for the implementation of the performance test for DFPC Reconstruction 3D
 *
 *
 * @{
 */

#ifndef POINT_CLOUD_MODEL_LOCALISATION_INTERFACE_HPP
#define POINT_CLOUD_MODEL_LOCALISATION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Converters/MatToFrameConverter.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/PointCloud.hpp>

#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <PerformanceTests/DFPCs/PerformanceTestInterface.hpp>

class PointCloudModelLocalisationTestInterface : public PerformanceTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		PointCloudModelLocalisationTestInterface(const std::string& folderPath, const std::string& baseConfigurationFileName, const std::string& performanceMeasuresFileName,
			CDFF::DFPC::PointCloudModelLocalisationInterface* localizer);
		~PointCloudModelLocalisationTestInterface();

		void SetInputFilesPath(const std::string& baseFolderPath, const std::string& inputListFileName);

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
		struct InputFilePathCollection
			{
			std::string sceneCloudFilePath;
			std::string modelCloudFilePath;
			std::string groundTruthPoseFilePath;
			};

		std::string baseFolderPath;
		std::string inputListFileName;
		std::vector<InputFilePathCollection> inputFileList;
		void ReadInputFileList();

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;

		CDFF::DFPC::PointCloudModelLocalisationInterface* localizer;
		PointCloudWrapper::PointCloudConstPtr LoadPointCloud(const std::string& cloudFilePath);
		void LoadGroundTruthPose(const std::string& poseFilePath);

		PointCloudWrapper::PointCloudConstPtr sceneCloud;
		PointCloudWrapper::PointCloudConstPtr modelCloud;
		PoseWrapper::Pose3D groundTruthPose;
		PoseWrapper::Pose3D outputPose;
		bool success = false;

		void ClearInputs();
		bool SetNextInputs() override;
		void ExecuteDfpc();
		MeasuresMap ExtractMeasures();

		float ComputeLocationError();
		float ComputeOrientationError(float modelSize);
	};

#endif

/* PointCloudModelLocalisationTestInterface.hpp */
/** @} */
