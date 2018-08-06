/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Reconstruction3D.hpp
 * @date 23/04/2018
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

#ifndef RECONSTRUCTION_3D_TEST_INTERFACE_HPP
#define RECONSTRUCTION_3D_TEST_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>
#include <Reconstruction3D/ObservedScene.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>

#include <PointCloudToPclPointCloudConverter.hpp>
#include <PerformanceTests/DFPCs/PerformanceTestInterface.hpp>

class Reconstruction3DTestInterface : public PerformanceTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		Reconstruction3DTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName, dfpc_ci::Reconstruction3DInterface* reconstructor);
		~Reconstruction3DTestInterface();

		void SetImageFilesPath(std::string baseFolderPath, std::string imagesListFileName);
		void SetCloudOutputFile(std::string outputCloudFileBaseName);

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
		std::string baseFolderPath;
		std::string imagesListFileName;
		std::string outputCloudFileBaseName;
		bool saveOutputCloud;
		Converters::PointCloudToPclPointCloudConverter pointCloudConverter;

		dfpc_ci::ObservedScene* map;
		dfpc_ci::Reconstruction3DInterface* reconstructor;
		void ReadImagesList();
	
		std::vector<std::string> leftImageFileNamesList;
		std::vector<std::string> rightImageFileNamesList;
		std::vector<FrameWrapper::FrameConstPtr> leftImagesList;
		std::vector<FrameWrapper::FrameConstPtr> rightImagesList;
		PointCloudWrapper::PointCloudConstPtr pointCloud;
		PoseWrapper::Pose3DConstPtr pose;
		bool success;

		void ClearInputs();
		bool SetNextInputs();
		void ExecuteDfpc();
		MeasuresMap ExtractMeasures();
	};

#endif

/* Reconstruction3DTestInterface.hpp */
/** @} */
