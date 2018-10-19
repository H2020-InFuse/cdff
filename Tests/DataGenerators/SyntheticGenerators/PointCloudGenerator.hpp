/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudGenerator.hpp
 * @date 16/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class will generator a set of point clouds from a set of stereo image pairs. It uses HirschmullerDisparityMapping DFN implementation.
 *  It also uses a set of poses, in order to produce an output file that contains the list of output clouds with associated poses.
 *  The output is meant to be given to the PointCloudAssembly DFN test in KeyPerformanceTest folder.
 *
 * @{
 */

#ifndef POINT_CLOUD_GENERATOR_HPP
#define POINT_CLOUD_GENERATOR_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <random>
#include <pcl/io/ply_io.h>
#include <Pose.hpp>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <StereoReconstruction/StereoReconstructionExecutor.hpp>

#include <PointCloudToPclPointCloudConverter.hpp>
#include <MatToFrameConverter.hpp>

namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class PointCloudGenerator
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	PointCloudGenerator(std::string inputFolderPath, std::string imageFileName, std::string poseFileName, std::string outputFolderPath, std::string cloudFileName, std::string dfnConfFile);
        	~PointCloudGenerator();

		void GenerateClouds();
		void EnablePlaneFiltering();

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
		struct InputEntry
			{
			std::string leftImagePath;
			std::string rightImagePath;
			PoseWrapper::Pose3D pose;
			};

		std::string inputFolderPath;
		std::string imageFileName;
		std::string poseFileName;
		std::string outputFolderPath;
		std::string cloudFileName;
		bool planeFilteringEnabled;

		CDFF::DFN::StereoReconstruction::HirschmullerDisparityMapping* disparityMapping;
		CDFF::DFN::StereoReconstructionExecutor* disparityMappingExecutor;

		std::vector<InputEntry> inputList;

		Converters::MatToFrameConverter matFrameConverter;
		Converters::PointCloudToPclPointCloudConverter pclPointCloudConverter;

		void ReadInputFiles();
		void ExecuteDisparityMapping(const InputEntry& inputEntry);
		FrameWrapper::FrameConstPtr LoadImage(std::string imageFilePath);
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr FilterDominantPlane(PointCloudWrapper::PointCloudConstPtr pointCloud);
		void SavePointCloud(PointCloudWrapper::PointCloudConstPtr pointCloud, const PoseWrapper::Pose3D& pose);
		void SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclOutputCloud, const PoseWrapper::Pose3D& pose);	
    };

}
#endif
/* PointCloudGenerator.hpp */
/** @} */
