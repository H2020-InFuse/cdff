/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrectAssemblyTester.hpp
 * @date 11/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test whether the point cloud assembly works correctly in the assumption of perfect alignment (i.e. the pose between two point clouds is known exactly).
 * The input point clouds may or may not contain different amount of noise.
 *
 * @{
 */

#ifndef CORRECT_ASSEMBLY_TESTER_HPP
#define CORRECT_ASSEMBLY_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudTransformation/PointCloudTransformationInterface.hpp>
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>

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
class CorrectAssemblyTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		CorrectAssemblyTester(const std::string& configurationFile, CDFF::DFN::PointCloudAssemblyInterface* assemblyDfn, const std::string& transformerConfigurationFile, 
			CDFF::DFN::PointCloudTransformationInterface* transformDfn);
		~CorrectAssemblyTester();

		void SetFiles(const std::string& dataFolderPath, const std::string& inputPointCloudListFile, const std::string& outputPointCloudFile);

		void ExecuteDfns();
		void SaveOutput();

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
		struct PointCloudFileEntry
			{
			std::string filePath;
			PoseWrapper::Pose3D pose;
			};

		std::string configurationFile, transformerConfigurationFile;
		std::string dataFolderPath, inputPointCloudListFile, outputPointCloudFile;
		CDFF::DFN::PointCloudAssemblyInterface* assemblyDfn;
		CDFF::DFN::PointCloudTransformationInterface* transformDfn;

		PointCloudWrapper::PointCloudConstPtr inputCloud;
		PointCloudWrapper::PointCloudConstPtr outputCloud;
		std::vector<PointCloudFileEntry> pointCloudList;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		Converters::PointCloudToPclPointCloudConverter pclPointCloudConverter;
		bool inputsWereLoaded;

		void LoadInputPointClouds();
		PointCloudWrapper::PointCloudConstPtr LoadPointCloud(std::string pointCloudFilePath);
		void ConfigureDfns();

		template <class Type> void DeleteIfNotNull(Type* &pointer)
			{
			if (pointer != NULL)
				{
				delete(pointer);
				}
			pointer = NULL;
			}
	};

#endif

/* CorrectAssemblyTester.hpp */
/** @} */
