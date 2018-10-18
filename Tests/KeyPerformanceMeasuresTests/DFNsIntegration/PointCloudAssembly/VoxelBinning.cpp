/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file NearestNeighbourAssembly.cpp
 * @date 18/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Correctness Test for DFN implementation VoxelBinning and CartesianSystemTransform.
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CorrectAssemblyTester.hpp"
#include <PointCloudAssembly/VoxelBinning.hpp>
#include <PointCloudTransform/CartesianSystemTransform.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::PointCloudAssembly;
using namespace CDFF::DFN::PointCloudTransform;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
You should provide four parameters: \n \
(i) the configuration file path of VoxelBinning \n \
(ii) the path to the folder containing the input file; \n \
(iii) the name of the point cloud input file: the file should contain one line for each point cloud, each line contains the cloud file and the cloud pose in the format: path/to/file/from/current/folder x y z qx qy qz qw; \n \
(iv) the name of the output point cloud file in ply format \n \n \
Example Usage: ./correctness_voxel_binning ../tests/ConfigurationFiles/DFNs/PointCloudAssembly/VoxelBinning_DlrHcru.yaml ../tests/Data/PointClouds PointCloudList.txt OutputPointCloud.ply \n \n";

int main(int argc, char** argv)
	{
	std::string assemblerConfigurationFile;
	std::string dataFolderPath, inputPointCloudListFilePath, outputPointCloudFilePath;

	ASSERT(argc >= 5, USAGE);
	assemblerConfigurationFile = argv[1];
	dataFolderPath = argv[2];
	inputPointCloudListFilePath = argv[3];
	outputPointCloudFilePath = argv[4];
	
	VoxelBinning* assemblerDfn = new VoxelBinning(); 
	CartesianSystemTransform* transformDfn = new CartesianSystemTransform();

	CorrectAssemblyTester tester(assemblerConfigurationFile, assemblerDfn, "", transformDfn);
	tester.SetFiles(dataFolderPath, inputPointCloudListFilePath, outputPointCloudFilePath);

	tester.ExecuteDfns();
	tester.SaveOutput();	

	return 0;
	}


/** @} */
