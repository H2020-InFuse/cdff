/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ScanlineOptimization.cpp
 * @date 24/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFN Scanline Optimization.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "StereoReconstruction.hpp"
#include <StereoReconstruction/ScanlineOptimization.hpp>

using namespace dfn_ci;

int main(int argc, char** argv)
	{
	std::string configurationFileName = "ScanlineOptimization_Performance1.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	StereoReconstructionInterface* reconstructor = new ScanlineOptimization();
	StereoReconstructionTestInterface interface("../tests/ConfigurationFiles/DFNs/StereoReconstruction", configurationFileName, "ScanlineOptimization.txt", reconstructor);
	
	if (argc >= 5)
		{
		std::string useReferenceDisparity = argv[4];
		interface.SetImageFilesPath(argv[2], argv[3], (useReferenceDisparity == "UseReferenceDisparity") );
		}

	if (argc >= 7)
		{
		interface.SetDisparityOutputFile(argv[5], argv[6]);
		}

	if (argc >= 9)
		{
		interface.SetCloudOutputFile(argv[7], argv[8]);
		}

	interface.Run();
	};

/** @} */
