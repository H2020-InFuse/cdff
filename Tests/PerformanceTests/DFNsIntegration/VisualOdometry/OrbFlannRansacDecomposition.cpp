/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbFlannRansacDecomposition.cpp
 * @date 26/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Orb extractor descriptor, Flann matcher, fundamental matrix RANSAC, and essential matrix decomposition
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
#include "VisualOdometry.hpp"

#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>

using namespace CDFF::DFN::FeaturesExtraction2D;
using namespace CDFF::DFN::FeaturesMatching2D;
using namespace CDFF::DFN::FundamentalMatrixComputation;
using namespace CDFF::DFN::CamerasTransformEstimation;

const std::string USAGE =
	" \n \
	This programs requires nine parameters: \n \
	(i) the folder path containing the configuration files of each DFN \n \
	(ii) the name of the OrbDetectorDescriptor Configuration file \n \
	(iii) the name of the Flann Configuration file \n \
	(iv) the name of the FundamentalMatrixComputation Configuration file \n \
	(v) the name of the CamerasTransformEstimation Configuration file \n \
	(vi) the name of the output performance file (that will be containing in the configuration folder) \n \
	(vii) the base folder containing the images; \n \
	(viii) the file containing the list of images in the format: (time relativeImagePath) on each line (except the first three lines that are ignored); \n \
	(ix) the file containing the list of poses in the format: (x y z qx qy qz qw) on each line (except the first three lines that are ignored); \n \
	(x) the number of images to process; \n \n \
	Example Usage: ./orb_flann_ransac_decomposition ../tests/ConfigurationFiles/DFNsIntegration/VisualOdometry OrbExtractorDescriptor_PerformanceTest_1.yaml \
	FlannMatcher_PerformanceTest_1.yaml FundamentalMatrixRansac_PerformanceTest_1.yaml EssentialMatrixDecomposition_PerformanceTest_2.yaml output.txt \
	../tests/Data/Images ImagesList.txt PosesList.txt \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 11, USAGE);
	
	std::string configurationFolderPath, orbConfigurationFileName, flannConfigurationFileName, fundamentalConfigurationFile, estimatorConfigurationFile, outputFileName;
	std::string imageFolderPath, imageListName, poseListName;
	unsigned imageLimit;
	configurationFolderPath = argv[1];
	orbConfigurationFileName = argv[2];
	flannConfigurationFileName = argv[3];
	fundamentalConfigurationFile = argv[4];
	estimatorConfigurationFile = argv[5];
	outputFileName = argv[6];
	imageFolderPath = argv[7];
	imageListName = argv[8];
	poseListName = argv[9];

	try 	
		{
		imageLimit = std::stoi(argv[10]);
		}
	catch (...)
		{
		ASSERT(false, "the 10th parameter has to be a positive integer number");
		}

	std::vector<std::string> baseConfigurationFiles;
	baseConfigurationFiles.push_back(orbConfigurationFileName);
	baseConfigurationFiles.push_back(flannConfigurationFileName);
	baseConfigurationFiles.push_back(fundamentalConfigurationFile);
	baseConfigurationFiles.push_back(estimatorConfigurationFile);	

	VisualOdometry* interface = new VisualOdometry(configurationFolderPath, baseConfigurationFiles, outputFileName);

	OrbDetectorDescriptor* orb = new OrbDetectorDescriptor();
	FlannMatcher* flann = new FlannMatcher();
	FundamentalMatrixRansac* ransac = new FundamentalMatrixRansac();
	EssentialMatrixDecomposition* essential = new EssentialMatrixDecomposition();
	interface->SetDfns(orb, NULL, flann, ransac, essential); 

	interface->SetInputFiles(imageFolderPath, (imageFolderPath+"/"+imageListName), (imageFolderPath+"/"+poseListName));
	interface->LoadInputFiles();

	interface->SetImageLimit(imageLimit);
	interface->Run();

	delete(interface);
	delete(orb);
	delete(flann);
	delete(ransac);
	delete(essential);
	}

/** @} */
