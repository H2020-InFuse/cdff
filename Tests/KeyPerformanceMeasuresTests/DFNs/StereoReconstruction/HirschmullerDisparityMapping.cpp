/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.cpp
 * @date 14/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Validity Test 4.1.1.6 for DFN implementation HirschmullerDisparityMapping.
 * "Resulting point cloud should be within the expected bounds of error described in D5.2", and
 * "Expected performance is no more than 10% outliers as estimated by a human inspecting the point cloud", and 
 * "position estimation less than 1% of R, where R is the maximum operational distance of the camera/sensor", and 
 * "90% similarity in shape to the object viewed with less than 10% error in dimensional analysis (only for components larger than 10% of the total size of the object)"
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "QualityTester.hpp"
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
The program has two usage depending on the number of parameters you will provide: \n \
(a) if you provide four string parameters the program will execute the stereo reconstruction dfn and save the result to a file, in this case the three parameters have the following meaning: \n \
(a.i) 1st parameter is the configuration file path of the dfn implementation HirschmullerDisparityMapping; \n \
(a.ii) 2nd parameter is the input left image file path; \n \
(a.iii) 3rd parameter is the input right image file path; \n \
(a.iv) 4th is the output cloud file path, in ply format; \n \n \
Example usage: ./quality_hirschmuller ../tests/ConfigurationFiles/DFNs/StereoReconstruction/HirschmullerDisparityMapping_DevonIsland.yaml \
../tests/Data/Images/RoadLeft.png ../tests/Data/Images/RoadRight.png ../tests/Data/PointClouds/Road.ply \n \n \n \
After you run (a), you should open the result with the tool DataGenerator/detect_outliers. Using the tool, you should detect those cloud points which should not appear in the scene. \
The result is used by the second step of the program to assess the quality of the point cloud. \n \n \
Example call: ./detect_outliers ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadOutliers.xml \n \n \n \
(b) if you provide two or three parameters, the program will assess whether the data meets the test requirements. In this case the parameters have the following meaning: \n \
(b.i) 1st parameter is the point cloud file produced by the application of the dfn, it is in ply format. \n \
(b.ii) 2nd parameter is the file containing the outliers in xml format \n \
Optionally you can add one more parameter: \n \
(b.iii) 3rd optional parameter is outliers percentage threshold for the quality check. It must be a number between 0 and 1; The default is 0.10. \n \n \
Example usage: ./quality_hirschmuller ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadOutliers.xml \n \n ";

float ExtractOutliersPercentageThreshold(char* argument)
	{
	const std::string errorMessage = "The 4th parameter numberPercentageThreshold has to be a floating point number between 0 and 1";
	float outliersPercentageThreshold;

	try 
		{
		outliersPercentageThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(outliersPercentageThreshold >= 0 && outliersPercentageThreshold <= 1, errorMessage);
	
	return outliersPercentageThreshold;
	}


int main(int argc, char** argv)
	{
	std::string configurationFilePath;
	std::string inputLeftImageFilePath;
	std::string inputRightImageFilePath;
	std::string outputPointCloudFilePath;
	std::string outliersReferenceFilePath;
	float outliersPercentageThreshold = 0.10;

	ASSERT(argc >= 3 && argc <= 5, USAGE);
	
	if (argc < 5)
		{
		outputPointCloudFilePath = argv[1];
		outliersReferenceFilePath = argv[2];
		if (argc == 4)
			{
			outliersPercentageThreshold = ExtractOutliersPercentageThreshold(argv[3]);
			}
		}
	else
		{
		configurationFilePath = argv[1];
		inputLeftImageFilePath = argv[2];
		inputRightImageFilePath = argv[3];
		outputPointCloudFilePath = argv[4];
		}

	HirschmullerDisparityMapping* hirschmuller = new HirschmullerDisparityMapping();
	QualityTester tester;

	if (argc == 5)
		{
		tester.SetDfn(configurationFilePath, hirschmuller);
		tester.SetInputFilesPaths(inputLeftImageFilePath, inputRightImageFilePath);
		tester.SetOutputFilePath(outputPointCloudFilePath);
		tester.ExecuteDfn();
		tester.SaveOutputPointCloud();
		}
	else
		{
		tester.SetOutputFilePath(outputPointCloudFilePath);
		tester.SetOutliersFilePath(outliersReferenceFilePath);
		bool success = tester.IsQualitySufficient(outliersPercentageThreshold);
		VERIFY_REQUIREMENT(success, "Quality of reconstructed cloud requirement 4.1.1.6 failed on the input images");
		}

	return 0;
	}


/** @} */
