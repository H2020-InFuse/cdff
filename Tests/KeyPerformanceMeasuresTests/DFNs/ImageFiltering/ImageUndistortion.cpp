/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortion.cpp
 * @date 09/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Linearity Test 4.1.1.1 for DFN implementation ImageUndistortion.
 * "Image should be the exact  resolution requested." and 
 * "Image should exhibit distortion of no more than 10% difference in relative distortion compared high-quality undistorted and calibrated reference of the same scene." and
 * "Note that no reference is perfect."
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "LinearityTester.hpp"
#include <ImageFiltering/ImageUndistortion.hpp>
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
(a) if you provide three string parameters the program will execute the image undistortion dfn and save the result to a file, in this case the three parameters have the following meaning: \n \
(a.i) 1st parameter is the configuration file path of the dfn implementation ImageUndistortion; \n \
(a.ii) 2nd parameter is the input image file path; \n \
(a.iii) 3rd is the output image file path; \n \n \
Example usage: ./linearity_image_undistortion ../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortion_LabCamera.yaml \
../tests/Data/Images/labChair.png ../tests/Data/Images/labChairUndistorted.png \n \n \n \
After you run (a), you should open the result with the tool DataGenerator/paint_lines. Using the tool, you should detect those image features that are supposed to be line in the real scene, even if \
they are not lines in the image itself. The result is used by the second step of the program to assess the amount of leftover distortion. \n \n \
Example call: ./paint_lines ../tests/Data/Images/labChairUndistorted.png ../tests/Data/Images/labChairLines.xml \n \n \n \
(b) if you provide one or two parameters, the program will assess whether the data meets the test requirements. In this case the parameters have the following meaning: \n \
(b.i) 1st parameter is the lines file path containing the lines you drew with the LinesPainter program. It has to be in opencv xml format. \n \
Optionally you can add one more parameter: \n \
(b.ii) 2nd optional parameter is the relative distortion difference. It must be a number between 0 and 1; The default is 0.10. \n \n \
Example usage: ./linearity_image_undistortion ../tests/Data/Images/labChairLines.xml \n \n ";

float ExtractRelativeDistortionDifference(char* argument)
	{
	const std::string errorMessage = "The 2nd parameter relativeDistortionDifference has to be a floating point number between 0 and 1";
	float relativeDistortionDifference;

	try 
		{
		relativeDistortionDifference = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(relativeDistortionDifference >= 0 && relativeDistortionDifference <= 1, errorMessage);
	
	return relativeDistortionDifference;
	}


int main(int argc, char** argv)
	{
	std::string configurationFilePath;
	std::string inputImageFilePath;
	std::string referenceLinesFilePath = "";
	std::string outputImageFilePath = "";
	float relativeDistortionDifference = 0.10;

	ASSERT(argc >= 2 && argc <= 4, USAGE);
	if (argc == 4)
		{
		configurationFilePath = argv[1];
		inputImageFilePath = argv[2];
		outputImageFilePath = argv[3];
		}
	else if (argc == 2)
		{
		referenceLinesFilePath = argv[1];
		}
	else if (argc == 3)
		{
		referenceLinesFilePath = argv[1];
		relativeDistortionDifference = ExtractRelativeDistortionDifference(argv[2]);
		}

	LinearityTester tester;
	if (argc == 4)
		{
		ImageUndistortion* undistortion = new ImageUndistortion();
		tester.SetDfn(configurationFilePath, undistortion);
		tester.SetFilesPaths(inputImageFilePath, outputImageFilePath);
		tester.ExecuteDfn();
		tester.SaveOutputImage();
		}
	else
		{
		bool success = tester.IsResultLinear(referenceLinesFilePath, relativeDistortionDifference);
		VERIFY_REQUIREMENT(success, "Regularity requirement 4.1.1.1 failed on the input point cloud");
		}

	return 0;
	}


/** @} */
