/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixRansac.cpp
 * @date 15/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Validity Test 4.1.1.4 for DFN implementation FundamentalMatrixRansac.
 * "A valid Fundamental Matrix should be found between two sets of features that are correctly located (validated by the eyes of a human) and correctly matched (validated by the eyes of a human)."
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "FundamentalMatrixTester.hpp"
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
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
You should provide two parameters: \n \
(i) the configuration file path; \n \
(ii) the input correspondences file path in xml format; \n \
Example Usage: ./fundamental_matrix_ransac_test ../tests/ConfigurationFiles/DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac_DevonIsland.yaml ../tests/Data/Images/devonIslandKeypointsMatches.xml \n \n"; 

int main(int argc, char** argv)
	{
	std::string configurationFilePath;
	std::string inputCorrespondenceFilePath;

	ASSERT(argc == 3, USAGE);
	configurationFilePath = argv[1];
	inputCorrespondenceFilePath = argv[2];

	FundamentalMatrixRansac* ransac = new FundamentalMatrixRansac();
	FundamentalMatrixTester tester(configurationFilePath, ransac);
	tester.SetInputFilePath(inputCorrespondenceFilePath);
	tester.ExecuteDfn();
	bool success = tester.IsDfnSuccessful();

	VERIFY_REQUIREMENT(success, "Correctness requirement 4.1.1.4 failed on the input correspondences");
	return 0;
	}


/** @} */
