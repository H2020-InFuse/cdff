/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixTester.hpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.4 of deliverable 5.5.
 * "A valid Fundamental Matrix should be found between two sets of features that are correctly located (validated by the eyes of a human) and correctly matched (validated by the eyes of a human)."
 *
 * @{
 */

#ifndef FUNDAMENTAL_MATRIX_TESTER_HPP
#define FUNDAMENTAL_MATRIX_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/highgui/highgui.hpp>

#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <Errors/Assert.hpp>

#include <CorrespondenceMap2D.hpp>
#include <Matrix.hpp>

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
class FundamentalMatrixTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		FundamentalMatrixTester(std::string configurationFilePath, dfn_ci::FundamentalMatrixComputationInterface* dfn);
		~FundamentalMatrixTester();

		void SetInputFilePath(std::string inputCorrespondenceFilePath);
		void ExecuteDfn();
		bool IsDfnSuccessful();
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
		std::string configurationFilePath;
		std::string inputCorrespondenceFilePath;

		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputCorrespondenceMap;
		MatrixWrapper::Matrix3dConstPtr outputFundamentalMatrix;
		bool outputComputationSuccess;

		dfn_ci::FundamentalMatrixComputationInterface* dfn;

		bool inputCorrespondencesWereLoaded;

		void LoadInputCorrespondences();
		void ConfigureDfn();

		void PrintOutputInformation();
		float ComputeError(cv::Mat cvFundamentalMatrix);
	};

#endif

/* FundamentalMatrixTester.hpp */
/** @} */
