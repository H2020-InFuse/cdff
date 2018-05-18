/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LinearityTester.hpp
 * @date 09/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.1 of deliverable 5.5.
 * "Image should be the exact  resolution requested." and 
 * "Image should exhibit distortion of no more than 10% difference in relative distortion compared high-quality undistorted and calibrated reference of the same scene." and
 * "Note that no reference is perfect."
 *
 * @{
 */

#ifndef LINEARITY_TESTER_HPP
#define LINEARITY_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <Errors/Assert.hpp>

#include <Frame.hpp>
#include <MatToFrameConverter.hpp>
#include <FrameToMatConverter.hpp>

#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>

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
class LinearityTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		LinearityTester();
		~LinearityTester();

		void SetDfn(std::string configurationFilePath, dfn_ci::ImageFilteringInterface* dfn);
		void SetFilesPaths(std::string inputImageFilePath, std::string outputImageFilePath);
		void ExecuteDfn();
		bool IsResultLinear(std::string referenceLinesFilePath, float relativeDistortionDifference);
		void SaveOutputImage();

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
		struct Point
			{
			int x;
			int y;
			};
		typedef std::vector<Point> Line;

		Stubs::CacheHandler<cv::Mat, FrameWrapper::FrameConstPtr>* stubFrameCache;
		Mocks::MatToFrameConverter* mockFrameConverter;

		Stubs::CacheHandler<FrameWrapper::FrameConstPtr, cv::Mat>* stubInverseFrameCache;
		Mocks::FrameToMatConverter* mockInverseFrameConverter;

		std::string configurationFilePath;
		std::string inputImageFilePath;
		std::string referenceLinesFilePath;
		std::string outputImageFilePath;

		FrameWrapper::FrameConstPtr inputFrame;
		FrameWrapper::FrameConstPtr outputFrame;

		cv::Mat inputImage;
		cv::Mat outputImage;
		std::vector<Line> referenceLinesList;
		std::vector<float> errorsList;
		std::vector<float> relativeErrorsList;

		Converters::MatToFrameConverter frameConverter;
		Converters::FrameToMatConverter inverseFrameConverter;
		dfn_ci::ImageFilteringInterface* dfn;

		bool dfnWasSet;
		bool inputImageWasLoaded;

		void SetUpMocksAndStubs();
		void LoadInputImage();
		void LoadReferenceLines();
		void ConfigureDfn();

		void ComputeErrors(float& totalError, float& averageError, float& averageRelativeError);
		float ComputeErrorOnLine(const Line& line);
		float ComputeLineLength(const Line& line);
	};

#endif

/* LinearityTester.hpp */
/** @} */
