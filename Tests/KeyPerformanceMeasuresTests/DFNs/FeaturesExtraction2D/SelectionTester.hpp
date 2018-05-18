/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.hpp
 * @date 04/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.2 of deliverable 5.5.
 * "The number of features detected should be within 10% of the number of features in a reference case for feature detection obtained by a standard detection algorithm (open source and in OpenCV)
 * implemented for a given descriptor.  These open-source descriptor implementations are considered to be industry standard, only parameters are expected to cause differences between implementations
 * of the same descriptor on the same data." and 
 * "90% of manually detected features should lay within a 5 pixel distance of the a detected feature.  Manually detected features are considered to be features identified by a human inspecting 
 * a close-up of an image to establish whether a feature is centered correctly on a part of the image.  A human will inspect features to identify any that are misplaced on a part of an image."
 *
 * @{
 */

#ifndef SELECTION_TESTER_HPP
#define SELECTION_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <Errors/Assert.hpp>

#include <VisualPointFeatureVector2D.hpp>
#include <Frame.hpp>
#include <Pose.hpp>
#include <MatToFrameConverter.hpp>

#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>

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
class SelectionTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		SelectionTester(std::string configurationFilePath, dfn_ci::FeaturesExtraction2DInterface* dfn);
		~SelectionTester();

		void SetFilesPaths(std::string inputImageFilePath, std::string numberReferenceFilePath, std::string precisionReferenceFilePath);
		void ExecuteDfn();
		bool IsSelectionValid(float numberPercentageThreshold, unsigned pixelOutlierThreshold, float outliersPercentageThreshold);

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
		Stubs::CacheHandler<cv::Mat, FrameWrapper::FrameConstPtr>* stubFrameCache;
		Mocks::MatToFrameConverter* mockFrameConverter;

		Stubs::CacheHandler<FrameWrapper::FrameConstPtr, cv::Mat>* stubInverseFrameCache;
		Mocks::FrameToMatConverter* mockInverseFrameConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr>* stubVector2dCache;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockVector2dConverter;

		std::string configurationFilePath;
		std::string inputImageFilePath;
		std::string numberReferenceFilePath;
		std::string precisionReferenceFilePath;
		FrameWrapper::FrameConstPtr inputFrame;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr outputFeaturesVector;
		cv::Mat numberKeypointsMatrix;
		cv::Mat precisionKeypointsMatrix;

		Converters::MatToFrameConverter frameConverter;
		dfn_ci::FeaturesExtraction2DInterface* dfn;

		bool inputImageWasLoaded;
		bool numberReferenceWasLoaded;
		bool precisionReferenceWasLoaded;

		void SetUpMocksAndStubs();
		void LoadInputImage();
		void LoadReferenceFeatures(std::string& filePath, cv::Mat& keypointsMatrix);
		void ConfigureDfn();

		bool ValidateNumberOfKeypoints(float numberPercentageThreshold);
		bool ValidateKeypoints(unsigned pixelOutlierThreshold, float outliersPercentageThreshold);
		unsigned ComputePixelDistance(unsigned outputKeypointIndex, unsigned precisionReferenceKeypointIndex);
	};

#endif

/* SelectionTester.hpp */
/** @} */
