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

#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/Pose.hpp>
#include <Converters/MatToFrameConverter.hpp>

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
		SelectionTester(const std::string& configurationFilePath, CDFF::DFN::FeaturesExtraction2DInterface* dfn);
		~SelectionTester();

		void SetFilesPaths(const std::string& inputImageFilePath, const std::string& numberReferenceFilePath, const std::string& precisionReferenceFilePath);
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
		std::string configurationFilePath;
		std::string inputImageFilePath;
		std::string numberReferenceFilePath;
		std::string precisionReferenceFilePath;
		FrameWrapper::FrameConstPtr inputFrame;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr outputFeaturesVector;
		cv::Mat numberKeypointsMatrix;
		cv::Mat precisionKeypointsMatrix;

		Converters::MatToFrameConverter frameConverter;
		CDFF::DFN::FeaturesExtraction2DInterface* dfn;

		bool inputImageWasLoaded;
		bool numberReferenceWasLoaded;
		bool precisionReferenceWasLoaded;

		void LoadInputImage();
		void LoadReferenceFeatures(const std::string& filePath, cv::Mat& keypointsMatrix);
		void ConfigureDfn();

		bool ValidateNumberOfKeypoints(float numberPercentageThreshold);
		bool ValidateKeypoints(unsigned pixelOutlierThreshold, float outliersPercentageThreshold);
		unsigned ComputePixelDistance(unsigned outputKeypointIndex, unsigned precisionReferenceKeypointIndex);
	};

#endif

/* SelectionTester.hpp */
/** @} */
