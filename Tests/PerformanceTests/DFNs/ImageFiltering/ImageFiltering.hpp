/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageFiltering.hpp
 * @date 25/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * This is the main class for the implementation of Performance Test for the DFN ImageFiltering
 * 
 * 
 * @{
 */

#ifndef IMAGE_FILTERING_TEST_INTERFACE_HPP
#define IMAGE_FILTERING_TEST_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNs/PerformanceTestInterface.hpp>


class ImageFilteringTestInterface : public PerformanceTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ImageFilteringTestInterface(const std::string& folderPath, const std::string& baseConfigurationFileName, const std::string& performanceMeasuresFileName, 
			CDFF::DFN::ImageFilteringInterface* filter);
		~ImageFilteringTestInterface();

		void SetImageFilePath(const std::string& baseImageFolder, const std::string& imagesListFileName);
		void SetOutputFile(const std::string& imagesOutputFileBaseName, const std::string& imagesOutputExtension);
	
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
		bool saveOutput;
		std::string baseImageFolder;
		std::string imagesListFileName;
		std::string imagesOutputFileBaseName;
		std::string imagesOutputExtension;
		std::vector<std::string> imageFileNamesList;

		FrameWrapper::FrameConstPtr inputFrame;

		CDFF::DFN::ImageFilteringInterface* filter;
		void ReadImageFileNamesList();

		bool SetNextInputs();
		MeasuresMap ExtractMeasures();
	};

#endif

/* ImageFilteringTestInterface.hpp */
/** @} */
