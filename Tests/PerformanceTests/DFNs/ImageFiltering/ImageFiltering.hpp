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
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <FrameToMatConverter.hpp>
#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNs/PerformanceTestInterface.hpp>


class ImageFilteringTestInterface : public PerformanceTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ImageFilteringTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName, dfn_ci::ImageFilteringInterface* filter);
		~ImageFilteringTestInterface();

		void SetImageFilePath(std::string baseImageFolder, std::string imagesListFileName);
		void SetOutputFile(std::string imagesOutputFileBaseName, std::string imagesOutputExtension);
	
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
		Stubs::CacheHandler<FrameWrapper::FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		Stubs::CacheHandler<cv::Mat, FrameWrapper::FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;

		bool saveOutput;
		std::string baseImageFolder;
		std::string imagesListFileName;
		std::string imagesOutputFileBaseName;
		std::string imagesOutputExtension;
		std::vector<std::string> imageFileNamesList;

		dfn_ci::ImageFilteringInterface* filter;
		void ReadImageFileNamesList();
		void SetupMocksAndStubs();

		bool SetNextInputs();
		MeasuresMap ExtractMeasures();
	};

#endif

/* ImageFilteringTestInterface.hpp */
/** @} */
