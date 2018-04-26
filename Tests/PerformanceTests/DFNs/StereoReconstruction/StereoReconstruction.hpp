/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoReconstruction.hpp
 * @date 25/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * This is the test interface for the implementation of the performance test for DFN Stereo Reconstruction
 * 
 * 
 * @{
 */

#ifndef STEREO_RECONSTRUCTION_TEST_INTERFACE_HPP
#define STEREO_RECONSTRUCTION_TEST_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNs/PerformanceTestInterface.hpp>
#include <pcl/io/ply_io.h>

class StereoReconstructionTestInterface : public PerformanceTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		StereoReconstructionTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName, 
			dfn_ci::StereoReconstructionInterface* reconstructor);
		~StereoReconstructionTestInterface();

		void SetImageFilesPath(std::string baseFolderPath, std::string imagesListFileName, bool useReferenceDisparity);
		void SetDisparityOutputFile(std::string outputDisparityFileBaseName, std::string outputDisparityFileExtension);
		void SetCloudOutputFile(std::string outputCloudFileBaseName, std::string outputCloudFileExtension);

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
		Stubs::CacheHandler<PointCloudWrapper::PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<FrameWrapper::FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		cv::Mat referenceDisparity;
		cv::Mat normalizedReferenceDisparity;
		bool useReferenceDisparity;
		
		std::string baseFolderPath;
		std::string imagesListFileName;
		std::vector<std::string> leftImagesNameList;
		std::vector<std::string> rightImagesNameList;
		std::vector<std::string> disparityImagesNameList;

		bool saveOutputDisparity;
		bool saveOutputCloud;
		std::string outputDisparityFileBaseName;
		std::string outputDisparityFileExtension;
		std::string outputCloudFileBaseName;
		std::string outputCloudFileExtension;

		dfn_ci::StereoReconstructionInterface* reconstructor;
		void ReadImagesList(bool useReferenceDisparity);
		void SetReferenceDisparity(std::string referenceDisparityFilePath);
		void SetupMocksAndStubs();

		bool SetNextInputs();
		MeasuresMap ExtractMeasures();

		void ComputeValidDisparityColumns(const cv::Mat& normalizedDisparity, unsigned& firstValidColumn, unsigned& numberOfValidColumns);
		bool IsBadDisparity(const cv::Mat& normalizedDisparity, unsigned firstValidColumn);
		double ComputeDisparityCost(const cv::Mat& normalizedDisparity, unsigned firstValidColumn, unsigned numberOfValidColumns);
		void SaveOutputDisparity(const cv::Mat& disparity, unsigned testId);
		void SaveOutputCloud(PointCloudWrapper::PointCloudConstPtr pointCloud, unsigned testId);
	};

#endif

/* StereoReconstructionTestInterface.hpp */
/** @} */
