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
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
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
		StereoReconstructionTestInterface(const std::string& folderPath, const std::string& baseConfigurationFileName, const std::string& performanceMeasuresFileName, 
			CDFF::DFN::StereoReconstructionInterface* reconstructor);
		~StereoReconstructionTestInterface();

		void SetImageFilesPath(const std::string& baseFolderPath, const std::string& imagesListFileName, bool useReferenceDisparity);
		void SetDisparityOutputFile(const std::string& outputDisparityFileBaseName, const std::string& outputDisparityFileExtension);
		void SetCloudOutputFile(const std::string& outputCloudFileBaseName, const std::string& outputCloudFileExtension);

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

		CDFF::DFN::StereoReconstructionInterface* reconstructor;
		void ReadImagesList(bool useReferenceDisparity);
		void SetReferenceDisparity(std::string referenceDisparityFilePath);

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
