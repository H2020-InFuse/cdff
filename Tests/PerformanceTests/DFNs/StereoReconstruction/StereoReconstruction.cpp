/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoReconstruction.cpp
 * @date 25/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Implementation of the Test interface for the performance test of DFN Stereo Reconstruction
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "StereoReconstruction.hpp"

using namespace CDFF::DFN::StereoReconstruction;
using namespace Converters;
using namespace FrameWrapper;
using namespace PointCloudWrapper;


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
StereoReconstructionTestInterface::StereoReconstructionTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName, 
	StereoReconstructionInterface* reconstructor) : PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	this->reconstructor = reconstructor;
	SetDfn(this->reconstructor);

	baseFolderPath = "../tests/Data/Images";
	
	std::string uniqueLeftImage = "RectifiedChair40Left.png";
	std::string uniqueRightImage = "RectifiedChair40Right.png";
	std::string uniqueDisparityImage = "chairDepth40.yml";

	leftImagesNameList.push_back(uniqueLeftImage);
	rightImagesNameList.push_back(uniqueRightImage);
	disparityImagesNameList.push_back(uniqueDisparityImage);

	outputDisparityFileBaseName = "../../ConfigurationFiles/DFNs/StereoReconstruction/Test_";
	outputDisparityFileExtension = ".jpg";

	useReferenceDisparity = true;
	saveOutputDisparity = true;
	saveOutputCloud = false;
	}

StereoReconstructionTestInterface::~StereoReconstructionTestInterface()
	{

	}

void StereoReconstructionTestInterface::SetImageFilesPath(std::string baseFolderPath, std::string imagesListFileName, bool useReferenceDisparity)
	{
	this->baseFolderPath = baseFolderPath;
	this->imagesListFileName = imagesListFileName;
	ReadImagesList(useReferenceDisparity);
	this->useReferenceDisparity = useReferenceDisparity;
	ASSERT( !useReferenceDisparity || disparityImagesNameList.size() == leftImagesNameList.size(), "Reference disparity required, but no disparity files in ImagesList");
	}

void StereoReconstructionTestInterface::SetDisparityOutputFile(std::string outputDisparityFileBaseName, std::string outputCloudFileExtension)
	{
	saveOutputDisparity = true;
	this->outputDisparityFileBaseName = outputDisparityFileBaseName;
	this->outputCloudFileExtension = outputCloudFileExtension;
	}

void StereoReconstructionTestInterface::SetCloudOutputFile(std::string outputCloudFileBaseName, std::string outputCloudFileExtension)
	{
	saveOutputCloud = true;
	this->outputCloudFileBaseName = outputCloudFileBaseName;
	this->outputCloudFileExtension = outputCloudFileExtension;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void StereoReconstructionTestInterface::ReadImagesList(bool useReferenceDisparity)
	{
	leftImagesNameList.clear();
	rightImagesNameList.clear();
	disparityImagesNameList.clear();

	std::stringstream imagesListFilePath;
	imagesListFilePath << baseFolderPath << "/" << imagesListFileName;
	std::ifstream containerFile(imagesListFilePath.str().c_str());

	std::string line;
	while (std::getline(containerFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		ASSERT(stringsList.size() >= 2, "Error reading file, bad line");
		
		leftImagesNameList.push_back( std::string(stringsList.at(0)) );
		rightImagesNameList.push_back( std::string(stringsList.at(1)) );
		if (useReferenceDisparity)
			{
			ASSERT(stringsList.size() >= 3, "Error reading file, bad line with no required reference disparity");
			disparityImagesNameList.push_back( std::string(stringsList.at(2)) );
			}
		}

	containerFile.close();
	}

void StereoReconstructionTestInterface::SetReferenceDisparity(std::string referenceDisparityFilePath)
	{
	cv::FileStorage file(referenceDisparityFilePath, cv::FileStorage::READ);
	file["depth"] >> referenceDisparity;

	cv::normalize(referenceDisparity, normalizedReferenceDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	std::stringstream stream;
	stream << "../tests/ConfigurationFiles/DFNs/StereoReconstruction/ReferenceDisparity.jpg"; 
	cv::imwrite(stream.str(), normalizedReferenceDisparity);
	}

bool StereoReconstructionTestInterface::SetNextInputs()
	{
	static unsigned time = 0;

	if (time < leftImagesNameList.size())
		{
		std::stringstream leftImageFilePath, rightImageFilePath;
		leftImageFilePath << baseFolderPath << "/" << leftImagesNameList.at(time);
		rightImageFilePath << baseFolderPath << "/" << rightImagesNameList.at(time);

		cv::Mat cvLeftImage = cv::imread(leftImageFilePath.str(), cv::IMREAD_COLOR);
		cv::Mat cvRightImage = cv::imread(rightImageFilePath.str(), cv::IMREAD_COLOR);
		if (useReferenceDisparity)
			{
			std::stringstream disparityFilePath;
			disparityFilePath << baseFolderPath << "/" << disparityImagesNameList.at(time);
			SetReferenceDisparity(disparityFilePath.str());
			}

		ASSERT( cvLeftImage.cols > 0 && cvLeftImage.rows > 0, "Performance Test Error: bad input images");
		ASSERT( cvLeftImage.size() == cvRightImage.size(), "Performance Test Error: input images do not have same size");
		ASSERT(!useReferenceDisparity || referenceDisparity.size() == cvRightImage.size(), "Performance Test Error: reference disparity does not have same size as input images");

		MatToFrameConverter converter;
		FrameConstPtr leftFrame = converter.Convert(cvLeftImage);
		FrameConstPtr rightFrame = converter.Convert(cvRightImage);

		reconstructor->leftInput(*leftFrame);
		reconstructor->rightInput(*rightFrame);

		time++;
		return true;
		}
	
	return false;
	}

StereoReconstructionTestInterface::MeasuresMap StereoReconstructionTestInterface::ExtractMeasures()
	{
	//testId has to start from 1 to match the inputId as saved in the output file.
	static unsigned testId = 0;
	testId++;

	MeasuresMap measuresMap;
	cv::Mat disparity = reconstructor->disparityMatrixOutput(); //Disparity type is CV_16SC1
	cv::Mat normalizedDisparity;
	cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	/** The output disparity has always a black border on the left side of the image, here we compute the size of the black border as well as its ending column **/
	unsigned firstValidColumn, numberOfValidColumns;
	ComputeValidDisparityColumns(normalizedDisparity, firstValidColumn, numberOfValidColumns);

	/** Sometimes completely white images are in the output, this loop determines whether the image is completely white**/
	measuresMap["OutputQuality"] = IsBadDisparity(normalizedDisparity, firstValidColumn) ? 0 : 1;	

	if (useReferenceDisparity)
		{
		measuresMap["DisparityCost"] = ComputeDisparityCost(normalizedDisparity, firstValidColumn, numberOfValidColumns);
		}

	if (saveOutputDisparity)
		{
		SaveOutputDisparity(normalizedDisparity, testId);
		}

	PointCloudPtr outputCloud = NewPointCloud();
	Copy( reconstructor->pointcloudOutput(), *outputCloud);
	if (saveOutputCloud)
		{
		SaveOutputCloud(outputCloud, testId);
		}
	delete(outputCloud);

	return measuresMap;
	}

void StereoReconstructionTestInterface::ComputeValidDisparityColumns(const cv::Mat& normalizedDisparity, unsigned& firstValidColumn, unsigned& numberOfValidColumns)
	{
	bool stop = false;
	for(unsigned column = 0; column < normalizedDisparity.cols && !stop; column++)
		{
		for(unsigned row = 0; row < normalizedDisparity.rows; row++)
			{
			stop = normalizedDisparity.at<uint8_t>(row, column) > 0;
			}
		if (stop)
			{
			firstValidColumn = column;
			}
		}
	numberOfValidColumns = normalizedDisparity.cols - firstValidColumn;
	}

bool StereoReconstructionTestInterface::IsBadDisparity(const cv::Mat& normalizedDisparity, unsigned firstValidColumn)
	{
	bool badOutput = true;
	for(unsigned column = firstValidColumn; column < normalizedDisparity.cols && badOutput; column++)
		{
		for(unsigned row = 0; row < normalizedDisparity.rows && badOutput; row++)
			{
			badOutput = normalizedDisparity.at<uint8_t>(row, column) == 255;
			}
		}
	return badOutput;
	}

double StereoReconstructionTestInterface::ComputeDisparityCost(const cv::Mat& normalizedDisparity, unsigned firstValidColumn, unsigned numberOfValidColumns)
	{
	//This loop computes the difference between the computed disparity and the reference disparity after normalization of both disparities in the interval [0, 255]
	uint64_t cost = 0;
	unsigned pixelsNumber = 0;
	for(unsigned row = 0; row < normalizedReferenceDisparity.rows; row++)
		{
		for(unsigned column = 0; column < normalizedReferenceDisparity.cols; column++)
			{
			uint8_t referenceDisparityValue = normalizedReferenceDisparity.at<uint8_t>(row, column);
			if (referenceDisparityValue > 0)
				{
				unsigned validColumn = firstValidColumn + (column * numberOfValidColumns / normalizedReferenceDisparity.cols );
				uint8_t disparityValue = normalizedDisparity.at<uint8_t>(row, validColumn);
				cost += std::abs(static_cast<int16_t>(disparityValue) - static_cast<int16_t>(255 - referenceDisparityValue));	
				pixelsNumber++;
				}
			}
		}
	return static_cast<double>(cost) / static_cast<double>(pixelsNumber);
	}

void StereoReconstructionTestInterface::SaveOutputDisparity(const cv::Mat& disparity, unsigned testId)
	{
	std::stringstream disparityOutputPath;
	disparityOutputPath << baseFolderPath << "/" << outputDisparityFileBaseName << testId << outputDisparityFileExtension; 
	cv::imwrite(disparityOutputPath.str(), disparity);
	}

void StereoReconstructionTestInterface::SaveOutputCloud(PointCloudWrapper::PointCloudConstPtr pointCloud, unsigned testId)
	{
	std::stringstream cloudOutputPath;
	cloudOutputPath << baseFolderPath << "/" << outputCloudFileBaseName << testId << outputCloudFileExtension; 

	PointCloudToPclPointCloudConverter pclConverter;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclCloud = pclConverter.Convert(pointCloud);

	pcl::PLYWriter writer;
	writer.write(cloudOutputPath.str(), *pclCloud, true);
	}

/** @} */
