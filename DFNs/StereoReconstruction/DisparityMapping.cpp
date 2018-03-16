/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DisparityMapping.cpp
 * @date 08/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the DisparityMapping class.
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
#include "DisparityMapping.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include "opencv2/calib3d.hpp"
#include <SupportTypes.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Eigen/Geometry>
#include <FrameToMatConverter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Visualizers/OpencvVisualizer.hpp>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace FrameWrapper;
using namespace SupportTypes;
using namespace BaseTypesWrapper;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DisparityMapping::DisparityMapping()
	{
	parametersHelper.AddParameter<float>("ReconstructionSpace", "LimitX", parameters.reconstructionSpace.limitX, DEFAULT_PARAMETERS.reconstructionSpace.limitX);
	parametersHelper.AddParameter<float>("ReconstructionSpace", "LimitY", parameters.reconstructionSpace.limitY, DEFAULT_PARAMETERS.reconstructionSpace.limitY);
	parametersHelper.AddParameter<float>("ReconstructionSpace", "LimitZ", parameters.reconstructionSpace.limitZ, DEFAULT_PARAMETERS.reconstructionSpace.limitZ);

	parametersHelper.AddParameter<int>("Prefilter", "Size", parameters.prefilter.size, DEFAULT_PARAMETERS.prefilter.size);
	parametersHelper.AddParameter<PrefilterType, PrefilterTypeHelper>("Prefilter", "Type", parameters.prefilter.type, DEFAULT_PARAMETERS.prefilter.type);
	parametersHelper.AddParameter<int>("Prefilter", "Maximum", parameters.prefilter.maximum, DEFAULT_PARAMETERS.prefilter.maximum);

	parametersHelper.AddParameter<int>("Disparities", "Minimum", parameters.disparities.minimum, DEFAULT_PARAMETERS.disparities.minimum);
	parametersHelper.AddParameter<int>("Disparities", "NumberOfIntervals", parameters.disparities.numberOfIntervals, DEFAULT_PARAMETERS.disparities.numberOfIntervals);
	parametersHelper.AddParameter<bool>("Disparities", "UseMaximumDifference", parameters.disparities.useMaximumDifference, DEFAULT_PARAMETERS.disparities.useMaximumDifference);
	parametersHelper.AddParameter<int>("Disparities", "MaximumDifference", parameters.disparities.maximumDifference, DEFAULT_PARAMETERS.disparities.maximumDifference);
	parametersHelper.AddParameter<int>("Disparities", "SpeckleRange", parameters.disparities.speckleRange, DEFAULT_PARAMETERS.disparities.speckleRange);
	parametersHelper.AddParameter<int>("Disparities", "SpeckleWindow", parameters.disparities.speckleWindow, DEFAULT_PARAMETERS.disparities.speckleWindow);

	parametersHelper.AddParameter<int>("FirstRegionOfInterest", "TopLeftColumn", parameters.firstRegionOfInterest.topLeftColumn, DEFAULT_PARAMETERS.firstRegionOfInterest.topLeftColumn);
	parametersHelper.AddParameter<int>("FirstRegionOfInterest", "TopLeftRow", parameters.firstRegionOfInterest.topLeftRow, DEFAULT_PARAMETERS.firstRegionOfInterest.topLeftRow);
	parametersHelper.AddParameter<int>("FirstRegionOfInterest", "NumberOfColumns", parameters.firstRegionOfInterest.numberOfColumns, DEFAULT_PARAMETERS.firstRegionOfInterest.numberOfColumns);
	parametersHelper.AddParameter<int>("FirstRegionOfInterest", "NumberOfRows", parameters.firstRegionOfInterest.numberOfRows, DEFAULT_PARAMETERS.firstRegionOfInterest.numberOfRows);

	parametersHelper.AddParameter<int>("SecondRegionOfInterest", "TopLeftColumn", parameters.secondRegionOfInterest.topLeftColumn, DEFAULT_PARAMETERS.secondRegionOfInterest.topLeftColumn);
	parametersHelper.AddParameter<int>("SecondRegionOfInterest", "TopLeftRow", parameters.secondRegionOfInterest.topLeftRow, DEFAULT_PARAMETERS.secondRegionOfInterest.topLeftRow);
	parametersHelper.AddParameter<int>("SecondRegionOfInterest", "NumberOfColumns", parameters.secondRegionOfInterest.numberOfColumns, DEFAULT_PARAMETERS.secondRegionOfInterest.numberOfColumns);
	parametersHelper.AddParameter<int>("SecondRegionOfInterest", "NumberOfRows", parameters.secondRegionOfInterest.numberOfRows, DEFAULT_PARAMETERS.secondRegionOfInterest.numberOfRows);

	parametersHelper.AddParameter<int>("BlocksMatching", "BlockSize", parameters.blocksMatching.blockSize, DEFAULT_PARAMETERS.blocksMatching.blockSize);
	parametersHelper.AddParameter<int>("BlocksMatching", "SmallerBlockSize", parameters.blocksMatching.smallerBlockSize, DEFAULT_PARAMETERS.blocksMatching.smallerBlockSize);
	parametersHelper.AddParameter<int>("BlocksMatching", "TextureThreshold", parameters.blocksMatching.textureThreshold, DEFAULT_PARAMETERS.blocksMatching.textureThreshold);
	parametersHelper.AddParameter<int>("BlocksMatching", "UniquenessRatio", parameters.blocksMatching.uniquenessRatio, DEFAULT_PARAMETERS.blocksMatching.uniquenessRatio);

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudSamplingDensity", parameters.pointCloudSamplingDensity, DEFAULT_PARAMETERS.pointCloudSamplingDensity);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseDisparityToDepthMap", parameters.useDisparityToDepthMap, DEFAULT_PARAMETERS.useDisparityToDepthMap);

	for(unsigned row = 0; row < 4; row++)
		{
		for (unsigned column = 0; column < 4; column++)
			{
			std::stringstream elementStream;
			elementStream << "Element_" << row <<"_"<< column;
			parametersHelper.AddParameter<float>("DisparityToDepthMap", elementStream.str(), parameters.disparityToDepthMap[4*row+column], DEFAULT_PARAMETERS.disparityToDepthMap[4*row+column]);
			}
		}

	parametersHelper.AddParameter<float>("StereoCamera", "LeftFocalLength", parameters.stereoCameraParameters.leftFocalLength, DEFAULT_PARAMETERS.stereoCameraParameters.leftFocalLength);
	parametersHelper.AddParameter<float>("StereoCamera", "LeftPrinciplePointX", parameters.stereoCameraParameters.leftPrinciplePointX, DEFAULT_PARAMETERS.stereoCameraParameters.leftPrinciplePointX);
	parametersHelper.AddParameter<float>("StereoCamera", "LeftPrinciplePointY", parameters.stereoCameraParameters.leftPrinciplePointY, DEFAULT_PARAMETERS.stereoCameraParameters.leftPrinciplePointY);
	parametersHelper.AddParameter<float>("StereoCamera", "Baseline", parameters.stereoCameraParameters.baseline, DEFAULT_PARAMETERS.stereoCameraParameters.baseline);

	disparityToDepthMap = Convert(parameters.disparityToDepthMap);
	configurationFilePath = "";
	}

DisparityMapping::~DisparityMapping()
	{

	}

void DisparityMapping::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	disparityToDepthMap = Convert(parameters.disparityToDepthMap);
	}

void DisparityMapping::process() 
	{
	cv::Mat inputLeftImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inLeftImage);
	cv::Mat inputRightImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inRightImage);

	cv::Mat pointCloud = ComputePointCloud(inputLeftImage, inputRightImage);
	outPointCloud = Convert(pointCloud);
	}

DisparityMapping::PrefilterTypeHelper::PrefilterTypeHelper(const std::string& parameterName, PrefilterType& boundVariable, const PrefilterType& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

DisparityMapping::PrefilterType DisparityMapping::PrefilterTypeHelper::Convert(const std::string& prefilterType)
	{
	if (prefilterType == "NormalizedResponse" || prefilterType == "0")
		{
		return NORMALIZED_RESPONSE;
		}
	else if (prefilterType == "Xsobel" || prefilterType == "1")
		{
		return XSOBEL;
		}
	ASSERT(false, "StereoTriangulation Error: unhandled prefilter type");
	return XSOBEL;
	}

const DisparityMapping::DisparityMappingOptionsSet DisparityMapping::DEFAULT_PARAMETERS =
	{
	.reconstructionSpace =
		{
		.limitX = 20,
		.limitY = 20,
		.limitZ = 40
		},
	.prefilter =
		{
		.size = 9,
		.type = XSOBEL,
		.maximum = 31
		},
	.disparities = 
		{
		.minimum = 0,
		.numberOfIntervals = 64,
		.useMaximumDifference = false,
		.maximumDifference = -1,
		.speckleRange = 0,
		.speckleWindow = 0
		},
	.firstRegionOfInterest =
		{
		.topLeftColumn = 0,
		.topLeftRow = 0,
		.numberOfColumns = 0,
		.numberOfRows = 0
		},
	.secondRegionOfInterest =
		{
		.topLeftColumn = 0,
		.topLeftRow = 0,
		.numberOfColumns = 0,
		.numberOfRows = 0
		},
	.blocksMatching =
		{
		.blockSize = 21,
		.smallerBlockSize = 0,
		.textureThreshold = 10,
		.uniquenessRatio = 15
		},
	.disparityToDepthMap = 
		{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 0, 1,
		0, 0, -1, 0
		},
	.pointCloudSamplingDensity = 1,
	.useDisparityToDepthMap = false,
	.stereoCameraParameters =
		{
		.leftFocalLength = 1,
		.leftPrinciplePointX = 0,
		.leftPrinciplePointY = 0,
		.baseline = 1
		}
	};

cv::Mat DisparityMapping::ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage)
	{
	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(parameters.disparities.numberOfIntervals, parameters.blocksMatching.blockSize);
	stereo->setPreFilterCap(parameters.prefilter.maximum);
	stereo->setPreFilterSize(parameters.prefilter.size);
	stereo->setPreFilterType( parameters.prefilter.type == XSOBEL ? cv::StereoBM::PREFILTER_XSOBEL : cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE );
	stereo->setTextureThreshold( parameters.blocksMatching.textureThreshold );
	stereo->setUniquenessRatio( parameters.blocksMatching.uniquenessRatio );
	stereo->setSmallerBlockSize( parameters.blocksMatching.blockSize );
	stereo->setMinDisparity( parameters.disparities.minimum );
	stereo->setSpeckleRange( parameters.disparities.speckleRange );
	stereo->setSpeckleWindowSize( parameters.disparities.speckleWindow );
	if ( parameters.disparities.useMaximumDifference )
		{
		stereo->setDisp12MaxDiff( parameters.disparities.maximumDifference );
		}
	stereo->setROI1( cv::Rect(parameters.firstRegionOfInterest.topLeftColumn, parameters.firstRegionOfInterest.topLeftRow, 
				parameters.firstRegionOfInterest.numberOfColumns, parameters.firstRegionOfInterest.numberOfRows) );
	stereo->setROI2( cv::Rect(parameters.secondRegionOfInterest.topLeftColumn, parameters.secondRegionOfInterest.topLeftRow, 
				parameters.secondRegionOfInterest.numberOfColumns, parameters.secondRegionOfInterest.numberOfRows) );

	cv::Mat greyLeftImage, greyRightImage;
	cv::cvtColor(leftImage, greyLeftImage, CV_BGR2GRAY);
	cv::cvtColor(rightImage, greyRightImage, CV_BGR2GRAY);

	cv::Mat disparity;
	stereo->compute(greyLeftImage, greyRightImage, disparity);
	DEBUG_SHOW_DISPARITY(disparity);

	cv::Mat pointCloud;
	if (parameters.useDisparityToDepthMap)
		{
		cv::reprojectImageTo3D(disparity, pointCloud, disparityToDepthMap);
		}
	else
		{
		pointCloud = ComputePointCloudFromDisparity(disparity);
		}

	return pointCloud;
	}

PointCloudConstPtr DisparityMapping::Convert(cv::Mat cvPointCloud)
	{
	PointCloudPtr pointCloud = new PointCloud();

	unsigned validPointCount = 0;
	unsigned pickUpPeriod = static_cast<unsigned>(1 / parameters.pointCloudSamplingDensity) ;
	for(unsigned row = 0; row < cvPointCloud.rows; row++)
		{
		for(unsigned column = 0; column < cvPointCloud.cols; column++)
			{
			cv::Vec3f point = cvPointCloud.at<cv::Vec3f>(row, column);

			bool validPoint = (point[0] == point[0] && point[1] == point[1] && point[2] == point[2]);
			validPoint = validPoint && ( std::abs(point[0]) <= parameters.reconstructionSpace.limitX ) && ( std::abs(point[1]) <= parameters.reconstructionSpace.limitY );
			validPoint = validPoint && ( point[2] >= 0 ) && ( point[2] <= parameters.reconstructionSpace.limitZ );
			if (validPoint)
				{
				validPointCount++;
				if (validPointCount % pickUpPeriod == 0)
					{
					AddPoint(*pointCloud, point[0], point[1], point[2]); 
					}
				}
			}
		}

	return pointCloud;
	}

cv::Mat DisparityMapping::Convert(DisparityToDepthMap disparityToDepthMap)
	{
	cv::Mat conversion(4, 4, CV_32FC1);
	
	for(unsigned row = 0; row < 4; row++)
		{
		for (unsigned column = 0; column < 4; column++)
			{
			conversion.at<float>(row, column) = disparityToDepthMap[4*row + column];
			}
		}

	return conversion;
	}

/** This algorithm is taken from PCL library from file /stereo/src/stereo_matching.cpp **/
cv::Mat DisparityMapping::ComputePointCloudFromDisparity(cv::Mat disparity)
	{
	float principlePointX = parameters.stereoCameraParameters.leftPrinciplePointX;
	float principlePointY = parameters.stereoCameraParameters.leftPrinciplePointY;
	float baseline = parameters.stereoCameraParameters.baseline;
	float focalLength = parameters.stereoCameraParameters.leftFocalLength;

	float depthScale = baseline * focalLength * 16;
	cv::Mat pointCloud(disparity.rows, disparity.cols, CV_32FC3, cv::Scalar(0,0,0));
	for(unsigned row = 0; row < disparity.rows; row++)
		{
		for(unsigned column = 0; column < disparity.cols; column++)
			{
			float disparityValue = ( static_cast<float>(disparity.at<int16_t>(row, column)) ) / 16;	
			if (disparityValue > 0)
				{	
				float depth = depthScale / disparityValue;
				pointCloud.at<cv::Vec3f>(row, column)[0] = ((static_cast<float> (column) - principlePointX) * depth) / focalLength;
				pointCloud.at<cv::Vec3f>(row, column)[1] = ((static_cast<float> (row) - principlePointY) * depth) / focalLength;
				pointCloud.at<cv::Vec3f>(row, column)[2] = depth;
				}
			else
				{
				pointCloud.at<cv::Vec3f>(row, column)[0] = std::numeric_limits<float>::quiet_NaN();
				pointCloud.at<cv::Vec3f>(row, column)[1] = std::numeric_limits<float>::quiet_NaN();
				pointCloud.at<cv::Vec3f>(row, column)[2] = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}
	return pointCloud;
	}

void DisparityMapping::ValidateParameters()
	{
	ASSERT(parameters.disparities.numberOfIntervals % 16 == 0, "DisparityMapping Configuration Error: number of disparities needs to be multiple of 16");
	ASSERT(parameters.blocksMatching.blockSize % 2 == 1, "DisparityMapping Configuration Error: blockSize needs to be odd");
	ASSERT(!parameters.disparities.useMaximumDifference || parameters.disparities.maximumDifference >= 0, "DisparityMapping Configuration, maximum disparity difference used but not set positive");
	ASSERT(parameters.pointCloudSamplingDensity > 0 && parameters.pointCloudSamplingDensity <= 1, "DisparityMapping Configuration Error: pointCloudSamplingDensity has to be in the set (0, 1]");
	ASSERT( parameters.reconstructionSpace.limitX > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.reconstructionSpace.limitY > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.reconstructionSpace.limitZ > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");

	ASSERT( parameters.stereoCameraParameters.leftFocalLength > 0, "DisparityMapping Configuration Error: Focal Length has to be positive");
	ASSERT( parameters.stereoCameraParameters.baseline > 0, "DisparityMapping Configuration Error: Baseline has to be positive");
	}


}


/** @} */
