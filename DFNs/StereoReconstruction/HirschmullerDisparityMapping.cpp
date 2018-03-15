/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.cpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the HirschmullerDisparityMapping class.
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
#include "HirschmullerDisparityMapping.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include "opencv2/calib3d.hpp"
#include <SupportTypes.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Eigen/Geometry>
#include <FrameToMatConverter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
HirschmullerDisparityMapping::HirschmullerDisparityMapping()
	{
	parametersHelper.AddParameter<float>("ReconstructionSpace", "LimitX", parameters.reconstructionSpace.limitX, DEFAULT_PARAMETERS.reconstructionSpace.limitX);
	parametersHelper.AddParameter<float>("ReconstructionSpace", "LimitY", parameters.reconstructionSpace.limitY, DEFAULT_PARAMETERS.reconstructionSpace.limitY);
	parametersHelper.AddParameter<float>("ReconstructionSpace", "LimitZ", parameters.reconstructionSpace.limitZ, DEFAULT_PARAMETERS.reconstructionSpace.limitZ);

	parametersHelper.AddParameter<int>("Prefilter", "Maximum", parameters.prefilter.maximum, DEFAULT_PARAMETERS.prefilter.maximum);

	parametersHelper.AddParameter<int>("Disparities", "Minimum", parameters.disparities.minimum, DEFAULT_PARAMETERS.disparities.minimum);
	parametersHelper.AddParameter<int>("Disparities", "NumberOfIntervals", parameters.disparities.numberOfIntervals, DEFAULT_PARAMETERS.disparities.numberOfIntervals);
	parametersHelper.AddParameter<bool>("Disparities", "UseMaximumDifference", parameters.disparities.useMaximumDifference, DEFAULT_PARAMETERS.disparities.useMaximumDifference);
	parametersHelper.AddParameter<int>("Disparities", "MaximumDifference", parameters.disparities.maximumDifference, DEFAULT_PARAMETERS.disparities.maximumDifference);
	parametersHelper.AddParameter<int>("Disparities", "SpeckleRange", parameters.disparities.speckleRange, DEFAULT_PARAMETERS.disparities.speckleRange);
	parametersHelper.AddParameter<int>("Disparities", "SpeckleWindow", parameters.disparities.speckleWindow, DEFAULT_PARAMETERS.disparities.speckleWindow);
	parametersHelper.AddParameter<int>("Disparities", "SmoothnessParameter1", parameters.disparities.smoothnessParameter1, DEFAULT_PARAMETERS.disparities.smoothnessParameter1);
	parametersHelper.AddParameter<int>("Disparities", "SmoothnessParameter2", parameters.disparities.smoothnessParameter2, DEFAULT_PARAMETERS.disparities.smoothnessParameter2);

	parametersHelper.AddParameter<int>("BlocksMatching", "BlockSize", parameters.blocksMatching.blockSize, DEFAULT_PARAMETERS.blocksMatching.blockSize);
	parametersHelper.AddParameter<int>("BlocksMatching", "UniquenessRatio", parameters.blocksMatching.uniquenessRatio, DEFAULT_PARAMETERS.blocksMatching.uniquenessRatio);

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudSamplingDensity", parameters.pointCloudSamplingDensity, DEFAULT_PARAMETERS.pointCloudSamplingDensity);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseFullScaleTwoPassAlgorithm", parameters.useFullScaleTwoPassAlgorithm, DEFAULT_PARAMETERS.useFullScaleTwoPassAlgorithm);

	for(unsigned row = 0; row < 4; row++)
		{
		for (unsigned column = 0; column < 4; column++)
			{
			std::stringstream elementStream;
			elementStream << "Element_" << row <<"_"<< column;
			parametersHelper.AddParameter<double>("DisparityToDepthMap",elementStream.str(), parameters.disparityToDepthMap[4*row+column], DEFAULT_PARAMETERS.disparityToDepthMap[4*row+column]);
			}
		}

	disparityToDepthMap = Convert(parameters.disparityToDepthMap);
	configurationFilePath = "";
	}

HirschmullerDisparityMapping::~HirschmullerDisparityMapping()
	{

	}

void HirschmullerDisparityMapping::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	disparityToDepthMap = Convert(parameters.disparityToDepthMap);
	}

void HirschmullerDisparityMapping::process() 
	{
	cv::Mat inputLeftImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inLeftImage);
	cv::Mat inputRightImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inRightImage);

	cv::Mat pointCloud = ComputePointCloud(inputLeftImage, inputRightImage);
	outPointCloud = Convert(pointCloud);
	}

const HirschmullerDisparityMapping::HirschmullerDisparityMappingOptionsSet HirschmullerDisparityMapping::DEFAULT_PARAMETERS =
	{
	.reconstructionSpace =
		{
		.limitX = 20,
		.limitY = 20,
		.limitZ = 40
		},
	.prefilter =
		{
		.maximum = 31
		},
	.disparities = 
		{
		.minimum = 0,
		.numberOfIntervals = 64,
		.useMaximumDifference = false,
		.maximumDifference = -1,
		.speckleRange = 0,
		.speckleWindow = 0,
		.smoothnessParameter1 = 0,
		.smoothnessParameter2 = 0
		},
	.blocksMatching =
		{
		.blockSize = 21,
		.uniquenessRatio = 15
		},
	.disparityToDepthMap = 
		{
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
		},
	.pointCloudSamplingDensity = 1,
	.useFullScaleTwoPassAlgorithm = false
	};

cv::Mat HirschmullerDisparityMapping::ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage)
	{	
	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create
		(
		parameters.disparities.minimum, 
		parameters.disparities.numberOfIntervals,
		parameters.blocksMatching.blockSize,
		parameters.disparities.smoothnessParameter1,
		parameters.disparities.smoothnessParameter2,
		(parameters.disparities.useMaximumDifference ? parameters.disparities.maximumDifference : -1),
		parameters.prefilter.maximum,
		parameters.blocksMatching.uniquenessRatio,
		parameters.disparities.speckleWindow,
		parameters.disparities.speckleRange,
		parameters.useFullScaleTwoPassAlgorithm
		);

	cv::Mat greyLeftImage, greyRightImage;
	cv::cvtColor(leftImage, greyLeftImage, CV_BGR2GRAY);
	cv::cvtColor(rightImage, greyRightImage, CV_BGR2GRAY);

	cv::Mat disparity;
	stereo->compute(greyLeftImage, greyRightImage, disparity);

	cv::Mat pointCloud;	
	cv::reprojectImageTo3D(disparity, pointCloud, disparityToDepthMap);

	return pointCloud;
	}

PointCloudConstPtr HirschmullerDisparityMapping::Convert(cv::Mat cvPointCloud)
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

cv::Mat HirschmullerDisparityMapping::Convert(DisparityToDepthMap disparityToDepthMap)
	{
	cv::Mat conversion(4, 4, CV_64FC1);
	
	for(unsigned row = 0; row < 4; row++)
		{
		for (unsigned column = 0; column < 4; column++)
			{
			conversion.at<double>(row, column) = disparityToDepthMap[4*row + column];
			}
		}

	return conversion;
	}

void HirschmullerDisparityMapping::ValidateParameters()
	{
	ASSERT(parameters.disparities.numberOfIntervals % 16 == 0, "HirschmullerDisparityMapping Configuration Error: number of disparities needs to be multiple of 16");
	ASSERT(parameters.blocksMatching.blockSize % 2 == 1, "HirschmullerDisparityMapping Configuration Error: blockSize needs to be odd");
	ASSERT(!parameters.disparities.useMaximumDifference || parameters.disparities.maximumDifference >= 0, "DisparityMapping Configuration, maximum disparity difference used but not set positive");
	ASSERT(parameters.pointCloudSamplingDensity > 0 && parameters.pointCloudSamplingDensity <= 1, "DisparityMapping Configuration Error: pointCloudSamplingDensity has to be in the set (0, 1]");
	ASSERT( (parameters.disparities.smoothnessParameter2 > parameters.disparities.smoothnessParameter1) ||
		(parameters.disparities.smoothnessParameter2 == 0 && parameters.disparities.smoothnessParameter1 == 0), 
		"HirschmullerDisparityMapping Configuration Error: SmoothnessParameters2 has to be greater than SmoothnessParameters1 or both parameters need to be zero");
	ASSERT( parameters.reconstructionSpace.limitX > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.reconstructionSpace.limitY > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.reconstructionSpace.limitZ > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	}


}


/** @} */
