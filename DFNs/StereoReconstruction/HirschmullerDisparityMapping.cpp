/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "HirschmullerDisparityMapping.hpp"

#include <Visualizers/OpencvVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sstream>

using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace StereoReconstruction
{

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
	parametersHelper.AddParameter<float>("GeneralParameters", "VoxelGridLeafSize", parameters.voxelGridLeafSize, DEFAULT_PARAMETERS.voxelGridLeafSize);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseFullScaleTwoPassAlgorithm", parameters.useFullScaleTwoPassAlgorithm, DEFAULT_PARAMETERS.useFullScaleTwoPassAlgorithm);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseDisparityToDepthMap", parameters.useDisparityToDepthMap, DEFAULT_PARAMETERS.useDisparityToDepthMap);

	for (unsigned row = 0; row < 4; row++)
	{
		for (unsigned column = 0; column < 4; column++)
		{
			std::stringstream elementStream;
			elementStream << "Element_" << row <<"_"<< column;
			parametersHelper.AddParameter<double>("DisparityToDepthMap",elementStream.str(), parameters.disparityToDepthMap[4*row+column], DEFAULT_PARAMETERS.disparityToDepthMap[4*row+column]);
		}
	}

	parametersHelper.AddParameter<float>("StereoCamera", "LeftFocalLength", parameters.stereoCameraParameters.leftFocalLength, DEFAULT_PARAMETERS.stereoCameraParameters.leftFocalLength);
	parametersHelper.AddParameter<float>("StereoCamera", "LeftPrinciplePointX", parameters.stereoCameraParameters.leftPrinciplePointX, DEFAULT_PARAMETERS.stereoCameraParameters.leftPrinciplePointX);
	parametersHelper.AddParameter<float>("StereoCamera", "LeftPrinciplePointY", parameters.stereoCameraParameters.leftPrinciplePointY, DEFAULT_PARAMETERS.stereoCameraParameters.leftPrinciplePointY);
	parametersHelper.AddParameter<float>("StereoCamera", "Baseline", parameters.stereoCameraParameters.baseline, DEFAULT_PARAMETERS.stereoCameraParameters.baseline);

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
	// Read data from input ports
	cv::Mat leftImage = frameToMat.Convert(&inLeft);
	cv::Mat rightImage = frameToMat.Convert(&inRight);

	// Process data
	cv::Mat pointcloud = ComputePointCloud(leftImage, rightImage);

	// Write data to output port
	const PointCloud* tmp = Convert(pointcloud);
	Copy(*tmp, outPointcloud);
	delete tmp;
}

const float HirschmullerDisparityMapping::EPSILON = 0.0001;

const HirschmullerDisparityMapping::HirschmullerDisparityMappingOptionsSet HirschmullerDisparityMapping::DEFAULT_PARAMETERS =
{
	.reconstructionSpace =
	{
		.limitX = 20,
		.limitY = 20,
		.limitZ = 10
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
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 0, 1,
		0, 0, -1, 0
	},
	.pointCloudSamplingDensity = 1,
	.voxelGridLeafSize = 0,
	.useFullScaleTwoPassAlgorithm = false,
	.useDisparityToDepthMap = false,
	.stereoCameraParameters =
	{
		.leftFocalLength = 1,
		.leftPrinciplePointX = 0,
		.leftPrinciplePointY = 0,
		.baseline = 1
	}
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
	DEBUG_SHOW_DISPARITY(disparity);
	SAVE_DISPARITY_MATRIX(disparity);

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

PointCloudConstPtr HirschmullerDisparityMapping::Convert(cv::Mat cvPointCloud)
{
	if (parameters.pointCloudSamplingDensity > EPSILON)
	{
		return ConvertWithPeriodicSampling(cvPointCloud);
	}
	else
	{
		return ConvertWithVoxelFilter(cvPointCloud);
	}
}

PointCloudConstPtr HirschmullerDisparityMapping::ConvertWithPeriodicSampling(cv::Mat cvPointCloud)
{
	PointCloudPtr pointCloud = new PointCloud();

	unsigned validPointCount = 0;
	unsigned pickUpPeriod = static_cast<unsigned>(1 / parameters.pointCloudSamplingDensity) ;
	for (unsigned row = 0; row < cvPointCloud.rows; row++)
	{
		for (unsigned column = 0; column < cvPointCloud.cols; column++)
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

PointCloudConstPtr HirschmullerDisparityMapping::ConvertWithVoxelFilter(cv::Mat cvPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (unsigned row = 0; row < cvPointCloud.rows; row++)
	{
		for (unsigned column = 0; column < cvPointCloud.cols; column++)
		{
			cv::Vec3f point = cvPointCloud.at<cv::Vec3f>(row, column);

			bool validPoint = (point[0] == point[0] && point[1] == point[1] && point[2] == point[2]);
			validPoint = validPoint && ( std::abs(point[0]) <= parameters.reconstructionSpace.limitX ) && ( std::abs(point[1]) <= parameters.reconstructionSpace.limitY );
			validPoint = validPoint && ( point[2] >= 0 ) && ( point[2] <= parameters.reconstructionSpace.limitZ );
			if (validPoint)
			{
				pcl::PointXYZ newPoint(point[0], point[1], point[2]);
				pclPointCloud->points.push_back(newPoint);
			}
		}
	}

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(pclPointCloud);
	grid.setLeafSize(parameters.voxelGridLeafSize, parameters.voxelGridLeafSize, parameters.voxelGridLeafSize);
	grid.filter(*pclPointCloud);

	PointCloudPtr pointCloud = new PointCloud();
	for (unsigned pointIndex = 0; pointIndex < pclPointCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ& point = pclPointCloud->points.at(pointIndex);
		AddPoint(*pointCloud, point.x, point.y, point.z);
	}

	return pointCloud;
}

cv::Mat HirschmullerDisparityMapping::Convert(DisparityToDepthMap disparityToDepthMap)
{
	cv::Mat conversion(4, 4, CV_64FC1);

	for (unsigned row = 0; row < 4; row++)
	{
		for (unsigned column = 0; column < 4; column++)
		{
			conversion.at<double>(row, column) = disparityToDepthMap[4*row + column];
		}
	}

	return conversion;
}

/**
 * This algorithm comes from the PCL library, file /stereo/src/stereo_matching.cpp
 */
cv::Mat HirschmullerDisparityMapping::ComputePointCloudFromDisparity(cv::Mat disparity)
{
	float principlePointX = parameters.stereoCameraParameters.leftPrinciplePointX;
	float principlePointY = parameters.stereoCameraParameters.leftPrinciplePointY;
	float baseline = parameters.stereoCameraParameters.baseline;
	float focalLength = parameters.stereoCameraParameters.leftFocalLength;

	float depthScale = baseline * focalLength;
	cv::Mat pointCloud(disparity.rows, disparity.cols, CV_32FC3, cv::Scalar(0,0,0));
	for (unsigned row = 0; row < disparity.rows; row++)
	{
		for (unsigned column = 0; column < disparity.cols; column++)
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

void HirschmullerDisparityMapping::ValidateParameters()
{
	ASSERT(parameters.disparities.numberOfIntervals % 16 == 0, "HirschmullerDisparityMapping Configuration Error: number of disparities needs to be multiple of 16");
	ASSERT(parameters.blocksMatching.blockSize % 2 == 1, "HirschmullerDisparityMapping Configuration Error: blockSize needs to be odd");
	ASSERT(!parameters.disparities.useMaximumDifference || parameters.disparities.maximumDifference >= 0, "DisparityMapping Configuration, maximum disparity difference used but not set positive");
	ASSERT( (parameters.disparities.smoothnessParameter2 > parameters.disparities.smoothnessParameter1) ||
		(parameters.disparities.smoothnessParameter2 == 0 && parameters.disparities.smoothnessParameter1 == 0),
		"HirschmullerDisparityMapping Configuration Error: SmoothnessParameters2 has to be greater than SmoothnessParameters1 or both parameters need to be zero");
	ASSERT( parameters.reconstructionSpace.limitX > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.reconstructionSpace.limitY > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.reconstructionSpace.limitZ > 0, "DisparityMapping Configuration Error: Limits for reconstruction space have to be positive");
	ASSERT( parameters.stereoCameraParameters.leftFocalLength > 0, "DisparityMapping Configuration Error: Focal Length has to be positive");
	ASSERT( parameters.stereoCameraParameters.baseline > 0, "DisparityMapping Configuration Error: Baseline has to be positive");

	ASSERT( (parameters.voxelGridLeafSize > EPSILON && parameters.pointCloudSamplingDensity <= EPSILON) ||
		(parameters.voxelGridLeafSize <= EPSILON && parameters.pointCloudSamplingDensity > EPSILON),
		"DisparityMapping Configuration Error: Only one between voxelGridLeafSize and pointCloudSamplingDensity can be greater than 0");
	ASSERT(parameters.pointCloudSamplingDensity <= 1, "DisparityMapping Configuration Error: pointCloudSamplingDensity has to be in the set (0, 1]");
}

}
}
}

/** @} */
