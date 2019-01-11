/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECONSTRUCTION_HIRSCHMULLERDISPARITYMAPPING_HPP
#define STEREORECONSTRUCTION_HIRSCHMULLERDISPARITYMAPPING_HPP

#include "StereoReconstructionInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>

namespace CDFF
{
namespace DFN
{
namespace StereoReconstruction
{
	/**
	 * Scene reconstruction (as a 3D pointcloud) from 2D stereo images, using
	 * the Hirschmueller disparity mapping algorithm.
	 *
	 * Processing steps: (i) grayscaling of the images, (ii) computation of the
	 * Hirschmueller disparity map, (iii) scene reconstruction based on the
	 * disparity map using a reprojection algorithm, (iv) downsampling of the
	 * generated pointcloud.
	 *
	 * @param prefilter.maximum
	 *
	 * @param disparities.minimum
	 *        disparity threshold
	 * @param disparities.numberOfIntervals
	 *        number of detected disparity intervals
	 * @param disparities.useMaximumDifference
	 * @param disparities.maximumDifference
	 * @param disparities.speckleRange
	 * @param disparities.speckleWindow
	 * @param disparities.smoothnessParameter1
	 * @param disparities.smoothnessParameter2
	 *
	 * @param blocksMatching.blockSize
	 *        dimension of the pixel blocks that are matched during disparity
	 *        computation, must be odd and larger than or equal to 5
	 * @param blocksMatching.uniquenessRatio
	 *
	 * @param useFullScaleTwoPassAlgorithm
	 *
	 * @param pointCloudSamplingDensity
	 *        downsampling ratio: a number between 0 and 1 that describes how
	 *        much downsampling of the generated pointcloud is desired. The
	 *        pointcloud is subsampled at positions that are multiples of n,
	 *        where n = 1/pointCloudSamplingDensity.
	 *
	 * @param useDisparityToDepthMap
	 *        defines whether the camera parameters are provided as a disparity-
	 *        to-depth matrix or as the focal length, principle points, and
	 *        baseline
	 *
	 * @param disparityToDepthMap
	 *        camera parameters in the form of a 4-by-4 disparity-to-depth
	 *        matrix: provide the elements of this matrix via parameters called
	 *        Element_X_Y, where X and Y are between 0 and 3
	 *
	 * @param stereoCameraParameters
	 *        camera parameters in the form of the focal length and principal
	 *        point of the left camera and the distance between the two cameras:
	 *        the parameters to use to provide this information are called
	 *        LeftFocalLength, LeftPrinciplePointX, LeftPrinciplePointY, and
	 *        Baseline, respectively
	 *
	 * @param reconstructionSpace
	 *        a bounding box for the reconstructed scene, provided via
	 *        parameters called LimitX, LimitY, LimitZ. A reconstructed point
	 *        of coordinates (x,y,z) is accepted into the pointcloud if
	 *        -LimitX <= x <= LimitX, -LimitY <= y <= LimitY, 0 < z <= LimitZ.
	 *
	 * @reference The algorithm is adapted from Heiko Hirschmueller (2008),
	 *            "Stereo Processing by Semiglobal Matching and Mutual
	 *            Information", IEEE Transactions on Pattern Analysis and
	 *            Machine Intelligence, 30(2), 328-341.
	 */
	class HirschmullerDisparityMapping : public StereoReconstructionInterface
	{
		public:

			HirschmullerDisparityMapping();
			virtual ~HirschmullerDisparityMapping();

			virtual void configure() override;
			virtual void process() override;

		private:

			static const float EPSILON;

			//DFN Parameters
			struct ReconstructionSpace
			{
				float limitX;
				float limitY;
				float limitZ;
			};

			struct PrefilterOptionsSet
			{
				int maximum;
			};

			struct DisparitiesOptionsSet
			{
				int minimum;
				int numberOfIntervals;
				bool useMaximumDifference;
				int maximumDifference;
				int speckleRange;
				int speckleWindow;
				int smoothnessParameter1;
				int smoothnessParameter2;
			};

			struct BlocksMatchingOptionsSet
			{
				int blockSize;
				int uniquenessRatio;
			};

			struct StereoCameraParameters
			{
				float leftFocalLength;
				float leftPrinciplePointX;
				float leftPrinciplePointY;
				float baseline;
			};

			typedef double DisparityToDepthMap[16];
			struct HirschmullerDisparityMappingOptionsSet
			{
				ReconstructionSpace reconstructionSpace;
				PrefilterOptionsSet prefilter;
				DisparitiesOptionsSet disparities;
				BlocksMatchingOptionsSet blocksMatching;
				DisparityToDepthMap disparityToDepthMap;
				float pointCloudSamplingDensity;
				float voxelGridLeafSize;
				bool useFullScaleTwoPassAlgorithm;
				bool useDisparityToDepthMap;
				StereoCameraParameters stereoCameraParameters;
			};

			Helpers::ParametersListHelper parametersHelper;
			HirschmullerDisparityMappingOptionsSet parameters;
			static const HirschmullerDisparityMappingOptionsSet DEFAULT_PARAMETERS;

			//Parameters Conversion
			cv::Mat disparityToDepthMap;
			cv::Mat Convert(DisparityToDepthMap disparityToDepthMap);

			//External conversion helpers
			Converters::FrameToMatConverter frameToMat;

			//Type conversion methods
			PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat cvPointCloud);
			PointCloudWrapper::PointCloudConstPtr ConvertWithPeriodicSampling(cv::Mat cvPointCloud);
			PointCloudWrapper::PointCloudConstPtr ConvertWithVoxelFilter(cv::Mat cvPointCloud);

			//Core computation methods
			cv::Mat ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage);
			cv::Mat ComputePointCloudFromDisparity(cv::Mat disparity);

			//Input Validation methods
			void ValidateParameters();

			//Testing methods for visualizing intermediate disparity map output.
			#ifdef TESTING
				#define SAVE_DISPARITY_MATRIX(disparity) disparityMatrix = disparity
			#else
				#define SAVE_DISPARITY_MATRIX(disparity)
			#endif
	};
}
}
}

#endif // STEREORECONSTRUCTION_HIRSCHMULLERDISPARITYMAPPING_HPP

/** @} */
