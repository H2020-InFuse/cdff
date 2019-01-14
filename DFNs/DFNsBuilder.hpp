/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DFNS_BUILDER_HPP
#define DFNS_BUILDER_HPP

#include <DFNCommonInterface.hpp>

#include <BundleAdjustment/BundleAdjustmentInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <ColorConversion/ColorConversion.hpp>
#include <DisparityImage/DisparityImageInterface.hpp>
#include <DisparityFiltering/DisparityFilteringInterface.hpp>
#include <DisparityToPointCloud/DisparityToPointCloudInterface.hpp>
#include <DisparityToPointCloudWithIntensity/DisparityToPointCloudWithIntensityInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <ImageDegradation/ImageDegradationInterface.hpp>
#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <ImageRectification/ImageRectificationInterface.hpp>
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <Registration3D/Registration3DInterface.hpp>
#include <StereoDegradation/StereoDegradationInterface.hpp>
#include <StereoMotionEstimation/StereoMotionEstimationInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <StereoRectification/StereoRectificationInterface.hpp>
#include <Transform3DEstimation/Transform3DEstimationInterface.hpp>
#include <PrimitiveMatching/PrimitiveMatchingInterface.hpp>
#include <DepthFiltering/DepthFilteringInterface.hpp>
#include <ForceMeshGenerator/ForceMeshGeneratorInterface.hpp>
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <PointCloudTransform/PointCloudTransformInterface.hpp>
#include <Voxelization/VoxelizationInterface.hpp>
#include <PointCloudFiltering/PointCloudFilteringInterface.hpp>

#include <stdlib.h>
#include <string>

namespace CDFF
{
namespace DFN
{
	/**
	 * This class provides static methods for instantiating DFN implementations
	 * using their names. It has a unique public method, which takes two strings
	 * as input (name of the DFN and name of the DFN's implementation) and
	 * returns a pointer to the DFN instance.
	 */
	class DFNsBuilder
	{
		public:
			static DFNCommonInterface* CreateDFN(const std::string& dfnType, const std::string& dfnImplementation);

		private:
			static BundleAdjustmentInterface* CreateBundleAdjustment(const std::string& dfnImplementation);
			static CamerasTransformEstimationInterface* CreateCamerasTransformEstimation(const std::string& dfnImplementation);
            static ColorConversionInterface* CreateColorConversion(const std::string& dfnImplementation);
            static DisparityImageInterface* CreateDisparityImage(const std::string& dfnImplementation);
            static DisparityFilteringInterface* CreateDisparityFiltering(const std::string& dfnImplementation);
            static DisparityToPointCloudInterface* CreateDisparityToPointCloud(const std::string& dfnImplementation);
            static DisparityToPointCloudWithIntensityInterface* CreateDisparityToPointCloudWithIntensity(const std::string& dfnImplementation);
			static FeaturesDescription2DInterface* CreateFeaturesDescription2D(const std::string& dfnImplementation);
			static FeaturesDescription3DInterface* CreateFeaturesDescription3D(const std::string& dfnImplementation);
			static FeaturesExtraction2DInterface* CreateFeaturesExtraction2D(const std::string& dfnImplementation);
			static FeaturesExtraction3DInterface* CreateFeaturesExtraction3D(const std::string& dfnImplementation);
			static FeaturesMatching2DInterface* CreateFeaturesMatching2D(const std::string& dfnImplementation);
			static FeaturesMatching3DInterface* CreateFeaturesMatching3D(const std::string& dfnImplementation);
			static FundamentalMatrixComputationInterface* CreateFundamentalMatrixComputation(const std::string& dfnImplementation);
            static ImageDegradationInterface* CreateImageDegradation(const std::string& dfnImplementation);
			static ImageFilteringInterface* CreateImageFiltering(const std::string& dfnImplementation);
            static ImageRectificationInterface* CreateImageRectification(const std::string& dfnImplementation);
			static PerspectiveNPointSolvingInterface* CreatePerspectiveNPointSolving(const std::string& dfnImplementation);
			static PointCloudReconstruction2DTo3DInterface* CreatePointCloudReconstruction2DTo3D(const std::string& dfnImplementation);
			static PrimitiveMatchingInterface* CreatePrimitiveMatching(const std::string& dfnImplementation);
			static Registration3DInterface* CreateRegistration3D(const std::string& dfnImplementation);
            static StereoDegradationInterface* CreateStereoDegradation(const std::string& dfnImplementation);
            static StereoMotionEstimationInterface* CreateStereoMotionEstimation(const std::string& dfnImplementation);
			static StereoReconstructionInterface* CreateStereoReconstruction(const std::string& dfnImplementation);
            static StereoRectificationInterface* CreateStereoRectification(const std::string& dfnImplementation);
			static Transform3DEstimationInterface* CreateTransform3DEstimation(const std::string& dfnImplementation);
			static DepthFilteringInterface* CreateDepthFiltering(const std::string& dfnImplementation);
			static ForceMeshGeneratorInterface* CreateForceMeshGenerator(const std::string& dfnImplementation);
			static PointCloudAssemblyInterface* CreatePointCloudAssembly(const std::string& dfnImplementation);
			static PointCloudTransformInterface* CreatePointCloudTransform(const std::string& dfnImplementation);
			static VoxelizationInterface* CreateVoxelization(const std::string& dfnImplementation);
			static PointCloudFilteringInterface* CreatePointCloudFiltering(const std::string& dfnImplementation);
	};
}
}

#endif // DFNS_BUILDER_HPP

/** @} */
