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
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <Registration3D/Registration3DInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <Transform3DEstimation/Transform3DEstimationInterface.hpp>
#include <PrimitiveMatching/PrimitiveMatchingInterface.hpp>
#include <DepthFiltering/DepthFilteringInterface.hpp>
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <PointCloudTransform/PointCloudTransformInterface.hpp>

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
			static DFNCommonInterface* CreateDFN(std::string dfnType, std::string dfnImplementation);

		private:
			static BundleAdjustmentInterface* CreateBundleAdjustment(std::string dfnImplementation);
			static CamerasTransformEstimationInterface* CreateCamerasTransformEstimation(std::string dfnImplementation);
			static FeaturesDescription2DInterface* CreateFeaturesDescription2D(std::string dfnImplementation);
			static FeaturesDescription3DInterface* CreateFeaturesDescription3D(std::string dfnImplementation);
			static FeaturesExtraction2DInterface* CreateFeaturesExtraction2D(std::string dfnImplementation);
			static FeaturesExtraction3DInterface* CreateFeaturesExtraction3D(std::string dfnImplementation);
			static FeaturesMatching2DInterface* CreateFeaturesMatching2D(std::string dfnImplementation);
			static FeaturesMatching3DInterface* CreateFeaturesMatching3D(std::string dfnImplementation);
			static FundamentalMatrixComputationInterface* CreateFundamentalMatrixComputation(std::string dfnImplementation);
			static ImageFilteringInterface* CreateImageFiltering(std::string dfnImplementation);
			static PerspectiveNPointSolvingInterface* CreatePerspectiveNPointSolving(std::string dfnImplementation);
			static PointCloudReconstruction2DTo3DInterface* CreatePointCloudReconstruction2DTo3D(std::string dfnImplementation);
			static PrimitiveMatchingInterface* CreatePrimitiveMatching(std::string dfnImplementation);
			static Registration3DInterface* CreateRegistration3D(std::string dfnImplementation);
			static StereoReconstructionInterface* CreateStereoReconstruction(std::string dfnImplementation);
			static Transform3DEstimationInterface* CreateTransform3DEstimation(std::string dfnImplementation);
			static DepthFilteringInterface* CreateDepthFiltering(std::string dfnImplementation);
			static PointCloudAssemblyInterface* CreatePointCloudAssembly(std::string dfnImplementation);
			static PointCloudTransformInterface* CreatePointCloudTransform(std::string dfnImplementation);

	};
}
}

#endif // DFNS_BUILDER_HPP

/** @} */
