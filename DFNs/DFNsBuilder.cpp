/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include <DFNsBuilder.hpp>

#include <BundleAdjustment/CeresAdjustment.hpp>
#include <BundleAdjustment/SvdDecomposition.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>
#include <FeaturesDescription2D/OrbDescriptor.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <FeaturesExtraction3D/IssDetector3D.hpp>
#include <FeaturesExtraction3D/CornerDetector3D.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FeaturesMatching3D/Icp3D.hpp>
#include <FeaturesMatching3D/Ransac3D.hpp>
#include <FeaturesMatching3D/BestDescriptorMatch.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <ImageFiltering/ImageUndistortion.hpp>
#include <ImageFiltering/ImageUndistortionRectification.hpp>
#include <ImageFiltering/CannyEdgeDetection.hpp>
#include <ImageFiltering/DerivativeEdgeDetection.hpp>
#include <ImageFiltering/BackgroundExtraction.hpp>
#include <ImageFiltering/NormalVectorExtraction.hpp>
#include <ImageFiltering/KMeansClustering.hpp>
#include <ImageRectification/ImageRectification.hpp>
#include <ImageRectification/ImageRectificationEdres.hpp>
#include <PerspectiveNPointSolving/IterativePnpSolver.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <PrimitiveMatching/HuInvariants.hpp>
#include <Registration3D/Icp3D.hpp>
#include <Registration3D/IcpCC.hpp>
#include <Registration3D/IcpMatcher.hpp>
#include <StereoReconstruction/DisparityMapping.hpp>
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <StereoReconstruction/ScanlineOptimization.hpp>
#include <Transform3DEstimation/CeresEstimation.hpp>
#include <Transform3DEstimation/LeastSquaresMinimization.hpp>
#include <DepthFiltering/ConvolutionFilter.hpp>
#include <ForceMeshGenerator/ThresholdForce.hpp>
#include <PointCloudAssembly/NeighbourPointAverage.hpp>
#include <PointCloudAssembly/NeighbourSinglePointAverage.hpp>
#include <PointCloudAssembly/VoxelBinning.hpp>
#include <PointCloudAssembly/MatcherAssembly.hpp>
#include <PointCloudTransform/CartesianSystemTransform.hpp>
#include <Voxelization/Octree.hpp>
#include <PointCloudFiltering/StatisticalOutlierRemoval.hpp>

#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{

DFNCommonInterface* DFNsBuilder::CreateDFN(const std::string& dfnType, const std::string& dfnImplementation)
{
	if (dfnType == "BundleAdjustment")
	{
		return CreateBundleAdjustment(dfnImplementation);
	}
	else if (dfnType == "CamerasTransformEstimation")
	{
		return CreateCamerasTransformEstimation(dfnImplementation);
	}
	else if (dfnType == "FeaturesDescription2D")
	{
		return CreateFeaturesDescription2D(dfnImplementation);
	}
	else if (dfnType == "FeaturesDescription3D")
	{
		return CreateFeaturesDescription3D(dfnImplementation);
	}
	else if (dfnType == "FeaturesExtraction2D")
	{
		return CreateFeaturesExtraction2D(dfnImplementation);
	}
	else if (dfnType == "FeaturesExtraction3D")
	{
		return CreateFeaturesExtraction3D(dfnImplementation);
	}
	else if (dfnType == "FeaturesMatching2D")
	{
		return CreateFeaturesMatching2D(dfnImplementation);
	}
	else if (dfnType == "FeaturesMatching3D")
	{
		return CreateFeaturesMatching3D(dfnImplementation);
	}
	else if (dfnType == "FundamentalMatrixComputation")
	{
		return CreateFundamentalMatrixComputation(dfnImplementation);
	}
	else if (dfnType == "ImageFiltering")
	{
		return CreateImageFiltering(dfnImplementation);
	}
	else if (dfnType == "PerspectiveNPointSolving")
	{
		return CreatePerspectiveNPointSolving(dfnImplementation);
	}
	else if (dfnType == "PointCloudReconstruction2DTo3D")
	{
		return CreatePointCloudReconstruction2DTo3D(dfnImplementation);
	}
	else if (dfnType == "Registration3D")
	{
		return CreateRegistration3D(dfnImplementation);
	}
	else if (dfnType == "StereoReconstruction")
	{
		return CreateStereoReconstruction(dfnImplementation);
	}
	else if (dfnType == "Transform3DEstimation")
	{
		return CreateTransform3DEstimation(dfnImplementation);
	}
	else if ( dfnType == "PrimitiveMatching" )
	{
		return CreatePrimitiveMatching(dfnImplementation);
	}
	else if (dfnType == "DepthFiltering")
	{
		return CreateDepthFiltering(dfnImplementation);
	}
	else if (dfnType == "ForceMeshGenerator")
	{
		return CreateForceMeshGenerator(dfnImplementation);
	}
	else if (dfnType == "PointCloudAssembly")
	{
		return CreatePointCloudAssembly(dfnImplementation);
	}
	else if (dfnType == "PointCloudTransform")
	{
		return CreatePointCloudTransform(dfnImplementation);
	}
	else if (dfnType == "Voxelization")
	{
	return CreateVoxelization(dfnImplementation);
	}
	else if (dfnType == "PointCloudFiltering")
	{
	return CreatePointCloudFiltering(dfnImplementation);
	}	

	PRINT_TO_LOG("DFN: ", dfnType);
	PRINT_TO_LOG("DFN implementation: ", dfnImplementation);
	ASSERT(false, "DFNsBuilder Error: unhandled DFN");
	return NULL;
}

BundleAdjustmentInterface* DFNsBuilder::CreateBundleAdjustment(const std::string& dfnImplementation)
{
	if (dfnImplementation == "CeresAdjustment")
	{
		return new BundleAdjustment::CeresAdjustment;
	}
	else if (dfnImplementation == "SvdDecomposition")
	{
		return new BundleAdjustment::SvdDecomposition;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN BundleAdjustment implementation");
	return NULL;
}

CamerasTransformEstimationInterface* DFNsBuilder::CreateCamerasTransformEstimation(const std::string& dfnImplementation)
{
	if (dfnImplementation == "EssentialMatrixDecomposition")
	{
		return new CamerasTransformEstimation::EssentialMatrixDecomposition;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN CameraTransformEstimation implementation");
	return NULL;
}

FeaturesDescription2DInterface* DFNsBuilder::CreateFeaturesDescription2D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "OrbDescriptor")
	{
		return new FeaturesDescription2D::OrbDescriptor;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesDescription2D implementation");
	return NULL;
}

FeaturesDescription3DInterface* DFNsBuilder::CreateFeaturesDescription3D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "ShotDescriptor3D")
	{
		return new FeaturesDescription3D::ShotDescriptor3D;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesDescription3D implementation");
	return NULL;
}

FeaturesExtraction2DInterface* DFNsBuilder::CreateFeaturesExtraction2D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "HarrisDetector2D")
	{
		return new FeaturesExtraction2D::HarrisDetector2D;
	}
	else if (dfnImplementation == "OrbDetectorDescriptor")
	{
		return new FeaturesExtraction2D::OrbDetectorDescriptor;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesMatching2D implementation");
	return NULL;
}

FeaturesExtraction3DInterface* DFNsBuilder::CreateFeaturesExtraction3D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "HarrisDetector3D")
	{
		return new FeaturesExtraction3D::HarrisDetector3D;
	}
	if (dfnImplementation == "IssDetector3D")
	{
		return new FeaturesExtraction3D::IssDetector3D;
	}
	if (dfnImplementation == "CornerDetector3D")
	{
		return new FeaturesExtraction3D::CornerDetector3D;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesExtraction3D implementation");
	return NULL;
}

FeaturesMatching2DInterface* DFNsBuilder::CreateFeaturesMatching2D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "FlannMatcher")
	{
		return new FeaturesMatching2D::FlannMatcher;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesMatching2D implementation");
	return NULL;
}

FeaturesMatching3DInterface* DFNsBuilder::CreateFeaturesMatching3D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "Icp3D")
	{
		return new FeaturesMatching3D::Icp3D;
	}
	else if (dfnImplementation == "Ransac3D")
	{
		return new FeaturesMatching3D::Ransac3D;
	}
	else if (dfnImplementation == "BestDescriptorMatch")
	{
		return new FeaturesMatching3D::BestDescriptorMatch;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesMatching3D implementation");
	return NULL;
}

FundamentalMatrixComputationInterface* DFNsBuilder::CreateFundamentalMatrixComputation(const std::string& dfnImplementation)
{
	if (dfnImplementation == "FundamentalMatrixRansac")
	{
		return new FundamentalMatrixComputation::FundamentalMatrixRansac;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FundamentalMatrixComputation implementation");
	return NULL;
}

ImageFilteringInterface* DFNsBuilder::CreateImageFiltering(const std::string& dfnImplementation)
{
	if (dfnImplementation == "ImageUndistortion")
	{
		return new ImageFiltering::ImageUndistortion;
	}
	else if (dfnImplementation == "ImageUndistortionRectification")
	{
		return new ImageFiltering::ImageUndistortionRectification;
	}
    else if (dfnImplementation == "CannyEdgeDetection")
    {
        return new ImageFiltering::CannyEdgeDetection;
    }
    else if (dfnImplementation == "DerivativeEdgeDetection")
    {
        return new ImageFiltering::DerivativeEdgeDetection;
    }
    else if (dfnImplementation == "BackgroundExtraction")
    {
        return new ImageFiltering::BackgroundExtraction;
    }
    else if (dfnImplementation == "NormalVectorExtraction")
    {
        return new ImageFiltering::NormalVectorExtraction;
    } else if (dfnImplementation == "KMeansClustering") {
    	return new ImageFiltering::KMeansClustering;
    }

	ASSERT(false, "DFNsBuilder Error: unhandled DFN ImageFiltering implementation");
	return NULL;
}

ImageRectificationInterface* DFNsBuilder::CreateImageRectification(const std::string& dfnImplementation)
{
    if (dfnImplementation == "ImageRectification")
    {
        return new ImageRectification::ImageRectification;
    }
    else if (dfnImplementation == "ImageRectificationEdres")
    {
        return new ImageRectification::ImageRectificationEdres;
    }
    ASSERT(false, "DFNsBuilder Error: unhandled DFN ImageRectification implementation");
    return NULL;
}

PerspectiveNPointSolvingInterface* DFNsBuilder::CreatePerspectiveNPointSolving(const std::string& dfnImplementation)
{
	if (dfnImplementation == "IterativePnpSolver")
	{
		return new PerspectiveNPointSolving::IterativePnpSolver;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PerspectiveNPointSolving implementation");
	return NULL;
}

PointCloudReconstruction2DTo3DInterface* DFNsBuilder::CreatePointCloudReconstruction2DTo3D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "Triangulation")
	{
		return new PointCloudReconstruction2DTo3D::Triangulation;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudReconstruction2DTo3D implementation");
	return NULL;
}

PrimitiveMatchingInterface* DFNsBuilder::CreatePrimitiveMatching(const std::string& dfnImplementation)
{
	if (dfnImplementation == "HuInvariants")
	{
		return new PrimitiveMatching::HuInvariants;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PrimitiveMatching implementation");
	return NULL;
}

Registration3DInterface* DFNsBuilder::CreateRegistration3D(const std::string& dfnImplementation)
{
	if (dfnImplementation == "Icp3D")
	{
		return new Registration3D::Icp3D;
	}
	else if (dfnImplementation == "IcpCC")
	{
		return new Registration3D::IcpCC;
	}
	else if (dfnImplementation == "IcpMatcher")
	{
		return new Registration3D::IcpMatcher;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN Registration3D implementation");
	return NULL;
}

StereoReconstructionInterface* DFNsBuilder::CreateStereoReconstruction(const std::string& dfnImplementation)
{
	if (dfnImplementation == "DisparityMapping")
	{
		return new StereoReconstruction::DisparityMapping;
	}
	else if (dfnImplementation == "HirschmullerDisparityMapping")
	{
		return new StereoReconstruction::HirschmullerDisparityMapping;
	}
	else if (dfnImplementation == "ScanlineOptimization")
	{
		return new StereoReconstruction::ScanlineOptimization;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN StereoReconstruction implementation");
	return NULL;
}

Transform3DEstimationInterface* DFNsBuilder::CreateTransform3DEstimation(const std::string& dfnImplementation)
{
	if (dfnImplementation == "CeresEstimation")
	{
		return new Transform3DEstimation::CeresEstimation;
	}
	else if (dfnImplementation == "LeastSquaresMinimization")
	{
		return new Transform3DEstimation::LeastSquaresMinimization;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN Transform3DEstimation implementation");
	return NULL;
}

DepthFilteringInterface* DFNsBuilder::CreateDepthFiltering(const std::string& dfnImplementation)
{
	if (dfnImplementation == "ConvolutionFilter")
	{
		return new DepthFiltering::ConvolutionFilter();
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN DepthFiltering implementation");
	return NULL;
}

ForceMeshGeneratorInterface* DFNsBuilder::CreateForceMeshGenerator(const std::string& dfnImplementation)
{
	if (dfnImplementation == "ThresholdForce")
	{
		return new ForceMeshGenerator::ThresholdForce();
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN ForceMeshGenerator implementation");
	return NULL;
}

PointCloudAssemblyInterface* DFNsBuilder::CreatePointCloudAssembly(const std::string& dfnImplementation)
{
	if (dfnImplementation == "NeighbourPointAverage")
	{
		return new PointCloudAssembly::NeighbourPointAverage;
	}
	else if (dfnImplementation == "VoxelBinning")
	{
		return new PointCloudAssembly::VoxelBinning;
	}
	else if (dfnImplementation == "MatcherAssembly")
	{
		return new PointCloudAssembly::MatcherAssembly;
	}
	if (dfnImplementation == "NeighbourSinglePointAverage")
	{
		return new PointCloudAssembly::NeighbourSinglePointAverage;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudAssembly implementation");
	return NULL;
}

PointCloudTransformInterface* DFNsBuilder::CreatePointCloudTransform(const std::string& dfnImplementation)
{
	if (dfnImplementation == "CartesianSystemTransform")
	{
		return new PointCloudTransform::CartesianSystemTransform;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudTransform implementation");
	return NULL;
}

VoxelizationInterface* DFNsBuilder::CreateVoxelization(const std::string& dfnImplementation)
{
	if (dfnImplementation == "Octree")
	{
		return new Voxelization::Octree;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN Voxelization implementation");
	return NULL;
}

PointCloudFilteringInterface* DFNsBuilder::CreatePointCloudFiltering(const std::string& dfnImplementation)
	{
	if (dfnImplementation == "StatisticalOutlierRemoval")
	{
		return new PointCloudFiltering::StatisticalOutlierRemoval;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudFiltering implementation");
	return NULL;
	}
}
}

/** @} */
