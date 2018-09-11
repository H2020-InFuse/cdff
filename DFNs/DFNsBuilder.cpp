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
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FeaturesMatching3D/Icp3D.hpp>
#include <FeaturesMatching3D/Ransac3D.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <ImageFiltering/ImageUndistortion.hpp>
#include <ImageFiltering/ImageUndistortionRectification.hpp>
#include <PerspectiveNPointSolving/IterativePnpSolver.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <Registration3D/Icp3D.hpp>
#include <Registration3D/IcpCC.hpp>
#include <StereoReconstruction/DisparityMapping.hpp>
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <StereoReconstruction/ScanlineOptimization.hpp>
#include <Transform3DEstimation/CeresEstimation.hpp>
#include <Transform3DEstimation/LeastSquaresMinimization.hpp>
#include <DepthFiltering/ConvolutionFilter.hpp>
#include <PointCloudAssembly/NeighbourPointAverage.hpp>
#include <PointCloudTransform/CartesianSystemTransform.hpp>

#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{

DFNCommonInterface* DFNsBuilder::CreateDFN(std::string dfnType, std::string dfnImplementation)
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
	else if (dfnType == "DepthFiltering")
	{
		return CreateDepthFiltering(dfnImplementation);
	}
	else if (dfnType == "PointCloudAssembly")
	{
		return CreatePointCloudAssembly(dfnImplementation);
	}
	else if (dfnType == "PointCloudTransform")
	{
		return CreatePointCloudTransform(dfnImplementation);
	}

	PRINT_TO_LOG("DFN: ", dfnType);
	PRINT_TO_LOG("DFN implementation: ", dfnImplementation);
	ASSERT(false, "DFNsBuilder Error: unhandled DFN");
	return NULL;
}

BundleAdjustmentInterface* DFNsBuilder::CreateBundleAdjustment(std::string dfnImplementation)
{
	if (dfnImplementation == "CeresAdjustment")
	{
		return new BundleAdjustment::CeresAdjustment;
	}
	else if (dfnImplementation == "SvdDecomposition")
	{
		return new BundleAdjustment::SvdDecomposition;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

CamerasTransformEstimationInterface* DFNsBuilder::CreateCamerasTransformEstimation(std::string dfnImplementation)
{
	if (dfnImplementation == "EssentialMatrixDecomposition")
	{
		return new CamerasTransformEstimation::EssentialMatrixDecomposition;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FeaturesDescription2DInterface* DFNsBuilder::CreateFeaturesDescription2D(std::string dfnImplementation)
{
	if (dfnImplementation == "OrbDescriptor")
	{
		return new FeaturesDescription2D::OrbDescriptor;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FeaturesDescription3DInterface* DFNsBuilder::CreateFeaturesDescription3D(std::string dfnImplementation)
{
	if (dfnImplementation == "ShotDescriptor3D")
	{
		return new FeaturesDescription3D::ShotDescriptor3D;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FeaturesExtraction2DInterface* DFNsBuilder::CreateFeaturesExtraction2D(std::string dfnImplementation)
{
	if (dfnImplementation == "HarrisDetector2D")
	{
		return new FeaturesExtraction2D::HarrisDetector2D;
	}
	else if (dfnImplementation == "OrbDetectorDescriptor")
	{
		return new FeaturesExtraction2D::OrbDetectorDescriptor;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FeaturesExtraction3DInterface* DFNsBuilder::CreateFeaturesExtraction3D(std::string dfnImplementation)
{
	if (dfnImplementation == "HarrisDetector3D")
	{
		return new FeaturesExtraction3D::HarrisDetector3D;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FeaturesMatching2DInterface* DFNsBuilder::CreateFeaturesMatching2D(std::string dfnImplementation)
{
	if (dfnImplementation == "FlannMatcher")
	{
		return new FeaturesMatching2D::FlannMatcher;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FeaturesMatching3DInterface* DFNsBuilder::CreateFeaturesMatching3D(std::string dfnImplementation)
{
	if (dfnImplementation == "Icp3D")
	{
		return new FeaturesMatching3D::Icp3D;
	}
	else if (dfnImplementation == "Ransac3D")
	{
		return new FeaturesMatching3D::Ransac3D;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

FundamentalMatrixComputationInterface* DFNsBuilder::CreateFundamentalMatrixComputation(std::string dfnImplementation)
{
	if (dfnImplementation == "FundamentalMatrixRansac")
	{
		return new FundamentalMatrixComputation::FundamentalMatrixRansac;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

ImageFilteringInterface* DFNsBuilder::CreateImageFiltering(std::string dfnImplementation)
{
	if (dfnImplementation == "ImageUndistortion")
	{
		return new ImageFiltering::ImageUndistortion;
	}
	else if (dfnImplementation == "ImageUndistortionRectification")
	{
		return new ImageFiltering::ImageUndistortionRectification;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

PerspectiveNPointSolvingInterface* DFNsBuilder::CreatePerspectiveNPointSolving(std::string dfnImplementation)
{
	if (dfnImplementation == "IterativePnpSolver")
	{
		return new PerspectiveNPointSolving::IterativePnpSolver;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

PointCloudReconstruction2DTo3DInterface* DFNsBuilder::CreatePointCloudReconstruction2DTo3D(std::string dfnImplementation)
{
	if (dfnImplementation == "Triangulation")
	{
		return new PointCloudReconstruction2DTo3D::Triangulation;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

Registration3DInterface* DFNsBuilder::CreateRegistration3D(std::string dfnImplementation)
{
	if (dfnImplementation == "Icp3D")
	{
		return new Registration3D::Icp3D;
	}
	else if (dfnImplementation == "IcpCC")
	{
		return new Registration3D::IcpCC;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

StereoReconstructionInterface* DFNsBuilder::CreateStereoReconstruction(std::string dfnImplementation)
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
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

Transform3DEstimationInterface* DFNsBuilder::CreateTransform3DEstimation(std::string dfnImplementation)
{
	if (dfnImplementation == "CeresEstimation")
	{
		return new Transform3DEstimation::CeresEstimation;
	}
	else if (dfnImplementation == "LeastSquaresMinimization")
	{
		return new Transform3DEstimation::LeastSquaresMinimization;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN implementation");
	return NULL;
}

DepthFilteringInterface* DFNsBuilder::CreateDepthFiltering(std::string dfnImplementation)
{
	return new DepthFiltering::ConvolutionFilter();
}

PointCloudAssemblyInterface* DFNsBuilder::CreatePointCloudAssembly(std::string dfnImplementation)
{
	if (dfnImplementation == "NeighbourPointAverage")
	{
		return new PointCloudAssembly::NeighbourPointAverage;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudAssembly implementation");
	return NULL;
}

PointCloudTransformInterface* DFNsBuilder::CreatePointCloudTransform(std::string dfnImplementation)
{
	if (dfnImplementation == "CartesianSystemTransform")
	{
		return new PointCloudTransform::CartesianSystemTransform;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudTransform implementation");
	return NULL;
}

}
}

/** @} */
