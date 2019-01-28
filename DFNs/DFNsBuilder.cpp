/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include <DFNsBuilder.hpp>

#ifdef HAVE_OPENCV
#include <BundleAdjustment/SvdDecomposition.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>
#include <ColorConversion/ColorConversion.hpp>
#include <DepthFiltering/ConvolutionFilter.hpp>
#include <DisparityImage/DisparityImage.hpp>
#include <DisparityToPointCloud/DisparityToPointCloud.hpp>
#include <DisparityToPointCloudWithIntensity/DisparityToPointCloudWithIntensity.hpp>
#include <FeaturesDescription2D/OrbDescriptor.hpp>
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <ImageDegradation/ImageDegradation.hpp>
#include <ImageFiltering/ImageUndistortion.hpp>
#include <ImageFiltering/ImageUndistortionRectification.hpp>
#include <ImageFiltering/CannyEdgeDetection.hpp>
#include <ImageFiltering/DerivativeEdgeDetection.hpp>
#include <ImageFiltering/BackgroundExtraction.hpp>
#include <ImageFiltering/NormalVectorExtraction.hpp>
#include <ImageFiltering/KMeansClustering.hpp>
#include <ImageRectification/ImageRectification.hpp>
#include <PerspectiveNPointSolving/IterativePnpSolver.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <StereoReconstruction/DisparityMapping.hpp>
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <StereoDegradation/StereoDegradation.hpp>
#include <StereoRectification/StereoRectification.hpp>
#include <Transform3DEstimation/LeastSquaresMinimization.hpp>
#include <Voxelization/Octree.hpp>
#include <PrimitiveMatching/HuInvariants.hpp>
#endif

#ifdef HAVE_CERES
#include <BundleAdjustment/CeresAdjustment.hpp>
#include <Transform3DEstimation/CeresEstimation.hpp>
#endif

#ifdef HAVE_PCL
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesDescription3D/PfhDescriptor3D.hpp>
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <FeaturesExtraction3D/IssDetector3D.hpp>
#include <FeaturesExtraction3D/CornerDetector3D.hpp>
#include <FeaturesMatching3D/Icp3D.hpp>
#include <FeaturesMatching3D/Ransac3D.hpp>
#include <PointCloudFiltering/StatisticalOutlierRemoval.hpp>
#include <PointCloudTransformation/CartesianSystemTransform.hpp>
#include <Registration3D/Icp3D.hpp>
#include <StereoReconstruction/ScanlineOptimization.hpp>
#include <PointCloudAssembly/NeighbourPointAverage.hpp>
#include <PointCloudAssembly/NeighbourSinglePointAverage.hpp>
#include <PointCloudAssembly/VoxelBinning.hpp>
#endif

#ifdef HAVE_POINTMATCHER
#include <Registration3D/IcpMatcher.hpp>
#include <PointCloudAssembly/MatcherAssembly.hpp>
#endif

#ifdef HAVE_CLOUDCOMPARE
#include <Registration3D/IcpCC.hpp>
#endif

#include <FeaturesMatching3D/BestDescriptorMatch.hpp>
#include <ForceMeshGenerator/ThresholdForce.hpp>

#ifdef HAVE_EDRES
#include <ImageDegradation/ImageDegradationEdres.hpp>
#include <DisparityImage/DisparityImageEdres.hpp>
#include <DisparityToPointCloud/DisparityToPointCloudEdres.hpp>
#include <DisparityToPointCloudWithIntensity/DisparityToPointCloudWithIntensityEdres.hpp>
#include <ImageRectification/ImageRectificationEdres.hpp>
#include <StereoDegradation/StereoDegradationEdres.hpp>
#include <StereoMotionEstimation/StereoMotionEstimationEdres.hpp>
#include <StereoRectification/StereoRectificationEdres.hpp>
#endif

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
	else if (dfnType == "ColorConversion")
	{
        	return CreateColorConversion(dfnImplementation);
	}
    	else if (dfnType == "DisparityImage")
    	{
        	return CreateDisparityImage(dfnImplementation);
    	}
    	else if (dfnType == "DisparityToPointCloud")
    	{
        	return CreateDisparityToPointCloud(dfnImplementation);
    	}
    	else if (dfnType == "DisparityToPointCloudWithIntensity")
    	{
        	return CreateDisparityToPointCloudWithIntensity(dfnImplementation);
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
    	else if (dfnType == "ImageDegradation")
    	{
        	return CreateImageDegradation(dfnImplementation);
    	}
	else if (dfnType == "ImageFiltering")
	{
		return CreateImageFiltering(dfnImplementation);
	}
    	else if (dfnType == "ImageRectification")
    	{
        return CreateImageRectification(dfnImplementation);
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
    	else if (dfnType == "StereoDegradation")
    	{
        	return CreateStereoDegradation(dfnImplementation);
    	}
    	else if (dfnType == "StereoMotionEstimation")
    	{
        	return CreateStereoMotionEstimation(dfnImplementation);
    	}
	else if (dfnType == "StereoReconstruction")
	{
		return CreateStereoReconstruction(dfnImplementation);
	}
    	else if (dfnType == "StereoRectification")
    	{
        	return CreateStereoRectification(dfnImplementation);
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
	else if (dfnType == "PointCloudTransformation")
	{
		return CreatePointCloudTransformation(dfnImplementation);
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
#ifdef HAVE_CERES
	if (dfnImplementation == "CeresAdjustment")
	{
		return new BundleAdjustment::CeresAdjustment;
	}
#endif
#ifdef HAVE_OPENCV
	if (dfnImplementation == "SvdDecomposition")
	{
		return new BundleAdjustment::SvdDecomposition;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN BundleAdjustment implementation");
	return NULL;
}

CamerasTransformEstimationInterface* DFNsBuilder::CreateCamerasTransformEstimation(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "EssentialMatrixDecomposition")
	{
		return new CamerasTransformEstimation::EssentialMatrixDecomposition;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN CameraTransformEstimation implementation");
	return NULL;
}

ColorConversionInterface* DFNsBuilder::CreateColorConversion(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "ColorConversion")
	{
		return new ColorConversion::ColorConversion;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN ColorConversion implementation");
	return NULL;
}

DisparityImageInterface* DFNsBuilder::CreateDisparityImage(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "DisparityImage")
    {
        return new DisparityImage::DisparityImage;
    }
#endif
#ifdef HAVE_EDRES
    if (dfnImplementation == "DisparityImageEdres")
    {
        return new DisparityImage::DisparityImageEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN DisparityImage implementation");
    return NULL;
}

DisparityToPointCloudInterface* DFNsBuilder::CreateDisparityToPointCloud(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "DisparityToPointCloud")
    {
        return new DisparityToPointCloud::DisparityToPointCloud;
    }
#endif
#ifdef HAVE_EDRES
    if (dfnImplementation == "DisparityToPointCloudEdres")
    {
        return new DisparityToPointCloud::DisparityToPointCloudEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN DisparityToPointCloud implementation");
    return NULL;
}

DisparityToPointCloudWithIntensityInterface* DFNsBuilder::CreateDisparityToPointCloudWithIntensity(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "DisparityToPointCloudWithIntensity")
    {
        return new DisparityToPointCloudWithIntensity::DisparityToPointCloudWithIntensity;
    }
#endif
#if HAVE_EDRES
    if (dfnImplementation == "DisparityToPointCloudWithIntensityEdres")
    {
        return new DisparityToPointCloudWithIntensity::DisparityToPointCloudWithIntensityEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN DisparityToPointCloudWithIntensity implementation");
    return NULL;
}

FeaturesDescription2DInterface* DFNsBuilder::CreateFeaturesDescription2D(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "OrbDescriptor")
	{
		return new FeaturesDescription2D::OrbDescriptor;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesDescription2D implementation");
	return NULL;
}

FeaturesDescription3DInterface* DFNsBuilder::CreateFeaturesDescription3D(const std::string& dfnImplementation)
{
#ifdef HAVE_PCL
	if (dfnImplementation == "ShotDescriptor3D")
	{
		return new FeaturesDescription3D::ShotDescriptor3D;
	}
	else if (dfnImplementation == "PfhDescriptor3D")
	{
		return new FeaturesDescription3D::PfhDescriptor3D;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesDescription3D implementation");
	return NULL;
}

FeaturesExtraction2DInterface* DFNsBuilder::CreateFeaturesExtraction2D(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "HarrisDetector2D")
	{
		return new FeaturesExtraction2D::HarrisDetector2D;
	}
	else if (dfnImplementation == "OrbDetectorDescriptor")
	{
		return new FeaturesExtraction2D::OrbDetectorDescriptor;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesMatching2D implementation");
	return NULL;
}

FeaturesExtraction3DInterface* DFNsBuilder::CreateFeaturesExtraction3D(const std::string& dfnImplementation)
{
#ifdef HAVE_PCL
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
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesExtraction3D implementation");
	return NULL;
}

FeaturesMatching2DInterface* DFNsBuilder::CreateFeaturesMatching2D(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "FlannMatcher")
	{
		return new FeaturesMatching2D::FlannMatcher;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesMatching2D implementation");
	return NULL;
}

FeaturesMatching3DInterface* DFNsBuilder::CreateFeaturesMatching3D(const std::string& dfnImplementation)
{
#ifdef HAVE_PCL
	if (dfnImplementation == "Icp3D")
	{
		return new FeaturesMatching3D::Icp3D;
	}
	else if (dfnImplementation == "Ransac3D")
	{
		return new FeaturesMatching3D::Ransac3D;
	}
#endif
	if (dfnImplementation == "BestDescriptorMatch")
	{
		return new FeaturesMatching3D::BestDescriptorMatch;
	}
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FeaturesMatching3D implementation");
	return NULL;
}

FundamentalMatrixComputationInterface* DFNsBuilder::CreateFundamentalMatrixComputation(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "FundamentalMatrixRansac")
	{
		return new FundamentalMatrixComputation::FundamentalMatrixRansac;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN FundamentalMatrixComputation implementation");
	return NULL;
}

ImageDegradationInterface* DFNsBuilder::CreateImageDegradation(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "ImageDegradation")
    {
        return new ImageDegradation::ImageDegradation;
    }
#endif
#ifdef HAVE_EDRES
    if (dfnImplementation == "ImageDegradationEdres")
    {
        return new ImageDegradation::ImageDegradationEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN ImageDegradation implementation");
    return NULL;
}

ImageFilteringInterface* DFNsBuilder::CreateImageFiltering(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
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
	} 
	else if (dfnImplementation == "KMeansClustering") 
	{
    		return new ImageFiltering::KMeansClustering;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN ImageFiltering implementation");
	return NULL;
}

ImageRectificationInterface* DFNsBuilder::CreateImageRectification(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "ImageRectification")
    {
        return new ImageRectification::ImageRectification;
    }
#endif
#ifdef HAVE_EDRES
    if (dfnImplementation == "ImageRectificationEdres")
    {
        return new ImageRectification::ImageRectificationEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN ImageRectification implementation");
    return NULL;
}

PerspectiveNPointSolvingInterface* DFNsBuilder::CreatePerspectiveNPointSolving(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "IterativePnpSolver")
	{
		return new PerspectiveNPointSolving::IterativePnpSolver;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PerspectiveNPointSolving implementation");
	return NULL;
}

PointCloudReconstruction2DTo3DInterface* DFNsBuilder::CreatePointCloudReconstruction2DTo3D(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "Triangulation")
	{
		return new PointCloudReconstruction2DTo3D::Triangulation;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudReconstruction2DTo3D implementation");
	return NULL;
}

PrimitiveMatchingInterface* DFNsBuilder::CreatePrimitiveMatching(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "HuInvariants")
	{
		return new PrimitiveMatching::HuInvariants;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PrimitiveMatching implementation");
	return NULL;
}

Registration3DInterface* DFNsBuilder::CreateRegistration3D(const std::string& dfnImplementation)
{
#ifdef HAVE_PCL
	if (dfnImplementation == "Icp3D")
	{
		return new Registration3D::Icp3D;
	}
#endif
#ifdef HAVE_CLOUDCOMPARE
	if (dfnImplementation == "IcpCC")
	{
		return new Registration3D::IcpCC;
	}
#endif
#ifdef HAVE_POINTMATCHER
	if (dfnImplementation == "IcpMatcher")
	{
		return new Registration3D::IcpMatcher;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN Registration3D implementation");
	return NULL;
}

StereoDegradationInterface* DFNsBuilder::CreateStereoDegradation(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "StereoDegradation")
    {
        return new StereoDegradation::StereoDegradation;
    }
#endif
#ifdef HAVE_EDRES
    if (dfnImplementation == "StereoDegradationEdres")
    {
        return new StereoDegradation::StereoDegradationEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN StereoDegradation implementation");
    return NULL;
}

StereoMotionEstimationInterface* DFNsBuilder::CreateStereoMotionEstimation(const std::string& dfnImplementation)
{
#ifdef HAVE_EDRES
    if (dfnImplementation == "StereoMotionEstimation")
    {
        return new StereoMotionEstimation::StereoMotionEstimationEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN StereoMotionEstimation implementation");
    return NULL;
}

StereoReconstructionInterface* DFNsBuilder::CreateStereoReconstruction(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "DisparityMapping")
	{
		return new StereoReconstruction::DisparityMapping;
	}
	else if (dfnImplementation == "HirschmullerDisparityMapping")
	{
		return new StereoReconstruction::HirschmullerDisparityMapping;
	}
#endif
#ifdef HAVE_PCL
	if (dfnImplementation == "ScanlineOptimization")
	{
		return new StereoReconstruction::ScanlineOptimization;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN StereoReconstruction implementation");
	return NULL;
}

StereoRectificationInterface* DFNsBuilder::CreateStereoRectification(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
    if (dfnImplementation == "StereoRectification")
    {
        return new StereoRectification::StereoRectification;
    }
#endif
#ifdef HAVE_EDRES
    if (dfnImplementation == "StereoRectificationEdres")
    {
        return new StereoRectification::StereoRectificationEdres;
    }
#endif
    ASSERT(false, "DFNsBuilder Error: unhandled DFN StereoRectification implementation");
    return NULL;
}

Transform3DEstimationInterface* DFNsBuilder::CreateTransform3DEstimation(const std::string& dfnImplementation)
{
#ifdef HAVE_CERES
	if (dfnImplementation == "CeresEstimation")
	{
		return new Transform3DEstimation::CeresEstimation;
	}
#endif
#ifdef HAVE_OPENCV
	if (dfnImplementation == "LeastSquaresMinimization")
	{
		return new Transform3DEstimation::LeastSquaresMinimization;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN Transform3DEstimation implementation");
	return NULL;
}

DepthFilteringInterface* DFNsBuilder::CreateDepthFiltering(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "ConvolutionFilter")
	{
		return new DepthFiltering::ConvolutionFilter();
	}
#endif
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
#ifdef HAVE_PCL
	if (dfnImplementation == "NeighbourPointAverage")
	{
		return new PointCloudAssembly::NeighbourPointAverage;
	}
	else if (dfnImplementation == "VoxelBinning")
	{
		return new PointCloudAssembly::VoxelBinning;
	}
	else if (dfnImplementation == "NeighbourSinglePointAverage")
	{
		return new PointCloudAssembly::NeighbourSinglePointAverage;
	}
#endif
#ifdef HAVE_POINTMATCHER
	if (dfnImplementation == "MatcherAssembly")
	{
		return new PointCloudAssembly::MatcherAssembly;
	}
#endif

	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudAssembly implementation");
	return NULL;
}

PointCloudTransformationInterface* DFNsBuilder::CreatePointCloudTransformation(const std::string& dfnImplementation)
{
#ifdef HAVE_PCL
	if (dfnImplementation == "CartesianSystemTransform")
	{
		return new PointCloudTransformation::CartesianSystemTransform;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudTransformation implementation");
	return NULL;
}

VoxelizationInterface* DFNsBuilder::CreateVoxelization(const std::string& dfnImplementation)
{
#ifdef HAVE_OPENCV
	if (dfnImplementation == "Octree")
	{
		return new Voxelization::Octree;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN Voxelization implementation");
	return NULL;
}

PointCloudFilteringInterface* DFNsBuilder::CreatePointCloudFiltering(const std::string& dfnImplementation)
{
#ifdef HAVE_PCL
	if (dfnImplementation == "StatisticalOutlierRemoval")
	{
		return new PointCloudFiltering::StatisticalOutlierRemoval;
	}
#endif
	ASSERT(false, "DFNsBuilder Error: unhandled DFN PointCloudFiltering implementation");
	return NULL;
}

}
}

/** @} */
