/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNsBuilder.cpp
 * @date 23/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the DFNsBuilder.
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
#include <DFNsBuilder.hpp>
#include <Errors/Assert.hpp>
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
#include <StereoReconstruction/DisparityMapping.hpp>
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DFNCommonInterface* DFNsBuilder::CreateDFN(std::string dfnType, std::string dfnImplementation)
	{		
	if (dfnType == "ImageFiltering")
		{
		return CreateImageFiltering(dfnImplementation);
		}
	else if (dfnType == "PointCloudReconstruction2DTo3D")
		{
		return CreatePointCloudReconstruction2DTo3D(dfnImplementation);
		}
	else if (dfnType == "FeaturesMatching2D")
		{
		return CreateFeaturesMatching2D(dfnImplementation);
		}
	else if (dfnType == "FeaturesExtraction2D")
		{
		return CreateFeaturesExtraction2D(dfnImplementation);
		}
	else if (dfnType == "FeaturesDescription2D")
		{
		return CreateFeaturesDescription2D(dfnImplementation);
		}
	else if (dfnType == "FundamentalMatrixComputation")
		{
		return CreateFundamentalMatrixComputation(dfnImplementation);
		}
	else if (dfnType == "CamerasTransformEstimation")
		{
		return CreateCamerasTransformEstimation(dfnImplementation);
		}
	else if (dfnType == "PerspectiveNPointSolving")
		{
		return CreatePerspectiveNPointSolving(dfnImplementation);
		}
	else if (dfnType == "FeaturesExtraction3D")
		{
		return CreateFeaturesExtraction3D(dfnImplementation);
		}
	else if (dfnType == "FeaturesDescription3D")
		{
		return CreateFeaturesDescription3D(dfnImplementation);
		}
	else if (dfnType == "FeaturesMatching3D")
		{
		return CreateFeaturesMatching3D(dfnImplementation);
		}
	else if (dfnType == "StereoReconstruction")
		{
		return CreateStereoReconstruction(dfnImplementation);
		}
	PRINT_TO_LOG("DFN: ", dfnType);
	PRINT_TO_LOG("Implementation: ", dfnImplementation);
	ASSERT(false, "DFNsBuilder Error: unhandled dfn Type");
	return NULL;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageFilteringInterface* DFNsBuilder::CreateImageFiltering(std::string dfnImplementation)
	{
	if (dfnImplementation == "ImageUndistortion")
		{
		return new ImageUndistortion();
		}
	else if (dfnImplementation == "ImageUndistortionRectification")
		{
		return new ImageUndistortionRectification();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

PointCloudReconstruction2DTo3DInterface* DFNsBuilder::CreatePointCloudReconstruction2DTo3D(std::string dfnImplementation)
	{
	if (dfnImplementation == "Triangulation")
		{
		return new Triangulation();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FeaturesMatching2DInterface* DFNsBuilder::CreateFeaturesMatching2D(std::string dfnImplementation)
	{
	if (dfnImplementation == "FlannMatcher")
		{
		return new FlannMatcher();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FeaturesExtraction2DInterface* DFNsBuilder::CreateFeaturesExtraction2D(std::string dfnImplementation)
	{
	if (dfnImplementation == "HarrisDetector2D")
		{
		return new HarrisDetector2D();
		}
	else if (dfnImplementation == "OrbDetectorDescriptor")
		{
		return new OrbDetectorDescriptor();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FeaturesDescription2DInterface* DFNsBuilder::CreateFeaturesDescription2D(std::string dfnImplementation)
	{
	if (dfnImplementation == "OrbDescriptor")
		{
		return new OrbDescriptor();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FundamentalMatrixComputationInterface* DFNsBuilder::CreateFundamentalMatrixComputation(std::string dfnImplementation)
	{
	if (dfnImplementation == "FundamentalMatrixRansac")
		{
		return new FundamentalMatrixRansac();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

CamerasTransformEstimationInterface* DFNsBuilder::CreateCamerasTransformEstimation(std::string dfnImplementation)
	{
	if (dfnImplementation == "EssentialMatrixDecomposition")
		{
		return new EssentialMatrixDecomposition();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

PerspectiveNPointSolvingInterface* DFNsBuilder::CreatePerspectiveNPointSolving(std::string dfnImplementation)
	{
	if (dfnImplementation == "IterativePnpSolver")
		{
		return new IterativePnpSolver();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FeaturesExtraction3DInterface* DFNsBuilder::CreateFeaturesExtraction3D(std::string dfnImplementation)
	{
	if (dfnImplementation == "HarrisDetector3D")
		{
		return new HarrisDetector3D();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FeaturesDescription3DInterface* DFNsBuilder::CreateFeaturesDescription3D(std::string dfnImplementation)
	{
	if (dfnImplementation == "ShotDescriptor3D")
		{
		return new ShotDescriptor3D();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

FeaturesMatching3DInterface* DFNsBuilder::CreateFeaturesMatching3D(std::string dfnImplementation)
	{
	if (dfnImplementation == "Icp3D")
		{
		return new Icp3D();
		}
	else if (dfnImplementation == "Ransac3D")
		{
		return new Ransac3D();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

StereoReconstructionInterface* DFNsBuilder::CreateStereoReconstruction(std::string dfnImplementation)
	{
	if (dfnImplementation == "DisparityMapping")
		{
		return new DisparityMapping();
		}
	else if (dfnImplementation == "HirschmullerDisparityMapping")
		{
		return new HirschmullerDisparityMapping();
		}
	ASSERT(false, "DFNsBuilder Error: unhandled dfn implementation");
	return NULL;
	}

}


/** @} */
