/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "IcpCC.hpp"

#include <PointCloudToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>
#include <pcl/registration/icp.h>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace CCLib;

namespace dfn_ci
{

IcpCC::IcpCC()
{
	parametersHelper.AddParameter<ConvergenceType, ConvergenceTypeHelper>("GeneralParameters", "ConvergenceType", parameters.convergenceType, DEFAULT_PARAMETERS.convergenceType);
	parametersHelper.AddParameter<double>("GeneralParameters", "MinimumErrorReduction", parameters.minimumErrorReduction, DEFAULT_PARAMETERS.minimumErrorReduction);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumNumberOfIterations", parameters.maximumNumberOfIterations, DEFAULT_PARAMETERS.maximumNumberOfIterations);
	parametersHelper.AddParameter<bool>("GeneralParameters", "ScaleIsAdjustable", parameters.scaleIsAdjustable, DEFAULT_PARAMETERS.scaleIsAdjustable);
	parametersHelper.AddParameter<bool>("GeneralParameters", "FarthestPointsAreFilteredOut", parameters.farthestPointsAreFilteredOut, DEFAULT_PARAMETERS.farthestPointsAreFilteredOut);
	parametersHelper.AddParameter<int>("GeneralParameters", "SamplingLimit", parameters.samplingLimit, DEFAULT_PARAMETERS.samplingLimit);
	parametersHelper.AddParameter<double>("GeneralParameters", "FinalOverlapRatio", parameters.finalOverlapRatio, DEFAULT_PARAMETERS.finalOverlapRatio);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumNumberOfThreads", parameters.maximumNumberOfThreads, DEFAULT_PARAMETERS.maximumNumberOfThreads);

	configurationFilePath = "";
}

IcpCC::~IcpCC()
{

}

void IcpCC::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	ConvertParametersToCCParametersList();
}

void IcpCC::process()
{
	if ( GetNumberOfPoints(inSourceCloud) == 0 || GetNumberOfPoints(inSinkCloud) == 0)
		{
		outSuccess = false;
		return;
		}

	ChunkedPointCloud* inputSourceCloud = Convert(&inSourceCloud);
	ChunkedPointCloud* inputSinkCloud = Convert(&inSinkCloud);	

	ValidateInputs(inputSourceCloud, inputSinkCloud);
	ComputeTransform(inputSourceCloud, inputSinkCloud);

	delete(inputSourceCloud);
	delete(inputSinkCloud);
}

IcpCC::ConvergenceTypeHelper::ConvergenceTypeHelper(const std::string& parameterName, ConvergenceType& boundVariable, const ConvergenceType& defaultValue)
	: ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

IcpCC::ConvergenceType IcpCC::ConvergenceTypeHelper::Convert(const std::string& outputConvergenceType)
{
	if (outputConvergenceType == "ErrorReduction" || outputConvergenceType == "0")
	{
		return MINIMUM_ERROR_REDUCTION;
	}
	else if (outputConvergenceType == "NumberOfIterations" || outputConvergenceType == "1")
	{
		return NUMBER_OF_ITERATIONS;
	}
	ASSERT(false, "Icp3D Error: unhandled convergence type: it should be either ErrorReduction or NumberOfIterations");
	return MINIMUM_ERROR_REDUCTION;
}

const IcpCC::IcpOptionsSet IcpCC::DEFAULT_PARAMETERS =
	{
	.convergenceType = MINIMUM_ERROR_REDUCTION,
	.minimumErrorReduction = 1e-5,
	.maximumNumberOfIterations = 20,
	.scaleIsAdjustable = false,
	.farthestPointsAreFilteredOut = false,
	.samplingLimit = 50000,
	.finalOverlapRatio = 1.0,
	.maximumNumberOfThreads = 0
	};


void IcpCC::ComputeTransform(ChunkedPointCloud* sourceCloud, ChunkedPointCloud* sinkCloud)
	{
	RegistrationTools::ScaledTransformation scaledTransform;
	if (inUseGuess)
		{
		scaledTransform = ConvertTrasformToCCTransform(inTransformGuess);
		}
	else
		{
		scaledTransform.R = SquareMatrix(3);
		scaledTransform.R.toIdentity();
		scaledTransform.T.x = 0;
		scaledTransform.T.y = 0;
		scaledTransform.T.z = 0;
		scaledTransform.s = 1.0;
		}

	double finalRMS;
	unsigned finalPointCount;
	ICPRegistrationTools::RESULT_TYPE result = ICPRegistrationTools::Register(sourceCloud, nullptr, sinkCloud, ccParametersList, scaledTransform, finalRMS, finalPointCount);

	if (result == ICPRegistrationTools::ICP_APPLY_TRANSFO || result == ICPRegistrationTools::ICP_NOTHING_TO_DO)
		{
		outTransform = ConvertCCTransformToTranform(scaledTransform);
		outSuccess = true;
		}
	else
		{
		outSuccess = false;
		}
	}

ChunkedPointCloud* IcpCC::Convert(PointCloudWrapper::PointCloudConstPtr cloud)
	{
	ChunkedPointCloud* ccCloud = new ChunkedPointCloud;
	ccCloud->reserve( GetNumberOfPoints(*cloud) );

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*cloud); pointIndex++)
		{
		CCVector3 newPoint( GetXCoordinate(*cloud, pointIndex), GetYCoordinate(*cloud, pointIndex), GetZCoordinate(*cloud, pointIndex) );
		ccCloud->addPoint(newPoint);
		}

	ccCloud->enableScalarField();
	return ccCloud;
	}

void IcpCC::ConvertParametersToCCParametersList()
	{
	if (parameters.convergenceType == MINIMUM_ERROR_REDUCTION)
		{
		ccParametersList.convType = ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
		ccParametersList.minRMSDecrease = parameters.minimumErrorReduction;
		}
	else
		{
		ccParametersList.convType = ICPRegistrationTools::MAX_ITER_CONVERGENCE;
		ccParametersList.nbMaxIterations = static_cast<unsigned>(parameters.maximumNumberOfIterations);
		}

	ccParametersList.adjustScale = parameters.scaleIsAdjustable;
	ccParametersList.filterOutFarthestPoints = parameters.farthestPointsAreFilteredOut;
	ccParametersList.samplingLimit = static_cast<unsigned>(parameters.samplingLimit);
	ccParametersList.finalOverlapRatio = parameters.finalOverlapRatio;
	ccParametersList.maxThreadCount = static_cast<unsigned>(parameters.maximumNumberOfThreads);
	}

//The expected initial guess / output  is the position of the source cloud in the frame of the sink cloud,
//This is the inverse of the transform that moves the source cloud in the location of the sink cloud
RegistrationTools::ScaledTransformation IcpCC::ConvertTrasformToCCTransform(const Transform3D& transform)
	{
	RegistrationTools::ScaledTransformation ccTransform;
	ccTransform.T.x = -GetXPosition(transform);
	ccTransform.T.y = -GetYPosition(transform);
	ccTransform.T.z = -GetZPosition(transform);
	ccTransform.s = 1.0;

	//We need this representation of the quaternion in order to use CC conversion method to rotation matrix.
	double quaternion[4];
	quaternion[0] = GetWOrientation(transform);
	quaternion[1] = GetXOrientation(transform);
	quaternion[2] = GetYOrientation(transform);
	quaternion[3] = GetZOrientation(transform);
	
	//Normalization, it is required for the computation of the rotation matrix;
	double norm = std::sqrt( quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
	quaternion[0] = quaternion[0] / norm;
	quaternion[1] = quaternion[1] / norm;
	quaternion[2] = quaternion[2] / norm;
	quaternion[3] = quaternion[3] / norm;

	SquareMatrix rotationMatrix(3);
	rotationMatrix.initFromQuaternion(quaternion);
	ccTransform.R = rotationMatrix.inv();
	return ccTransform;
	}

Transform3D IcpCC::ConvertCCTransformToTranform(const RegistrationTools::ScaledTransformation& ccTransform)
	{
	ASSERT(ccTransform.s >= 0.99999 && ccTransform.s <= 1.00001, "IcpCC error, ccTransform does not have expected scale of 1");

	Transform3D transform;
	SetPosition(transform, -ccTransform.T.x, -ccTransform.T.y, -ccTransform.T.z);

	//We need a non-cost matrix for calling the toQuatenion method. (It is not expected indeed).
	SquareMatrix rotationMatrixCopy = ccTransform.R.inv();

	double quaternion[4];
	bool success = 	rotationMatrixCopy.toQuaternion(quaternion);
	ASSERT(success, "IcpCC error, could not convert ccTransform rotation matrix to Transform quaternion");
	
	SetOrientation(transform, quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
	return transform;
	}

void IcpCC::ValidateParameters()
	{
	ASSERT( parameters.minimumErrorReduction > 0 || parameters.convergenceType != MINIMUM_ERROR_REDUCTION, "IcpCC Configuration error, Minimum Error Reduction has to be positive");
	ASSERT( parameters.maximumNumberOfIterations > 0 || parameters.convergenceType != NUMBER_OF_ITERATIONS, "IcpCC Configuration error, Maximum number of iterations has to be positive");
	ASSERT( parameters.samplingLimit > 0, "IcpCC Configuration error, Sampling limit has to be positive");
	ASSERT( parameters.finalOverlapRatio >= 0 && parameters.finalOverlapRatio <= 1, "IcpCC Configuration error, finalOverlapRatio has to be between 0 and 1");
	ASSERT( parameters.maximumNumberOfThreads >= 0, "IcpCC Configuration Error, maximumNumberOfThreads has to be greater or equal to zero");
	}

void IcpCC::ValidateInputs(ChunkedPointCloud* sourceCloud, ChunkedPointCloud* sinkCloud)
	{
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
	}

void IcpCC::ValidateCloud(ChunkedPointCloud* cloud)
	{
	for(unsigned pointIndex = 0; pointIndex < cloud->size(); pointIndex++)
		{
		const CCVector3* point = cloud->getPoint(pointIndex);
		ASSERT_EQUAL(point->x, point->x, "IcpCC Error, Cloud contains an NaN point");
		ASSERT_EQUAL(point->y, point->y, "IcpCC Error, Cloud contains an NaN point");
		ASSERT_EQUAL(point->z, point->z, "IcpCC Error, Cloud contains an NaN point");
		}
	}

}

/** @} */
