/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "LeastSquaresMinimization.hpp"

#include <Errors/Assert.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include <stdlib.h>
#include <fstream>

using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap3DWrapper;
using namespace Helpers;
using namespace BaseTypesWrapper;

namespace CDFF
{
namespace DFN
{
namespace Transform3DEstimation
{

LeastSquaresMinimization::LeastSquaresMinimization()
{
        parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("GeneralParameters", "MaximumAllowedError", parameters.maximumAllowedError, DEFAULT_PARAMETERS.maximumAllowedError);

	configurationFilePath = "";
	SetPosition(emptyPose, 0, 0, 0);
	SetOrientation(emptyPose, 0, 0, 0, 0);
}

LeastSquaresMinimization::~LeastSquaresMinimization()
{
}

void LeastSquaresMinimization::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void LeastSquaresMinimization::process()
{
	int numberOfCorrespondences = GetNumberOfCorrespondenceMaps(inMatches);
	Clear(outTransforms);
	for(int mapIndex = 0; mapIndex < numberOfCorrespondences; mapIndex++)
		{
		const CorrespondenceMap3D& map = GetCorrespondenceMap(inMatches, mapIndex);

		cv::Mat coefficientMatrix, valueMatrix;
		bool success = CreateLinearSystem(map, coefficientMatrix, valueMatrix);

		cv::Mat transformMatrix;
		if (success)
			{
			transformMatrix = SolveLinearSystem(coefficientMatrix, valueMatrix, outError);
			}

		if (!success || outError > parameters.maximumAllowedError)
			{
			if (numberOfCorrespondences == 1)
				{
				outSuccess = false;
				return;
				}
			else
				{
				AddPose(outTransforms, emptyPose);
				}
			}
		else
			{
			AddTransformOutput(transformMatrix);
			}
		}
	outSuccess = true;
}

bool LeastSquaresMinimization::CreateLinearSystem(const CorrespondenceMap3D& map, cv::Mat& coefficientMatrix, cv::Mat& valueMatrix)
	{
	const float EPSILON = 0.00001;
	const int NUMBER_OF_DEGREES_OF_FREEDOM = 6;
	const int NUMBER_OF_VARIABLES = 12;

	coefficientMatrix = cv::Mat();
	valueMatrix = cv::Mat();
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(map); correspondenceIndex++)
		{
		Point3D source = GetSource(map, correspondenceIndex);
		Point3D sink = GetSink(map, correspondenceIndex);
	
		if (source.x != source.x || source.y != source.y || source.z != source.z || sink.x != sink.x || sink.y != sink.y || sink.z != sink.z)
			{
			continue;
			}

		cv::Mat coefficientMatrixPart = ( cv::Mat_<float>(3, 12) << 
			source.x, source.y, source.z, 1,    0, 0, 0, 0,    0, 0, 0, 0,
			0, 0, 0, 0,   source.x, source.y, source.z, 1,     0, 0, 0, 0,
			0, 0, 0, 0,   0, 0, 0, 0,     source.x, source.y, source.z, 1 );
		cv::Mat valueMatrixPart = ( cv::Mat_<float>(3, 1) << sink.x, sink.y, sink.z);
		
		if (coefficientMatrix.rows == 0)
			{
			coefficientMatrix = coefficientMatrixPart;
			valueMatrix = valueMatrixPart;
			}		
		else
			{
			cv::Mat matrixList[2] = {coefficientMatrix, coefficientMatrixPart};
			cv::vconcat(matrixList, 2, coefficientMatrix);
			matrixList[0] = valueMatrix;
			matrixList[1] = valueMatrixPart;
			cv::vconcat(matrixList, 2, valueMatrix);
			}
		}

	if ( coefficientMatrix.rows < NUMBER_OF_VARIABLES )
		{
		return false;
		}

	cv::Mat singulaValueMatrix;
	cv::SVD::compute(coefficientMatrix, singulaValueMatrix);
	int rank = cv::countNonZero( singulaValueMatrix > EPSILON );
	return ( rank >= NUMBER_OF_DEGREES_OF_FREEDOM );
	}

cv::Mat LeastSquaresMinimization::SolveLinearSystem(cv::Mat coefficientMatrix, cv::Mat valueMatrix, float& error)
	{
	cv::Mat pseudoInverse;
	cv::invert(coefficientMatrix, pseudoInverse, cv::DECOMP_SVD);
	cv::Mat transformMatrix = pseudoInverse * valueMatrix;

	cv::Mat errorMatrix = coefficientMatrix * transformMatrix - valueMatrix;
	error = cv::norm(errorMatrix);
	return transformMatrix;
	}

void LeastSquaresMinimization::AddTransformOutput(cv::Mat transformMatrix)
	{
	cv::Mat rotationMatrix = (cv::Mat_<float>(3, 3) << 
		transformMatrix.at<float>(0), transformMatrix.at<float>(1), transformMatrix.at<float>(2),
		transformMatrix.at<float>(4), transformMatrix.at<float>(5), transformMatrix.at<float>(6),
		transformMatrix.at<float>(8), transformMatrix.at<float>(9), transformMatrix.at<float>(10) );
	cv::Mat translationVector = (cv::Mat_<float>(3,1) << transformMatrix.at<float>(3), transformMatrix.at<float>(7), transformMatrix.at<float>(11));
	
	cv::Mat position = - rotationMatrix.inv() * translationVector;

	Pose3D pose;
	SetPosition(pose, position.at<float>(0, 0), position.at<float>(1, 0), position.at<float>(2, 0) );

	float qw = std::sqrt(1.00 + rotationMatrix.at<float>(0,0) + rotationMatrix.at<float>(1,1) + rotationMatrix.at<float>(2,2)) / 2;
	float qx = (rotationMatrix.at<float>(2,1) - rotationMatrix.at<float>(1,2)) / ( 4 * qw );
	float qy = (rotationMatrix.at<float>(0,2) - rotationMatrix.at<float>(2,0)) / ( 4 * qw );
	float qz = (rotationMatrix.at<float>(1,0) - rotationMatrix.at<float>(0,1)) / ( 4 * qw );
	SetOrientation(pose, qx, qy, qz, qw);

	AddPose(outTransforms, pose);
	}

const LeastSquaresMinimization::LeastSquaresMinimizationOptionsSet LeastSquaresMinimization::DEFAULT_PARAMETERS =
{
	/*.maximumAllowedError =*/ 0.001
};

void LeastSquaresMinimization::ValidateParameters()
{
	ASSERT(parameters.maximumAllowedError > 0, "LeastSquaresMinimization Configuration Error: maximumAllowedError has to be positive");
}

void LeastSquaresMinimization::ValidateInputs(const CorrespondenceMap3D& map)
{
	
}

}
}
}

/** @} */
