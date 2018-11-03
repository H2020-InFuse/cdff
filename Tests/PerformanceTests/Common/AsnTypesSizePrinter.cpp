/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file AsnTypesSizePrinter.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup PerformaceTests
 * 
 * The application will print the size of the ASN defined types
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
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/CorrespondenceMap3D.hpp>
#include <Types/CPP/Matrix.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <DepthMap.h>
#include <LaserScan.h>
#include <Errors/Assert.hpp>
#include <string>

using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace FrameWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace CorrespondenceMap3DWrapper;
using namespace MatrixWrapper;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Main
 *
 * --------------------------------------------------------------------------
 */

std::string PrintSize(unsigned long size)
	{
	static const unsigned K_THRESHOLD = 1024;
	static const unsigned M_THRESHOLD = 1024*1024;
	static const unsigned G_THRESHOLD = 1024*1024*1024;

	std::stringstream stream;
	if (size < K_THRESHOLD)
		{
		stream << size << " bytes";
		}
	else if (size < M_THRESHOLD)
		{
		stream << ((float)size) / ((float)K_THRESHOLD) << " kbytes";
		}
	else if (size < G_THRESHOLD)
		{
		stream << ((float)size) / ((float)M_THRESHOLD) << " Mbytes";
		}
	else
		{
		stream << ((float)size) / ((float)G_THRESHOLD) << " Gbytes";
		}

	return stream.str();
	}

int main(int argc, char** argv)
	{
	PRINT_TO_LOG("Frame Size:                      ", PrintSize(sizeof(Frame)) );
	PRINT_TO_LOG("PointCloud Size:                 ", PrintSize(sizeof(PointCloud)) );
	PRINT_TO_LOG("VisualPointFeatureVector2D Size: ", PrintSize(sizeof(VisualPointFeatureVector2D)) );
	PRINT_TO_LOG("VisualPointFeatureVector3D Size: ", PrintSize(sizeof(VisualPointFeatureVector3D)) );
	PRINT_TO_LOG("CorrespondenceMap2D Size:        ", PrintSize(sizeof(CorrespondenceMap2D)) );
	PRINT_TO_LOG("CorrespondenceMap3D Size:        ", PrintSize(sizeof(CorrespondenceMap3D)) );
	PRINT_TO_LOG("Matrix3d Size:                   ", PrintSize(sizeof(Matrix3d)) );
	PRINT_TO_LOG("Pose3D Size:                     ", PrintSize(sizeof(Pose3D)) );
	PRINT_TO_LOG("Pose2D Size:                     ", PrintSize(sizeof(Pose2D)) );
        PRINT_TO_LOG("asn1SccDepthMap Size:            ", PrintSize(sizeof(asn1SccDepthMap)) );
        PRINT_TO_LOG("asn1SccLaserScan Size:           ", PrintSize(sizeof(asn1SccLaserScan)) );
	};

/** @} */
