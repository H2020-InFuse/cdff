/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup PoseWrapper
 * @{
 */

#include "Pose.hpp"
#include "Errors/AssertOnTest.hpp"
#include <cmath>
#include <stdlib.h>
#include <sstream>

namespace PoseWrapper
{

using namespace BaseTypesWrapper;

void Copy(const Pose3D& source, Pose3D& destination)
{
	SetPosition(destination, GetXPosition(source), GetYPosition(source), GetZPosition(source));
	SetOrientation(destination, GetXOrientation(source), GetYOrientation(source), GetZOrientation(source), GetWOrientation(source));
}

void SetPosition(Pose3D& pose, T_Double x, T_Double y, T_Double z)
{
	pose.pos.arr[0] = x;
	pose.pos.arr[1] = y;
	pose.pos.arr[2] = z;
}

Pose3DPtr NewPose3D()
{
	Pose3DPtr pose = new Pose3D();
	Reset(*pose);
	return pose;
}

Pose3DSharedPtr NewSharedPose3D()
{
	Pose3DSharedPtr sharedPose = std::make_shared<Pose3D>();
	Reset(*sharedPose);
	return sharedPose;
}

void Reset(Pose3D& pose)
{
	SetPosition(pose, 0, 0, 0);
	SetOrientation(pose, 0, 0, 0, 0);
}

std::string ToString(const Pose3D& pose)
{
	std::stringstream poseStream;
	poseStream << "Position: (" << GetXPosition(pose) << ", " << GetYPosition(pose) << ", " << GetZPosition(pose) << ") ";
	poseStream << "Orientation: (" << GetXOrientation(pose) << ", " << GetYOrientation(pose) << ", " << GetZOrientation(pose) << ", " << GetWOrientation(pose) << ")";
	return poseStream.str();
}

T_Double GetXPosition(const Pose3D& pose)
{
	return pose.pos.arr[0];
}

T_Double GetYPosition(const Pose3D& pose)
{
	return pose.pos.arr[1];
}

T_Double GetZPosition(const Pose3D& pose)
{
	return pose.pos.arr[2];
}

void SetOrientation(Pose3D& pose, T_Double x, T_Double y, T_Double z, T_Double w)
{
	pose.orient.arr[0] = x;
	pose.orient.arr[1] = y;
	pose.orient.arr[2] = z;
	pose.orient.arr[3] = w;
}

T_Double GetXOrientation(const Pose3D& pose)
{
	return pose.orient.arr[0];
}

T_Double GetYOrientation(const Pose3D& pose)
{
	return pose.orient.arr[1];
}

T_Double GetZOrientation(const Pose3D& pose)
{
	return pose.orient.arr[2];
}

T_Double GetWOrientation(const Pose3D& pose)
{
	return pose.orient.arr[3];
}

void SetTranslation(Pose3D& pose, T_Double x, T_Double y, T_Double z)
{
	SetPosition(pose, x, y, z);
}

T_Double GetXTranslation(const Pose3D& pose)
{
	return GetXPosition(pose);
}

T_Double GetYTranslation(const Pose3D& pose)
{
	return GetYPosition(pose);
}

T_Double GetZTranslation(const Pose3D& pose)
{
	return GetZPosition(pose);
}

void SetRotation(Pose3D& pose, T_Double x, T_Double y, T_Double z, T_Double w)
{
	SetOrientation(pose, x, y, z, w);
}

T_Double GetXRotation(const Pose3D& pose)
{
	return GetXOrientation(pose);
}

T_Double GetYRotation(const Pose3D& pose)
{
	return GetYOrientation(pose);
}

T_Double GetZRotation(const Pose3D& pose)
{
	return GetZOrientation(pose);
}

T_Double GetWRotation(const Pose3D& pose)
{
	return GetWOrientation(pose);
}

Pose3D Sum(const Pose3D& pose1, const Pose3D& pose2)
	{
	T_Double x1 = GetXPosition(pose1);
	T_Double y1 = GetYPosition(pose1);
	T_Double z1 = GetZPosition(pose1);
	T_Double qx1 = GetXOrientation(pose1);
	T_Double qy1 = GetYOrientation(pose1);
	T_Double qz1 = GetZOrientation(pose1);
	T_Double qw1 = GetWOrientation(pose1);

	T_Double x2 = GetXPosition(pose2);
	T_Double y2 = GetYPosition(pose2);
	T_Double z2 = GetZPosition(pose2);
	T_Double qx2 = GetXOrientation(pose2);
	T_Double qy2 = GetYOrientation(pose2);
	T_Double qz2 = GetZOrientation(pose2);
	T_Double qw2 = GetWOrientation(pose2);

	T_Double x = x1 + x2;
	T_Double y = y1 + y2;
	T_Double z = z1 + z2;
	T_Double qx = (qw2 * qx1 + qx2 * qw1 - qy2 * qz1 + qz2 * qy1);
	T_Double qy = (qw2 * qy1 + qx2 * qz1 + qy2 * qw1 - qz2 * qx1);
	T_Double qz = (qw2 * qz1 - qx2 * qy1 + qy2 * qx1 + qz2 * qw1);
	T_Double qw = (qw2 * qw1 - qx2 * qx1 - qy2 * qy1 - qz2 * qz1);

	Pose3D sum;
	SetPosition(sum, x, y, z);
	SetOrientation(sum, qx, qy, qz, qw);
	return sum;
	}

T_Double ComputeTranslationDistance(const Pose3D& pose1, const Pose3D& pose2)
	{
	T_Double x1 = GetXPosition(pose1);
	T_Double y1 = GetYPosition(pose1);
	T_Double z1 = GetZPosition(pose1);

	T_Double x2 = GetXPosition(pose2);
	T_Double y2 = GetYPosition(pose2);
	T_Double z2 = GetZPosition(pose2);

	T_Double diffX = x1 - x2;
	T_Double diffY = y1 - y2;
	T_Double diffZ = z1 - z2;

	return std::sqrt( diffX*diffX + diffY*diffY + diffZ*diffZ);
	}

BaseTypesWrapper::T_Double ComputeOrientationDistance(const Pose3D& pose1, const Pose3D& pose2)
	{
	T_Double qx1 = GetXOrientation(pose1);
	T_Double qy1 = GetYOrientation(pose1);
	T_Double qz1 = GetZOrientation(pose1);
	T_Double qw1 = GetWOrientation(pose1);

	T_Double qx2 = GetXOrientation(pose2);
	T_Double qy2 = GetYOrientation(pose2);
	T_Double qz2 = GetZOrientation(pose2);
	T_Double qw2 = GetWOrientation(pose2);

	T_Double norm1 = std::sqrt(qx1*qx1 + qy1*qy1 + qz1*qz1 + qw1*qw1);
	T_Double norm2 = std::sqrt(qx2*qx2 + qy2*qy2 + qz2*qz2 + qw2*qw2);

	T_Double normalizedScalarProduct = (qx1*qx2 + qy1*qy2 + qz1*qz2 + qw1*qw2) / (norm1 * norm2);
	return (1 - normalizedScalarProduct);
	}

BitStream ConvertToBitStream(const Pose3D& pose)
	{
	return BaseTypesWrapper::ConvertToBitStream(pose, asn1SccPose_REQUIRED_BYTES_FOR_ENCODING, asn1SccPose_Encode);
	}

void ConvertFromBitStream(BitStream bitStream, Pose3D& pose)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccPose_REQUIRED_BYTES_FOR_ENCODING, pose, asn1SccPose_Decode);
	}

void Copy(const Pose2D& source, Pose2D& destination)
{
	SetPosition(destination, GetXPosition(source), GetYPosition(source));
	SetOrientation(destination, GetOrientation(source));
}

Pose2DPtr NewPose2D()
{
	Pose2DPtr pose = new Pose2D();
	Reset(*pose);
	return pose;
}

Pose2DSharedPtr NewSharedPose2D()
{
	Pose2DSharedPtr sharedPose = std::make_shared<Pose2D>();
	Reset(*sharedPose);
	return sharedPose;
}

void Reset(Pose2D& pose)
{
	SetPosition(pose, 0, 0);
	SetOrientation(pose, 0);
}

std::string ToString(const Pose2D& pose)
{
	std::stringstream poseStream;
	poseStream << "Position: (" << GetXPosition(pose) << ", " << GetYPosition(pose) << ") ";
	poseStream << "Orientation: " << GetOrientation(pose);
	return poseStream.str();
}

void SetPosition(Pose2D& pose, T_Double x, T_Double y)
{
	pose.position.arr[0] = x;
	pose.position.arr[1] = y;
}

T_Double GetXPosition(const Pose2D& pose)
{
	return pose.position.arr[0];
}

T_Double GetYPosition(const Pose2D& pose)
{
	return pose.position.arr[1];
}

void SetOrientation(Pose2D& pose, T_Double angle)
{
	pose.orientation = angle;
}

T_Double GetOrientation(const Pose2D& pose)
{
	return pose.orientation;
}

void SetTranslation(Pose2D& pose, T_Double x, T_Double y)
{
	SetPosition(pose, x, y);
}

T_Double GetXTranslation(const Pose2D& pose)
{
	return GetXPosition(pose);
}

T_Double GetYTranslation(const Pose2D& pose)
{
	return GetYPosition(pose);
}

void SetRotation(Pose2D& pose, T_Double angle)
{
	SetOrientation(pose, angle);
}

T_Double GetRotation(const Pose2D& pose)
{
	return GetOrientation(pose);
}

BitStream ConvertToBitStream(const Pose2D& pose)
	{
	return BaseTypesWrapper::ConvertToBitStream(pose, asn1SccPose2D_REQUIRED_BYTES_FOR_ENCODING, asn1SccPose2D_Encode);
	}

void ConvertFromBitStream(BitStream bitStream, Pose2D& pose)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccPose2D_REQUIRED_BYTES_FOR_ENCODING, pose, asn1SccPose2D_Decode);
	}

}

/** @} */
