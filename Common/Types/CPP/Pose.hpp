/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup PoseWrapper
 *
 * Wrapper for ASN.1 Pose type
 *
 * @{
 */

#ifndef POSE_HPP
#define POSE_HPP

#include <Types/C/Pose.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

namespace PoseWrapper
{

// Types

typedef asn1SccPose Pose3D;
typedef asn1SccPose Transform3D;
typedef asn1SccPosition Position3D;
typedef asn1SccOrientation Orientation3D;
typedef asn1SccQuaterniond Quaternion;

typedef asn1SccPose2D Pose2D;
typedef asn1SccPose2D Transform2D;
typedef asn1SccPosition2D Position2D;
typedef asn1SccT_Double Orientation2D;

// Pointer types

typedef Pose3D* Pose3DPtr;
typedef Pose3D const* Pose3DConstPtr;
typedef std::shared_ptr<Pose3D> Pose3DSharedPtr;
typedef std::shared_ptr<const Pose3D> Pose3DSharedConstPtr;

typedef Transform3D* Transform3DPtr;
typedef Transform3D const* Transform3DConstPtr;
typedef std::shared_ptr<Transform3D> Transform3DSharedPtr;
typedef std::shared_ptr<const Transform3D> Transform3DSharedConstPtr;

typedef Pose2D* Pose2DPtr;
typedef Pose2D const* Pose2DConstPtr;
typedef std::shared_ptr<Pose2D> Pose2DSharedPtr;
typedef std::shared_ptr<const Pose2D> Pose2DSharedConstPtr;

typedef Transform2D* Transform2DPtr;
typedef Transform2D const* Transform2DConstPtr;
typedef std::shared_ptr<Transform2D> Transform2DSharedPtr;
typedef std::shared_ptr<const Transform2D> Transform2DSharedConstPtr;

// Functions

void Copy(const Pose3D& source, Pose3D& destination);
Pose3DPtr NewPose3D();
Pose3DSharedPtr NewSharedPose3D();
void Reset(Pose3D& pose);
std::string ToString(const Pose3D& pose);

void SetPosition(Pose3D& pose, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y, BaseTypesWrapper::T_Double z);
BaseTypesWrapper::T_Double GetXPosition(const Pose3D& pose);
BaseTypesWrapper::T_Double GetYPosition(const Pose3D& pose);
BaseTypesWrapper::T_Double GetZPosition(const Pose3D& pose);

void SetOrientation(Pose3D& pose, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y, BaseTypesWrapper::T_Double z, BaseTypesWrapper::T_Double w);
BaseTypesWrapper::T_Double GetXOrientation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetYOrientation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetZOrientation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetWOrientation(const Pose3D& pose);

void SetTranslation(Pose3D& pose, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y, BaseTypesWrapper::T_Double z);
BaseTypesWrapper::T_Double GetXTranslation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetYTranslation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetZTranslation(const Pose3D& pose);

void SetRotation(Pose3D& pose, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y, BaseTypesWrapper::T_Double z, BaseTypesWrapper::T_Double w);
BaseTypesWrapper::T_Double GetXRotation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetYRotation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetZRotation(const Pose3D& pose);
BaseTypesWrapper::T_Double GetWRotation(const Pose3D& pose);

Pose3D Sum(const Pose3D& pose1, const Pose3D& pose2);

BitStream ConvertToBitStream(const Pose3D& pose);
void ConvertFromBitStream(BitStream bitStream, Pose3D& pose);

void Copy(const Pose2D& source, Pose2D& destination);
Pose2DPtr NewPose2D();
Pose2DSharedPtr NewSharedPose2D();
void Reset(Pose2D& pose);
std::string ToString(const Pose2D& pose);

void SetPosition(Pose2D& pose, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y);
BaseTypesWrapper::T_Double GetXPosition(const Pose2D& pose);
BaseTypesWrapper::T_Double GetYPosition(const Pose2D& pose);

void SetOrientation(Pose2D& pose, BaseTypesWrapper::T_Double angle);
BaseTypesWrapper::T_Double GetOrientation(const Pose2D& pose);

void SetTranslation(Pose2D& pose, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y);
BaseTypesWrapper::T_Double GetXTranslation(const Pose2D& pose);
BaseTypesWrapper::T_Double GetYTranslation(const Pose2D& pose);

void SetRotation(Pose2D& pose, BaseTypesWrapper::T_Double angle);
BaseTypesWrapper::T_Double GetRotation(const Pose2D& pose);

BitStream ConvertToBitStream(const Pose2D& pose);
void ConvertFromBitStream(BitStream bitStream, Pose2D& pose);

}

#endif // POSE_HPP

/** @} */
