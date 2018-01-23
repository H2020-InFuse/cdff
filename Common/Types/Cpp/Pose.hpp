/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Pose.hpp
 * @date 18/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup PoseWrapper
 * 
 * Pose namespace wrapper for Pose type
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
namespace CTypes {
#include <Pose.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

#ifndef POSE_HPP
#define POSE_HPP


/* --------------------------------------------------------------------------
 *
 * Frame namespace
 *
 * --------------------------------------------------------------------------
 */
namespace PoseWrapper
{

/* --------------------------------------------------------------------------
 *
 * Types definition
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::Pose Pose3D;
typedef CTypes::Pose Transform3D;
typedef CTypes::Position Position3D;
typedef CTypes::Orientation Orientation3D;
typedef CTypes::Quaterniond Quaternion;

typedef CTypes::Pose2D Pose2D;
typedef CTypes::Pose2D Transform2D;
typedef CTypes::Position2D Position2D;
typedef CTypes::T_Double Orientation2D;


/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<Pose3D> Pose3DSharedPtr;
typedef std::shared_ptr<const Pose3D> Pose3DSharedConstPtr;
typedef Pose3D* Pose3DPtr;
typedef Pose3D const* Pose3DConstPtr;

typedef std::shared_ptr<Transform3D> Transform3DSharedPtr;
typedef std::shared_ptr<const Transform3D> Transform3DSharedConstPtr;
typedef Transform3D* Transform3DPtr;
typedef Transform3D const* Transform3DConstPtr;

typedef std::shared_ptr<Pose2D> Pose2DSharedPtr;
typedef std::shared_ptr<const Pose2D> Pose2DSharedConstPtr;
typedef Pose2D* Pose2DPtr;
typedef Pose2D const* Pose2DConstPtr;

typedef std::shared_ptr<Transform2D> Transform2DSharedPtr;
typedef std::shared_ptr<const Transform2D> Transform2DSharedConstPtr;
typedef Transform2D* Transform2DPtr;
typedef Transform2D const* Transform2DConstPtr;



/* --------------------------------------------------------------------------
 *
 * Access Functions definition
 *
 * --------------------------------------------------------------------------
 */
void Copy(const Pose3D& source, Pose3D& destination);

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

void Reset(Pose3D& pose);

void Copy(const Pose2D& source, Pose2D& destination);

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

void Reset(Pose2D& pose);
}

#endif

/* TransformWithCovarianceWrapper.hpp */
/** @} */
