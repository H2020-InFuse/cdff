/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Matrix.hpp
 * @date 26/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup MatrixWrapper
 * 
 * Wrapper for ASN.1 matrix types
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

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
namespace CTypes {
#include <Geometry.h>
#include <CorrespondenceMap3D.h>
#include <Eigen.h>
}

#include <memory>
#include "BaseTypes.hpp"

#ifndef MATRIX_HPP
#define MATRIX_HPP

namespace MatrixWrapper 
{


/* --------------------------------------------------------------------------
 *
 * Types definition
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::Matrix3d Matrix3d;

/* --------------------------------------------------------------------------
 *
 * type initialization enum
 *
 * --------------------------------------------------------------------------
 */
enum InitializationType
	{
	ALL_ZEROES,
	ALL_ONES,
	IDENTITY
	};


/* --------------------------------------------------------------------------
 *
 * Pointer Types
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::Matrix3d* Matrix3dPtr;
typedef CTypes::Matrix3d const* Matrix3dConstPtr;
typedef std::shared_ptr<Matrix3d> Matrix3dSharedPtr;
typedef std::shared_ptr<const Matrix3d> Matrix3dSharedConstPtr;

/* --------------------------------------------------------------------------
 *
 * Helper functions
 *
 * --------------------------------------------------------------------------
 */
void Copy(const Matrix3d& source, Matrix3d& destination);
Matrix3dPtr NewMatrix3d(InitializationType initializationType = ALL_ZEROES);
Matrix3dSharedPtr NewSharedMatrix3d(InitializationType initializationType = ALL_ZEROES);

void SetZeroMatrix(Matrix3d& matrix);
void SetOneMatrix(Matrix3d& matrix);
void SetIdentityMatrix(Matrix3d& matrix);

BaseTypesWrapper::T_Double GetElement(const Matrix3d& matrix, unsigned rowIndex, unsigned columnIndex);
void SetElement(Matrix3d& matrix, unsigned rowIndex, unsigned columnIndex, BaseTypesWrapper::T_Double value);
 


}

#endif


/* Matrix.hpp */
/** @} */
