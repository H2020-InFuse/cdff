/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup MatrixWrapper
 * @{
 */

#include "Matrix.hpp"
#include <Errors/Assert.hpp>

namespace MatrixWrapper
{

using namespace BaseTypesWrapper;

void Copy(const Matrix3d& source, Matrix3d& destination)
{
	for (unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
	{
		for (unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
		{
			SetElement(destination, rowIndex, columnIndex, GetElement(source, rowIndex, columnIndex));
		}
	}
}

Matrix3dPtr NewMatrix3d(InitializationType initializationType)
{
	Matrix3dPtr newMatrix = new Matrix3d();

	switch(initializationType)
	{
		case ALL_ZEROES:
		{
			SetZeroMatrix(*newMatrix);
			break;
		}
		case ALL_ONES:
		{
			SetOneMatrix(*newMatrix);
			break;
		}
		case IDENTITY:
		{
			SetIdentityMatrix(*newMatrix);
			break;
		}
		default:
		{
			ASSERT(false, "Matrix3d, unhandled initialization type");
		}
	}

	return newMatrix;
}

void SetZeroMatrix(Matrix3d& matrix)
{
	for (unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
	{
		for (unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
		{
			SetElement(matrix, rowIndex, columnIndex, 0);
		}
	}
}

void SetOneMatrix(Matrix3d& matrix)
{
	for (unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
	{
		for (unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
		{
			SetElement(matrix, rowIndex, columnIndex, 1);
		}
	}
}

void SetIdentityMatrix(Matrix3d& matrix)
{
	for (unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
	{
		for (unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
		{
			SetElement(matrix, rowIndex, columnIndex, (columnIndex == rowIndex) ? 1 : 0);
		}
	}
}

Matrix3dSharedPtr NewSharedMatrix3d(InitializationType initializationType)
{
	Matrix3dPtr newMatrix = NewMatrix3d(initializationType);
	Matrix3dSharedPtr sharedMatrix(newMatrix);
	return sharedMatrix;
}

T_Double GetElement(const Matrix3d& matrix, unsigned rowIndex, unsigned columnIndex)
{
	return matrix.arr[rowIndex].arr[columnIndex];
}

void SetElement(Matrix3d& matrix, unsigned rowIndex, unsigned columnIndex, T_Double value)
{
	matrix.arr[rowIndex].arr[columnIndex] = value;
}

BitStream ConvertToBitStream(const Matrix3d& matrix)
	CONVERT_TO_BIT_STREAM(matrix, asn1SccMatrix3d_REQUIRED_BYTES_FOR_ENCODING, asn1SccMatrix3d_Encode)

void ConvertFromBitStream(BitStream bitStream, Matrix3d& matrix)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccMatrix3d_REQUIRED_BYTES_FOR_ENCODING, matrix, asn1SccMatrix3d_Decode)

}

/** @} */
