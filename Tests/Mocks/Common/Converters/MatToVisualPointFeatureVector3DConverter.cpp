/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector3DConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of the MatToVisualPointFeatureVector3DConverter class.
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
#include "MatToVisualPointFeatureVector3DConverter.hpp"
#include <Errors/Assert.hpp>
#include "Mocks/MockMacro.hpp"

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
MatToVisualPointFeatureVector3DConverter::~MatToVisualPointFeatureVector3DConverter()
	{

	}

CppTypes::VisualPointFeatureVector3D::ConstPtr MatToVisualPointFeatureVector3DConverter::Convert(const cv::Mat featuresVector)
	MOCK_METHOD(Converters::MatToVisualPointFeatureVector3DConverter, Convert, CppTypes::VisualPointFeatureVector3D::ConstPtr, (featuresVector) )

}

/** @} */
