/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector2DConverter.cpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of the MatToVisualPointFeatureVector2DConverter class.
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
#include "MatToVisualPointFeatureVector2DConverter.hpp"
#include <Errors/Assert.hpp>


namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualPointFeatureVector2D* MatToVisualPointFeatureVector2DConverter::Convert(cv::Mat featuresVector)
	{
	static unsigned time = 0;
	time++;

	VisualPointFeatureVector2D** plannedOutput = (VisualPointFeatureVector2D**) Mock::GetBehaviour("Convert", time); 

	if (plannedOutput == NULL)
		{
		return Types::MatToVisualPointFeatureVector2DConverter::Convert(featuresVector);
		}
	else
		{
		return (*plannedOutput);
		}
	}

}

/** @} */
