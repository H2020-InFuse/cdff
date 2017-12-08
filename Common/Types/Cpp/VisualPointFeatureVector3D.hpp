/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3D.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * C++ wrapper for the VisualPointFeatureVector3D
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
#include <VisualPointFeatureVector3D.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef VISUAL_POINT_FEATURE_VECTOR_3D_HPP
#define VISUAL_POINT_FEATURE_VECTOR_3D_HPP

namespace CppTypes 
{
/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */

typedef CTypes::VisualPointFeature3D VisualPointFeature3D;
typedef CTypes::VisualPointFeature3D_descriptor VisualPointDescriptor3D;



/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector3D
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:

		typedef std::shared_ptr<VisualPointFeatureVector3D> Ptr;
		typedef std::shared_ptr<const VisualPointFeatureVector3D> ConstPtr;

		VisualPointFeatureVector3D();
		~VisualPointFeatureVector3D();

		void AddPoint(float x, float y, float z);
		void ClearPoints();
		int GetNumberOfPoints() const;
		float GetXCoordinate(int pointIndex) const;
		float GetYCoordinate(int pointIndex) const;
		float GetZCoordinate(int pointIndex) const;

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:

		static const T_UInt32 MAX_FEATURE_3D_POINTS;
		static const T_UInt32 MAX_DESCRIPTOR_3D_LENGTH;

		CTypes::VisualPointFeatureVector3D featuresVector;
		

	};



}
#endif

/* VisualPointFeatureVector3D.hpp */
/** @} */
