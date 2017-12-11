/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2D.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * C++ wrapper for the VisualPointFeatureVector2D
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
#include <VisualPointFeatureVector2D.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef VISUAL_POINT_FEATURE_VECTOR_HPP
#define VISUAL_POINT_FEATURE_VECTOR_HPP

namespace CppTypes 
{
/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */

typedef CTypes::VisualPointFeature2D VisualPointFeature2D;
typedef CTypes::VisualPointFeature2D_descriptor VisualPointDescriptor2D;



/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector2D
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:

		typedef std::shared_ptr<VisualPointFeatureVector2D> Ptr;
		typedef std::shared_ptr<const VisualPointFeatureVector2D> ConstPtr;

		VisualPointFeatureVector2D();
		~VisualPointFeatureVector2D();

		void AddPoint(uint16_t x, uint16_t y);
		void ClearPoints();
		int GetNumberOfPoints() const;
		int GetXCoordinate(int pointIndex) const;
		int GetYCoordinate(int pointIndex) const;

		void AddDescriptorComponent(int pointIndex, float component);
		void ClearDescriptor(int pointIndex);
		int GetNumberOfDescriptorComponents(int pointIndex) const;
		float GetDescriptorComponent(int pointIndex, int componentIndex) const;

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

		static const T_UInt32 MAX_FEATURE_2D_POINTS;
		static const T_UInt32 MAX_DESCRIPTOR_2D_LENGTH;

		CTypes::VisualPointFeatureVector2D featuresVector;
		

	};



}
#endif

/* VisualPointFeatureVector2D.hpp */
/** @} */
