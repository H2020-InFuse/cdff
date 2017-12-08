/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * C++ wrapper for the PointCloud
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
#include <Pointcloud.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

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
class PointCloud
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:

		typedef std::shared_ptr<PointCloud> Ptr;
		typedef std::shared_ptr<const PointCloud> ConstPtr;

		PointCloud();
		~PointCloud();

		void AddPoint(T_Double x, T_Double y, T_Double z);
		void ClearPoints();
		int GetNumberOfPoints() const;
		T_Double GetXCoordinate(int pointIndex) const;
		T_Double GetYCoordinate(int pointIndex) const;
		T_Double GetZCoordinate(int pointIndex) const;

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
		static const T_UInt32 MAX_CLOUD_SIZE;

		CTypes::Pointcloud pointCloud;
		bool isColored;

	};



}
#endif

/* PointCloud.hpp */
/** @} */
