/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2DSub.hpp
 * @date 15/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * C++ wrapper for the VisualPointFeatureVector2D. Substitutive proposal.
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

typedef CTypes::Point2D Point2D


/* --------------------------------------------------------------------------
 *
 * Iterator class definition
 *
 * --------------------------------------------------------------------------
 */
class Iterator
	{
	public:
		Iterator& operator++();
		static bool operator==(const Iterator& iterator1, const Iterator& iterator2);
	protected:
		int index;
		Iterator(int index);
	private:
	};

/* --------------------------------------------------------------------------
 *
 * ReadDescriptor2DIterator class definition
 *
 * --------------------------------------------------------------------------
 */
class ReadDescriptor2DIterator : public Iterator()
	{
	friend class ReadVisualPointFeature2DIterator;
	public:
		ReadDescriptor2DIterator& operator++();
		float GetValue() const;
	protected:
		ReadDescriptorIterator(const CTypes::VisualPointFeature2D_descriptor const* descriptor, int index);
	private:
		const CTypes::VisualPointFeature2D_descriptor const* descriptor; 
	};

/* --------------------------------------------------------------------------
 *
 * WriteDescriptor2DIterator class definition
 *
 * --------------------------------------------------------------------------
 */
class WriteDescriptor2DIterator : public Iterator()
	{
	friend class WriteVisualPointFeature2DIterator;
	public:
		WriteDescriptor2DIterator& operator++();
		void SetValue(const float value);
	protected:
		WriteDescriptorIterator(const CTypes::VisualPointFeature2D_descriptor* descriptor, int index);
	private:
		const CTypes::VisualPointFeature2D_descriptor* descriptor;
	};

/* --------------------------------------------------------------------------
 *
 * ReadVisualPointFeature2DIterator class definition
 *
 * --------------------------------------------------------------------------
 */
class ReadVisualPointFeature2DIterator : public Iterator()
	{
	friend class VisualPointFeatureVector2D;
	public:
		ReadVisualPointFeature2DIterator& operator++();
		Point2D GetPoint() const;
		ReadDescriptor2DIterator BeginDescriptor() const;
		ReadDescriptor2DIterator EndDescriptor() const;
	protected:
		ReadVisualPointFeature2DIterator(const CTypes::VisualPointFeatureVector2D const* featuresVector, int index);
	private:
		const CTypes::VisualPointFeatureVector2D const* featuresVector;
	};

/* --------------------------------------------------------------------------
 *
 * WriteVisualPointFeature2DIterator class definition
 *
 * --------------------------------------------------------------------------
 */
class WriteVisualPointFeature2DIterator : public Iterator()
	{
	friend class VisualPointFeatureVector2D;
	public:
		WriteVisualPointFeature2DIterator& operator++();
		SetPoint(const Point2D& point);
		WriteDescriptor2DIterator BeginDescriptor();
		WriteDescriptor2DIterator EndDescriptor();
	protected:
		WriteVisualPointFeature2DIterator(const CTypes::VisualPointFeatureVector2D* featuresVector, int index);
	private:
		const CTypes::VisualPointFeatureVector2D* featuresVector;
	};

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

		VisualPointFeatureVector2D(int descriptorSize);
		~VisualPointFeatureVector2D();
	
		WriteVisualPointFeature2DIterator AddPoint(const Point2D& point);
		void Clear();
		int GetSize() const;
		int GetDescriptorSize() const;
		ReadVisualPointFeature2DIterator BeginRead() const;
		ReadVisualPointFeature2DIterator EndRead() const;

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

		class 

		static const int MAX_FEATURE_2D_POINTS;
		static const int MAX_DESCRIPTOR_2D_LENGTH;

		CTypes::VisualPointFeatureVector2D featuresVector;
		int descriptorSize;

	};



}
#endif

/* VisualPointFeatureVector2DSub.hpp */
/** @} */
