/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CppTypes
 *
 * C++ wrapper for VisualPointFeatureVector2D: substitutive proposal
 *
 * @{
 */

#ifndef VISUAL_POINT_FEATURE_VECTOR_2D_HPP
#define VISUAL_POINT_FEATURE_VECTOR_2D_HPP

#include <VisualPointFeatureVector2D.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

namespace CppTypes
{

// Types

typedef asn1SccPoint2D Point2D

// Classes

class Iterator
{
	public:
		Iterator& operator++();
		static bool operator==(const Iterator& iterator1, const Iterator& iterator2);
	protected:
		int index;
		Iterator(int index);
};

class ReadDescriptor2DIterator : public Iterator()
{
	friend class ReadVisualPointFeature2DIterator;
	public:
		ReadDescriptor2DIterator& operator++();
		float GetValue() const;
	protected:
		ReadDescriptorIterator(const asn1SccVisualPointFeature2D_descriptor const* descriptor, int index);
	private:
		const asn1SccVisualPointFeature2D_descriptor const* descriptor;
};

class WriteDescriptor2DIterator : public Iterator()
{
	friend class WriteVisualPointFeature2DIterator;
	public:
		WriteDescriptor2DIterator& operator++();
		void SetValue(const float value);
	protected:
		WriteDescriptorIterator(const asn1SccVisualPointFeature2D_descriptor* descriptor, int index);
	private:
		const asn1SccVisualPointFeature2D_descriptor* descriptor;
};

class ReadVisualPointFeature2DIterator : public Iterator()
{
	friend class VisualPointFeatureVector2D;
	public:
		ReadVisualPointFeature2DIterator& operator++();
		Point2D GetPoint() const;
		ReadDescriptor2DIterator BeginDescriptor() const;
		ReadDescriptor2DIterator EndDescriptor() const;
	protected:
		ReadVisualPointFeature2DIterator(const asn1SccVisualPointFeatureVector2D const* featuresVector, int index);
	private:
		const asn1SccVisualPointFeatureVector2D const* featuresVector;
};

class WriteVisualPointFeature2DIterator : public Iterator()
{
	friend class VisualPointFeatureVector2D;
	public:
		WriteVisualPointFeature2DIterator& operator++();
		SetPoint(const Point2D& point);
		WriteDescriptor2DIterator BeginDescriptor();
		WriteDescriptor2DIterator EndDescriptor();
	protected:
		WriteVisualPointFeature2DIterator(const asn1SccVisualPointFeatureVector2D* featuresVector, int index);
	private:
		const asn1SccVisualPointFeatureVector2D* featuresVector;
};

// Main class

class VisualPointFeatureVector2D
{
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

	private:

		static const int MAX_FEATURE_2D_POINTS;
		static const int MAX_DESCRIPTOR_2D_LENGTH;

		asn1SccVisualPointFeatureVector2D featuresVector;
		int descriptorSize;

};

}

#endif // VISUAL_POINT_FEATURE_VECTOR_2D_HPP

/** @} */
