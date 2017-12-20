/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2DSub.cpp
 * @date 15/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * Implementation of VisualPointFeatureVector2DSub class.
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
#include "VisualPointFeatureVector2D.hpp"
#include <Errors/Assert.hpp>

namespace CppTypes
{

/* --------------------------------------------------------------------------
 *
 * Iterator class methods
 *
 * --------------------------------------------------------------------------
 */
Iterator& Iterator::operator++()
	{
	index++;
	return this;
	}

bool Iterator::operator==(const Iterator& iterator1, const Iterator& iterator2)
	{
	return iterator1.index == iterator2.index;
	}

Iterator::Iterator(int index)
	{
	this->index = index;
	}

/* --------------------------------------------------------------------------
 *
 * ReadDescriptor2DIterator class methods
 *
 * --------------------------------------------------------------------------
 */
ReadDescriptor2DIterator& ReadDescriptor2DIterator::operator++()
	{
	Iterator::operator++();
	return this;
	}

float ReadDescriptor2DIterator::GetValue() const
	{
	ASSERT_ON_TEST(index < descriptor.nCount, "Descriptor2D trying to access a missing descriptor element");
	return descriptor.arr[index];
	}

ReadDescriptor2DIterator::ReadDescriptorIterator(const CTypes::VisualPointFeature2D_descriptor const* descriptor, int index)
	: descriptor(descriptor), Index(index);
	{

	}


/* --------------------------------------------------------------------------
 *
 * WriteDescriptorIterator class methods
 *
 * --------------------------------------------------------------------------
 */
WriteDescriptorIterator& WriteDescriptorIterator::operator++()
	{
	Iterator::operator++();
	return this;
	}

void WriteDescriptorIterator::SetValue(const float value)
	{
	ASSERT_ON_TEST(index < descriptor.nCount, "Descriptor2D trying to access a missing descriptor element");
	descriptor.arr[index] = value;
	}

WriteDescriptorIterator::WriteDescriptorIterator(const CTypes::VisualPointFeature2D_descriptor* descriptor, int index)
	: descriptor(descriptor), Index(index);
	{

	}


/* --------------------------------------------------------------------------
 *
 * ReadVisualPointFeature2DIterator class methods
 *
 * --------------------------------------------------------------------------
 */
ReadVisualPointFeature2DIterator& ReadVisualPointFeature2DIterator::operator++()
	{
	Iterator::operator++();
	return this;
	}

Point2D ReadVisualPointFeature2DIterator::GetPoint() const
	{
	ASSERT_ON_TEST(index < featuresVector.nCount, "VisualPointDescriptorVector2D trying to access a missing point");
	return featuresVector.arr[index].point;
	}

ReadDescriptor2DIterator ReadVisualPointFeature2DIterator::BeginDescriptor() const
	{
	return ReadDescriptor2DIterator(&featuresVector, 0);
	}

ReadDescriptor2DIterator ReadVisualPointFeature2DIterator::EndDescriptor() const
	{
	return ReadDescriptor2DIterator(&featuresVector, featuresVector.arr[index].descriptor.nCount);
	}

ReadVisualPointFeature2DIterator::ReadVisualPointFeature2DIterator(const CTypes::VisualPointFeatureVector2D const* featuresVector, int index)
	:featuresVector(featuresVector), Index(index);
	{

	}


/* --------------------------------------------------------------------------
 *
 * WriteVisualPointFeature2DIterator class methods
 *
 * --------------------------------------------------------------------------
 */
WriteVisualPointFeature2DIterator& WriteVisualPointFeature2DIterator::operator++()
	{
	Iterator::operator++();
	return this;
	}

void WriteVisualPointFeature2DIterator::SetPoint(const Point2D& point)
	{
	ASSERT_ON_TEST(index < featuresVector.nCount, "VisualPointDescriptorVector2D trying to access a missing point");
	featuresVector.arr[index].point = point;
	}

WriteDescriptor2DIterator WriteVisualPointFeature2DIterator::BeginDescriptor()
	{
	return WriteDescriptor2DIterator(&featuresVector, 0);
	}

WriteDescriptor2DIterator WriteVisualPointFeature2DIterator::EndDescriptor()
	{
	return WriteDescriptor2DIterator(&featuresVector, featuresVector.arr[index].descriptor.nCount);
	}

WriteVisualPointFeature2DIterator::WriteVisualPointFeature2DIterator(const CTypes::VisualPointFeatureVector2D* featuresVector, int index)
	:featuresVector(featuresVector), Index(index);
	{

	}


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

VisualPointFeatureVector2D(int descriptorSize)
	{
	ASSERT(descriptorSize <= MAX_DESCRIPTOR_2D_LENGTH, "2D Descriptor length is larger than allowed");
	this->descriptorSize = descriptorSize;
	Clear();
	}

~VisualPointFeatureVector2D()
	{

	}
	
WriteVisualPointFeature2DIterator AddPoint(const Point2D point&)
	{
	ASSERT_ON_TEST(featuresVector.nCount < MAX_FEATURE_2D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point = point;
	featuresVector.arr[currentIndex].descriptor.nCount = descriptorSize;
	featuresVector.nCount++;	
	return WriteVisualPointFeature3DIterator(&featuresVector, currentIndex);
	}

void Clear()
	{
	featuresVector.nCount = 0;	
	}

int GetSize() const
	{
	return featuresVector.nCount;
	}

int GetDescriptorSize() const
	{
	return descriptorSize;
	}

ReadVisualPointFeature3DIterator BeginRead() const
	{
	return ReadVisualPointFeature3DIterator(&featuresVector, 0);
	}

ReadVisualPointFeature3DIterator EndRead() const
	{
	return ReadVisualPointFeature3DIterator(&featuresVector, featuresVector.nCount);
	}

VisualPointFeatureVector2D::VisualPointFeatureVector2D()
	{
	ClearPoints();
	}

VisualPointFeatureVector2D::~VisualPointFeatureVector2D()
	{

	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const int VisualPointFeatureVector2D::MAX_FEATURE_2D_POINTS = static_cast<int>(CTypes::features2DElementsMax);
const int VisualPointFeatureVector2D::MAX_DESCRIPTOR_2D_LENGTH = static_cast<int>(CTypes::descriptor2DNameLength);


}

/** @} */
