/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ConversionCache.hpp
 * @date 20/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is the class for Conversion Cache. It is supposed to store the most recent conversions.
 *  This is a template class, it depends on the origin type, then end type and the specific conversion class used.
 *
 * @{
 */

#ifndef CONVERSION_CACHE_HPP
#define CONVERSION_CACHE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Errors/Assert.hpp>
#include "CacheHandler.hpp"



namespace Common {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
template<class FromType, class ToType, class Converter>
class ConversionCache
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		static ConversionCache<FromType, ToType, Converter>& Instance() 
			{
			if (instance == NULL)
				{
				instance = new ConversionCache<FromType, ToType, Converter>();
				}
			return *instance;
			}

		static ToType Convert(const FromType& from)
			{
			ConversionCache<FromType, ToType, Converter>& theInstance =  ConversionCache<FromType, ToType, Converter>::Instance();
			return theInstance.PrivateConvert(from);
			}

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
		static ConversionCache<FromType, ToType, Converter>* instance;
		CacheHandler<FromType, ToType>* cacheHandler;
		Converter* converter;

		ConversionCache()
			{
			cacheHandler = new CacheHandler<FromType, ToType>();
			converter = new Converter();
			}

		~ConversionCache()
			{
			delete (cacheHandler);
			delete (converter);
			}

		ToType PrivateConvert(const FromType& from)
			{
			ToType conversion;
			if ( !cacheHandler->Find(from, conversion) )
				{
				conversion = converter->Convert(from);
				cacheHandler->Insert(from, conversion);
				}
			return conversion;
			}


	/* --------------------------------------------------------------------
	 * Testing Only
	 * --------------------------------------------------------------------
	 */
	#ifdef TESTING
	public:
	static ConversionCache<FromType, ToType, Converter>& Instance(CacheHandler<FromType, ToType>* cacheHandler, Converter* converter) 
			{
			if (instance != NULL)
				{
				PRINT_TO_LOG("Deleting cache", "");
				delete(instance);
				}
			instance = new ConversionCache<FromType, ToType, Converter>(cacheHandler, converter);
			return *instance;
			}

	private:
		ConversionCache(CacheHandler<FromType, ToType>* cacheHandler, Converter* converter)
			{
			this->cacheHandler = cacheHandler;
			this->converter = converter;
			}		
	#endif

	};

template<class FromType, class ToType, class Converter>
ConversionCache<FromType, ToType, Converter>* ConversionCache<FromType, ToType, Converter>::instance = NULL;



}

#endif

/* ConversionCache.hpp */
/** @} */
