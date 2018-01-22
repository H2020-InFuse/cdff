/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CacheHandler.hpp
 * @date 20/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is the class for Handling the cache. It is supposed to manage the pairs of previous conversions.
 *  This is a template class, it depends on the origin type, and then end type.
 *
 * @{
 */

#ifndef CACHE_HANDLER_HPP
#define CACHE_HANDLER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Errors/Assert.hpp>



namespace Common {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
template<class FromType, class ToType>
class CacheHandler
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		CacheHandler()
			{

			}

		virtual ~CacheHandler()
			{

			}

		virtual bool Find(const FromType& from, ToType& to)
			{
			ASSERT(false, "Find method of cache handler was not implemented");
			return false;
			}
		
		virtual void Insert(const FromType& from, const ToType& to)
			{
			ASSERT(false, "Insert method of cache handler was not implemented");
			ASSERT(false, "Write a test to make sure that a this method creates a copy of the shared pointers, otherwise the objects will be eventually deleted");
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
	
	};


}

#endif

/* CacheHandler.hpp */
/** @} */
