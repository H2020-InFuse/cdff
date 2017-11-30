/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CacheHandler.hpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Stubs
 * 
 *  This is a stub for the cache handler. It always says that the conversion was not previously computed.
 *  
 *
 * @{
 */

#ifndef STUB_CACHE_HANDLER
#define STUB_CACHE_HANDLER


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ConversionCache/CacheHandler.hpp>

namespace Stubs {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
template<class FromType, class ToType>
class CacheHandler : public Common::CacheHandler<FromType, ToType>
	{

	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		bool Find(const FromType& from, ToType& to)
			{

			return false;
			}
		
		void Insert(const FromType& from, const ToType& to)
			{

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
