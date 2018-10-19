
/**
 * @author Raul Dominguez
 */

#ifndef CENTRAL_DPM_HPP
#define CENTRAL_DPM_HPP

#include <Pointcloud.h>

namespace CDFF
{

	class CentralDPM
	{
		public:

			CentralDPM();
			virtual ~CentralDPM();

			/**
			 * Save the received pointcloud in disk associated to the id provided 
			 * 
			 */
			void storePointcloud(const asn1SccPointcloud& pcl, const asn1SccT_String& pcl_id);





	};

}

#endif // CENTRAL_DPM_HPP