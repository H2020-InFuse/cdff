
/**
 * @author Raul Dominguez
 */

#ifndef CENTRAL_DPM_HPP
#define CENTRAL_DPM_HPP

#include <Pointcloud.h>
#include <Map.h>

namespace CDFF
{

	class CentralDPM
	{
		public:

			CentralDPM(){};
			virtual ~CentralDPM(){};

			/**
			 * Save the received pointcloud in disk associated to the id provided 
			 * 
			 */
			void storePointcloud(const asn1SccPointcloud& pcl, const asn1SccT_String& pcl_id);

			/**
			 * Save the received map in disk associated to the id provided 
			 * 
			 */
			void storeMap(const asn1SccMap& map, const asn1SccT_String& map_id);


	};

}

#endif // CENTRAL_DPM_HPP