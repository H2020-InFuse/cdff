
/**
 * @author Raul Dominguez
 */

#ifndef CENTRAL_DPM_HPP
#define CENTRAL_DPM_HPP

#include <Types/C/Pointcloud.h>
#include <Types/C/Map.h>

namespace CDFF
{
namespace Support
{

	class CentralDPM
	{
		public:

			CentralDPM(){};
			virtual ~CentralDPM(){};

			/**
			 * Store the received pointcloud in disk associated to the id provided 
			 * 
			 */
			void savePointcloud(const asn1SccPointcloud& pcl, const asn1SccT_String& pcl_id);

			/**
			 * Store the received map in disk associated to the id provided 
			 * 
			 */
			void saveMap(const asn1SccMap& map, const asn1SccT_String& map_id);


	};

}
}
#endif // CENTRAL_DPM_HPP