/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CentralDPM.cpp
 * @date 23/10/2018
 * @author Raul Dominguez
 */

/*!
 * @addtogroup CentralDPMTests
 *
 * Unit Test for the Central Data Products Manager
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
#include <catch.hpp>
#include <CentralDPM.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <string>

using namespace PointCloudWrapper;
using namespace CDFF::Support;

void set_name(const std::string & name,  asn1SccT_String & map_id)
{
    int name_length = name.length();
    map_id.nCount = name_length;
    std::copy(name.begin(), name.end(), map_id.arr);
    map_id.arr[name_length] = '0';
}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Start a Central DPM and store a Pointcloud in it with a given name", "[SavePointCloudCentralDPM]" )
{
    //Initialize Inputs
    PointCloudPtr asnPointCloud = NewPointCloud();
    AddPoint(*asnPointCloud, 0, 0, 0);

    std::string pcl_id = "Cloud_Point_007";
    asn1SccT_String pointcloud_id;
    set_name(pcl_id, pointcloud_id);

    // Instantiate Central DPM
    CentralDPM* dpm = new CentralDPM(); 
    dpm->savePointcloud( *asnPointCloud, pointcloud_id);

    // Cleanup
    delete(dpm);
}

TEST_CASE( "Start a Central DPM and store a Map in it with a given name", "[StoreMapCentralDPM]" )
{
    asn1SccMap* asn1Map = new asn1SccMap();
    asn1SccMap_Initialize(asn1Map);

    std::string map_id_std = "Treasure_Map_001";
    asn1SccT_String map_id;
    set_name(map_id_std, map_id);

    CentralDPM* dpm = new CentralDPM(); 
    dpm->saveMap(*asn1Map, map_id);

    // Cleanup
    delete(dpm);

}