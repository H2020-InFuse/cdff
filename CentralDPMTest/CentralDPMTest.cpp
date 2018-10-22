#include <iostream>
#include <CentralDPM.hpp>
#include <PointCloud.hpp>
#include <string.h>

using namespace CDFF;
using namespace PointCloudWrapper;


void set_name(const std::string & name,  asn1SccT_String & map_id)
{
    int name_length = name.length();
    map_id.nCount = name_length;
    std::copy(name.begin(), name.end(), map_id.arr);
    map_id.arr[name_length] = '0';
}

int main(){

    CentralDPM* dpm; 

	PointCloudPtr asnPointCloud = NewPointCloud();
	AddPoint(*asnPointCloud, 0, 0, 0);

    //asn1SccT_String pointcloud_id{ strlen("Cloud_Point_007"), "Cloud_Point_007"};

    std::string pcl_id = "Cloud_Point_007";

    asn1SccT_String pointcloud_id;
    set_name(pcl_id, pointcloud_id);
    
    dpm->storePointcloud( *asnPointCloud, pointcloud_id);


    std::string map_name = "Treasure_Map_001";

    int name_length = map_name.length();

    asn1SccT_String map_id;
    map_id.nCount = name_length;
    std::copy(map_name.begin(), map_name.end(), map_id.arr);
    map_id.arr[name_length] = '0';

    asn1SccMap* asn1Map = new asn1SccMap();
    asn1SccMap_Initialize(asn1Map);

    dpm->storeMap(*asn1Map, map_id);
}