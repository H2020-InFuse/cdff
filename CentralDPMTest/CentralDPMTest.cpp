#include <iostream>
#include <CentralDPM.hpp>
#include <PointCloud.hpp>
#include <string.h>



using namespace CDFF;
using namespace PointCloudWrapper;

int main(){

    CentralDPM* dpm; 

	PointCloudPtr asnPointCloud = NewPointCloud();
	AddPoint(*asnPointCloud, 0, 0, 0);

    asn1SccT_String pointcloud_id{ strlen("Cloud_Point_007"), "Cloud_Point_007"};
    
    dpm->storePointcloud( *asnPointCloud, pointcloud_id);

}