#include "CentralDPM.hpp"
#include <iostream>
#include <stdlib.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/plugin/ClassLoader.hpp>
#include <envire_core/items/Item.hpp>

using namespace envire::core;
namespace CDFF
{

    void CentralDPM::storePointcloud(const asn1SccPointcloud& pcl, const asn1SccT_String& pcl_id)
    {
        EnvireGraph g;

        std::string itemname = "envire::core::Item<asn1SccPointcloud>";
        
        envire::core::ClassLoader *loader = envire::core::ClassLoader::getInstance();

        envire::core::Item<asn1SccPointcloud>::Ptr item;

        if (loader->hasClass(itemname)) // Check that the class can be used with class_loader?
        {
            if (loader->createEnvireItem(itemname, item)) // Creates the intsance of the item behind thje pointer
            {

                item->setData(pcl);
                item->contentsChanged();
                std::cout << "We created an Item with a pcl in it" << std::endl;

                const FrameId frame("PCL");
                g.addFrame(frame);
                g.addItemToFrame(frame, item);
                std::cout << "We created a frame and added the pcl to it" << std::endl;

                //EnvireGraph h;
                char charname[pcl_id.nCount];
                memcpy( charname, &pcl_id.arr, pcl_id.nCount);
                std::string graph_id(charname);
                std::string store_file("./"+graph_id+".graph");
                g.saveToFile(store_file);
                std::cout << "Graph saved to file "<< store_file  << std::endl;
            }
        }
        else
        {
            throw std::invalid_argument("no class with name asn1SccPointcloud found in plugins");
        }

    }

    void CentralDPM::storeMap(const asn1SccMap& map, const asn1SccT_String& map_id)
    {
        EnvireGraph g;
        std::string itemname = "envire::core::Item<asn1SccMap>";
        envire::core::ClassLoader *loader = envire::core::ClassLoader::getInstance();
        envire::core::Item<asn1SccMap>::Ptr item;
        if (loader->hasClass(itemname)) // Check that the class can be used with class_loader?
        {
            if (loader->createEnvireItem(itemname, item)) // Creates the intsance of the item behind thje pointer
            {
                item->setData(map);
                item->contentsChanged();
                std::cout << "[CentralDPM::storeMap] We created an Item with a map in it" << std::endl;

                const FrameId frame("Map");
                g.addFrame(frame);
                g.addItemToFrame(frame, item);
                std::cout << "[CentralDPM::storeMap] We created a frame and added the map to it" << std::endl;

                //EnvireGraph h;
                char charname[map_id.nCount+1];
                memcpy( charname, &map_id.arr, map_id.nCount);
                charname[map_id.nCount] = '\0';
                std::string graph_id(charname);

                std::string store_file("./"+graph_id+".graph");
                g.saveToFile(store_file);
                std::cout << "[CentralDPM::storeMap] Graph saved to file "<< store_file  << std::endl;
            }
        }
        else
        {
            throw std::invalid_argument("[CentralDPM::storeMap] No class with name asn1SccMap found in plugins");
        }

    }

}
