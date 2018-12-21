#include "CentralDPM.hpp"

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/plugin/ClassLoader.hpp>
#include <envire_core/items/Item.hpp>

#include <Loggers/StandardOutputLogger.hpp>

#include <string>
#include <stdexcept>

using namespace envire::core;
namespace CDFF
{
namespace Support
{

    void CentralDPM::savePointcloud(const asn1SccPointcloud& pcl, const asn1SccT_String& pcl_id)
    {
        std::string log_prefix = "[CentralDPM::storePointcloud] ";
        StandardOutputLogger* logger = new StandardOutputLogger();
        EnvireGraph* g = new EnvireGraph();
        {
            std::string itemname = "envire::core::Item<asn1SccPointcloud>";
            envire::core::ClassLoader *loader = envire::core::ClassLoader::getInstance();
            envire::core::Item<asn1SccPointcloud>::Ptr item;
            if (loader->hasClass(itemname)) // Check that the class can be used with class_loader?
            {
                if (loader->createEnvireItem(itemname, item)) // Creates the intsance of the item behind thje pointer
                {
                    item->setData(pcl);
                    item->contentsChanged();
                    logger->AddEntry(log_prefix + "We created an Item with a pcl in it", Logger::SUCCESS );
                    logger->Print();
                    const FrameId frame("PCL");
                    g->addFrame(frame);
                    g->addItemToFrame(frame, item);
                    logger->AddEntry(log_prefix + "We created a frame and added the pcl to it", Logger::SUCCESS );
                    logger->Print();
                    //EnvireGraph h;
                    char charname[pcl_id.nCount];
                    memcpy( charname, &pcl_id.arr, pcl_id.nCount);
                    std::string graph_id(charname);
                    std::string store_file("./"+graph_id+".graph");
                    g->saveToFile(store_file);
                    logger->AddEntry(log_prefix + "Graph saved to file: " + store_file, Logger::SUCCESS );
                    logger->Print();
                }
            }
            else
            {
                throw std::invalid_argument("no class with name asn1SccPointcloud found in plugins");
            }
        }
        envire::core::ClassLoader::destroyInstance();
        logger->AddEntry(log_prefix + "Loader has been destroyed", Logger::SUCCESS );
        logger->Print();
    }

    void CentralDPM::saveMap(const asn1SccMap& map, const asn1SccT_String& map_id)
    {
        StandardOutputLogger* logger = new StandardOutputLogger();
        std::string log_prefix = "[CentralDPM::storeMap] ";
        EnvireGraph* g = new EnvireGraph();
        std::string itemname = "envire::core::Item<asn1SccMap>";
        envire::core::ClassLoader *loader = envire::core::ClassLoader::getInstance();
        {

            envire::core::Item<asn1SccMap>::Ptr item;
            if (loader->hasClass(itemname)) // Check that the class can be used with class_loader
            {
                if (loader->createEnvireItem(itemname, item)) // Creates the intsance of the item behind thje pointer
                {
                    item->setData(map);
                    item->contentsChanged();
                    logger->AddEntry(log_prefix + "We created an Item with a map in it", Logger::SUCCESS );
                    logger->Print();
                    const FrameId frame("Map");
                    g->addFrame(frame);
                    g->addItemToFrame(frame, item);
                    logger->AddEntry(log_prefix + "We created a frame and added the map to it", Logger::SUCCESS );
                    logger->Print();
                    char charname[map_id.nCount+1];
                    memcpy( charname, &map_id.arr, map_id.nCount);
                    charname[map_id.nCount] = '\0';
                    std::string graph_id(charname);
                    std::string store_file("./"+graph_id+".graph");
                    g->saveToFile(store_file);
                    logger->AddEntry(log_prefix + "Graph saved to file " +store_file, Logger::SUCCESS );
                    logger->Print();

                }
            }
            else
            {
                throw std::invalid_argument("[CentralDPM::storeMap] No class with name asn1SccMap found in plugins");
            }

        }

        envire::core::ClassLoader::destroyInstance();
        logger->AddEntry(log_prefix + "Loader has been destroyed", Logger::SUCCESS );
        logger->Print();

    }

}
}
