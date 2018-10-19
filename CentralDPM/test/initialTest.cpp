#include <iostream>
#include <PointCloud.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/util/Demangle.hpp>
#include <envire_core/plugin/ClassLoader.hpp>
#include <envire_core/items/Item.hpp>
#include <boost/pointer_cast.hpp>
#include <fstream>

using namespace envire::core;
using namespace PointCloudWrapper;

int main() {
    EnvireGraph g;
    std::cout << "You instantiated an Envire Graph" << std::endl;

	PointCloudPtr asnPointCloud = NewPointCloud();

	AddPoint(*asnPointCloud, 0, 0, 0);
    std::cout << "Poincloud Created" << std::endl;

    //envire::core::ItemBase::Ptr item;
    envire::core::Item<asn1SccPointcloud>::Ptr item;

    std::string name = envire::core::demangleTypeName(typeid(*asnPointCloud));
    std::string itemname = "envire::core::Item<" + name + ">";

    envire::core::ClassLoader *loader = envire::core::ClassLoader::getInstance();

    if (loader->hasClass(itemname)) // Check that the class can be used with class_loader?
    {
        if (loader->createEnvireItem(itemname, item)) // Creates the intsance of the item behind thje pointer
        {

            //envire::core::Item<asn1SccPointcloud>::Ptr pclItem; // = boost::dynamic_pointer_cast<envire::core::Item<asn1SccPointcloud>>(item);
            //envire::core::Item<asn1SccPointcloud>::Ptr pclItem = boost::dynamic_pointer_cast<envire::core::Item<asn1SccPointcloud>>(item);
            item->setData(*asnPointCloud);
            item->contentsChanged();
            std::cout << "We created an Item with a pcl in it" << std::endl;

            const FrameId frame("PCL");
            g.addFrame(frame);
            g.addItemToFrame(frame, item);
            std::cout << "We created a frame and added the pcl to it" << std::endl;

            //EnvireGraph h;
            g.saveToFile("/home/appuser/example.graph");
            std::cout << "Graph saved to file" << std::endl;

            /*
            std::stringstream stream;
            boost::archive::binary_oarchive oa(stream);
            // Breaks in the next line
            oa << g;
            */



            //delete &g;

            //std::cout << "Graph deleted" << std::endl;
            // Save the graph to disk

            //std::string filename = "/home/appuser/outputCentralDPM.bin";
            //std::ofstream myFile(filename, std::ofstream::out);
            //boost::archive::binary_oarchive oa(myFile);
            //myFile.close();

        }
    }
    else
    {
        throw std::invalid_argument("no class with name " + name + " found in plugins");
    }

}
