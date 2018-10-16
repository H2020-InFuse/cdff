#include <iostream>
#include <PointCloud.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/util/Demangle.hpp>
#include <envire_core/plugin/ClassLoader.hpp>
#include <envire_core/items/Item.hpp>
#include <boost/pointer_cast.hpp>


using namespace envire::core;
using namespace PointCloudWrapper;

int main() {
    EnvireGraph g;
    std::cout << "You instantiated an Envire Graph" << std::endl;

	PointCloudPtr asnPointCloud = NewPointCloud();

	AddPoint(*asnPointCloud, 0, 0, 0);
    std::cout << "Poincloud Created" << std::endl;

    envire::core::ItemBase::Ptr item;
    std::string name = envire::core::demangleTypeName(typeid(*asnPointCloud));
    std::string itemname = "envire::core::Item<" + name + ">";

    envire::core::ClassLoader *loader = envire::core::ClassLoader::getInstance();

    if (loader->hasClass(itemname))
    {
        if (loader->createEnvireItem(itemname, item))
        {
            envire::core::Item<asn1SccPointcloud>::Ptr pclItem = boost::dynamic_pointer_cast<envire::core::Item<asn1SccPointcloud>>(item);
            pclItem->setData(*asnPointCloud);
            pclItem->contentsChanged();
            std::cout << "We created an Item with a pcl in it" << std::endl;

            const FrameId frame("PCL");
            g.addFrame(frame);
            g.addItemToFrame(frame, pclItem);
            std::cout << "We created a frame and added the pcl to it" << std::endl;
        }
    }
    else
    {
        throw std::invalid_argument("no class with name " + name + " found in plugins");
    }

}
