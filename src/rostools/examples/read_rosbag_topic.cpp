#include <iostream>

#include "rosbag/bag.h"
#include <rosbag/view.h>

int main(int argc, char** argv) {
    std::string path;
    if (argc < 2)
        path = "/media/afei/Samsung_T3/dataset/timmo/timmo-fix.bag";
    else
        path = argv[1];

    std::cout << "Input Rosbag Path: " << path << std::endl;

    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        std::cout << "LOADING BAG FAILED: " << e.what() << std::endl;
        return 0;
    }

    std::vector<std::string> topics;

    rosbag::View view(bag);
    std::vector<const rosbag::ConnectionInfo*> info;
    info = view.getConnections();

    printf("Get topic size: %ld\n", info.size());

    for(auto i : info)
        std::cout << i->topic << std::endl;

    bag.close();
    return 0;
}