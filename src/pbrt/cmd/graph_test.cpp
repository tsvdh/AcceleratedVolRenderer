#pragma once

#include "pbrt/graph/deps/nanoflann.hpp"
#include "pbrt/graph/util.h"

using TreeType = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

void main(int argc, char* argv[]) {
    util::VerticesHolder vHolder;
    vHolder.GetList().emplace_back(0, pbrt::Point3f{0, 0, 0});
    TreeType tree(3, vHolder);
    tree.addPoints(0, 0);

    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(3.f, resultItems);
    float bla[3] = {2, 0, 0};

    tree.findNeighbors(resultSet, bla);
    if (resultItems.empty())
        std::cout << "nothing";
    else
        std::cout << resultItems[0].second;
    std::cout << std::endl;

    std::unordered_map<int, bool> testMap;
    for (int i = 0; i < 1000; ++i)
        testMap.emplace(i, true);

    for (auto pair : testMap)
        std::cout << pair.first << " ";
    std::cout << std::endl;
}