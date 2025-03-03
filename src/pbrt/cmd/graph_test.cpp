#pragma once

#include <iostream>

#include "pbrt/graph/util.h"
#include "pbrt/graph/deps/Eigen/SparseCore"

#include "pbrt/util/vecmath.h"

void main(int argc, char* argv[]) {
    using DynamicTreeType = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

    util::VerticesHolder vHolder;
    auto searchTree = std::make_unique<DynamicTreeType>(3, vHolder);

    for (int i = 0; i < 10; ++i) {
        vHolder.GetList().push_back(pbrt::Point3f(i, 0, 0));

    }


    searchTree->addPoints(0, 5);

}
