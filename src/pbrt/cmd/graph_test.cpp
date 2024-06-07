#pragma once

#include <pbrt/graph/graph.h>
#include <iostream>
#include <fstream>
#include <iomanip>

int main(int argc, char* argv[]) {
    graph::FreeGraph g;

    auto v1 = graph::Vertex{0, pbrt::Point3f{1, 1, 1}};
    auto v2 = graph::Vertex{0, pbrt::Point3f{2, 2, 2}};
    auto v3 = graph::Vertex{0, pbrt::Point3f{3, 3, 3}};
    auto v4 = graph::Vertex{0, pbrt::Point3f{4, 4, 4}};

    auto e1 = graph::Edge{0, &v1, &v2};
    auto e2 = graph::Edge{0, &v2, &v3};
    auto e3 = graph::Edge{0, &v3, &v4};

    graph::Path path;
    path.edges = {&e1, &e2, &e3};
    g.AddPath(path);

    std::ofstream oFile1("files/test.txt");
    oFile1 << g;
    oFile1.close();

    // graph::UniformGraph g2;
    // std::ifstream iFile("files/test.txt");
    // iFile >> g2;
    // iFile.close();
    //
    // std::ofstream oFile2("files/test2.txt");
    // oFile2 << g2;
    // oFile2.close();

    return 0;
}
