// pbrt is Copyright(c) 1998-2020 Matt Pharr, Wenzel Jakob, and Greg Humphreys.
// The pbrt source code is licensed under the Apache License, Version 2.0.
// SPDX: Apache-2.0

#include <pbrt/options.h>
#include <pbrt/pbrt.h>
#include <pbrt/util/args.h>
#include <pbrt/util/file.h>
#include <pbrt/util/image.h>
#include <pbrt/util/mesh.h>
#include <pbrt/util/string.h>
#include <pbrt/util/transform.h>
#include <pbrt/util/vecmath.h>
#include <stdlib.h>

#include <array>
#include <cstdarg>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace pbrt;

static void help() {
    printf(R"(plytool provides a number of operations on PLY meshes.

usage: plytool <command> [options]

where <command> is:

cat: Print a text representation of the mesh.

displace: Apply displacement mapping to a mesh.

info: Print general information about the mesh.

split: Split the mesh into multiple PLY files.

"plytool help <command>" provides detailed information about <command>.
)");
}

static void usage(const char *msg = nullptr, ...) {
    if (msg != nullptr) {
        va_list args;
        va_start(args, msg);
        fprintf(stderr, "plytool: ");
        vfprintf(stderr, msg, args);
        fprintf(stderr, "\n\n");
    }

    help();

    exit(1);
}

int help(std::vector<std::string> args) {
    if (args.empty()) {
        help();
        return 0;
    }
    for (std::string cmd : args) {
        if (cmd == "cat") {
            printf("usage: plytool cat <filename>\n");
        } else if (cmd == "info") {
            printf("usage: plytool info <filename...>\n");
        } else if (cmd == "displace") {
            printf(R"(usage: plytool displace [options] <filename>

options:
  --scale <s>       Scale to apply to displacement value in image.
                    (Default: 1)
  --uvscale <s>     Scale to apply to uv texture coordinates in image.
                    (Default: 1)
  --edge-length <s> Maximum length of an edge in the undisplaced mesh.
                    (Default: 1)
  --image <name>    Filename for image used to define displacements.
  --outfile <name>  Filename name for emitted PLY file.
)");
        } else if (cmd == "split") {
            printf(R"(usage: plytool split [options] <filename>

options:
  --maxfaces <n>    Maximum number of faces in an output PLY file.
                    (Default: 1000000)
  --outbase <name>  Base name for emitted PLY files.  Consecutive numbers
                    and a ".ply" suffix will be appended to <name>.
                    (Default: based on <source.ply>.)
)");
        }
        else {
            usage("%s: command unknown", cmd.c_str());
            return 1;
        }
    }

    return 0;
}

int cat(std::vector<std::string> args) {
    std::string filename;
    for (auto iter = args.begin(); iter != args.end(); ++iter) {
        auto onError = [](const std::string &err) {
            usage("%s", err.c_str());
            exit(1);
        };
        if (filename.empty())
            filename = *iter;
        else
            usage("%s: unexpected argument", iter->c_str());
    }

    if (filename.empty()) usage("must specify PLY filename.");

    TriQuadMesh mesh = TriQuadMesh::ReadPLY(filename);

    for (size_t i = 0; i < mesh.triIndices.size(); i += 3)
        Printf("Triangle: %d %d %d\n", mesh.triIndices[i], mesh.triIndices[i+1],
               mesh.triIndices[i+2]);
    for (size_t i = 0; i < mesh.quadIndices.size(); i += 4)
        Printf("Quad: %d %d %d %d\n", mesh.quadIndices[i], mesh.quadIndices[i+1],
               mesh.quadIndices[i+2], mesh.quadIndices[i+3]);
    for (size_t i = 0; i < mesh.p.size(); ++i)
        Printf("Vertex position %d: %s\n", i, mesh.p[i]);
    for (size_t i = 0; i < mesh.n.size(); ++i)
        Printf("Vertex normal %d: %s\n", i, mesh.n[i]);
    for (size_t i = 0; i < mesh.uv.size(); ++i)
        Printf("Vertex uv %d: %s\n", i, mesh.uv[i]);

    return 0;
}

int info(std::vector<std::string> args) {
    for (std::string filename : args) {
        TriQuadMesh mesh = TriQuadMesh::ReadPLY(filename);

        Printf("%s:\n", filename);
        Printf("\tTriangles: %d\n", mesh.triIndices.size() / 3);
        Printf("\tQuads: %d\n", mesh.quadIndices.size() / 4);
        Printf("\tVertex positions: %d\n", mesh.p.size());
        Printf("\tVertex normals: %d\n", mesh.n.size());
        Printf("\tVertex uvs: %d\n", mesh.uv.size());
        Printf("\tFace indices: %d\n", mesh.faceIndices.size());

        std::vector<bool> vertexUsed(mesh.p.size(), false);
        for (int v : mesh.triIndices) {
            CHECK(v >= 0 && v < mesh.p.size());
            vertexUsed[v] = true;
        }
        for (int v : mesh.quadIndices) {
            CHECK(v >= 0 && v < mesh.p.size());
            vertexUsed[v] = true;
        }
        for (size_t i = 0; i < vertexUsed.size(); ++i)
            if (!vertexUsed[i])
                Printf("Notice: vertex %d is not used.\n", i);

        Bounds3f bounds;
        for (Point3f p : mesh.p)
            bounds = Union(bounds, p);
        Printf("\tBounding box: %s\n", bounds);
    }

    return 0;
}

static void refine(int v0, int v1, int v2, TriQuadMesh &mesh, Float edgeLength,
                   std::map<std::pair<int, int>, int> &edgeSplit) {
    Point3f p0 = mesh.p[v0], p1 = mesh.p[v1], p2 = mesh.p[v2];
    Float d01 = Distance(p0, p1), d12 = Distance(p1, p2), d20 = Distance(p2, p0);

    if (d01 < edgeLength && d12 < edgeLength && d20 < edgeLength) {
        mesh.triIndices.push_back(v0);
        mesh.triIndices.push_back(v1);
        mesh.triIndices.push_back(v2);
        return;
    }

    // order so that the first two vertices have the longest edge
    std::array<int, 3> v;
    if (d01 > d12) {
        if (d01 > d20)
            v = { v0, v1, v2 };
        else
            v = { v2, v0, v1 };
    } else {
        if (d12 > d20)
            v = { v1, v2, v0 };
        else
            v = { v2, v0, v1 };
    }

    // has the edge been spilt before?
    std::pair<int, int> edge(v[0], v[1]);
    if (v[0] > v[1]) std::swap(edge.first, edge.second);
    int vmid;
    if (auto iter = edgeSplit.find(edge); iter != edgeSplit.end()) {
        vmid = iter->second;
    } else {
        vmid = mesh.p.size();
        edgeSplit[edge] = vmid;
        mesh.p.push_back((mesh.p[v[0]] + mesh.p[v[1]]) / 2);
        if (!mesh.n.empty())
            mesh.n.push_back(Normalize(mesh.n[v[0]] + mesh.n[v[1]]));
        if (!mesh.uv.empty())
            mesh.uv.push_back((mesh.uv[v[0]] + mesh.uv[v[1]]) / 2);
    }

    refine(v[0], vmid, v[2], mesh, edgeLength, edgeSplit);
    refine(vmid, v[1], v[2], mesh, edgeLength, edgeSplit);
}

int displace(std::vector<std::string> args) {
    Float scale = 1, uvScale = 1, edgeLength = 1;
    std::string filename, imageFilename, outFilename;
    int maxFaces = 1000000;
    for (auto iter = args.begin(); iter != args.end(); ++iter) {
        auto onError = [](const std::string &err) {
            usage("%s", err.c_str());
            exit(1);
        };
        if (ParseArg(&iter, args.end(), "scale", &scale, onError) ||
            ParseArg(&iter, args.end(), "uvscale", &uvScale, onError) ||
            ParseArg(&iter, args.end(), "image", &imageFilename, onError) ||
            ParseArg(&iter, args.end(), "outfile", &outFilename, onError) ||
            ParseArg(&iter, args.end(), "edge-length", &edgeLength, onError))
            ;  // yaay
        else if (filename.empty())
            filename = *iter;
        else
            usage("unexpected argument \"%s\"", iter->c_str());
    }

    if (filename.empty()) usage("must specify source PLY filename.");
    if (outFilename.empty()) usage("must specify output PLY filename.");
    if (imageFilename.empty()) usage("must specify image displacement map.");

    TriQuadMesh mesh = TriQuadMesh::ReadPLY(filename);
    ImageAndMetadata immeta = Image::Read(imageFilename);

    if (!mesh.quadIndices.empty()) {
        fprintf(stderr, "%s: quad faces are not currently supported by \"displace\". Sorry.\n",
            filename.c_str());
        return 1;
    }
    if (mesh.n.empty()) {
        fprintf(stderr, "%s: vertex normals are currently required by \"displace\". Sorry.\n",
            filename.c_str());
        return 1;
    }
    if (mesh.uv.empty()) {
        fprintf(stderr, "%s: vertex uvs are currently required by \"displace\". Sorry.\n",
            filename.c_str());
        return 1;
    }

    TriQuadMesh outputMesh = mesh;
    outputMesh.triIndices.clear();
    std::map<std::pair<int, int>, int> edgeSplit;
    for (size_t i = 0; i < mesh.triIndices.size(); i += 3)
        refine(mesh.triIndices[i], mesh.triIndices[i+1], mesh.triIndices[i+2],
               outputMesh, edgeLength, edgeSplit);

    // Displace the vertices...
    for (size_t i = 0; i < outputMesh.p.size(); ++i) {
        Float d = immeta.image.Bilerp(uvScale * outputMesh.uv[i],
                                      WrapMode::Repeat).Average();
        d *= scale;
        outputMesh.p[i] += Vector3f(d * outputMesh.n[i]);
    }

    // Recompute surface normals.
    for (size_t i = 0; i < outputMesh.n.size(); ++i)
        outputMesh.n[i] = Normal3f(0, 0, 0);
    for (size_t i = 0; i < outputMesh.triIndices.size(); i += 3) {
        int v[3] = { outputMesh.triIndices[i], outputMesh.triIndices[i + 1],
                     outputMesh.triIndices[i + 2] };
        Vector3f v10 = outputMesh.p[v[1]] - outputMesh.p[v[0]];
        Vector3f v21 = outputMesh.p[v[2]] - outputMesh.p[v[1]];

        Normal3f n(Cross(v10, v21));
        if (LengthSquared(n) > 0) {
            n = Normalize(n);
            outputMesh.n[v[0]] += n;
            outputMesh.n[v[1]] += n;
            outputMesh.n[v[2]] += n;
        }
    }
    for (size_t i = 0; i < outputMesh.n.size(); ++i)
        if (LengthSquared(outputMesh.n[i]) > 0)
            outputMesh.n[i] = Normalize(outputMesh.n[i]);

    if (!WritePLY(outFilename, outputMesh.triIndices, {}, outputMesh.p,
                  outputMesh.n, outputMesh.uv, {}))
        return 1;

    return 0;
}

int split(std::vector<std::string> args) {
    std::string inPLY, outPLYBase;
    int maxFaces = 1000000;
    for (auto iter = args.begin(); iter != args.end(); ++iter) {
        auto onError = [](const std::string &err) {
            usage("%s", err.c_str());
            exit(1);
        };
        if (ParseArg(&iter, args.end(), "maxfaces", &maxFaces, onError) ||
            ParseArg(&iter, args.end(), "outbase", &outPLYBase, onError))
            ;  // yaay
        else if (inPLY.empty())
            inPLY = *iter;
        else
            usage("unexpected argument \"%s\"", iter->c_str());
    }

    if (inPLY.empty()) usage("must specify source PLY filename.");

    if (outPLYBase.empty()) outPLYBase = RemoveExtension(inPLY);

    TriQuadMesh mesh = TriQuadMesh::ReadPLY(inPLY);

    if (mesh.quadIndices.size() > 0) {
        fprintf(stderr,
                "%s: sorry, mesh has quad faces. plytool currently only "
                "supports triangle meshes.\n",
                inPLY.c_str());
        return 1;
    }
    if (mesh.faceIndices.size() > 0) {
        fprintf(stderr,
                "%s: sorry, mesh has faceIndices, which are not currently "
                "supported by plytool.\n",
                inPLY.c_str());
        return 1;
    }

    int nFaces = mesh.triIndices.size() / 3;
    if (nFaces <= maxFaces) {
        fprintf(stderr, "%s: mesh has %d faces and so has not been split up.\n",
                inPLY.c_str(), nFaces);
        return 0;
    }

    int nMeshes = (nFaces + maxFaces - 1) / maxFaces;
    fprintf(stderr, "%s: mesh has %d faces and will be split into %d meshes.\n",
            inPLY.c_str(), nFaces, nMeshes);

    int nFacesPerMesh = nFaces / nMeshes;  // more or less...
    for (int i = 0; i < nMeshes; ++i) {
        int firstFaceIndex = i * nFacesPerMesh;
        int lastFaceIndex = (i + 1) * nFacesPerMesh;
        if (i == nMeshes - 1) lastFaceIndex = nFaces;

        std::map<int, int> vertexIndexRemap;
        std::vector<int> indices;
        std::vector<Point3f> p;
        std::vector<Normal3f> n;
        std::vector<Point2f> uv;
        for (int vertexIndex = 3 * firstFaceIndex;
             vertexIndex < 3 * lastFaceIndex; ++vertexIndex) {
            int index = mesh.triIndices[vertexIndex];
            if (auto iter = vertexIndexRemap.find(index); iter != vertexIndexRemap.end())
                indices.push_back(iter->second);
            else {
                int newIndex = int(vertexIndexRemap.size());
                vertexIndexRemap[index] = newIndex;
                indices.push_back(newIndex);

                p.push_back(mesh.p[index]);
                if (!mesh.n.empty()) n.push_back(mesh.n[index]);
                // TODO: there's no "s" in TriQuadMesh???
                if (!mesh.uv.empty()) uv.push_back(mesh.uv[index]);
            }
        }

        TriangleMesh triMesh(Transform(), false /* reverse orientation */,
                             indices, p, std::vector<Vector3f>(), n, uv, std::vector<int>());
        if (!triMesh.WritePLY(StringPrintf("%s-%03d.ply", outPLYBase, i)))
            return 1;
    }

    return 0;
}

int main(int argc, char *argv[]) {
    InitPBRT(PBRTOptions());

    if (argc < 2) {
        help();
        return 0;
    }

    std::vector<std::string> args = GetCommandLineArguments(argv);

    std::string cmd = args[0];
    args.erase(args.begin());

    int ret;
    if (cmd == "help")
        ret = help(args);
    else if (cmd == "cat")
        ret = cat(args);
    else if (cmd == "displace")
        ret = displace(args);
    else if (cmd == "info")
        ret = info(args);
    else if (cmd == "split")
        ret = split(args);
    else
        usage("%s: command unknown", cmd.c_str());

    CleanupPBRT();

    return ret;
}