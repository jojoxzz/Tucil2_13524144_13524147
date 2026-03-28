#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <unordered_set>
#include <iomanip>

struct Vec3 {
    double x, y, z;

    Vec3 (double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& armtk) const {
        return {x + armtk.x, y + armtk.y, z + armtk.z};
    }

    Vec3 operator-(const Vec3& armtk) const {
        return {x - armtk.x, y - armtk.y, z - armtk.z};
    }

    Vec3 operator*(double t) const {
        return {x * t, y * t, z * t};
    }

    Vec3 absolut() const {
        return {std::abs(x), std::abs(y), std::abs(z)};
    }

    double dotproduct (const Vec3& armtk) const {
        return (x * armtk.x) + (y * armtk.y) + (z * armtk.z);
    }

    Vec3 crossproduct (const Vec3& armtk) const {
        return {y * armtk.z - z * armtk.y, z * armtk.x - x * armtk.z, x * armtk.y - y * armtk.x};
    }
};

struct Triangle {
    Vec3 v0, v1, v2;
};

struct boundingBox {
    Vec3 center;
    Vec3 half;

    std::array<boundingBox, 8> subdivide() const {
        Vec3 q = half * 0.5;
        return {{
            {center + Vec3(-q.x, -q.y, -q.z), q},
            {center + Vec3(+q.x, -q.y, -q.z), q},
            {center + Vec3(-q.x, +q.y, -q.z), q},
            {center + Vec3(+q.x, +q.y, -q.z), q},
            {center + Vec3(-q.x, -q.y, +q.z), q},
            {center + Vec3(+q.x, -q.y, +q.z), q},
            {center + Vec3(-q.x, +q.y, +q.z), q},
            {center + Vec3(+q.x, +q.y, +q.z), q},
        }};
    }

    Vec3 min() const {
        return (center - half);
    }

    Vec3 max() const {
        return (center + half);
    }
};

bool boundingBoxIntersectsTriangle (const boundingBox& box, const Triangle& tri) {
    Vec3 v0 = tri.v0 - box.center;
    Vec3 v1 = tri.v1 - box.center;
    Vec3 v2 = tri.v2 - box.center;

    Vec3 r0 = v1 - v0;
    Vec3 r1 = v2 - v1;
    Vec3 r2 = v0 - v2;

    double rx = box.half.x, ry = box.half.y, rz = box.half.z;

    auto testAxis = [&](Vec3 axis, Vec3 va, Vec3 vb, Vec3 vc) -> bool {
        double p0 = axis.dotproduct(va);
        double p1 = axis.dotproduct(vb);
        double p2 = axis.dotproduct(vc);
        double r = (rx * std::abs(axis.x)) + (ry * std::abs(axis.y)) + (rz * std::abs(axis.z));
        double mn = std::min({p0, p1, p2});
        double mx = std::max({p0, p1, p2});

        return !(mn > r || mx < -r);
    };

    if (!testAxis({0, -r0.z, r0.y}, v0, v1, v2)) return false;
    if (!testAxis({0, -r1.z, r1.y}, v0, v1, v2)) return false;
    if (!testAxis({0, -r2.z, r2.y}, v0, v1, v2)) return false;

    if (!testAxis({r0.z, 0, -r0.x}, v0, v1, v2)) return false;
    if (!testAxis({r1.z, 0, -r1.x}, v0, v1, v2)) return false;
    if (!testAxis({r2.z, 0, -r2.x}, v0, v1, v2)) return false;

    if (!testAxis({-r0.y, r0.x, 0}, v0, v1, v2)) return false;
    if (!testAxis({-r1.y, r1.x, 0}, v0, v1, v2)) return false;
    if (!testAxis({-r2.y, r2.x, 0}, v0, v1, v2)) return false;

    if (std::max({v0.x, v1.x, v2.x}) < -rx || std::min({v0.x, v1.x, v2.x}) > rx) return false;
    if (std::max({v0.y, v1.y, v2.y}) < -ry || std::min({v0.y, v1.y, v2.y}) > ry) return false;
    if (std::max({v0.z, v1.z, v2.z}) < -rz || std::min({v0.z, v1.z, v2.z}) > rz) return false;

    Vec3 n = r0.crossproduct(r1);
    double d = n.dotproduct(v0);
    double r = (rx * std::abs(n.x)) + (ry * std::abs(n.y)) + (rz * std::abs(n.z));

    if (std::abs(d) > r) {
        return false;
    }

    return true;
}

struct OctreeNode {
    boundingBox box;
    int depth;
    bool isLeaf;
    bool isVoxel;

    OctreeNode (boundingBox box, int depth) : box (box), depth(depth), isLeaf(true), isVoxel(false) {}
};

struct Stats {
    int maxdepth;
    std::vector <long long> nodeCount;
    std::vector <long long> prunedCount;

    long long voxelCount = 0, vertexCount = 0, faceCount = 0;
    Stats(int maxdepth) : maxdepth(maxdepth), nodeCount(maxdepth + 1, 0), prunedCount(maxdepth + 1, 0) {}
};

void subdivide (
    const boundingBox box,
    int depth,
    int maxdepth,
    const std::vector <Triangle>& triangles,
    std::vector <boundingBox>& voxels,
    Stats& stats
) {
    stats.nodeCount[depth]++;

    bool hasInterSection = false;
    for (const auto& tri : triangles) {
        if (boundingBoxIntersectsTriangle(box, tri)) {
            hasInterSection = true;
            break;
        }
    }

    if (!hasInterSection) {
        stats.prunedCount[depth]++;
        return;
    }

    if (depth == maxdepth) {
        voxels.push_back(box);
        stats.voxelCount++;
        return;
    }

    auto children = box.subdivide();
    for (const auto& child : children) {
        subdivide(child, depth + 1, maxdepth, triangles, voxels, stats);
    }
}

bool readOBJ (
    const std::string& path,
    std::vector <Vec3>& vertices,
    std::vector <Triangle>& triangles
) {
    std::fstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Tidak dapat membuka file: " << path << "\n";
        return false;
    }

    std::string line;
    int lineNumber = 0;
    bool valid = true;

    while (std::getline(file, line)) {
        lineNumber++;
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;
        ss >> token;

        if (token == "v") {
            double x, y, z;
            
            if(!(ss >> x >> y >> z)) {
                std::cerr << "[ERROR] Format vertex tidak valid pada baris " << lineNumber << "\n";
                valid = false;
                continue;
            }
            vertices.push_back({x, y, z});
        } else if (token == "f") {
            std::vector<int> idx;
            std::string part;

            while (ss >> part) {
                int id = std::stoi(part.substr(0, part.find('/')));
                idx.push_back(id);
            }
            if (idx.size() < 3) {
                std::cerr << "[ERROR] Face butuh minimal 3 vertex pada baris " << lineNumber << "\n";
                valid = false;
                continue;
            }
            for (size_t i = 1; i + 1 < idx.size(); i++) {
                int i0 = idx[0] - 1; 
                int i1 = idx[i] - 1;
                int i2 = idx[i+1] - 1;

                if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= (int)vertices.size() || i1 >= (int)vertices.size() || i2 >= (int)vertices.size()) {
                    std::cerr << "[ERROR] Indeks face di luar range pada baris " << lineNumber << "\n";
                    valid = false;
                    continue;
                }
                triangles.push_back({vertices[i0], vertices[i1], vertices[i2]});
            }
        } else {

        }
    }
    if (vertices.empty()) {
        std::cerr << "[ERROR] Tidak ada vertex yang ditemukan.\n";
        return false;
    }

    if (triangles.empty()) {
        std::cerr << "[ERROR] Tidak ada face yang ditemukan.\n";
        return false;
    }

    return valid;
}

void writeVoxelOBJ (const std::string& path, const std::vector <boundingBox>& voxels, Stats& stats) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Tidak dapat menulis ke: " << path << "\n";
        return;
    }

    file << "# Hasil voxelization\n";
    file << "# Jumlah voxel: " << voxels.size() << "\n\n";

    int vertexOffset = 1;

    for (const auto& box : voxels) {
        Vec3 min = box.min();
        Vec3 max = box.max();

        file << "v " << min.x << " " << min.y << " " << min.z << "\n"; // 0: ---
        file << "v " << max.x << " " << min.y << " " << min.z << "\n"; // 1: +--
        file << "v " << max.x << " " << max.y << " " << min.z << "\n"; // 2: ++-
        file << "v " << min.x << " " << max.y << " " << min.z << "\n"; // 3: -+-
        file << "v " << min.x << " " << min.y << " " << max.z << "\n"; // 4: --+
        file << "v " << max.x << " " << min.y << " " << max.z << "\n"; // 5: +-+
        file << "v " << max.x << " " << max.y << " " << max.z << "\n"; // 6: +++
        file << "v " << min.x << " " << max.y << " " << max.z << "\n"; // 7: -++

        int verOf = vertexOffset;

        file << "f " << verOf + 0 << " " << verOf + 1 << " " << verOf + 2 << "\n";
        file << "f " << verOf + 0 << " " << verOf + 2 << " " << verOf + 3 << "\n";

        file << "f " << verOf + 4 << " " << verOf + 6 << " " << verOf + 5 << "\n";
        file << "f " << verOf + 4 << " " << verOf + 7 << " " << verOf + 6 << "\n";

        file << "f " << verOf + 0 << " " << verOf + 3 << " " << verOf + 7 << "\n";
        file << "f " << verOf + 0 << " " << verOf + 7 << " " << verOf + 4 << "\n";

        file << "f " << verOf + 1 << " " << verOf + 5 << " " << verOf + 6 << "\n";
        file << "f " << verOf + 1 << " " << verOf + 6 << " " << verOf + 2 << "\n";

        file << "f " << verOf + 0 << " " << verOf + 4 << " " << verOf + 5 << "\n";
        file << "f " << verOf + 0 << " " << verOf + 5 << " " << verOf + 1 << "\n";

        file << "f " << verOf + 3 << " " << verOf + 2 << " " << verOf + 6 << "\n";
        file << "f " << verOf + 3 << " " << verOf + 6 << " " << verOf + 7 << "\n";

        vertexOffset += 8;
    }
    stats.vertexCount = (long long) voxels.size() * 8;
    stats.faceCount = (long long)voxels.size() * 12;

    file.close();
}

boundingBox computeBoundingBox(const std::vector<Vec3>& vertices) {
    double xMin = vertices[0].x, xMax = vertices[0].x;
    double yMin = vertices[0].y, yMax = vertices[0].y;
    double zMin = vertices[0].z, zMax = vertices[0].z;
 
    for (const auto& v : vertices) {
        xMin = std::min(xMin, v.x); xMax = std::max(xMax, v.x);
        yMin = std::min(yMin, v.y); yMax = std::max(yMax, v.y);
        zMin = std::min(zMin, v.z); zMax = std::max(zMax, v.z);
    }

    double sizeX = xMax - xMin;
    double sizeY = yMax - yMin;
    double sizeZ = zMax - zMin;
    double maxSize = std::max({sizeX, sizeY, sizeZ});
 
    maxSize *= 1.01;
 
    Vec3 center = {
        (xMin + xMax) / 2.0,
        (yMin + yMax) / 2.0,
        (zMin + zMax) / 2.0
    };
    Vec3 half = {maxSize / 2.0, maxSize / 2.0, maxSize / 2.0};
 
    return {center, half};
}

void printReport (const Stats& stats, const std::string& outputPath, double elapsedMs) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "         LAPORAN VOXELIZATION          \n";
    std::cout << "========================================\n";

    std::cout << "Jumlah voxel   : " << stats.voxelCount   << "\n";
    std::cout << "Jumlah vertex  : " << stats.vertexCount  << "\n";
    std::cout << "Jumlah faces   : " << stats.faceCount    << "\n";
    std::cout << "Kedalaman tree : " << stats.maxdepth     << "\n";
 
    std::cout << "\n--- Statistik node octree per depth ---\n";
    for (int d = 1; d <= stats.maxdepth; d++) {
        std::cout << "  " << d << " : " << stats.nodeCount[d] << "\n";
    }
 
    std::cout << "\n--- Node yang tidak perlu ditelusuri (pruned) per depth ---\n";
    for (int d = 1; d <= stats.maxdepth; d++) {
        std::cout << "  " << d << " : " << stats.prunedCount[d] << "\n";
    }
 
    std::cout << "\n";
    std::cout << "Waktu berjalan : " << std::fixed << std::setprecision(2) << elapsedMs << " ms\n";
    std::cout << "Output disimpan: " << outputPath << "\n";
    std::cout << "========================================\n";
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Penggunaan: " << argv[0] << " <path_input.obj> <maxDepth>\n";
        std::cerr << "Contoh    : " << argv[0] << " model.obj 5\n";
        return 1;
    }
 
    std::string inputPath = argv[1];
    int maxDepth;
 
    try {
        maxDepth = std::stoi(argv[2]);
    } catch (...) {
        std::cerr << "[ERROR] maxDepth harus berupa bilangan bulat positif.\n";
        return 1;
    }
 
    if (maxDepth < 1 || maxDepth > 10) {
        std::cerr << "[ERROR] maxDepth harus antara 1 dan 10.\n";
        return 1;
    }
 
    size_t lastSlash = inputPath.find_last_of("/\\");
    std::string baseName = (lastSlash == std::string::npos) ? inputPath : inputPath.substr(lastSlash + 1);

    size_t dot = baseName.rfind('.');
    if (dot != std::string::npos) {
        baseName = baseName.substr(0, dot);
    }

    std::string outputFolder = "./test/"; 
    std::string outputPath = outputFolder + baseName + "-voxelized.obj";
 
    auto startTime = std::chrono::high_resolution_clock::now();
 
    std::cout << "[INFO] Membaca file: " << inputPath << "\n";
    std::vector<Vec3>     vertices;
    std::vector<Triangle> triangles;
 
    if (!readOBJ(inputPath, vertices, triangles)) {
        std::cerr << "[ERROR] Gagal membaca file OBJ.\n";
        return 1;
    }
 
    std::cout << "[INFO] Vertex   : " << vertices.size()  << "\n";
    std::cout << "[INFO] Segitiga : " << triangles.size() << "\n";

    boundingBox rootBox = computeBoundingBox(vertices);
    std::cout << "[INFO] Bounding box:\n";
    std::cout << "       Min: (" << rootBox.min().x << ", " << rootBox.min().y << ", " << rootBox.min().z << ")\n";
    std::cout << "       Max: (" << rootBox.max().x << ", " << rootBox.max().y << ", " << rootBox.max().z << ")\n";

    std::cout << "[INFO] Menjalankan subdivisi octree (maxDepth=" << maxDepth << ")...\n";
    Stats stats(maxDepth);
    std::vector<boundingBox> voxels;
    voxels.reserve(1024);
 
    subdivide(rootBox, 0, maxDepth, triangles, voxels, stats);
 
    std::cout << "[INFO] Menulis file output...\n";
    writeVoxelOBJ(outputPath, voxels, stats);
 
    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
 
    printReport(stats, outputPath, elapsedMs);
 
    return 0;
}