#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <map>
#include <chrono>

using namespace std;

struct Vec3 {
    double x, y, z;
};

struct Face {
    int v1, v2, v3;
};

struct BoundingBox {
    Vec3 min, max;
};

struct OctreeNode {
    BoundingBox box;
    int depth;
    bool isLeaf;
    bool isSurface;
    OctreeNode* children[8];

    OctreeNode(BoundingBox b, int d) {
        box = b;
        depth = d;
        isLeaf = true;
        isSurface = false;
        for (int i = 0; i < 8; i++) children[i] = nullptr;
    }
};

vector<Vec3> g_vertices;
vector<Face> g_faces;
map<int, int> stats_nodes;
map<int, int> stats_pruned;
int g_maxDepth;
int g_voxelCount = 0;

bool isPointInBox(Vec3 v, BoundingBox box) {
    return (v.x >= box.min.x && v.x <= box.max.x &&
            v.y >= box.min.y && v.y <= box.max.y &&
            v.z >= box.min.z && v.z <= box.max.z);
}

bool checkIntersection(Face f, BoundingBox box) {
    Vec3 p1 = g_vertices[f.v1 - 1];
    Vec3 p2 = g_vertices[f.v2 - 1];
    Vec3 p3 = g_vertices[f.v3 - 1];

    if (isPointInBox(p1, box) || isPointInBox(p2, box) || isPointInBox(p3, box)) {
        return true;
    }
    return false;
}

void subdivide(OctreeNode* node) {
    if (node->depth >= g_maxDepth) return;

    node->isLeaf = false;
    Vec3 min = node->box.min;
    Vec3 max = node->box.max;
    Vec3 mid = {(min.x + max.x)/2.0, (min.y + max.y)/2.0, (min.z + max.z)/2.0};

    for (int i = 0; i < 8; i++) {
        BoundingBox cBox;
        cBox.min.x = (i & 1) ? mid.x : min.x;
        cBox.max.x = (i & 1) ? max.x : mid.x;
        cBox.min.y = (i & 2) ? mid.y : min.y;
        cBox.max.y = (i & 2) ? max.y : mid.y;
        cBox.min.z = (i & 4) ? mid.z : min.z;
        cBox.max.z = (i & 4) ? max.z : mid.z;

        node->children[i] = new OctreeNode(cBox, node->depth + 1);
    }
}

void buildOctree(OctreeNode* node) {
    stats_nodes[node->depth]++;

    bool hit = false;
    for (int i = 0; i < g_faces.size(); i++) {
        if (checkIntersection(g_faces[i], node->box)) {
            hit = true;
            break;
        }
    }

    if (!hit) {
        stats_pruned[node->depth]++;
        return;
    }

    if (node->depth == g_maxDepth) {
        node->isSurface = true;
        g_voxelCount++;
        return;
    }

    subdivide(node);
    for (int i = 0; i < 8; i++) {
        buildOctree(node->children[i]);
    }
}

vector<Vec3> outV;
vector<Face> outF;

void createVoxelMesh(BoundingBox b) {
    int startIdx = outV.size() + 1;
    outV.push_back({b.min.x, b.min.y, b.min.z});
    outV.push_back({b.max.x, b.min.y, b.min.z});
    outV.push_back({b.max.x, b.max.y, b.min.z});
    outV.push_back({b.min.x, b.max.y, b.min.z});
    outV.push_back({b.min.x, b.min.y, b.max.z});
    outV.push_back({b.max.x, b.min.y, b.max.z});
    outV.push_back({b.max.x, b.max.y, b.max.z});
    outV.push_back({b.min.x, b.max.y, b.max.z});

    int logic[12][3] = {
        {0,1,2},{0,2,3}, {4,5,6},{4,6,7}, {0,1,5},{0,5,4},
        {2,3,7},{2,7,6}, {1,2,6},{1,6,5}, {0,3,7},{0,7,4}
    };
    for(int i=0; i<12; i++) {
        outF.push_back({startIdx+logic[i][0], startIdx+logic[i][1], startIdx+logic[i][2]});
    }
}

void collect(OctreeNode* node) {
    if (!node) return;
    if (node->isLeaf && node->isSurface) {
        createVoxelMesh(node->box);
        return;
    }
    for (int i = 0; i < 8; i++) collect(node->children[i]);
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cout << "Format: ./voxelizer <file.obj> <depth>" << endl;
        return 1;
    }

    string inputPath = argv[1];
    g_maxDepth = stoi(argv[2]);

    ifstream infile(inputPath);
    if (!infile.is_open()) {
        cout << "File tidak ditemukan!" << endl;
        return 1;
    }

    auto start = chrono::high_resolution_clock::now();

    string line;
    while (getline(infile, line)) {
        stringstream ss(line);
        string tag;
        ss >> tag;
        if (tag == "v") {
            Vec3 v;
            ss >> v.x >> v.y >> v.z;
            g_vertices.push_back(v);
        } else if (tag == "f") {
            Face f;
            ss >> f.v1 >> f.v2 >> f.v3;
            g_faces.push_back(f);
        }
    }

    BoundingBox rootBox;
    rootBox.min = rootBox.max = g_vertices[0];
    for (auto &v : g_vertices) {
        rootBox.min.x = min(rootBox.min.x, v.x); rootBox.max.x = max(rootBox.max.x, v.x);
        rootBox.min.y = min(rootBox.min.y, v.y); rootBox.max.y = max(rootBox.max.y, v.y);
        rootBox.min.z = min(rootBox.min.z, v.z); rootBox.max.z = max(rootBox.max.z, v.z);
    }

    OctreeNode* root = new OctreeNode(rootBox, 1);
    buildOctree(root);
    collect(root);

    string outPath = "test/result_voxel.obj";
    ofstream outfile(outPath);
    for (auto &v : outV) outfile << "v " << v.x << " " << v.y << " " << v.z << endl;
    for (auto &f : outF) outfile << "f " << f.v1 << " " << f.v2 << " " << f.v3 << endl;
    outfile.close();

    auto end = chrono::high_resolution_clock::now();
    double timeTaken = chrono::duration<double>(end - start).count();

    cout << "Banyaknya voxel yang terbentuk: " << g_voxelCount << endl;
    cout << "Banyaknya vertex yang terbentuk: " << outV.size() << endl;
    cout << "Banyaknya faces yang terbentuk: " << outF.size() << endl << endl;

    cout << "Statistik node octree yang terbentuk:" << endl;
    for (int i = 1; i <= g_maxDepth; i++) cout << i << " : " << stats_nodes[i] << endl;

    cout << "\nStatistik node yang tidak perlu ditelusuri:" << endl;
    for (int i = 1; i <= g_maxDepth; i++) cout << i << " : " << stats_pruned[i] << endl;

    cout << "\nKedalaman octree: " << g_maxDepth << endl;
    cout << "Lama waktu program berjalan: " << timeTaken << " detik" << endl;
    cout << "Path dimana file .obj disimpan: " << outPath << endl;

    return 0;
}