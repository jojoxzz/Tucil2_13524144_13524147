CXX = g++
CXXFLAGS = -std=c++11 -O3
SRC = src/VoxelObjek.cpp
OBJ = bin/voxelizer
OUT_DIR = bin test

all: directories $(OBJ)

directories:
	mkdir -p bin
	mkdir -p test/Initial_Object

$(OBJ): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(OBJ)

run: all
	./$(OBJ) $(FILE) $(DEPTH)

clean:
	rm -rf bin/*.exe bin/voxelizer test/result_voxel.obj