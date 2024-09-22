#pragma once
#include <vector>
#include <string>
#include <optional>
#include <random>

namespace map_generation
{

using size_type = std::vector<int>::size_type;

const float noiseProb = 0.4;
const float artificialObstacleProb = 0.05;
const float terrainObstacleProb = 0.05;
const int minTerrainObstacleHeight = 20;
const int maxTerrainObstacleHeight = 50;

struct Map {
    private:
    std::vector<std::vector<int>> map;
    size_type mapSize;
    std::mt19937 gen;

    public:
    Map(size_type mapSize);
    std::vector<std::vector<int>> getMap() const {return map;}
    std::optional<int> getTile(int row, int col) const; 
    void setTile(int row, int col, int val);
    size_type getMapSize() const noexcept {return mapSize;}
    std::string toString() const noexcept;
};

/**
 * Generates random map with following properties
 * - costs from set of {0, 10, 20, 30, 40, 50, 100}
 * - has randomly placed terrain obstacles which rover is able
 *   traverse (cost differences between tiles no more than 10)
 * - has size of mapSize x mapSize
 * - point (0,0) has cost 0
 */
Map genrateRandomMap(size_type mapSize);
void addFloorNoiseToMap(Map& map);
void addTerrainObstaclesToMap(Map& map);
void addArtificialObstaclesToMap(Map& map) ;

} //namesapce map_generation

