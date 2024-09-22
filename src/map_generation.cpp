#include "map_generation.hpp"
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <random>
#include <optional>
#include <functional>

namespace map_generation
{
    
    Map genrateRandomMap(size_type mapSize) {
        Map map{mapSize};

        addFloorNoiseToMap(map);
        addTerrainObstaclesToMap(map);
        addArtificialObstaclesToMap(map);
        map.setTile(0,0,0);
        return map;
    }

    void addRandomlyToMap(Map& map, std::function<void(Map&, int, int)> addFunction, float prob) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(0.0, 1.0);
        for(int row = 0; row < map.getMapSize(); row++) {
            for(int col = 0; col < map.getMapSize(); col++) {
                if(dist(gen) <= prob) {
                    addFunction(map, row, col);
                }
            }
        }
    }

    void addFloorNoiseToMap(Map& map) {
        const auto setNoise = [](Map& map, int row, int col){
            map.setTile(row, col, 10);
        };
        addRandomlyToMap(map, setNoise, noiseProb);
    }

    void addTerrainObstaclesToMap(Map& map) {
        const auto addTerrainObstacle = [](Map& map, int row, int col){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dist(minTerrainObstacleHeight/10, maxTerrainObstacleHeight/10);
            const auto height = dist(gen) * 10;
            const auto maxDist = height / 10;
            for(int i = -maxDist; i < maxDist; i++) {
                for(int j = -maxDist; j < maxDist; j++) {
                    auto toSet = height - (std::abs(i) + std::abs(j)) * 10;
                    if (toSet > 0) {
                        map.setTile(row + i, col + j, toSet);
                    }
                }
            }
        };
        addRandomlyToMap(map, addTerrainObstacle, terrainObstacleProb);
    }

    void addArtificialObstaclesToMap(Map& map) {
        const auto setArtificialObstacle = [](Map& map, int row, int col){
            map.setTile(row, col, 100);
        };
        addRandomlyToMap(map, setArtificialObstacle, artificialObstacleProb);
    }

    std::string Map::toString() const noexcept
    {
        std::stringstream ss;
        for(const auto& row : map) {
            for(auto& tile : row) {
                const auto toPrint = tile == 0 ? "---" : std::to_string(tile);
                ss << std::setw(3) << std::setfill(' ') << toPrint << ' ';
            }
            ss << std::endl;
        }
        return ss.str();
    }

    std::optional<int> Map::getTile(int row, int col) const {
        if(row < mapSize and col < mapSize and row >= 0 and col >= 0) {
            return std::optional<int>(map[row][col]);
        }
        return std::optional<int>();
    }

    void Map::setTile(int row, int col, int val) {
        if(row < mapSize and col < mapSize and row >= 0 and col >= 0) {
            map[row][col] = val;
        }
    }

    Map::Map(size_type mapSize)
    {
        this->mapSize = mapSize;
        map = std::vector<std::vector<int>>(mapSize);
        for(auto& row : map) {
            row = std::vector<int>(mapSize);
        }
    }

} // namespace map_generation