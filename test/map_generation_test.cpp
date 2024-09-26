#include "map_generation.hpp"
#include "gtest/gtest.h"
#include <functional>

using namespace map_generation;

bool checkPredicate(const Map& map, std::function<bool(const Map&, int, int)> f) {
    for(int row = 0; row < map.getMapSize(); row++) {
        for(int col = 0; col < map.getMapSize(); col++) {
            if(not f(map, row, col) ) {
                return false;
            }
        }
    }
    return true;
}

TEST(MapGenerationTests, generateMapDebug) {
    auto map = genrateRandomMap(20);
    EXPECT_EQ(map.getTile(0,0), 0);
    std::cout << map.toString() << std::endl;
}

TEST(MapGenerationTests, setFunction) {
    auto map = Map(10);
    EXPECT_EQ(map.getTile(3,6), 0);
    map.setTile(3,6, 20);
    EXPECT_EQ(map.getTile(3,6), 20);
}

TEST(MapGenerationTests, getFunction)
{
    auto map = Map(5);
    EXPECT_EQ(map.getTile(1,2), 100);
    EXPECT_EQ(map.getTile(1,2), 0);
    EXPECT_EQ(map.getTile(-1,0), 100);
}

TEST(MapGenerationTests, noiseGeneration)
{
    auto map = Map(10);
    addFloorNoiseToMap(map);
    const auto isOnlyNoise = [](const Map& map, int row, int col){
        return map.getTile(row, col) <= 10 ? true : false;
    };
    EXPECT_TRUE(checkPredicate(map, isOnlyNoise));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}