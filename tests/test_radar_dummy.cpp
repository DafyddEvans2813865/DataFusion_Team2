#include <cassert>
#include <iostream>

#include "../src/dummy/RadarDummy.h"

void test_point_count() {
    RadarDummy radar(25);
    auto data = radar.generateData();

    assert(data.size() == 25);
}

void test_non_empty() {
    RadarDummy radar(1);
    auto data = radar.generateData();

    assert(!data.empty());
}

void test_value_ranges() {
    RadarDummy radar(50);
    auto data = radar.generateData();

    for (const auto& d : data) {
        assert(d.x >= 0.0f);
        assert(d.y >= 0.0f);
        assert(d.x <= 100.0f);
        assert(d.y <= 100.0f);
    }
}

int main() {
    std::cout << "Running RadarDummy tests...\n";

    test_point_count();
    test_non_empty();
    test_value_ranges();

    std::cout << "All RadarDummy tests passed ✅\n";
    return 0;
}
