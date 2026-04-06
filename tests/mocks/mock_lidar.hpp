#pragma once

#include <gmock/gmock.h>

#include <lidar/lidar_base.hpp>

class MockLidar : public LidarBase {
   public:
    MockLidar() {
        ON_CALL(*this, set_callback)
            .WillByDefault(
                [this](std::function<void(const LaserScan &)> on_scan) -> void {
                    this->on_scan = on_scan;
                });
        ON_CALL(*this, ok).WillByDefault([this]() -> bool { return true; });
        ON_CALL(*this, spin).WillByDefault([this]() -> void { return; });
    }

    MOCK_METHOD(void, set_callback, (std::function<void(const LaserScan &)> on_scan),
                (override));
    MOCK_METHOD(bool, ok, (), (const, override));
    MOCK_METHOD(void, spin, (), (override));

    std::function<void(const LaserScan &)> on_scan;
};