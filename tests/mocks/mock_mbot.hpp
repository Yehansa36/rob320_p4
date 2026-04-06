#pragma once

#include <gmock/gmock.h>

#include <condition_variable>
#include <mbot/mbot_base.hpp>
#include <mutex>

class MockMBot : public MBotBase {
   public:
    MockMBot() {
        ON_CALL(*this, set_callback).WillByDefault([this](std::function<void(const Pose2DStamped &)> on_pose) -> void {
            this->on_pose = on_pose;
        });
        ON_CALL(*this, ok).WillByDefault([this]() -> bool { return true; });
        ON_CALL(*this, spin).WillByDefault([this]() -> void { return; });
        ON_CALL(*this, drive).WillByDefault([this](const Twist2DStamped &cmd) -> void { twists.push_back(cmd); });
        ON_CALL(*this, drive_to).WillByDefault([this](const Pose2DStamped &cmd) -> void { 
            std::lock_guard<std::mutex> guard(mtx);
            poses.push_back(cmd); 
            cv.notify_all();
        });
    }

    MOCK_METHOD(void, set_callback, (std::function<void(const Pose2DStamped &)> on_pose), (override));
    MOCK_METHOD(bool, ok, (), (const, override));
    MOCK_METHOD(void, spin, (), (override));
    MOCK_METHOD(void, drive, (const Twist2DStamped &cmd), (const, override));
    MOCK_METHOD(void, drive_to, (const Pose2DStamped &cmd), (const, override));

    std::function<void(const Pose2DStamped &)> on_pose;
    std::vector<rix::msg::geometry::Twist2DStamped> twists;
    std::vector<rix::msg::geometry::Pose2DStamped> poses;

    bool wait_for_poses(size_t n, const rix::util::Duration &d) {
        std::unique_lock<std::mutex> lock(mtx);
        auto pred = [&]() { return poses.size() == n; };
        if (pred()) return true;
        return cv.wait_for(lock, d.get(), pred);
    }   

   private:
    std::mutex mtx;
    std::condition_variable cv;
};