#pragma once 

#include <gmock/gmock.h>

#include <rix/ipc/interfaces/notification.hpp>

class MockNotification : public rix::ipc::interfaces::Notification {
   public: 
    /**
     * @brief MockNotification object used to simulate a Signal in project 1
     * testing
     * 
     * @param counter_max The number of times for wait to be called before 
     * returning true (default -1, never return true). If at any time raise is
     * called, wait will return true and ignore the counter.
     */
    MockNotification(int counter_max = -1) : status(false), counter(0), counter_max(counter_max) {
        ON_CALL(*this, raise).WillByDefault([this]() -> bool {
            status = true;
            return true;
        });
        ON_CALL(*this, wait).WillByDefault([this](const rix::util::Duration &d) -> bool {
            // If the signal has not been raised and our counter_max is greater than 0
            if (!status && this->counter_max > 0) {
                // Increment the counter
                counter++;
                // If our counter has reached the max, set status to true
                status = counter > this->counter_max;
            }
            bool tmp = status;
            if (tmp) {
                counter = 0;
                status = false;
            }
            return tmp;
        });
    }

    MOCK_METHOD(bool, raise, (), (const));
    MOCK_METHOD(bool, wait, (const rix::util::Duration &duration), (const));

   private:
    mutable int counter;
    mutable int counter_max;
    mutable bool status;
};
