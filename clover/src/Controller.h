//
// Created by lpl on 1/24/26.
//

#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "Error.h"
#include "clover.pb.h"

#include <expected>

class Controller {
private:
    Controller() = default;

public:
    void init();

    std::expected<void, Error> handle_load_motor_sequence(const LoadMotorSequenceRequest& req);
    std::expected<void, Error> handle_start_sequence(const StartSequenceRequest& req);
    std::expected<void, Error> handle_halt_sequence(const HaltSequenceRequest& req);
};

#endif  // APP_CONTROLLER_H
