#include <actions_helpers.h>
#include <Arduino.h>

#include <config.h>
#include <queues.h>
#include <state.h>
#include <Logger.h>

// note : pumps function use state to determine if pumps should be on or off
namespace {
constexpr uint8_t PUMPS_ALL_MASK = 0b1111;
bool frontPumpsActive = false;
bool backPumpsActive = false;
}

namespace robot::actions::action_helpers {
void togglePWM(const PWMControl& actuator_info, bool on) {
    PWMCommand cmd;

    cmd.controller = actuator_info.controller;
    cmd.pin = actuator_info.pin;
    cmd.value = on ? 4095 : 0;

    xQueueSend(robot::queues::pwm_command_queue, &cmd, 0);
}

void toggle_pumps_front(uint8_t state) {
    if (state == 0b0000) {
        robot::state::setPumpsStatus(false);
        frontPumpsActive = false;
    } else {
        robot::state::setPumpsStatus(true);
        frontPumpsActive = true;
    }

    if (frontPumpsActive || backPumpsActive) {
        robot::state::setPumpsStatus(true);
    } else {
        robot::state::setPumpsStatus(false);
    }
    state &= PUMPS_ALL_MASK;

    IOExpanderCommand cmd2;
    cmd2.expander = robot::config::IOExpander::KINETIC;
    cmd2.pin = 7;
    cmd2.level = (state & (1 << 1)) != 0;

    IOExpanderCommand cmd3;
    cmd3.expander = robot::config::IOExpander::KINETIC;
    cmd3.pin = 6;
    cmd3.level = (state & (1 << 0)) != 0;

    IOExpanderCommand cmd4;
    cmd4.expander = robot::config::IOExpander::KINETIC;
    cmd4.pin = 13;
    cmd4.level = (state & (1 << 3)) != 0;

    IOExpanderCommand cmd5;
    cmd5.expander = robot::config::IOExpander::KINETIC;
    cmd5.pin = 12;
    cmd5.level = (state & (1 << 2)) != 0;

    togglePWM(robot::config::front_interior_left_ev, (state & (1 << 0)) != 0);
    togglePWM(robot::config::front_interior_right_ev, (state & (1 << 1)) != 0);
    togglePWM(robot::config::front_exterior_left_ev, (state & (1 << 2)) != 0);
    togglePWM(robot::config::front_exterior_right_ev, (state & (1 << 3)) != 0);

    xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd3, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd4, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd5, 0);
}

void toggle_pumps_back(uint8_t state) {
    if (state == 0b0000) {
        robot::state::setPumpsStatus(false);
        backPumpsActive = false;
    } else {
        robot::state::setPumpsStatus(true);
        backPumpsActive = true;
    }

    state &= PUMPS_ALL_MASK;
    
    IOExpanderCommand cmd2;
    cmd2.expander = robot::config::IOExpander::KINETIC;
    cmd2.pin = 14;
    cmd2.level = (state & (1 << 0)) != 0;

    IOExpanderCommand cmd3;
    cmd3.expander = robot::config::IOExpander::KINETIC;
    cmd3.pin = 15;
    cmd3.level = (state & (1 << 1)) != 0;

    IOExpanderCommand cmd4;
    cmd4.expander = robot::config::IOExpander::KINETIC;
    cmd4.pin = 5;
    cmd4.level = (state & (1 << 2)) != 0;

    IOExpanderCommand cmd5;
    cmd5.expander = robot::config::IOExpander::KINETIC;
    cmd5.pin = 4;
    cmd5.level = (state & (1 << 3)) != 0;

    Serial.println(state, BIN);

    togglePWM(robot::config::back_exterior_left_ev, (state & (1 << 3)) == 0);
    togglePWM(robot::config::back_interior_left_ev, (state & (1 << 2)) == 0);
    togglePWM(robot::config::back_interior_right_ev, (state & (1 << 0)) == 0);
    togglePWM(robot::config::back_exterior_right_ev, (state & (1 << 1)) == 0);

    xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd3, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd4, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd5, 0);
}

void performRotation(const PWMControl& actuator_info, uint8_t angle) {
    PWMCommand cmd;

    cmd.controller = actuator_info.controller;
    cmd.pin = actuator_info.pin;
    cmd.value = robot::actions::action_helpers::angleToPWMValue(angle);

    xQueueSend(robot::queues::pwm_command_queue, &cmd, 0);
}

void rotate_turner_front(ArmState state) {
    uint8_t angle = 0;
    switch (state) {
        case ArmState::IDLE:
            angle = 75;
            break;
        case ArmState::TAKING:
            angle = 155;
            break;
        case ArmState::TURNING:
            action_helpers::rotate_turner_back(ArmState::IDLE);
            angle = 15; 
            break;
        case ArmState::DROPPING:
            action_helpers::rotate_turner_back(ArmState::IDLE);
            angle = 140;
            break; 
    }
    performRotation(robot::config::front_left_turner, angle);
    performRotation(robot::config::front_right_turner, 180 - angle + 3);
}

void rotate_turner_back(ArmState state) {
    uint8_t angle = 0;
    switch (state) {
        case ArmState::IDLE:
            angle = 75;
            break;
        case ArmState::TAKING:
            angle = 155;
            break;
        case ArmState::TURNING:
            action_helpers::rotate_turner_front(ArmState::IDLE);
            angle = 15;
            break;
        case ArmState::DROPPING:
            action_helpers::rotate_turner_front(ArmState::IDLE);
            angle = 140;
            break;
    }
    performRotation(robot::config::back_left_turner, angle);
    performRotation(robot::config::back_right_turner, 180 - angle + 3);
}

void rotate_grabber_front(GrabberState state) {
    robot::state::setFrontGrabberState(state);
    switch (state) {
        case GrabberState::CATCHING:
            performRotation(robot::config::front_left_grabber, 137);
            performRotation(robot::config::front_right_grabber, 30);
            break;
        case GrabberState::UNFOLDED:
            performRotation(robot::config::front_left_grabber, 65);
            performRotation(robot::config::front_right_grabber, 106);
            break;
        case GrabberState::FOLDED:
            performRotation(robot::config::front_left_grabber, 161);
            performRotation(robot::config::front_right_grabber, 10);
            break;
    }
}

void rotate_grabber_back(GrabberState state) {
    robot::state::setBackGrabberState(state);
    switch (state) {
        case GrabberState::UNFOLDED:
            performRotation(robot::config::back_left_grabber, 137);
            performRotation(robot::config::back_right_grabber, 30);
            break;
        case GrabberState::CATCHING:
            performRotation(robot::config::back_left_grabber, 65);
            performRotation(robot::config::back_right_grabber, 106);
            break;
        case GrabberState::FOLDED:
            performRotation(robot::config::back_left_grabber, 41);
            performRotation(robot::config::back_right_grabber, 130);
            break;
    }
}

void endAction() {
    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}

uint16_t angleToPWMValue(uint8_t angle) {
    return map(angle, 0, 180, 115, 545);
}

bool readColor(robot::config::ColorSensor sensor, ColorResponse& response) {
    ColorCommand colorCmd;
    colorCmd.sensor = sensor;
    xQueueSend(robot::queues::color_command_queue, &colorCmd, 0);

    ColorResponse colorResp{};
    const BaseType_t result = xQueueReceive(robot::queues::color_response_queue, &colorResp, pdMS_TO_TICKS(1500));
    response = colorResp;
    return result == pdPASS;
}

bool move(short fwd, short strafe, short rotate) {
    MotionCommand cmd;
    cmd.target = {fwd, strafe, rotate};
    return xQueueSend(robot::queues::motion_command_queue, &cmd, 0) == pdPASS;
}

bool mustBeTurned(const ColorResponse& color) {
    if (color.s < 0.1f) {
        return false;
    }

    const GlobalState state = robot::state::get();

    if (!state.isYellowTeam && (color.h >= 50.0f) && (color.h <= 80.0f)) {
        return true;
    }

    if (state.isYellowTeam && (color.h >= 210.0f) && (color.h <= 230.0f)) {
        return true;
    }

    return false;
}
}