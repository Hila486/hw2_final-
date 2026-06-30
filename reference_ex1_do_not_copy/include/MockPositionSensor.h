#pragma once
// MockPositionSensor.h

#include "IPositionSensor.h"
#include "DroneState.h"

// MockPositionSensor is a fake position sensor for the simulator.
// It reads the current pose from DroneState and returns it to the drone.
class MockPositionSensor : public IPositionSensor {
public:
    // Constructor takes a reference to the drone state, which it will read to get the current pose.
    explicit MockPositionSensor(const DroneState& droneState);

    // Virtual destructor from the interface.
    ~MockPositionSensor() override = default;
    
    // Prevent copying
    MockPositionSensor(const MockPositionSensor&) = delete;
    MockPositionSensor& operator=(const MockPositionSensor&) = delete;
    
    // Prevent moving
    MockPositionSensor(MockPositionSensor&&) = delete;
    MockPositionSensor& operator=(MockPositionSensor&&) = delete;   

    // IPositionSensor interface implementation:
    Pose getPose() const override;

private:
    //The sensor does not own the drone state.
    //It only keeps a reference to the existing state.
    const DroneState& droneState;
};

