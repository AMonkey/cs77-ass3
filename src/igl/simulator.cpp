#include "simulator.h"

#include "intersect.h"

///@file igl/simulator.cpp Simulation. @ingroup igl

/// Updates the simulated data by stepping over the time delta given simulator->steps_per_sec
/// @param simulator Contains data to update and functions to perform the update
/// @param dt The number of seconds to advance the simulator (time delta)
void simulator_update(ParticleSimulator* simulator, float dt) {
    int steps = round(simulator->steps_per_sec * dt);
    float ddt = dt / steps;
    simulator->begin_update(dt);
    for(int i = 0; i < steps; i ++) {
        simulator->begin_step(ddt);
        simulator_update_step(simulator,ddt);
        simulator->end_step(ddt);
    }
    simulator->end_update(dt);
}

/// Updates the simulated data in one step that covers dt seconds by:
/// 1) computing outside forces (ex: gravity),
/// 2) applying internal constraints (ex: spring forces),
/// 3) performing Euler integration,
/// 4) handling collisions, and
/// 5) updating timers.
/// @param simulator Contains data to update and functions to perform the update
/// @param dt The number of seconds to advance the simulator (time delta)
void simulator_update_step(ParticleSimulator* simulator, float dt) {
    put_your_code_here("Particles Update");
}

