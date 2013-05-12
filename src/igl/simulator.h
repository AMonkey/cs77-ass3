#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include "vmath/vmath.h"
#include "common/std.h"
#include "common/debug.h"

///@file igl/simulator.h Simulation. @ingroup igl
///@defgroup simulator Simulation
///@ingroup igl
///@{

struct Shape;

/// Simulated particle
struct Particle {
    vec3f   pos = zero3f; ///< position
    vec3f   norm = zero3f; ///< normal
    vec3f   vel = zero3f; ///< velocity
    float   mass = 1; ///< mass
    float   radius = 0.01; ///< radius
    float   timer = 0; ///< timer (when <= 0, the particle is removed)
    bool    pinned = false; ///< whether the particle can move
    bool    oriented = false; ///< whether the particle is an oriented disk
    vec3f   _force = zero3f; ///< particle force
};

/// Spring between two particles
struct ParticleSpring {
    float ks; ///< static coefficient
    float kd; ///< dynamic coefficient
    float l; ///< rest length
    int i; ///< particle index
    int j; ///< particle index
};

/// Particle collision object
struct ParticleCollider {
    Shape*   shape = nullptr; ///< collider shape
    frame3f  frame; ///< collider frame
};

/// Particle simulator
struct ParticleSimulator {
    vector<Particle>                    particles; ///< list of particles
    vector<ParticleSpring>              springs; ///< list of springs
    vector<ParticleCollider>            colliders; ///< list of collision objects
    
    function<vec3f (const Particle&)>   force; ///< function that evaluates the force
    
    function<void (float)>              begin_update = [](float){}; ///< function called at the start of each simulation update
    function<void (float)>              end_update = [](float){}; ///< function called at the end of each simulation update
    function<void (float)>              begin_step = [](float){}; ///< function called at the start of each simulation step
    function<void (float)>              end_step = [](float){}; ///< function called at the end of each simulation step
    
    int                                 steps_per_sec = 1000; ///< simulation steps per second
};

///@name simulation interface
///@{
void simulator_update(ParticleSimulator* simulator, float dt);
void simulator_update_step(ParticleSimulator* simulator, float dt);
///@}

///@}

#endif