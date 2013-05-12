#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include "node.h"
#include "shape.h"
#include "material.h"

#include "keyframed.h"
#include "simulator.h"

///@file igl/primitive.h Primitives. @ingroup igl
///@defgroup primitive Primitives
///@ingroup igl
///@{

/// Abstract Primitive
struct Primitive : Node {
    frame3f              frame; ///< frame
    Material*            material = nullptr; ///< material
};

/// Group of Primitives
struct PrimitiveGroup : Node {
	vector<Primitive*>       prims; ///< primitives
};

/// Basic Surface
struct Surface : Primitive {
    Shape*               shape = nullptr; ///< shape
};

/// Surface Transformed with aributrary and animated transformations
struct TransformedSurface : Primitive {
    Shape*              shape = nullptr; ///< shape
    
    frame3f             pivot = identity_frame3f; ///< transformation center and orientation
    
    vec3f               translation = zero3f; ///< translation
    KeyframedValue*     anim_translation = nullptr; ///< translation keyframed animation
    vec3f               rotation_euler = zero3f; ///< rotation along main axis (ZYX order)
    KeyframedValue*     anim_rotation_euler = nullptr; ///< rotation keyframed animation
    vec3f               scale = one3f; ///< scaling
    KeyframedValue*     anim_scale = nullptr; ///< scaling keyframed animation
};

/// Surface keyframed with a shape per frame
struct InterpolatedSurface : Primitive {
    vector<Shape*>       shapes; ///< keyframed shapes
    float                fps = 1; ///< animation speed
};

/// Bone used for skinning
struct Bone : Node {
    frame3f                 frame_rest = identity_frame3f; ///< rest frame
    frame3f                 frame_pose = identity_frame3f; ///< pose frame
    vec3f                   rotation_euler = zero3f; ///< rotation wrt pose frame
    KeyframedValue*         anim_rotation_euler = nullptr; ///< animation for rotation wrt frame
    vec3f                   display_endpoint = z3f; ///< bone display end point
    int                     parent = -1; ///< parent bone index
};

/// Bone weights
struct BoneWeights : Node {
    vector<int>             idx;        ///< bone index
    vector<float>           weight;     ///< bone weight
};

/// Surface skinned with a bone hierarchy
struct SkinnedSurface : Primitive {
    Shape*                  shape = nullptr; ///< undeformed shape
    vector<Bone*>           bones; ///< bone hierarchy
    vector<BoneWeights*>    weights; ///< per-vertex bone weights
    
    Shape*                  _posed_cached = nullptr; ///< posed shape
    float                   _posed_cached_time = -1; ///< when the pose cache was last updated
};

/// Surface with physically-based simulation
struct SimulatedSurface : Primitive {
    vec3f                   force_gravity = vec3f(0,0,-9.81); ///< gravity direction and magnitude
    vec3f                   force_wind = vec3f(100,0,0); ///< wind direction and magnitude
    float                   force_airfriction = 0.1; ///< air friction corfficient
    
    float                   simulation_dumping = 0.5; ///< dumping coeffieicnt for simulation
    int                     simulation_fps = 1000; ///< simulation steps per second
    
    ParticleSimulator*      _simulator = nullptr; ///< simulator
    Shape*                  _shape = nullptr; ///< simulated shape
};

/// Particle generator and simulator
struct ParticleSystem : SimulatedSurface {
    Shape*                  source_shape = nullptr; ///< emissive source shape (must support sampling)
    
    range1i                 particles_per_sec = range1i(50,150); ///< particles per second
    
    range1f                 particles_init_timer = range1f(1,2); ///< range of particle creation timer
    range1f                 particles_init_vel = range1f(0.1,0.2); ///< range of particle cretion velocities
    range1f                 particles_init_radius = range1f(0.01, 0.02); ///< range of particle creation radia
    range1f                 particles_init_density = range1f(1000,1000); ///< range of particle creation density
    
    PointSet*               _points = nullptr; ///< typed simulated shape
    Rng                     _rng; ///< random number generator for particle creation
};

struct Cloth : SimulatedSurface {
    vec2i                   source_grid = vec2i(10,10); ///< source grid resolution
    vec2f                   source_size = vec2f(2,2); ///< source size
    
    float                   cloth_stretch = 1000000; ///< cloth stretch coefficient
    float                   cloth_shear = 1000000; ///< cloth shear coefficient
    float                   cloth_bend = 1000000; ///< cloth bend coefficient
    float                   cloth_dump = 5; ///< cloth dumping coefficient
    
    float                   cloth_density = 1; ///< cloth density
    
    vector<int>             pinned; ///< indices of pinned vertices
    
    Mesh*                   _mesh = nullptr; ///< typed simulated shape
};

///@name TransformedShape animation support
///@{
inline bool transformed_animated(TransformedSurface* transformed) {
    return transformed->anim_translation or transformed->anim_rotation_euler or transformed->anim_scale;
}
inline range1f transformed_animation_interval(TransformedSurface* transformed) {
    range1f ret;
    if(transformed->anim_translation) ret = runion(ret, keyframed_interval(transformed->anim_translation));
    if(transformed->anim_rotation_euler) ret = runion(ret, keyframed_interval(transformed->anim_rotation_euler));
    if(transformed->anim_scale) ret = runion(ret, keyframed_interval(transformed->anim_scale));
    return ret;
}
mat4f transformed_matrix(TransformedSurface* transformed, float time);
mat4f transformed_matrix_inv(TransformedSurface* transformed, float time);
///@}

///@name InterpolatedSurface animation support
///@{
inline bool interpolated_animated(InterpolatedSurface* interpolated) {
    return interpolated->shapes.size() > 1;
}
inline int interpolated_shapeidx(InterpolatedSurface* interpolated, float time) {
     return clamp((int)round(time/interpolated->fps),0,interpolated->shapes.size()-1);
}
///@}

///@name SkinnedSurface animation support
///@{
inline bool skinned_animated(SkinnedSurface* skinned) {
    for(auto bone : skinned->bones) if(bone->anim_rotation_euler) return true;
    return false;
}
inline range1f skinned_animation_interval(SkinnedSurface* skinned) {
    auto interval = range1f();
    for(auto bone : skinned->bones) if(bone->anim_rotation_euler) interval = runion(interval, keyframed_interval(bone->anim_rotation_euler));
    return interval;
}
void skinned_bone_frames(SkinnedSurface* skinned, float time, vector<frame3f>& pose, vector<frame3f>& rest);
void skinned_update_pose(SkinnedSurface* skinned, float time);
///@}

///@name animation interface
///@{
range1f primitive_animation_interval(Primitive* prim);
inline range1f primitives_animation_interval(PrimitiveGroup* group) {
    auto ret = range1f();
    for(auto p : group->prims) ret = runion(ret,primitive_animation_interval(p));
    return ret;
}
///@}

///@name simulation interface
///@{
inline bool primitive_simulation_has(Primitive* prim) {
    if(is<SimulatedSurface>(prim)) return true;
    else return false;
}
inline bool primitives_simulation_has(PrimitiveGroup* group) {
    for(auto p : group->prims) if(primitive_simulation_has(p)) return true;
    return false;
}
void primitive_simulation_init(Primitive* prim);
inline void primitives_simulation_init(PrimitiveGroup* group) {
    for(auto p : group->prims) primitive_simulation_init(p);
}
void primitive_simulation_update(Primitive* prim, float dt);
inline void primitives_simulation_update(PrimitiveGroup* group, float dt) {
    for(auto p : group->prims) primitive_simulation_update(p,dt);
}
///@}

///@}

#endif
