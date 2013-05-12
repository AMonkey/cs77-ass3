#include "primitive.h"

///@file igl/primitive.cpp Primitives. @ingroup igl

mat4f transformed_matrix(TransformedSurface* transformed, float time) {
    auto m = identity_mat4f;
    
    put_your_code_here("Animated Transformations");

    return m;
}

mat4f transformed_matrix_inv(TransformedSurface* transformed, float time) {
    auto mi = identity_mat4f;
    
    put_your_code_here("Animated Transformations (inverse)");
    
    return mi;
}

/// Computes the pose and rest frames at time for each bone
/// @param skinned Where rest bone data (wrt to parent bone's frame) and the pose rotation data are stored
/// @param time The time at which to compute each bone's pose frames
/// @param pose_frames The frame of each bone in pose position wrt the frame of root bone (object frame)
/// @param rest_frames The frame of each bone in rest position wrt the frame of root bone (object frame)
void skinned_bone_frames(SkinnedSurface* skinned, float time, vector<frame3f>& pose_frames, vector<frame3f>& rest_frames) {
    pose_frames.resize(skinned->bones.size(), identity_frame3f);
    rest_frames.resize(skinned->bones.size(), identity_frame3f);

    put_your_code_here("Bone Frames");      // compute the pose and rest frames for each bone wrt to root bone frame
                                            // NOTE: bone->frame_rest and bone->frame_pose are wrt to the frame of their parent bone
}

/// Computes the pose position for skinned mesh
/// @param skinned The skinned mesh to update
/// @param time The time at which to compute each bone's pose frame
void skinned_update_pose(SkinnedSurface* skinned, float time) {
    if(time == skinned->_posed_cached_time) return;
    skinned->_posed_cached_time = time;

    vector<frame3f> pose_frames, rest_frames;
    skinned_bone_frames(skinned, time, pose_frames, rest_frames);

    auto& rest_pos = *shape_get_pos(skinned->shape);            // vertex rest position
    auto& pose_pos = *shape_get_pos(skinned->_posed_cached);    // vertex pose position

    put_your_code_here("Mesh Skinning");        // compute the pose position for each vertex
}

range1f primitive_animation_interval(Primitive* prim) {
    if(is<TransformedSurface>(prim)) return transformed_animation_interval(cast<TransformedSurface>(prim));
    else if(is<InterpolatedSurface>(prim)) return range1f(0,cast<InterpolatedSurface>(prim)->shapes.size()/cast<InterpolatedSurface>(prim)->fps);
    else if(is<SkinnedSurface>(prim)) return skinned_animation_interval(cast<SkinnedSurface>(prim));
    else return range1f();
}

void primitive_simulation_init(Primitive* prim) {
    if(is<SimulatedSurface>(prim)) {
        auto simulated = cast<SimulatedSurface>(prim);
        
        // clear the particles
        simulated->_simulator = new ParticleSimulator();
        
        // setup forces
        simulated->_simulator->force = [simulated](const Particle& p) -> vec3f {
            vec3f force = zero3f;
            put_your_code_here("Particles Forces");     // compute the forces acting on the particle (ex: gravity, wind)
            return force;
        };

        if(is<ParticleSystem>(simulated)) {
            auto psys = cast<ParticleSystem>(prim);
            psys->_simulator->begin_update = [psys](float dt){
                int n = round(dt*psys->_rng.next_int(psys->particles_per_sec));     // number of particles to create

                put_your_code_here("Particle Creation/Deletion");                   // delete "dead" particles, create new particles
            };
            psys->_simulator->end_update = [psys](float dt){
                psys->_points->pos.resize(psys->_simulator->particles.size());
                psys->_points->radius.resize(psys->_simulator->particles.size());
                for(int i = 0; i < psys->_simulator->particles.size(); i ++) {
                    psys->_points->pos[i] = psys->_simulator->particles[i].pos;
                    psys->_points->radius[i] = psys->_simulator->particles[i].radius;
                }
            };
        }
        else if(is<Cloth>(simulated)) {
            auto cloth = cast<Cloth>(prim);
            
            // synch particles
            cloth->_simulator->particles.resize(cloth->_mesh->pos.size());
            for(int i = 0; i < cloth->_simulator->particles.size(); i ++) {
                cloth->_simulator->particles[i] = Particle();
                cloth->_simulator->particles[i].pos = cloth->_mesh->pos[i];
                cloth->_simulator->particles[i].norm = cloth->_mesh->norm[i];
                cloth->_simulator->particles[i].vel = zero3f;
                cloth->_simulator->particles[i].radius = (cloth->source_size.x/cloth->source_grid.x+cloth->source_size.y/cloth->source_grid.y)/2;
                cloth->_simulator->particles[i].mass = cloth->cloth_density/(cloth->_simulator->particles[i].radius*cloth->_simulator->particles[i].radius);
                cloth->_simulator->particles[i].oriented = true;
                cloth->_simulator->particles[i].pinned = false;
                cloth->_simulator->particles[i].timer = 0;
            }
            for(auto p : cloth->pinned) cloth->_simulator->particles[p].pinned = true;
            
            // setup constraints
            for(int j = 0; j < cloth->source_grid.y; j ++) {
                for(int i = 0; i < cloth->source_grid.x; i ++) {
                    int idx0 = (j+0)*(cloth->source_grid.x+1)+(i+0);
                    int idx1 = (j+0)*(cloth->source_grid.x+1)+(i+1);
                    int idx2 = (j+1)*(cloth->source_grid.x+1)+(i+1);
                    int idx3 = (j+1)*(cloth->source_grid.x+1)+(i+0);
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_stretch, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx1].pos), idx0, idx1 });
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_stretch, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx3].pos), idx0, idx3 });
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_shear, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx2].pos), idx0, idx2 });
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_shear, cloth->cloth_dump, dist(cloth->_simulator->particles[idx1].pos,cloth->_simulator->particles[idx3].pos), idx1, idx3 });
                    if(j+2 <= cloth->source_grid.y) {
                        int idx3p = (j+2)*(cloth->source_grid.x+1)+(i+0);
                        cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_bend, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx3p].pos), idx0, idx3p });
                    }
                    if(i+2 <= cloth->source_grid.x) {
                        int idx1p = (j+0)*(cloth->source_grid.x+1)+(i+2);
                        cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_bend, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx1p].pos), idx0, idx1p });
                    }
                }
            }
            
            cloth->_simulator->end_step = [cloth](float dt){
                for(int i = 0; i < cloth->_mesh->pos.size(); i ++) {
                    cloth->_mesh->pos[i] = cloth->_simulator->particles[i].pos;
                }
                shape_smooth_frames(cloth->_mesh);
                for(int i = 0; i < cloth->_mesh->pos.size(); i ++) {
                    cloth->_simulator->particles[i].norm = cloth->_mesh->norm[i];
                }
            };
        }
        else not_implemented_error();
    }
    else return;
}

void primitive_simulation_update(Primitive* prim, float dt) {
    if(is<SimulatedSurface>(prim)) simulator_update(cast<SimulatedSurface>(prim)->_simulator,dt);
}
