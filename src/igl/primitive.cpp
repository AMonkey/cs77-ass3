#include "primitive.h"

///@file igl/primitive.cpp Primitives. @ingroup igl

mat4f transformed_matrix(TransformedSurface* transformed, float time) {
    auto t = transformed->translation;
    auto r = transformed->rotation_euler;
    auto s = transformed->scale;

    if (transformed->anim_translation)
        t += keyframed_value(transformed->anim_translation, time);

    if (transformed->anim_rotation_euler)
        r += keyframed_value(transformed->anim_rotation_euler, time);

    if (transformed->anim_scale)
        s *= keyframed_value(transformed->anim_scale, time);

    // Translation
    auto mat_t = mat4f(vec4f(1, 0, 0, t.x),
                       vec4f(0, 1, 0, t.y),
                       vec4f(0, 0, 1, t.z),
                       vec4f(0, 0, 0, 1)

                 );

    // Scale
    auto mat_s = mat4f(vec4f(s.x, 0, 0, 0),
                       vec4f(0, s.y, 0, 0),
                       vec4f(0, 0, s.z, 0),
                       vec4f(0, 0, 0, 1)

                 );

    // Rotation about z
    auto mat_r_z = mat4f(vec4f(cos(r.z), -1*sin(r.z), 0, 0),
                         vec4f(sin(r.z), cos(r.z), 0, 0),
                         vec4f(0, 0, 1, 0),
                         vec4f(0, 0, 0, 1)

                   );

    // Rotation about y
    auto mat_r_y = mat4f(vec4f(cos(r.y), 0, sin(r.y), 0),
                         vec4f(0, 1, 0, 0),
                         vec4f(-1*sin(r.y), 0, cos(r.y), 0),
                         vec4f(0, 0, 0, 1)

                   );

    // Rotation about x
    auto mat_r_x = mat4f(vec4f(1, 0, 0, 0),
                         vec4f(0, cos(r.x), -1*sin(r.x), 0),
                         vec4f(0, sin(r.x), cos(r.x), 0),
                         vec4f(0, 0, 0, 1)

                   );

    auto m = mat_t * mat_r_z * mat_r_y * mat_r_x * mat_s;

    return m;
    //return m * transformed_matrix_inv(transformed, time); // uncomment to check inv

}

mat4f transformed_matrix_inv(TransformedSurface* transformed, float time) {
    auto mi = identity_mat4f;
    
    auto t = transformed->translation;
    auto r = transformed->rotation_euler;
    auto s = transformed->scale;

    if (transformed->anim_translation)
        t += keyframed_value(transformed->anim_translation, time);

    if (transformed->anim_rotation_euler)
        r += keyframed_value(transformed->anim_rotation_euler, time);

    if (transformed->anim_scale)
        s *= keyframed_value(transformed->anim_scale, time);

    // To invert, subtract instead of adding
    auto mat_t = mat4f(vec4f(1, 0, 0, -1*t.x),
                       vec4f(0, 1, 0, -1*t.y),
                       vec4f(0, 0, 1, -1*t.z),
                       vec4f(0, 0, 0, 1)

                 );

    // To invert, take ^-1 of scale factors
    auto mat_s = mat4f(vec4f((1/s.x), 0, 0, 0),
                       vec4f(0, (1/s.y), 0, 0),
                       vec4f(0, 0, (1/s.z), 0),
                       vec4f(0, 0, 0, 1)

                 );

    // To invert, take opposite of theta (only affects sin as cos is even)
    auto mat_r_z = mat4f(vec4f(cos(r.z), sin(r.z), 0, 0),
                         vec4f(-1*sin(r.z), cos(r.z), 0, 0),
                         vec4f(0, 0, 1, 0),
                         vec4f(0, 0, 0, 1)

                   );

    auto mat_r_y = mat4f(vec4f(cos(r.y), 0, -1*sin(r.y), 0),
                         vec4f(0, 1, 0, 0),
                         vec4f(sin(r.y), 0, cos(r.y), 0),
                         vec4f(0, 0, 0, 1)

                   );

    auto mat_r_x = mat4f(vec4f(1, 0, 0, 0),
                         vec4f(0, cos(r.x), sin(r.x), 0),
                         vec4f(0, -1*sin(r.x), cos(r.x), 0),
                         vec4f(0, 0, 0, 1)

                   );

    return mat_s * mat_r_x * mat_r_y * mat_r_z * mat_t;

}

/// Computes the pose and rest frames at time for each bone
/// @param skinned Where rest bone data (wrt to parent bone's frame) and the pose rotation data are stored
/// @param time The time at which to compute each bone's pose frames
/// @param pose_frames The frame of each bone in pose position wrt the frame of root bone (object frame)
/// @param rest_frames The frame of each bone in rest position wrt the frame of root bone (object frame)
void skinned_bone_frames(SkinnedSurface* skinned, float time, vector<frame3f>& pose_frames, vector<frame3f>& rest_frames) {
    pose_frames.resize(skinned->bones.size(), identity_frame3f);
    rest_frames.resize(skinned->bones.size(), identity_frame3f);

    // compute the pose and rest frames for each bone wrt to root bone frame
    // NOTE: bone->frame_rest and bone->frame_pose are wrt to the frame of their parent bone

    for (int i = 0; i < skinned->bones.size(); i++) {
        // Calculate rest frame
        auto parent = skinned->bones[i]->parent;
        if (parent >= 0) {
            rest_frames[i] = transform_frame(rest_frames[parent], skinned->bones[i]->frame_rest);

        }
        else {
            rest_frames[i] = skinned->bones[i]->frame_rest;

        }

        // Calculate pose frame
        // rotating the identity frame about the major axes in ZYX order
        TransformedSurface *t = new TransformedSurface();
        t->rotation_euler = skinned->bones[i]->rotation_euler;
        t->anim_rotation_euler = skinned->bones[i]->anim_rotation_euler;
        pose_frames[i] = transform_frame(transformed_matrix(t, time), pose_frames[i]);

        // translating by the origin of the rest frame
        pose_frames[i].o += skinned->bones[i]->frame_rest.o;

        // changing to object's frame by transforming wrt to the pose frame of parent bone
        if (parent >= 0)
            pose_frames[i] = transform_frame(pose_frames[parent], pose_frames[i]);

        delete t;

    }
}

/// Computes the pose position for skinned mesh
/// @param skinned The skinned mesh to update
/// @param time The time at which to compute each bone's pose frame
void skinned_update_pose(SkinnedSurface* skinned, float time) {
    // Nifty baby-cache
    if(time == skinned->_posed_cached_time) return;
    skinned->_posed_cached_time = time;

    //return;

    vector<frame3f> pose_frames, rest_frames;
    skinned_bone_frames(skinned, time, pose_frames, rest_frames);

    auto &rest_pos = *shape_get_pos(skinned->shape);            // vertex rest position as a mesh
    auto &pose_pos = *shape_get_pos(skinned->_posed_cached);    // vertex pose position

    for (int i = 0; i < pose_pos.size(); i++) { // Vertices
        // Need to zero out position
        pose_pos[i] = zero3f;

        //for (int *j = skinned->weights[i]->idx.begin();  j < skinned->weights[i]->idx.begin()+skinned->weights[i]->idx.size(); j++) { // Bones
        for (int j = 0; j < skinned->weights[i]->idx.size(); j++) { // Bones
            auto bone_index = skinned->weights[i]->idx[j];
            auto w = skinned->weights[i]->weight[j];

            /* From Notes:
             * you will transform the rest position of each vertex from object frame to rest frame of a bone,
             * then transform from pose frame of the bone back to object frame
             */
            auto p = transform_point( pose_frames[bone_index], transform_point_inverse(rest_frames[bone_index], rest_pos[i]) );
            pose_pos[i] += w * p;

        }
    }
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

            // F = ma
            return p.mass * simulated->force_gravity;
        };

        if(is<ParticleSystem>(simulated)) {
            auto psys = cast<ParticleSystem>(prim);
            psys->_simulator->begin_update = [psys](float dt){
                int n = round(dt*psys->_rng.next_int(psys->particles_per_sec));     // number of particles to create

                // Delete particles
                //for (Particle p : psys->_simulator->particles) {
                for (int i = 0; i < psys->_simulator->particles.size(); i++) {
                    auto p = psys->_simulator->particles[i];
                    if (p.timer <= 0)
                        psys->_simulator->particles.erase(psys->_simulator->particles.begin()+i);

                }

                // Create particles
                for (int i = 0; i < n; i++) {
                    Particle *p = new Particle();

                    ShapeSample ss = shape_sample_uniform(psys->source_shape, psys->_rng.next_vec2f());

                    p->pos = ss.frame.o;
                    p->norm = ss.frame.z;

                    // Starts with random values within given range in simulator
                    p->vel = vec3f(0, 0, psys->_rng.next_float(psys->particles_init_vel));
                    p->radius = psys->_rng.next_float(psys->particles_init_radius);
                    p->timer = psys->_rng.next_float(psys->particles_init_timer);
                    // m = rho * V, V = 4/3 pi r^3
                    p->mass = psys->_rng.next_float(psys->particles_init_density) * (4/3) * pi * pow(p->radius, 3);

                    psys->_simulator->particles.push_back(*p);

                }
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
