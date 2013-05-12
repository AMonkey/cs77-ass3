#ifndef _SCENE_H_
#define _SCENE_H_

#include "node.h"
#include "camera.h"
#include "light.h"
#include "primitive.h"
#include "gizmo.h"

///@file igl/scene.h Scene. @ingroup igl
///@defgroup scene Scene
///@ingroup igl
///@{

struct Scene : Node {
	Camera*              camera = nullptr;
	LightGroup*          lights = nullptr;
	PrimitiveGroup*      prims = nullptr;
    GizmoGroup*          gizmos = nullptr;
        
    GizmoGroup*         _defaultgizmos = nullptr;
    LightGroup*         _cameralights = nullptr;
    
    uint                _shade_vert_id = 0;
    uint                _shade_frag_id = 0;
    uint                _shade_prog_id = 0;
};

///@name animation interface
///@{
inline range1f scene_animation_interval(Scene* scene) { return primitives_animation_interval(scene->prims); }
///@}

///@name simulation support
///@{
inline bool scene_simulation_has(Scene* scene) { return primitives_simulation_has(scene->prims); }
inline void scene_simulation_init(Scene* scene) { primitives_simulation_init(scene->prims); }
inline void scene_simulation_update(Scene* scene, float dt) { primitives_simulation_update(scene->prims,dt); }
///@}

///@name scene member initialization
///@{
inline void scene_defaultgizmos_init(Scene* scene) {
    if(scene->_defaultgizmos) return;
    scene->_defaultgizmos = new GizmoGroup();
    scene->_defaultgizmos->gizmos.push_back(new Grid());
    scene->_defaultgizmos->gizmos.push_back(new Axes());
}

inline void scene_cameralights_update(Scene* scene, const vector<vec3f>& dir, const vector<vec3f>& c) {
    if(not scene->_cameralights) scene->_cameralights = new LightGroup();
    if(scene->_cameralights->lights.size() != dir.size()) {
        scene->_cameralights->lights.clear();
        for(int i = 0; i < dir.size(); i ++) scene->_cameralights->lights.push_back(new DirectionalLight());
    }
    for(int i = 0; i < dir.size(); i++) {
        scene->_cameralights->lights[i]->frame = scene->camera->frame;
        scene->_cameralights->lights[i]->frame.z = transform_direction(scene->camera->frame, normalize(dir[i]));
        scene->_cameralights->lights[i]->frame = orthonormalize(scene->_cameralights->lights[i]->frame);
        cast<DirectionalLight>(scene->_cameralights->lights[i])->intensity = c[i];
    }
}
///@}

///@}

#endif
