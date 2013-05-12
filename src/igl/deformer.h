#ifndef _DEFORMER_H_
#define _DEFORMER_H_

#include "vmath/vmath.h"
#include "node.h"

///@file igl/deformer.h Shape Deformers. @ingroup igl
///@defgroup deformer Shape Deformers
///@ingroup igl
///@{

// a description on how to compute normals can be found here
// http://http.developer.nvidia.com/GPUGems/gpugems_ch42.html

/// Abstract Deformer
struct Deformer : Node {
};

/// Twist Deformer
struct Twist : Deformer {
    float                   angle = 0; ///< twist angle
};

/// Lattce Deformer
struct Lattice : Deformer {
    range3f                         bbox = range3f(vec3f(-1,-1,-1),vec3f(+1,+1,+1)); ///< reference bounding box
    vec3i                           grid = vec3i(3,3,3); ///< lattice grid size
    vector<vec3f>                   pos; ///< control points
};

inline int _lattice_cpidx(Lattice* lattice, int i, int j, int k) { return k*lattice->grid.x*lattice->grid.y+j*lattice->grid.x+i; }
inline void _lattice_initcp(Lattice* lattice) {
    if(not lattice->pos.empty()) return;
    lattice->pos.resize(lattice->grid.x*lattice->grid.y*lattice->grid.z);
    for(int i = 0; i < lattice->grid.x; i ++) {
        for(int j = 0; j < lattice->grid.y; j ++) {
            for(int k = 0; k < lattice->grid.z; k ++) {
                lattice->pos[_lattice_cpidx(lattice,i,j,k)] = lattice->bbox.min + size(lattice->bbox) * vec3f(i,j,k) /
                            (vec3f(lattice->grid.x,lattice->grid.y,lattice->grid.z)-one3f);
            }
        }
    }
}

///@name deformer interface
///@{
/// apply the deformer to a point
inline vec3f deformer_apply(Deformer* deformer, const vec3f& p) {
    if(not deformer) return zero3f;
    else if(is<Twist>(deformer)) {
        auto twist = cast<Twist>(deformer);
        float r = sqrt(p.x*p.x+p.y*p.y);
        float phi = atan2(p.y,p.x);
        float h = p.z;
        phi += twist->angle * h;
        return vec3f(r*cos(phi),r*sin(phi),h);
    }
    else if(is<Lattice>(deformer)) {
        auto lattice = cast<Lattice>(deformer);
        error_if_not(lattice->grid.x*lattice->grid.y*lattice->grid.z == lattice->pos.size(), "wrong number of control points");
        vec3f pl = (p - lattice->bbox.min) / size(lattice->bbox);
        vec3f ret = zero3f;
        for(int i = 0; i < lattice->grid.x; i ++) {
            for(int j = 0; j < lattice->grid.y; j ++) {
                for(int k = 0; k < lattice->grid.z; k ++) {
                    float u = bernstein(pl.x, i, lattice->grid.x-1);
                    float v = bernstein(pl.y, j, lattice->grid.y-1);
                    float w = bernstein(pl.z, k, lattice->grid.z-1);
                    ret += (u*v*w)*lattice->pos[_lattice_cpidx(lattice,i,j,k)];
                }
            }
        }
        return ret;
    }
    else { not_implemented_error(); return zero3f; }
}
///@}

///@}

#endif
