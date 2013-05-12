#ifndef _KEYFRAMED_H_
#define _KEYFRAMED_H_

#include "node.h"

///@file igl/keyframed.h Keyframed Values. @ingroup igl
///@defgroup keyframed Keyframed Values
///@ingroup igl
///@{

/// Keyframed Value (interpolated like a bezier)
struct KeyframedValue : Node {
    const float         _epsilon = 0.00000001f; ///< epsilon
    
    vector<float>       times; ///< keyframe times
    vector<vec3f>       values; ///< keyframe values
    int                 degree = 1; ///< bezier interpolation degrees
    
    int segments() const { return values.size() / (degree+1); }
};

/// keyfamed animation interval
inline range1f keyframed_interval(KeyframedValue* keyframed) {
    return range1f( keyframed->times.front(), keyframed->times.back() );
}

/// eval keyframed value
inline vec3f keyframed_value(KeyframedValue* keyframed, float time) {
    time = clamp(time,keyframed_interval(keyframed).min,keyframed_interval(keyframed).max-keyframed->_epsilon);
    auto value = zero3f;

    int k = -1;
    float cont_pts[keyframed->degree+ 1 ];

    // Get K
    for  (int i = 0; (i + 1) < keyframed->times.size(); i++) {
        if (keyframed->times[i] >= time && keyframed->times[i+1] <= time) {
            k = i;
            break;

        }
    }


    auto u = (time - keyframed->times[k]) / (keyframed->times[k+1] - keyframed->times[k]);
    for (int i = 0; i <= keyframed->degree; i++) {
        value += bernstein(u, i, keyframed->degree) * keyframed->values[k * (keyframed->degree + 1) + i];

    }

    return value;
}

///@}

#endif
