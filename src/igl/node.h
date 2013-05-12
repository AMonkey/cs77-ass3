#ifndef _NODE_H_
#define _NODE_H_

#include "common/common.h"
#include "vmath/vmath.h"

///@file igl/node.h Scene Nodes. @ingroup igl
///@defgroup node Scene Nodes
///@ingroup igl
///@{

/// Abstract Scene Node
struct Node {
    virtual ~Node() { }
};

/// cast to a subtype
template<typename T, typename U>
inline T* cast(U* ptr) {
    return static_cast<T*>(ptr);
}

/// check if a pointer is a pairtcular type (uses rtti)
template <typename T, typename U>
inline bool is(U* ptr) {
    return bool(dynamic_cast<T*>(ptr));
}

///@}

#endif
