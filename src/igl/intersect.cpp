#include "intersect.h"

#include "scene.h"

///@file igl/intersect.cpp Intersection. @ingroup igl

bool intersect_pointset_element_first(PointSet* pointset, int elementid, const ray3f& ray, intersection3f& intersection) {
    if(pointset->approximate) {
        float t;
        if(not intersect_point_approximate(ray, pointset->pos[elementid], pointset->radius[elementid], t)) return false;
        
        intersection.ray_t = t;
        intersection.uv = zero2f;
        
        intersection.frame.o = ray.eval(t);
        intersection.frame.z = -ray.d;
        intersection.frame.y = y3f;
        intersection.frame.x = x3f;
        intersection.frame = orthonormalize(intersection.frame);
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = (pointset->texcoord.empty()) ? zero2f : pointset->texcoord[elementid];
        
        return true;
    } else {
        float t;
        if(not intersect_sphere(ray, pointset->pos[elementid], pointset->radius[elementid], t)) return false;
        
        intersection.ray_t = t;
        auto pl = (ray.eval(t) - pointset->pos[elementid]) / pointset->radius[elementid];
        intersection.uv = vec2f(atan2pos(pl.y,pl.x)/(2*pi),acos(pl.z)/pi);
        
        intersection.frame = pointset_frame(pointset, elementid, intersection.uv);
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = (pointset->texcoord.empty()) ? zero2f : pointset->texcoord[elementid];
        
        return true;
    }
}

bool intersect_lineset_element_first(LineSet* lines, int elementid, const ray3f& ray, intersection3f& intersection) {
    if(lines->approximate) {
        auto l = lines->line[elementid];
        
        float t, s;
        if(not intersect_line_approximate(ray, lines->pos[l.x], lines->pos[l.y], lines->radius[l.x], lines->radius[l.y], t, s)) return false;
        
        intersection.ray_t = t;        
        intersection.uv = vec2f(s,0);
        
        intersection.frame.o = ray.eval(t);
        intersection.frame.z = -ray.d;
        intersection.frame.x = normalize(lines->pos[l.y]-lines->pos[l.x]);
        intersection.frame.y = y3f;
        orthonormalize(intersection.frame.y, intersection.frame.z, intersection.frame.x);
        // TODO: check frame after -> left handed?
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = (lines->texcoord.empty()) ? vec2f(intersection.uv.x,0) : (lines->texcoord[l.x]*(1-intersection.uv.x)+lines->texcoord[l.y]*intersection.uv.x);
        
        return true;
    } else {
        auto l = lines->line[elementid];
        auto f = lineset_cylinder_frame(lines, elementid);
        auto r = (lines->radius[l.y]+lines->radius[l.x])/2;
        auto h = length(lines->pos[l.y]-lines->pos[l.x]);
        
        float t;
        auto tray = transform_ray_inverse(f, ray);
        if(not intersect_cylinder(tray, r, h, t)) return false;
        
        intersection.ray_t = t;
        
        auto pl = tray.eval(t) / vec3f(r,r,h);
        intersection.uv = vec2f(atan2pos(pl.y,pl.x)/(2*pi),pl.z);
        
        intersection.frame = lineset_frame(lines, elementid, intersection.uv);
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = (lines->texcoord.empty()) ? vec2f(intersection.uv.y,0) : (lines->texcoord[l.x]*(1-intersection.uv.y)+lines->texcoord[l.y]*intersection.uv.y);
        
        return true;        
    }
}

bool intersect_trianglemesh_element_first(TriangleMesh* mesh, int elementid, const ray3f& ray, intersection3f& intersection) {
    auto f = mesh->triangle[elementid];
    float t; vec2f uv;
    bool hit = intersect_triangle(ray, mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z], t, uv.x, uv.y);
    if(not hit) return false;
    
    intersection.ray_t = t;
    intersection.uv = uv;
    
    intersection.frame = trianglemesh_frame(mesh, elementid, intersection.uv);
    intersection.geom_norm = triangle_normal(mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z]);
    intersection.texcoord = (mesh->texcoord.empty()) ? vec2f{uv.y,1-uv.x-uv.y} : interpolate_baricentric_triangle(mesh->texcoord, f, uv);
    
    return true;
}

bool intersect_mesh_element_first(Mesh* mesh, int elementid, const ray3f& ray, intersection3f& intersection) {
    auto f = mesh_triangle_face(mesh,elementid);

    float t; vec2f uv;
    bool hit = intersect_triangle(ray, mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z], t, uv.x, uv.y);
    if(not hit) return false;
    
    intersection.ray_t = t;
    intersection.uv = uv;
    
    intersection.frame = mesh_frame(mesh, elementid, intersection.uv);
    intersection.geom_norm = triangle_normal(mesh->pos[f.x],mesh->pos[f.y],mesh->pos[f.z]);
    if(not mesh->texcoord.empty()) intersection.texcoord = interpolate_baricentric_triangle(mesh->texcoord, f, uv);
    else if(elementid < mesh->triangle.size()) intersection.texcoord = interpolate_baricentric_triangle(zero2f, x2f, y2f, uv);
    else if((elementid - mesh->triangle.size()) % 2 == 0) intersection.texcoord = interpolate_baricentric_triangle(zero2f, x2f, one2f, uv);
    else intersection.texcoord = interpolate_baricentric_triangle(zero2f, one2f, y2f, uv);
    
    return true;        
}

bool intersect_facemesh_element_first(FaceMesh* mesh, int elementid, const ray3f& ray, intersection3f& intersection) {
    auto f = facemesh_triangle_face(mesh,elementid);
    
    float t; vec2f uv;
    bool hit = intersect_triangle(ray, mesh->pos[mesh->vertex[f.x].x], mesh->pos[mesh->vertex[f.y].x], mesh->pos[mesh->vertex[f.z].x], t, uv.x, uv.y);
    if(not hit) return false;
    
    intersection.ray_t = t;
    intersection.uv = uv;
    
    intersection.frame = facemesh_frame(mesh, elementid, intersection.uv);
    intersection.geom_norm = triangle_normal(mesh->pos[mesh->vertex[f.x].x],mesh->pos[mesh->vertex[f.y].x],mesh->pos[mesh->vertex[f.z].x]);
    if(not mesh->texcoord.empty()) intersection.texcoord = interpolate_baricentric_triangle(mesh->texcoord[mesh->vertex[f.x].z],mesh->texcoord[mesh->vertex[f.y].z],mesh->texcoord[mesh->vertex[f.z].z],uv);
    else if(elementid < mesh->triangle.size()) intersection.texcoord = interpolate_baricentric_triangle(zero2f, x2f, y2f, uv);
    else if((elementid - mesh->triangle.size()) % 2 == 0) intersection.texcoord = interpolate_baricentric_triangle(zero2f, x2f, one2f, uv);
    else intersection.texcoord = interpolate_baricentric_triangle(zero2f, one2f, y2f, uv);
    
    return true;
}

bool intersect_pointset_element_any(PointSet* pointset, int elementid, const ray3f& ray) {
    return intersect_sphere(ray, pointset->pos[elementid], pointset->radius[elementid]);
}

bool intersect_lineset_element_any(LineSet* lines, int elementid, const ray3f& ray) {
    auto l = lines->line[elementid];
    auto f = lineset_cylinder_frame(lines, elementid);
    auto r = (lines->radius[l.y]+lines->radius[l.x])/2;
    auto h = length(lines->pos[l.y]-lines->pos[l.x]);
    
    auto tray = transform_ray_inverse(f, ray);
    return intersect_cylinder(tray, r, h);
}

bool intersect_trianglemesh_element_any(TriangleMesh* mesh, int elementid, const ray3f& ray) {
    auto f = mesh->triangle[elementid];
    return intersect_triangle(ray, mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z]);
}

bool intersect_mesh_element_any(Mesh* mesh, int elementid, const ray3f& ray) {
    auto f = mesh_triangle_face(mesh,elementid);
    return intersect_triangle(ray, mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z]);
}

bool intersect_facemesh_element_any(FaceMesh* mesh, int elementid, const ray3f& ray) {
    auto f = facemesh_triangle_face(mesh,elementid);
    return intersect_triangle(ray, mesh->pos[mesh->vertex[f.x].x], mesh->pos[mesh->vertex[f.y].x], mesh->pos[mesh->vertex[f.z].x]);
}

range3f intersect_pointset_element_bounds(PointSet* pointset, int elementid) {
    return sphere_bounds(pointset->pos[elementid],pointset->radius[elementid]);
}

range3f intersect_lineset_element_bounds(LineSet* lines, int elementid) {
    auto l = lines->line[elementid];
    auto f = lineset_cylinder_frame(lines, elementid);
    auto r = (lines->radius[l.y]+lines->radius[l.x])/2;
    auto h = length(lines->pos[l.y]-lines->pos[l.x]);
    
    return transform_bbox(f,cylinder_bounds(r,h));
}

range3f intersect_trianglemesh_element_bounds(TriangleMesh* mesh, int elementid) {
    auto f = mesh->triangle[elementid];
    return triangle_bounds(mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z]);
}

range3f intersect_mesh_element_bounds(Mesh* mesh, int elementid) {
    auto f = mesh_triangle_face(mesh,elementid);
    return triangle_bounds(mesh->pos[f.x], mesh->pos[f.y], mesh->pos[f.z]);
}

range3f intersect_facemesh_element_bounds(FaceMesh* mesh, int elementid) {
    auto f = facemesh_triangle_face(mesh,elementid);
    return triangle_bounds(mesh->pos[mesh->vertex[f.x].x], mesh->pos[mesh->vertex[f.y].x], mesh->pos[mesh->vertex[f.z].x]);
}

range3f intersect_shape_bounds(Shape* shape) {
    if(shape->_tesselation) return intersect_shape_bounds(shape->_tesselation);
    
    if(is<PointSet>(shape)) {
        auto pointset = cast<PointSet>(shape);
        range3f bbox;
        for(int i = 0; i < pointset->pos.size(); i ++) bbox = runion(bbox,intersect_pointset_element_bounds(pointset, i));
        return bbox;
    }
    else if(is<LineSet>(shape)) {
        auto lines = cast<LineSet>(shape);
        range3f bbox;
        for(int i = 0; i < lines->line.size(); i ++) bbox = runion(bbox,intersect_lineset_element_bounds(lines, i));
        return bbox;
    }
    else if(is<TriangleMesh>(shape)) {
        return range_from_values(cast<TriangleMesh>(shape)->pos);
    }
    else if(is<Mesh>(shape)) {
        return range_from_values(cast<Mesh>(shape)->pos);
    }
    else if(is<FaceMesh>(shape)) {
        return range_from_values(cast<FaceMesh>(shape)->pos);
    }
    else if(is<Sphere>(shape)) return sphere_bounds(cast<Sphere>(shape)->center, cast<Sphere>(shape)->radius);
    else if(is<Cylinder>(shape)) return cylinder_bounds(cast<Cylinder>(shape)->radius, cast<Cylinder>(shape)->height);
    else if(is<Quad>(shape)) return quad_bounds(cast<Quad>(shape)->width,cast<Quad>(shape)->height);
    else if(is<Triangle>(shape)) return triangle_bounds(cast<Triangle>(shape)->v0, cast<Triangle>(shape)->v1, cast<Triangle>(shape)->v2);
    else { not_implemented_error(); return range3f(); }
}


bool _intersect_element_first(int nelements, const function<bool(int,const ray3f&,intersection3f&)>& intersect_element, const ray3f& ray, intersection3f& intersection) {
    bool hit = false;
    float mint = ray3f::rayinf;
    ray3f sray = ray;
    for(int i = 0; i < nelements; i ++) {
        intersection3f sintersection;
        if(intersect_element(i, sray, sintersection)) {
            if(mint > sintersection.ray_t) {
                hit = true;
                mint = sintersection.ray_t;
                sray.tmax = mint;
                intersection = sintersection;
            }
        }
    }
    return hit;
}

bool intersect_shape_first(Shape* shape, const ray3f& ray, intersection3f& intersection) {
    if(shape->_tesselation) return intersect_shape_first(shape->_tesselation, ray, intersection);
    
    if(is<PointSet>(shape)) {
        auto pointset = cast<PointSet>(shape);
        return _intersect_element_first(pointset->pos.size(),
                                        [pointset](int elementid, const ray3f& ray, intersection3f& intersection){ return intersect_pointset_element_first(pointset,elementid,ray,intersection); },
                                        ray, intersection);
    }
    else if(is<LineSet>(shape)) {
        auto lines = cast<LineSet>(shape);
        return _intersect_element_first(lines->pos.size(),
                                        [lines](int elementid, const ray3f& ray, intersection3f& intersection){ return intersect_lineset_element_first(lines,elementid,ray,intersection); },
                                        ray, intersection);
    }
    else if(is<TriangleMesh>(shape)) {
        auto mesh = cast<TriangleMesh>(shape);
        return _intersect_element_first(mesh->triangle.size(),
                [mesh](int elementid, const ray3f& ray, intersection3f& intersection){ return intersect_trianglemesh_element_first(mesh,elementid,ray,intersection); },
                ray, intersection);
    }
    else if(is<Mesh>(shape)) {
        auto mesh = cast<Mesh>(shape);
        return _intersect_element_first(mesh->triangle.size() + mesh->quad.size()*2,
                                        [mesh](int elementid, const ray3f& ray, intersection3f& intersection){ return intersect_mesh_element_first(mesh,elementid,ray,intersection); },
                                        ray, intersection);
    }
    else if(is<FaceMesh>(shape)) {
        auto mesh = cast<FaceMesh>(shape);
        return _intersect_element_first(mesh->triangle.size() + mesh->quad.size()*2,
                                        [mesh](int elementid, const ray3f& ray, intersection3f& intersection){ return intersect_facemesh_element_first(mesh,elementid,ray,intersection); },
                                        ray, intersection);
    }
    else if(is<Sphere>(shape)) {
        auto sphere = cast<Sphere>(shape);
        
        float t;
        if(not intersect_sphere(ray, sphere->center, sphere->radius, t)) return false;
        
        intersection.ray_t = t;
        auto pl = (ray.eval(t) - sphere->center) / sphere->radius;
        intersection.uv = vec2f(atan2pos(pl.y,pl.x)/(2*pi),acos(pl.z)/pi);
        
        intersection.frame = sphere_frame(sphere, intersection.uv);
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = intersection.uv;
        
        return true;
    }
    else if(is<Cylinder>(shape)) {
        auto cylinder = cast<Cylinder>(shape);
        
        float t;
        if(not intersect_cylinder(ray, cylinder->radius, cylinder->height, t)) return false;
        
        intersection.ray_t = t;
        
        auto pl = ray.eval(t) / vec3f(cylinder->radius,cylinder->radius,cylinder->height);
        intersection.uv = vec2f(atan2pos(pl.y,pl.x)/(2*pi),pl.z);
        
        intersection.frame = cylinder_frame(cylinder, intersection.uv);
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = intersection.uv;
        
        return true;
    }
    else if(is<Quad>(shape)) {
        auto quad = cast<Quad>(shape);
        
        float t; vec2f uv;
        if(not intersect_quad(ray, quad->width, quad->height, t, uv.x, uv.y)) return false;
        
        intersection.ray_t = t;
        intersection.uv = uv;
        
        intersection.frame = quad_frame(quad,intersection.uv);
        intersection.geom_norm = z3f;
        intersection.texcoord = uv;
        
        return true;
    }
    else if(is<Triangle>(shape)) {
        auto triangle = cast<Triangle>(shape);
        
        float t; vec2f uv;
        if(not intersect_triangle(ray, triangle->v0, triangle->v1, triangle->v2, t, uv.x, uv.y)) return false;
        
        intersection.ray_t = t;
        intersection.uv = uv;
        
        intersection.frame = triangle_frame(triangle,intersection.uv);
        intersection.geom_norm = intersection.frame.z;
        intersection.texcoord = zero2f*uv.x+x2f*uv.y+y2f*(1-uv.x-uv.y);
        
        return true;
    }
    else { not_implemented_error(); return false; }
}

bool intersect_shape_any(Shape* shape, const ray3f& ray) {
    if(shape->_tesselation) return intersect_shape_any(shape->_tesselation, ray);
    
    if(is<PointSet>(shape)) {
        for(int i = 0; i < cast<PointSet>(shape)->pos.size(); i ++)
            if(intersect_pointset_element_any(cast<PointSet>(shape),i,ray)) return true;
        return false;
    }
    else if(is<LineSet>(shape)) {
        for(int i = 0; i < cast<LineSet>(shape)->line.size(); i ++)
            if(intersect_lineset_element_any(cast<LineSet>(shape),i,ray)) return true;
        return false;
    }
    else if(is<TriangleMesh>(shape)) {
        for(int i = 0; i < cast<TriangleMesh>(shape)->triangle.size(); i ++)
            if(intersect_trianglemesh_element_any(cast<TriangleMesh>(shape),i,ray)) return true;
        return false;
    }
    else if(is<Mesh>(shape)) {
        for(int i = 0; i < cast<Mesh>(shape)->triangle.size() + cast<Mesh>(shape)->quad.size()*2; i ++)
            if(intersect_mesh_element_any(cast<Mesh>(shape),i,ray)) return true;
        return false;
    }
    else if(is<FaceMesh>(shape)) {
        for(int i = 0; i < cast<FaceMesh>(shape)->triangle.size() + cast<Mesh>(shape)->quad.size()*2; i ++)
            if(intersect_facemesh_element_any(cast<FaceMesh>(shape),i,ray)) return true;
        return false;
    }
    else if(is<Sphere>(shape)) return intersect_sphere(ray, cast<Sphere>(shape)->center, cast<Sphere>(shape)->radius);
    else if(is<Cylinder>(shape)) return intersect_cylinder(ray, cast<Cylinder>(shape)->radius, cast<Cylinder>(shape)->height);
    else if(is<Quad>(shape)) return intersect_quad(ray, cast<Quad>(shape)->width, cast<Quad>(shape)->height);
    else if(is<Triangle>(shape)) return intersect_triangle(ray, cast<Triangle>(shape)->v0, cast<Triangle>(shape)->v1, cast<Triangle>(shape)->v2);
    else { not_implemented_error(); return false; }
}

range3f intersect_primitive_bounds(Primitive* prim) {
    auto bbox = range3f();
    if(is<Surface>(prim)) bbox = intersect_shape_bounds(cast<Surface>(prim)->shape);
    else if(is<TransformedSurface>(prim)) {
        auto transformed = cast<TransformedSurface>(prim);
        error_if_not(not transformed_animated(transformed), "intersect does not support animation");
        bbox = transform_bbox(transformed_matrix(transformed, 0), intersect_shape_bounds(transformed->shape));
    }
    else if(is<SimulatedSurface>(prim)) bbox = intersect_shape_bounds(cast<SimulatedSurface>(prim)->_shape);
    else if(is<SkinnedSurface>(prim)) bbox = intersect_shape_bounds(cast<SkinnedSurface>(prim)->_posed_cached);
    else not_implemented_error();
    return transform_bbox(prim->frame, bbox);
}

bool intersect_primitive_first(Primitive* prim, const ray3f& ray, intersection3f& intersection) {
    auto hit = false;
    auto rayl = transform_ray_inverse(prim->frame,ray);
    if(is<Surface>(prim)) hit = intersect_shape_first(cast<Surface>(prim)->shape, rayl, intersection);
    else if(is<TransformedSurface>(prim)) {
        auto transformed = cast<TransformedSurface>(prim);
        error_if_not(not transformed_animated(transformed), "intersect does not support animation");
        hit = intersect_shape_first(transformed->shape, transform_ray(transformed_matrix_inv(transformed,0), rayl),intersection);
        if(hit) intersection = transform_intersection(transformed_matrix(transformed,0),transformed_matrix_inv(transformed,0),intersection);
    }
    else if(is<SimulatedSurface>(prim)) hit = intersect_shape_first(cast<SimulatedSurface>(prim)->_shape, rayl, intersection);
    else if(is<InterpolatedSurface>(prim)) {
        auto interpolated = cast<InterpolatedSurface>(prim);
        error_if_not(not interpolated_animated(interpolated), "intersect does not support animation");
        hit = intersect_shape_first(interpolated->shapes[0], rayl, intersection);
    }
    else if(is<SkinnedSurface>(prim)) {
        auto skinned = cast<SkinnedSurface>(prim);
        error_if_not(not skinned_animated(skinned), "intersect does not support animation");
        hit = intersect_shape_first(skinned->_posed_cached, rayl, intersection);
    }
    else not_implemented_error();
    if(hit) {
        intersection = transform_intersection(prim->frame,intersection);
        intersection.material = prim->material;
    }
    return hit;
}

bool intersect_primitive_first(Primitive* prim, const ray3f& ray, float time, intersection3f& intersection) {
    if(is<Surface>(prim) or is<SimulatedSurface>(prim)) return intersect_primitive_first(prim, ray, intersection);
    
    auto hit = false;
    auto rayl = transform_ray_inverse(prim->frame,ray);
    if(is<TransformedSurface>(prim)) {
        auto transformed = cast<TransformedSurface>(prim);
        hit = intersect_shape_first(transformed->shape, transform_ray(transformed_matrix_inv(transformed,time), rayl),intersection);
        if(hit) intersection = transform_intersection(transformed_matrix(transformed,time),transformed_matrix_inv(transformed,time),intersection);
    }
    else if(is<InterpolatedSurface>(prim)) {
        auto interpolated = cast<InterpolatedSurface>(prim);
        return intersect_shape_first(interpolated->shapes[interpolated_shapeidx(interpolated, time)], rayl, intersection);
    }
    else not_implemented_error();
    if(hit) {
        intersection = transform_intersection(prim->frame,intersection);
        intersection.material = prim->material;
    }
    return hit;
}

bool intersect_primitive_any(Primitive* prim, const ray3f& ray) {
    auto rayl = transform_ray_inverse(prim->frame,ray);
    if(is<Surface>(prim)) return intersect_shape_any(cast<Surface>(prim)->shape,rayl);
    else if(is<TransformedSurface>(prim)) {
        auto transformed = cast<TransformedSurface>(prim);
        error_if_not(not transformed_animated(transformed), "intersect does not support animation");
        return intersect_shape_any(transformed->shape,transform_ray(transformed_matrix_inv(transformed,0), rayl));
    }
    else if(is<SimulatedSurface>(prim)) return intersect_shape_any(cast<SimulatedSurface>(prim)->_shape,rayl);
    else if(is<InterpolatedSurface>(prim)) {
        auto interpolated = cast<InterpolatedSurface>(prim);
        error_if_not(not interpolated_animated(interpolated), "intersect does not support animation");
        return intersect_shape_any(interpolated->shapes[0], rayl);
    }
    else if(is<SkinnedSurface>(prim)) {
        auto skinned = cast<SkinnedSurface>(prim);
        error_if_not(not skinned_animated(skinned), "intersect does not support animation");
        return intersect_shape_any(skinned->_posed_cached,rayl);
    }
    else { not_implemented_error(); return false; }
}

bool intersect_primitive_any(Primitive* prim, const ray3f& ray, float time) {
    if(is<Surface>(prim) or is<SimulatedSurface>(prim)) return intersect_primitive_any(prim, ray);
    
    auto rayl = transform_ray_inverse(prim->frame,ray);
    if(is<TransformedSurface>(prim)) {
        auto transformed = cast<TransformedSurface>(prim);
        return intersect_shape_any(transformed->shape,transform_ray(transformed_matrix_inv(transformed,time), rayl));
    }
    else if(is<InterpolatedSurface>(prim)) {
        auto interpolated = cast<InterpolatedSurface>(prim);
        return intersect_shape_any(interpolated->shapes[interpolated_shapeidx(interpolated, time)], rayl);
    }
    else { not_implemented_error(); return false; }
}

range3f intersect_primitives_bounds(PrimitiveGroup* group) {
    range3f bbox;
    for(auto p : group->prims) bbox = runion(bbox,intersect_primitive_bounds(p));
    return bbox;
}

bool intersect_primitives_first(PrimitiveGroup* group, const ray3f& ray, intersection3f& intersection) {
    bool hit = false;
    float mint = ray3f::rayinf;
    ray3f sray = ray;
    for(auto p : group->prims) {
        intersection3f sintersection;
        if(intersect_primitive_first(p, sray, sintersection)) {
            if(mint > sintersection.ray_t) {
                hit = true;
                mint = sintersection.ray_t;
                sray.tmax = mint;
                intersection = sintersection;
            }
        }
    }
    return hit;
}

bool intersect_primitives_any(PrimitiveGroup* group, const ray3f& ray) {
    for(auto p : group->prims) if(intersect_primitive_any(p,ray)) return true;
    return false;
}

bool intersect_primitives_first(PrimitiveGroup* group, const ray3f& ray, float time, intersection3f& intersection) {
    bool hit = false;
    float mint = ray3f::rayinf;
    ray3f sray = ray;
    for(auto p : group->prims) {
        intersection3f sintersection;
        if(intersect_primitive_first(p, sray, time, sintersection)) {
            if(mint > sintersection.ray_t) {
                hit = true;
                mint = sintersection.ray_t;
                sray.tmax = mint;
                intersection = sintersection;
            }
        }
    }
    return hit;
}

bool intersect_primitives_any(PrimitiveGroup* group, const ray3f& ray, float time) {
    for(auto p : group->prims) if(intersect_primitive_any(p,ray,time)) return true;
    return false;
}

range3f intersect_scene_bounds(Scene* scene) { return intersect_primitives_bounds(scene->prims); }

bool intersect_scene_first(Scene* scene, const ray3f& ray, intersection3f& intersection) { return intersect_primitives_first(scene->prims, ray, intersection); }
bool intersect_scene_any(Scene* scene, const ray3f& ray) { return intersect_primitives_any(scene->prims, ray); }

bool intersect_scene_first(Scene* scene, const ray3f& ray, float time, intersection3f& intersection) { return intersect_primitives_first(scene->prims, ray, time, intersection); }
bool intersect_scene_any(Scene* scene, const ray3f& ray, float time) { return intersect_primitives_any(scene->prims, ray, time); }

