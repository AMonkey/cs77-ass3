#include "serialize.h"          

#include "draw.h"

///@file igl/serialize.cpp Serialization. @ingroup igl

Serializer::_Registry Serializer::_registry;

void Serializer::register_object_types() {
    static bool done = false;
    if(done) return;
    register_object_type<KeyframedValue>();
    register_object_type<PointSet>();
    register_object_type<LineSet>();
    register_object_type<TriangleMesh>();
    register_object_type<Mesh>();
    register_object_type<FaceMesh>();
    register_object_type<CatmullClarkSubdiv>();
    register_object_type<Subdiv>();
    register_object_type<TesselationOverride>();
    register_object_type<Spline>();
    register_object_type<Patch>();
    register_object_type<DeformedShape>();
    register_object_type<Sphere>();
    register_object_type<Cylinder>();
    register_object_type<Quad>();
    register_object_type<Triangle>();
    register_object_type<Twist>();
    register_object_type<Lattice>();
    register_object_type<Lambert>();
    register_object_type<Phong>();
    register_object_type<Material>();
    register_object_type<Camera>();
    register_object_type<LightGroup>();
    register_object_type<PointLight>();
    register_object_type<DirectionalLight>();
    register_object_type<Surface>();
    register_object_type<InterpolatedSurface>();
    register_object_type<ParticleSystem>();
    register_object_type<Cloth>();
    register_object_type<TransformedSurface>();
    register_object_type<SkinnedSurface>();
    register_object_type<PrimitiveGroup>();
    register_object_type<Bone>();
    register_object_type<BoneWeights>();
    register_object_type<GizmoGroup>();
    register_object_type<Grid>();
    register_object_type<Axes>();
    register_object_type<Line>();
    register_object_type<Dot>();
    register_object_type<Scene>();
    register_object_type<DrawOptions>();
    done = true;
}

const char* serialize_typename(Node* node) {
    if(not node) return nullptr;
    else if(is<KeyframedValue>(node)) return "KeyframedValue";
    else if(is<Shape>(node)) {
        if(not node) return nullptr;
        else if(is<PointSet>(node)) return "PointSet";
        else if(is<LineSet>(node)) return "LineSet";
        else if(is<TriangleMesh>(node)) return "TriangleMesh";
        else if(is<Mesh>(node)) return "Mesh";
        else if(is<FaceMesh>(node)) return "FaceMesh";
        else if(is<CatmullClarkSubdiv>(node)) return "CatmullClarkSubdiv";
        else if(is<Subdiv>(node)) return "Subdiv";
        else if(is<TesselationOverride>(node)) return "TesselationOverride";
        else if(is<Spline>(node)) return "Spline";
        else if(is<Patch>(node)) return "Patch";
        else if(is<DeformedShape>(node)) return "DeformedShape";
        else if(is<Sphere>(node)) return "Sphere";
        else if(is<Cylinder>(node)) return "Cylinder";
        else if(is<Quad>(node)) return "Quad";
        else if(is<Triangle>(node)) return "Triangle";
        else return "Shape";
    }
    else if(is<Deformer>(node)) {
        if(not node) return nullptr;
        else if(is<Twist>(node)) return "Twist";
        else if(is<Lattice>(node)) return "Lattice";
        return "Deformer";
    }
    else if(is<Material>(node)) {
        if(not node) return nullptr;
        else if(is<Lambert>(node)) return "Lambert";
        else if(is<Phong>(node)) return "Phong";
        else return "Material";
    }
    else if(is<Camera>(node)) return "Camera";
    else if(is<LightGroup>(node)) return "LightGroup";
    else if(is<Light>(node)) {
        if(not node) return nullptr;
        else if(is<PointLight>(node)) return "PointLight";
        else if(is<DirectionalLight>(node)) return "DirectionalLight";
        else return "Light";
    }
    else if(is<Primitive>(node)) {
        if(not node) return nullptr;
        else if(is<Surface>(node)) return "Surface";
        else if(is<InterpolatedSurface>(node)) return "InterpolatedSurface";
        else if(is<TransformedSurface>(node)) return "TransformedSurface";
        else if(is<SkinnedSurface>(node)) return "SkinnedSurface";
        else if(is<SimulatedSurface>(node)) {
            if(not node) return nullptr;
            else if(is<ParticleSystem>(node)) return "ParticleSystem";
            else if(is<Cloth>(node)) return "Cloth";
            return "SimulatedSurface";
        }
        else return "Primitive";
    }
    else if(is<PrimitiveGroup>(node)) return "PrimitiveGroup";
    else if(is<Bone>(node)) return "Bone";
    else if(is<BoneWeights>(node)) return "BoneWeights";
    else if(is<Gizmo>(node)) {
        if(not node) return nullptr;
        else if(is<Grid>(node)) return "Grid";
        else if(is<Axes>(node)) return "Axes";
        else if(is<Line>(node)) return "Line";
        else if(is<Dot>(node)) return "Dot";
        else return "Gizmo";
    }
    else if(is<GizmoGroup>(node)) return "GizmoGroup";
    else if(is<Scene>(node)) return "Scene";
    else if(is<DrawOptions>(node)) return "DrawOptions";
    else return "Node";
}

void serialize_members(Node* node, Serializer& ser) {
    if(not node) warning("node is null");
    else if(is<KeyframedValue>(node)) {
        auto keyframed = cast<KeyframedValue>(node);
        ser.serialize_member("values",keyframed->values);
        ser.serialize_member("times",keyframed->times);
        ser.serialize_member("degree",keyframed->degree);
    }
    else if(is<Shape>(node)) {
        auto shape = cast<Shape>(node);
        if(not shape) error("node is null");
        if(is<PointSet>(node)) {
            auto points = cast<PointSet>(node);
            ser.serialize_member("pos",points->pos);
            ser.serialize_member("radius",points->radius);
            ser.serialize_member("texcoord",points->texcoord);
            ser.serialize_member("approximate",points->approximate);
            ser.serialize_member("approximate_radius",points->approximate_radius);
        }
        else if(is<LineSet>(node)) {
            auto lines = cast<LineSet>(node);
            ser.serialize_member("pos",lines->pos);
            ser.serialize_member("radius",lines->radius);
            ser.serialize_member("texcoord",lines->texcoord);
            ser.serialize_member("line",lines->line);
            ser.serialize_member("approximate",lines->approximate);
            ser.serialize_member("approximate_radius",lines->approximate_radius);
        }
        else if(is<TriangleMesh>(node)) {
            auto mesh = cast<TriangleMesh>(node);
            ser.serialize_member("pos",mesh->pos);
            ser.serialize_member("norm",mesh->norm);
            ser.serialize_member("texcoord",mesh->texcoord);
            ser.serialize_member("triangle",mesh->triangle);
        }
        else if(is<Mesh>(node)) {
            auto mesh = cast<Mesh>(node);
            ser.serialize_member("pos",mesh->pos);
            ser.serialize_member("norm",mesh->norm);
            ser.serialize_member("texcoord",mesh->texcoord);
            ser.serialize_member("triangle",mesh->triangle);
            ser.serialize_member("quad",mesh->quad);
        }
        else if(is<FaceMesh>(node)) {
            auto mesh = cast<FaceMesh>(node);
            ser.serialize_member("pos",mesh->pos);
            ser.serialize_member("norm",mesh->norm);
            ser.serialize_member("texcoord",mesh->texcoord);
            ser.serialize_member("vertex",mesh->vertex);
            ser.serialize_member("triangle",mesh->triangle);
            ser.serialize_member("quad",mesh->quad);
        }
        else if(is<CatmullClarkSubdiv>(node)) {
            auto subdiv = cast<CatmullClarkSubdiv>(node);
            ser.serialize_member("pos",subdiv->pos);
            ser.serialize_member("norm",subdiv->norm);
            ser.serialize_member("texcoord",subdiv->texcoord);
            ser.serialize_member("quad",subdiv->quad);
            ser.serialize_member("level",subdiv->level);
            ser.serialize_member("smooth",subdiv->smooth);
        }
        else if(is<Subdiv>(node)) {
            auto subdiv = cast<Subdiv>(node);
            ser.serialize_member("pos",subdiv->pos);
            ser.serialize_member("norm",subdiv->norm);
            ser.serialize_member("texcoord",subdiv->texcoord);
            ser.serialize_member("triangle",subdiv->triangle);
            ser.serialize_member("quad",subdiv->quad);
            ser.serialize_member("crease_edge",subdiv->crease_edge);
            ser.serialize_member("crease_vertex",subdiv->crease_vertex);
            ser.serialize_member("level",subdiv->level);
            ser.serialize_member("smooth",subdiv->smooth);
        }
        else if(is<TesselationOverride>(node)) {
            auto override = cast<TesselationOverride>(node);
            ser.serialize_member("shape",override->shape);
            ser.serialize_member("level",override->level);
            ser.serialize_member("smooth",override->smooth);
        }
        else if(is<Spline>(node)) {
            auto spline = cast<Spline>(node);
            ser.serialize_member("pos",spline->pos);
            ser.serialize_member("radius",spline->radius);
            ser.serialize_member("cubic",spline->cubic);
            ser.serialize_member("continous",spline->continous);
            ser.serialize_member("level",spline->level);
            ser.serialize_member("smooth",spline->smooth);
        }
        else if(is<Patch>(node)) {
            auto patch = cast<Patch>(node);
            ser.serialize_member("pos",patch->pos);
            ser.serialize_member("texcoord",patch->texcoord);
            ser.serialize_member("cubic",patch->cubic);
            ser.serialize_member("continous_stride",patch->continous_stride);
            ser.serialize_member("level",patch->level);
            ser.serialize_member("smooth",patch->smooth);
        }
        else if(is<DeformedShape>(node)) {
            auto deformed = cast<DeformedShape>(node);
            ser.serialize_member("shape",deformed->shape);
            ser.serialize_member("deformers",deformed->deformers);
            ser.serialize_member("level",deformed->level);
            ser.serialize_member("smooth",deformed->smooth);
        }
        else if(is<Sphere>(node)) {
            auto sphere = cast<Sphere>(node);
            ser.serialize_member("center",sphere->center);
            ser.serialize_member("radius",sphere->radius);
        }
        else if(is<Cylinder>(node)) {
            auto cylinder = cast<Cylinder>(node);
            ser.serialize_member("radius",cylinder->radius);
            ser.serialize_member("height",cylinder->height);
        }
        else if(is<Quad>(node)) {
            auto quad = cast<Quad>(node);
            ser.serialize_member("width",quad->width);
            ser.serialize_member("height",quad->height);
        }
        else if(is<Triangle>(node)) {
            auto triangle = cast<Triangle>(node);
            ser.serialize_member("v0",triangle->v0);
            ser.serialize_member("v1",triangle->v1);
            ser.serialize_member("v2",triangle->v2);
        }
        else not_implemented_error();
    }
    else if(is<Deformer>(node)) {
        auto deformer = cast<Deformer>(node);
        if(not deformer) error("node is null");
        else if(is<Twist>(node)) {
            auto twist = cast<Twist>(node);
            ser.serialize_member("angle",twist->angle);
        }
        else if(is<Lattice>(node)) {
            auto lattice = cast<Lattice>(node);
            ser.serialize_member("bbox",lattice->bbox);
            ser.serialize_member("grid",lattice->grid);
            ser.serialize_member("pos",lattice->pos);
        }
        else not_implemented_error();
    }
    else if(is<Material>(node)) {
        auto material = cast<Material>(node);
        if(not material) error("node is null");
        else if(is<Lambert>(node)) {
            auto lambert = cast<Lambert>(node);
            ser.serialize_member("diffuse",lambert->diffuse);
        }
        else if(is<Phong>(node)) {
            auto phong = cast<Phong>(node);
            ser.serialize_member("diffuse",phong->diffuse);
            ser.serialize_member("specular",phong->specular);
            ser.serialize_member("reflection",phong->reflection);
            ser.serialize_member("exponent",phong->exponent);
            ser.serialize_member("use_reflected",phong->use_reflected);
        }
        else not_implemented_error();
    }
    else if(is<Camera>(node)) {
        auto camera = cast<Camera>(node);
        ser.serialize_member("frame",camera->frame);
        ser.serialize_member("view_dist",camera->view_dist);
        ser.serialize_member("image_width",camera->image_width);
        ser.serialize_member("image_height",camera->image_height);
        ser.serialize_member("image_dist",camera->image_dist);
        ser.serialize_member("focus_dist",camera->focus_dist);
        ser.serialize_member("focus_aperture",camera->focus_aperture);
        ser.serialize_member("orthographic",camera->orthographic);
    }
    else if(is<Light>(node)) {
        auto light = cast<Light>(node);
        ser.serialize_member("frame",light->frame);
        if(is<PointLight>(node)) {
            auto points = cast<PointLight>(node);
            ser.serialize_member("intensity",points->intensity);
        }
        else if(is<DirectionalLight>(node)) {
            auto directional = cast<DirectionalLight>(node);
            ser.serialize_member("intensity",directional->intensity);
        }
        else not_implemented_error();
    }
    else if(is<LightGroup>(node)) {
        auto lights = cast<LightGroup>(node);
        ser.serialize_member("lights",lights->lights);
    }
    else if(is<Primitive>(node)) {
        auto prim = cast<Primitive>(node);
        ser.serialize_member("frame",prim->frame);
        ser.serialize_member("material",prim->material);
        if(not prim) error("node is null");
        else if(is<Surface>(node)) {
            auto surface = cast<Surface>(node);
            ser.serialize_member("shape",surface->shape);
        }
        else if(is<TransformedSurface>(node)) {
            auto transformed = cast<TransformedSurface>(node);
            ser.serialize_member("shape",transformed->shape);
            ser.serialize_member("pivot",transformed->pivot);
            ser.serialize_member("translation",transformed->translation);
            ser.serialize_member("scale",transformed->scale);
            ser.serialize_member("rotation_euler",transformed->rotation_euler);
            ser.serialize_member("anim_translation",transformed->anim_translation);
            ser.serialize_member("anim_scale",transformed->anim_scale);
            ser.serialize_member("anim_rotation_euler",transformed->anim_rotation_euler);
        }
        else if(is<InterpolatedSurface>(node)) {
            auto interpolated = cast<InterpolatedSurface>(node);
            ser.serialize_member("shapes",interpolated->shapes);
            ser.serialize_member("fps",interpolated->fps);
        }
        else if(is<SkinnedSurface>(node)) {
            auto skinned = cast<SkinnedSurface>(node);
            ser.serialize_member("shape",skinned->shape);
            ser.serialize_member("bones",skinned->bones);
            ser.serialize_member("weights",skinned->weights);
        }
        else if(is<SimulatedSurface>(node)) {
            auto simulated = cast<SimulatedSurface>(node);
            ser.serialize_member("force_gravity",simulated->force_gravity);
            ser.serialize_member("force_wind",simulated->force_wind);
            ser.serialize_member("force_airfriction",simulated->force_airfriction);
            ser.serialize_member("simulation_fps",simulated->simulation_fps);
            ser.serialize_member("simulation_dumping",simulated->simulation_dumping);
            if(not simulated) error("node is null");
            else if(is<ParticleSystem>(node)) {
                auto particles = cast<ParticleSystem>(node);
                ser.serialize_member("source_shape",particles->source_shape);
                ser.serialize_member("particles_per_sec",particles->particles_per_sec);
                ser.serialize_member("particles_init_timer",particles->particles_init_timer);
                ser.serialize_member("particles_init_vel",particles->particles_init_vel);
                ser.serialize_member("particles_init_radius",particles->particles_init_radius);
                ser.serialize_member("particles_init_density",particles->particles_init_density);
            }
            else if(is<Cloth>(node)) {
                auto cloth = cast<Cloth>(node);
                ser.serialize_member("source_grid",cloth->source_grid);
                ser.serialize_member("source_size",cloth->source_size);
                ser.serialize_member("cloth_stretch",cloth->cloth_stretch);
                ser.serialize_member("cloth_shear",cloth->cloth_shear);
                ser.serialize_member("cloth_bend",cloth->cloth_bend);
                ser.serialize_member("cloth_dump",cloth->cloth_dump);
                ser.serialize_member("cloth_density",cloth->cloth_density);
                ser.serialize_member("pinned",cloth->pinned);
            }
            else not_implemented_error();
        }
        else not_implemented_error();
    }
    else if(is<PrimitiveGroup>(node)) {
        auto group = cast<PrimitiveGroup>(node);
        ser.serialize_member("prims",group->prims);
    }
    else if(is<Bone>(node)) {
        auto bone = cast<Bone>(node);
        ser.serialize_member("frame_rest",bone->frame_rest);
        ser.serialize_member("frame_pose",bone->frame_pose);
        ser.serialize_member("display_endpoint",bone->display_endpoint);
        ser.serialize_member("rotation_euler",bone->rotation_euler);
        ser.serialize_member("anim_rotation_euler",bone->anim_rotation_euler);
        ser.serialize_member("parent",bone->parent);
    }
    else if(is<BoneWeights>(node)) {
        auto weights = cast<BoneWeights>(node);
        ser.serialize_member("weight",weights->weight);
        ser.serialize_member("idx",weights->idx);
    }
    else if(is<Gizmo>(node)) {
        auto gizmo = cast<Gizmo>(node);
        if(not gizmo) error("node is null");
        else if(is<Grid>(node)) {
            auto grid = cast<Grid>(node);
            ser.serialize_member("frame",grid->frame);
            ser.serialize_member("color",grid->color);
            ser.serialize_member("steps",grid->steps);
            ser.serialize_member("size",grid->size);
            ser.serialize_member("thickness",grid->thickness);
        }
        else if(is<Axes>(node)) {
            auto axes = cast<Axes>(node);
            ser.serialize_member("frame",axes->frame);
            ser.serialize_member("color_x",axes->color_x);
            ser.serialize_member("color_y",axes->color_y);
            ser.serialize_member("color_z",axes->color_z);
            ser.serialize_member("size",axes->size);
            ser.serialize_member("thickness",axes->thickness);
        }
        else if(is<Line>(node)) {
            auto line = cast<Line>(node);
            ser.serialize_member("pos0",line->pos0);
            ser.serialize_member("pos1",line->pos1);
            ser.serialize_member("color",line->color);
            ser.serialize_member("thickness",line->thickness);
        }
        else if(is<Dot>(node)) {
            auto dot = cast<Dot>(node);
            ser.serialize_member("pos",dot->pos);
            ser.serialize_member("color",dot->color);
            ser.serialize_member("thickness",dot->thickness);
        }
        else not_implemented_error();
    }
    else if(is<GizmoGroup>(node)) {
        auto gizmogroup = cast<GizmoGroup>(node);
        ser.serialize_member("gizmos",gizmogroup->gizmos);
    }
    else if(is<Scene>(node)) {
        auto scene = cast<Scene>(node);
        ser.serialize_member("camera",scene->camera);
        ser.serialize_member("lights",scene->lights);
        ser.serialize_member("prims",scene->prims);
    }
    else if(is<DrawOptions>(node)) {
        auto opts = cast<DrawOptions>(node);
        ser.serialize_member("res", opts->res);
        ser.serialize_member("samples", opts->samples);
        ser.serialize_member("doublesided", opts->doublesided);
        ser.serialize_member("time", opts->time);
        ser.serialize_member("background", opts->background);
        ser.serialize_member("ambient", opts->ambient);
        ser.serialize_member("cameralights", opts->cameralights);
        ser.serialize_member("cameralights_dir", opts->cameralights_dir);
        ser.serialize_member("cameralights_col", opts->cameralights_col);
        ser.serialize_member("faces", opts->faces);
        ser.serialize_member("edges", opts->edges);
        ser.serialize_member("lines", opts->lines);
        ser.serialize_member("control", opts->control);
        ser.serialize_member("control_no_depth", opts->control_no_depth);
        ser.serialize_member("gizmos", opts->gizmos);
    }
    else not_implemented_error();
}
