#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "node.h"

///@file igl/material.h Materials. @ingroup igl
///@defgroup material Materials
///@ingroup igl
///@{

/// Abstract Material
struct Material : Node {
};

/// Lambert Material
struct Lambert : Material {
    vec3f        diffuse = vec3f(0.75,0.75,0.75); ///< diffuse color
};

/// Phong Material
struct Phong : Material {
	vec3f        diffuse = vec3f(0.75,0.75,0.75); ///< diffuse color
    vec3f        specular = vec3f(0.25,0.25,0.25); ///< specular color
    float        exponent =  10; ///< specular exponent
    vec3f        reflection = zero3f; ///< reflection color
    
    bool use_reflected = false; ///< use reflected or bisector
};

///@name eval interface
///@{

/// check whether a material has textures
inline bool material_has_textures(Material* material) {
    if(is<Lambert>(material)) {
        return false;
    }
    else if(is<Phong>(material)) {
        return false;
    }
    else { not_implemented_error(); return nullptr; }
}

/// evalute perturbed shading frame
inline frame3f material_shading_frame(Material* material, const frame3f& frame, const vec2f& texcoord) {
    return frame;
}

/// resolve texture coordinates
inline Material* material_shading_textures(Material* material, const vec2f& texcoord) {
    if(is<Lambert>(material)) {
        auto lambert = cast<Lambert>(material);
        auto ret = new Lambert();
        ret->diffuse = lambert->diffuse;
        return ret;
    }
    else if(is<Phong>(material)) {
        auto phong = cast<Phong>(material);
        auto ret = new Phong();
        ret->diffuse = phong->diffuse;
        ret->specular = phong->specular;
        ret->exponent = mean_component(vec3f(phong->exponent,phong->exponent,phong->exponent));
        ret->reflection = phong->reflection;
        return ret;
    }
    else { not_implemented_error(); return nullptr; }
}

/// evaluate the material color
inline vec3f material_diffuse_albedo(Material* material) {
    error_if_not(not material_has_textures(material), "cannot support textures");
    if(is<Lambert>(material)) return cast<Lambert>(material)->diffuse;
    else if(is<Phong>(material)) return cast<Phong>(material)->diffuse;
    else { not_implemented_error(); return zero3f; }
}

/// evaluete the emission of the material
inline vec3f material_emission(Material* material, const frame3f& frame, const vec3f& wo) {
    return zero3f;
}

/// evaluate an approximation of the fresnel model
inline vec3f _schlickFresnel(const vec3f& rhos, float iDh) {
    return rhos + (vec3f(1,1,1)-rhos) * pow(1.0f-iDh,5.0f);
}

/// evaluate an approximation of the fresnel model
inline vec3f _schlickFresnel(const vec3f& rhos, const vec3f& w, const vec3f& wh) {
    return _schlickFresnel(rhos, dot(wh,w));
}

/// evaluate product of BRDF and cosine
inline vec3f material_brdfcos(Material* material, const frame3f& frame, const vec3f& wi, const vec3f& wo) {
    error_if_not(not material_has_textures(material), "cannot support textures");
    if(is<Lambert>(material)) {
        auto lambert = cast<Lambert>(material);
        if(dot(wi,frame.z) <= 0 or dot(wo,frame.z) <= 0) return zero3f;
        return lambert->diffuse * abs(dot(wi,frame.z)) / pif;
    }
    else if(is<Phong>(material)) {
        auto phong = cast<Phong>(material);
        if(dot(wi,frame.z) <= 0 or dot(wo,frame.z) <= 0) return zero3f;
        if(phong->use_reflected) {
            vec3f wr = reflect(-wi,frame.z);
            return (phong->diffuse / pif + (phong->exponent + 8) * phong->specular*pow(max(dot(wo,wr),0.0f),phong->exponent) / (8*pif)) * abs(dot(wi,frame.z));
        } else {
            vec3f wh = normalize(wi+wo);
            return (phong->diffuse / pif + (phong->exponent + 8) * phong->specular*pow(max(dot(frame.z,wh),0.0f),phong->exponent) / (8*pif)) * abs(dot(wi,frame.z));
        }
    }
    else { not_implemented_error(); return zero3f; }
}

/// material average color for interactive drawing
inline vec3f material_display_color(Material* material) {
    if(is<Lambert>(material)) return cast<Lambert>(material)->diffuse;
    else if(is<Phong>(material)) return cast<Phong>(material)->diffuse;
    else { not_implemented_error(); return zero3f; }
}
///@}

///@}

#endif
