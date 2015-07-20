/**
 * @file sphere.hpp
 * @brief Class defnition for Sphere.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_SPHERE_HPP_
#define _462_SCENE_SPHERE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * A sphere, centered on its position with a certain radius.
 */
class Sphere : public Geometry
{
public:

    real_t radius;
    const Material* material;

    Sphere();
    virtual ~Sphere();
    virtual void render() const;
	virtual bool Hit(Ray ray, real_t t0, real_t t1, HitRecord &rec);
	Vector2 get_coord(Vector3 v);
	Material get_material();

	void SetMaterialColors(Color3 &ambient, Color3 &diffuse, Color3 &specular);
	Color3 GetTexture(Vector3 position);
};

} /* _462 */

#endif /* _462_SCENE_SPHERE_HPP_ */

