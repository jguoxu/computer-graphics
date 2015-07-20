/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
}

Triangle::~Triangle() { }

bool Triangle::Hit(Ray ray, real_t t0, real_t t1, HitRecord &rec)
{
	Ray localRay;

	Matrix4 transMatrix, inverseMatrix;
	Matrix3 normalMatrix;
	// make all the transform matrix
	make_transformation_matrix(&transMatrix, position, orientation, scale);
	make_inverse_transformation_matrix(&inverseMatrix, position, orientation, scale);
	make_normal_matrix(&normalMatrix, transMatrix);
	// localize the ray
    localRay.origin = inverseMatrix.transform_point(ray.origin);
    localRay.dir = inverseMatrix.transform_vector(ray.dir);

	// calculate all beta, gamma, theta
	real_t a,b,c,d,e,f,g,h,i,j,k,l;
	a = vertices[0].position.x - vertices[1].position.x;
	b = vertices[0].position.y - vertices[1].position.y;
	c = vertices[0].position.z - vertices[1].position.z;

	d = vertices[0].position.x - vertices[2].position.x;
	e = vertices[0].position.y - vertices[2].position.y;
	f = vertices[0].position.z - vertices[2].position.z;

	g = localRay.dir.x;
	h = localRay.dir.y;
	i = localRay.dir.z;

	j = vertices[0].position.x - localRay.origin.x;
	k = vertices[0].position.y - localRay.origin.y;
	l = vertices[0].position.z - localRay.origin.z;

	real_t m;
	m = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);

	real_t beta, gamma, theta;
	beta = (j*(e*i - h*f) + k*(g*f - d*i) + l*(d*h - e*g))/m;
	gamma = (i*(a*k - j*b) + h*(j*c - a*l) + g*(b*l - k*c))/m;
	theta = -(f*(a*k - j*b) + e*(j*c - a*l) + d*(b*l - k*c))/m;
	
	// interection detect
	if(theta < t0 || theta > t1)
		return false;
	if(gamma < 0 || gamma >1)
		return false;
	if (beta < 0 || beta > (1-gamma))
		return false;

	Vector3 localPos;
	localPos = (1 - beta - gamma)*vertices[0].position + beta*vertices[1].position + gamma*vertices[2].position;

	// normal
	rec.normal = (1 - beta - gamma)*vertices[0].normal + beta*vertices[1].normal + gamma*vertices[2].normal;
	rec.normal = normalize(normalMatrix*rec.normal);

	// material
	rec.ambientColor = (1 - beta - gamma)*vertices[0].material->ambient + beta*vertices[1].material->ambient + gamma*vertices[2].material->ambient;
	rec.diffuseColor = (1 - beta - gamma)*vertices[0].material->diffuse + beta*vertices[1].material->diffuse + gamma*vertices[2].material->diffuse;
	rec.specularColor = (1 - beta - gamma)*vertices[0].material->specular + beta*vertices[1].material->specular + gamma*vertices[2].material->specular;
	rec.refractiveIdx = vertices[0].material->refractive_index;

	// pixel color 
	Vector2 coord = (1 - beta - gamma)*vertices[0].tex_coord + beta*vertices[1].tex_coord + gamma*vertices[2].tex_coord;
	coord.x = coord.x - (int)coord.x;
	coord.y = coord.y - (int)coord.y;
	int width, height;
	vertices[0].material->get_texture_size(&width, &height);
	Color3 c0 = vertices[0].material->get_texture_pixel(coord.x*width, coord.y*height);
	Color3 c1 = vertices[1].material->get_texture_pixel(coord.x*width, coord.y*height);
	Color3 c2 = vertices[2].material->get_texture_pixel(coord.x*width, coord.y*height);
	rec.textureColor =  (1 - beta - gamma)*c0 + beta*c1 + gamma*c2;

	// position
	rec.position = transMatrix.transform_point(localPos);

	// t
	rec.t = theta;
	return true;

}


void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}


} /* _462 */

