/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "scene/material.hpp"
#include "application/opengl.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ) { }
Model::~Model() { }

void Model::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    mesh->render();
    if ( material )
        material->reset_gl_state();
}


bool Model::Hit(Ray ray, real_t t0, real_t t1, HitRecord &rec)
{
	Ray localRay;

	Matrix4 transMatrix, inverseMatrix;
	Matrix3 normalMatrix;
	// make all the useful matrix
	make_transformation_matrix(&transMatrix, position, orientation, scale);
	make_inverse_transformation_matrix(&inverseMatrix, position, orientation, scale);
	make_normal_matrix(&normalMatrix, transMatrix);

    localRay.origin = inverseMatrix.transform_point(ray.origin);
    localRay.dir = inverseMatrix.transform_vector(ray.dir);
	
	size_t vertexIdx0,vertexIdx1,vertexIdx2;
	real_t a,b,c,d,e,f,g,h,i,j,k,l;
	real_t m;
	real_t beta, gamma, theta;
	real_t curTheta = INFINITE;

	// traverse all the triangles in the mesh
	bool isHit = false;
	for (size_t ii = 0; ii < mesh->num_triangles(); ii++)
	{
		vertexIdx0 = mesh->get_triangles()[ii].vertices[0];
		vertexIdx1 = mesh->get_triangles()[ii].vertices[1];
		vertexIdx2 = mesh->get_triangles()[ii].vertices[2];	

		// calculate gamma, beta, theta
		a = mesh->get_vertices()[vertexIdx0].position.x - mesh->get_vertices()[vertexIdx1].position.x;
		b = mesh->get_vertices()[vertexIdx0].position.y - mesh->get_vertices()[vertexIdx1].position.y;
		c = mesh->get_vertices()[vertexIdx0].position.z - mesh->get_vertices()[vertexIdx1].position.z;

		d = mesh->get_vertices()[vertexIdx0].position.x - mesh->get_vertices()[vertexIdx2].position.x;
		e = mesh->get_vertices()[vertexIdx0].position.y - mesh->get_vertices()[vertexIdx2].position.y;
		f = mesh->get_vertices()[vertexIdx0].position.z - mesh->get_vertices()[vertexIdx2].position.z;

		g = localRay.dir.x;
		h = localRay.dir.y;
		i = localRay.dir.z;

		j = mesh->get_vertices()[vertexIdx0].position.x - localRay.origin.x;
		k = mesh->get_vertices()[vertexIdx0].position.y - localRay.origin.y;
		l = mesh->get_vertices()[vertexIdx0].position.z - localRay.origin.z;

		m = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
		
		beta = (j*(e*i - h*f) + k*(g*f - d*i) + l*(d*h - e*g))/m;
		gamma = (i*(a*k - j*b) + h*(j*c - a*l) + g*(b*l - k*c))/m;
		theta = -(f*(a*k - j*b) + e*(j*c - a*l) + d*(b*l - k*c))/m;

		// intersection detect
		if(theta < t0 || theta > t1)
			continue;
		if(gamma < 0 || gamma >1)
			continue;
		if (beta < 0 || beta > (1-gamma))
			continue;

		// set the record if the theta is smaller than last one,
		// means the triangle at front of last one
		if (theta<curTheta)
		{
			// set t to hit record and reset current t
			curTheta = theta;
			isHit = true;
			rec.t = theta;

			// position
			Vector3 localPos;
			localPos = (1 - beta - gamma)*mesh->get_vertices()[vertexIdx0].position + beta*mesh->get_vertices()[vertexIdx1].position + gamma*mesh->get_vertices()[vertexIdx2].position;
			rec.position = transMatrix.transform_point(localPos);

			// normal
			rec.normal = (1 - beta - gamma)*mesh->get_vertices()[vertexIdx0].normal + beta*mesh->get_vertices()[vertexIdx1].normal + gamma*mesh->get_vertices()[vertexIdx2].normal;
			rec.normal=normalize(normalMatrix*rec.normal);

			// material
			rec.ambientColor = material->ambient;
			rec.diffuseColor = material->diffuse;
			rec.specularColor = material->specular;
			rec.refractiveIdx = material->refractive_index;

			// pixel color
			Vector2 coord = (1 - beta - gamma)*mesh->get_vertices()[vertexIdx0].tex_coord + beta*mesh->get_vertices()[vertexIdx1].tex_coord + gamma*mesh->get_vertices()[vertexIdx2].tex_coord;
			coord.x = coord.x - (int)coord.x;
			coord.y = coord.y - (int)coord.y;
			int width, height;
			material->get_texture_size(&width, &height);
			Color3 c0 = material->get_texture_pixel(coord.x*width, coord.y*height);
			Color3 c1 = material->get_texture_pixel(coord.x*width, coord.y*height);
			Color3 c2 = material->get_texture_pixel(coord.x*width, coord.y*height);
			rec.textureColor =  (1 - beta - gamma)*c0 + beta*c1 + gamma*c2;
			
			
		}
	}
	return isHit;
}

} /* _462 */

