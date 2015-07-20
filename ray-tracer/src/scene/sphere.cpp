/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "application/opengl.hpp"

namespace _462 {

#define SPHERE_NUM_LAT 80
#define SPHERE_NUM_LON 100

#define SPHERE_NUM_VERTICES ( ( SPHERE_NUM_LAT + 1 ) * ( SPHERE_NUM_LON + 1 ) )
#define SPHERE_NUM_INDICES ( 6 * SPHERE_NUM_LAT * SPHERE_NUM_LON )
// index of the x,y sphere where x is lat and y is lon
#define SINDEX(x,y) ((x) * (SPHERE_NUM_LON + 1) + (y))
#define VERTEX_SIZE 8
#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5

static unsigned int Indices[SPHERE_NUM_INDICES];
static float Vertices[VERTEX_SIZE * SPHERE_NUM_VERTICES];

static void init_sphere()
{
    static bool initialized = false;
    if ( initialized )
        return;

    for ( int i = 0; i <= SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j <= SPHERE_NUM_LON; j++ ) {
            real_t lat = real_t( i ) / SPHERE_NUM_LAT;
            real_t lon = real_t( j ) / SPHERE_NUM_LON;
            float* vptr = &Vertices[VERTEX_SIZE * SINDEX(i,j)];

            vptr[TCOORD_OFFSET + 0] = lon;
            vptr[TCOORD_OFFSET + 1] = 1-lat;

            lat *= PI;
            lon *= 2 * PI;
            real_t sinlat = sin( lat );

            vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0] = sinlat * sin( lon );
            vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1] = cos( lat ),
            vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2] = sinlat * cos( lon );
        }
    }

    for ( int i = 0; i < SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j < SPHERE_NUM_LON; j++ ) {
            unsigned int* iptr = &Indices[6 * ( SPHERE_NUM_LON * i + j )];

            unsigned int i00 = SINDEX(i,  j  );
            unsigned int i10 = SINDEX(i+1,j  );
            unsigned int i11 = SINDEX(i+1,j+1);
            unsigned int i01 = SINDEX(i,  j+1);

            iptr[0] = i00;
            iptr[1] = i10;
            iptr[2] = i11;
            iptr[3] = i11;
            iptr[4] = i01;
            iptr[5] = i00;
        }
    }

    initialized = true;
}

Sphere::Sphere()
    : radius(0), material(0) {}

Sphere::~Sphere() {}


// set the material's color in hit record
void Sphere::SetMaterialColors(Color3 &ambient, Color3 &diffuse, Color3 &specular)
{
	ambient = material->ambient;
	diffuse = material->diffuse;
	specular = material->specular;
}

// return the texture color at certain position
Color3 Sphere::GetTexture(Vector3 pos)
{
	real_t angle, p, x, y;
	int width, height;
	material->get_texture_size(&width,&height);	
	angle = acos((pos.z-position.z)/radius);
	p = atan2((pos.y - position.y), (pos.x - position.x));
	x = p/(2.0*PI);
	y = (PI - angle)/PI;
	Color3 texColor = material->get_texture_pixel((int)(x*width),(int)(y*height));
	return texColor;
}

bool Sphere::Hit(Ray ray, real_t t0, real_t t1, HitRecord &rec)
{
	
	Vector3 localPos;
	Matrix4 inverseMatrix, transMatrix;
	Matrix3 temp, normalMatrix;
	Ray localRay;

	// calculate transform/ transform inverse/ normal matrix
	make_transformation_matrix(&transMatrix, position, orientation, scale);
	make_inverse_transformation_matrix(&inverseMatrix, position, orientation, scale);
	make_normal_matrix(&normalMatrix, transMatrix);
	
	// get the local ray
	localRay.origin = inverseMatrix.transform_point(ray.origin);
	localRay.dir = inverseMatrix.transform_vector(ray.dir);

	// do the intersection calculation
	real_t result, x1, x2, oriDotOri , dirDotDir, oriDotDir;
	oriDotDir = dot(localRay.dir , localRay.origin); 
	dirDotDir = dot(localRay.dir , localRay.dir); 
	oriDotOri = dot(localRay.origin , localRay.origin); 
	result = oriDotDir*oriDotDir-dirDotDir*(oriDotOri-radius*radius);

	// no intersection
	if(result < 0)
		return false;
	else 
	{
		x1=(-oriDotDir - sqrt(result)) / dirDotDir;
		x2=(-oriDotDir + sqrt(result)) / dirDotDir;
		// x1 is the place is the intersection
		if(x1>t0 && x1<t1)
		{
			localPos = localRay.origin + x1*localRay.dir;
			// set material 
			SetMaterialColors(rec.ambientColor, rec.diffuseColor,rec.specularColor);
			rec.refractiveIdx = material->refractive_index;

			// set normal
			rec.normal=(localPos)/radius;
			rec.normal=normalize(normalMatrix*rec.normal);
			
			// set texture color
			rec.textureColor = GetTexture(localPos);

			// set position
			rec.position = transMatrix.transform_point(localPos);

			// set t
			rec.t = x1;
			return true;
		}
		else
		{
			// x2 is the place is the intersection
			if(x2>t0 && x2<t1)
			{
				localPos = localRay.origin + x2*localRay.dir;
				//set material
				SetMaterialColors(rec.ambientColor, rec.diffuseColor,rec.specularColor);
				rec.refractiveIdx = material->refractive_index;

				// set normal
				rec.normal=(localPos)/radius;
				rec.normal=normalize(normalMatrix*rec.normal);

				// set texture
				rec.textureColor = GetTexture(localPos);
				
				// set position
				rec.position = transMatrix.transform_point(localPos);
				
				// set t
				rec.t = x2;
				return true;
			}
			return false;
	}

	}
}


void Sphere::render() const
{
    // create geometry if we haven't already
    init_sphere();

    if ( material )
        material->set_gl_state();

    // just scale by radius and draw unit sphere
    glPushMatrix();
    glScaled( radius, radius, radius );
    glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE * sizeof Vertices[0], Vertices );
    glDrawElements( GL_TRIANGLES, SPHERE_NUM_INDICES, GL_UNSIGNED_INT, Indices );
    glPopMatrix();

    if ( material )
        material->reset_gl_state();
}


} /* _462 */

