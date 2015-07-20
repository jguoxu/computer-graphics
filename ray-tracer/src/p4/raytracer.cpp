/**
 * @file raytacer.cpp
 * @brief Raytracer class
 *
 * Implement these functions for project 4.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "raytracer.hpp"
#include "scene/scene.hpp"

//#include <SDL/SDL_timer.h>
#include "SDL_timer.h"
#include <iostream>

#ifdef OPENMP // just a defense in case OpenMP is not installed.

#include <omp.h>

#endif
namespace _462 {

// max number of threads OpenMP can use. Change this if you like.
#define MAX_THREADS 1

static const unsigned STEP_SIZE = 8;

Raytracer::Raytracer()
    : scene(0), width(0), height(0) { }

Raytracer::~Raytracer() { }

/**
 * Initializes the raytracer for the given scene. Overrides any previous
 * initializations. May be invoked before a previous raytrace completes.
 * @param scene The scene to raytrace.
 * @param width The width of the image being raytraced.
 * @param height The height of the image being raytraced.
 * @return true on success, false on error. The raytrace will abort if
 *  false is returned.
 */
bool Raytracer::initialize( Scene* scene, size_t width, size_t height )
{
    /*
     * omp_set_num_threads sets the maximum number of threads OpenMP will
     * use at once.
     */
#ifdef OPENMP
    omp_set_num_threads(MAX_THREADS);
#endif
    this->scene = scene;
    this->width = width;
    this->height = height;

    current_row = 0;

    // TODO any initialization or precompuation before the trace
    return true;
}

// Test if there's object block the light to create shadow
static bool IsShadowed(const Scene* scene, Vector3 pos0, Vector3 pos1)
{
	Ray ray;
	ray.origin = pos0;
	ray.dir = normalize(pos1 - pos0) ;

	HitRecord r;
	for(size_t i=0;i<scene->num_geometries(); i++){
		if(scene->get_geometries()[i]->Hit (ray, 0.00001, distance(pos1,pos0), r))
		{
			return true;
		}
	}
	return false;
}

// check if is refract
static bool IsRefract(Vector3 d, Vector3 normal, real_t n, Vector3 &t)
{
       real_t result;
       result = 1-n*n*(1-dot(d,normal)*dot(d,normal));

       if( result < 0 )
            return false;
       else
       {
            t = n*( d - normal*dot(d,normal)) - normal*sqrt(result);
            return true;
       }
}

// reflection times
size_t reflectionCounter = 0;
static Color3 RayTrace(Ray ray, const Scene* scene)
{
	reflectionCounter++;
	if (reflectionCounter>3
		)
	{
		return Color3(0,0,0);
	}

	// hit record used for record the hit information
	HitRecord hitRecord;
	HitRecord tempHitRecord;
	Ray reflectionRay;
	Ray refractionRay;
	// traverse all the geometries
	bool isHit = false;
	for(size_t i=0;i<scene->num_geometries(); i++){
		if(scene->get_geometries()[i]->Hit (ray,scene->camera.get_near_clip(),scene->camera.get_far_clip(), tempHitRecord))
		{
			if (tempHitRecord.t < hitRecord.t)
			{
				hitRecord = tempHitRecord;
			}			
			isHit = true;
		}
	}
	// if ray hit a geometry
	if (isHit)
	{

		// recursive for the reflection
		reflectionRay.dir = normalize(ray.dir - 2*dot(ray.dir, hitRecord.normal)*hitRecord.normal);
		reflectionRay.origin = hitRecord.position;
		// reflection case
		if (hitRecord.refractiveIdx == 0)
		{
			Color3 lightAmbientColor = scene->ambient_light;
			Color3 matAmbientColor = hitRecord.ambientColor;
			Color3 texColor = hitRecord.textureColor;
			Color3 totalBCK = Color3(0,0,0);

			// traverse all the lights
			for (size_t i = 0; i < scene->num_lights(); i++)
			{
				real_t d = length(hitRecord.position - scene->get_lights()[i].position);
				real_t base = (scene->get_lights()[i].attenuation.constant + scene->get_lights()[i].attenuation.linear*d +
					scene->get_lights()[i].attenuation.quadratic*d*d);

				Color3 lightColor = 1.0/base*scene->get_lights()[i].color;
				Color3 matDiffuseColor = hitRecord.diffuseColor;
				Vector3 lightVec = normalize(scene->get_lights()[i].position - hitRecord.position);
				real_t NL = dot(lightVec, hitRecord.normal);

				if (NL<=0)
				{
					NL = 0;
				}
				real_t shadowFlag = 1.0;
				// check if object block the light
				if (IsShadowed(scene, hitRecord.position, scene->get_lights()[i].position))
				{
					shadowFlag = 0.0;
				}	
				totalBCK += shadowFlag*lightColor*matDiffuseColor*NL;
			}

			Color3 color = texColor*(lightAmbientColor*matAmbientColor + totalBCK);
			
			color += RayTrace(reflectionRay, scene)*hitRecord.specularColor;

			return(color);
		}
		// refraction case
		else
		{
			real_t n, c, R0, R1;
            Vector3 t;
            
			n = scene->refractive_index/hitRecord.refractiveIdx;
			// check if the ray inside the mesh 
			if(dot(normalize(ray.dir), hitRecord.normal) < 0)
            {
                IsRefract(normalize(ray.dir), hitRecord.normal, n, t);
				c = dot(-normalize(ray.dir), hitRecord.normal);

            }
            else
            {
				// rafraction case inside the material
				if(IsRefract(normalize(ray.dir), -hitRecord.normal, 1.0/n, t))
                    c = dot(t, hitRecord.normal);
				// reflection case inside the material
                else
					return RayTrace(reflectionRay, scene);
            }

            R0 = (n-1)*(n-1)/(n+1)*(n+1);
            R1 = R0 + (1-R0)*pow(1-c,5);

			refractionRay.origin = hitRecord.position;
            refractionRay.dir = t;
            
			return (R1*RayTrace( reflectionRay, scene) + (1-R1)*RayTrace(refractionRay, scene));
		}
		
	}
	// if hit nothing, return background color
	return scene->background_color;
}

/**
 * Performs a raytrace on the given pixel on the current scene.
 * The pixel is relative to the bottom-left corner of the image.
 * @param scene The scene to trace.
 * @param x The x-coordinate of the pixel to trace.
 * @param y The y-coordinate of the pixel to trace.
 * @param width The width of the screen in pixels.
 * @param height The height of the screen in pixels.
 * @return The color of that pixel in the final image.
 */

real_t top, bot, left, right;
Vector3 w,u,v;


static Color3 trace_pixel(const Scene* scene,
                          size_t x,
                          size_t y,
                          size_t width,
                          size_t height)
{
    assert(x < width);
    assert(y < height);
    // TODO return the color of the given pixel
    // you don't have to use this stub function if you prefer to
    // write your own version of Raytracer::raytrace.

	// Compute u,v,w basis vectors
	Vector3 lookDirection=scene->camera.get_direction();
	Vector3 upDirection= scene ->camera.get_up();
	w = -lookDirection;
	u = cross(upDirection, w);
	v = cross(w,u);
	// calculate the view plane
	top = scene->camera.get_near_clip()*tan((scene->camera.get_fov_radians())/2.0); 
	right = top * (scene->camera.get_aspect_ratio());
	left = -right;
	bot = -top;   

	//calculate the world space position of the pixel
	real_t uSpace, vSpace, wSpace;
	uSpace = left + (right - left)*(x + 0.5) / width;
	vSpace = bot + (top - bot)*(y + 0.5) / height;
	wSpace = scene->camera.get_near_clip ();

	// calculate the ray to send out
	Ray ray;  
	Vector3 direction = normalize(-wSpace*w+uSpace*u + vSpace*v) ;
	ray.origin = scene->camera.get_position();
	ray.dir = direction ;
    reflectionCounter = 0;
	return RayTrace(ray, scene);
}



/**
 * Raytraces some portion of the scene. Should raytrace for about
 * max_time duration and then return, even if the raytrace is not copmlete.
 * The results should be placed in the given buffer.
 * @param buffer The buffer into which to place the color data. It is
 *  32-bit RGBA (4 bytes per pixel), in row-major order.
 * @param max_time, If non-null, the maximum suggested time this
 *  function raytrace before returning, in seconds. If null, the raytrace
 *  should run to completion.
 * @return true if the raytrace is complete, false if there is more
 *  work to be done.
 */
bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time)
{
    // TODO Add any modifications to this algorithm, if needed.

    static const size_t PRINT_INTERVAL = 64;

    // the time in milliseconds that we should stop
    unsigned int end_time = 0;
    bool is_done;

    if (max_time)
    {
        // convert duration to milliseconds
        unsigned int duration = (unsigned int) (*max_time * 1000);
        end_time = SDL_GetTicks() + duration;
    }

    // until time is up, run the raytrace. we render an entire group of
    // rows at once for simplicity and efficiency.
    for (; !max_time || end_time > SDL_GetTicks(); current_row += STEP_SIZE)
    {
        // we're done if we finish the last row
        is_done = current_row >= height;
        // break if we finish
        if (is_done) break;

        int loop_upper = std::min(current_row + STEP_SIZE, height);

        // This tells OpenMP that this loop can be parallelized.
        #pragma omp parallel for
        for (int c_row = current_row; c_row < loop_upper; c_row++)
        {
            /*
             * This defines a critical region of code that should be
             * executed sequentially.
             */
            #pragma omp critical
            {
                if (c_row % PRINT_INTERVAL == 0)
                    printf("Raytracing (Row %d)\n", c_row);
            }

            for (size_t x = 0; x < width; x++)
            {
                // trace a pixel
                Color3 color = trace_pixel(scene, x, c_row, width, height);
                // write the result to the buffer, always use 1.0 as the alpha
                color.to_array(&buffer[4 * (c_row * width + x)]);
            }
        }
    }

    if (is_done) printf("Done raytracing!\n");

    return is_done;
}

} /* _462 */
