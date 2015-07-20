#include "p5/physics.hpp"
#define EPCILON 0.0001
namespace _462 {

// class Rk4objec used for calculate the position, orienttation, and velocity value
class RK4Object
{
public:
	Vector3 k1, k2, k3, k4;

	RK4Object()
	{
		k1 = k2 = k3 = k4 = Vector3::Zero();
	}

	Vector3 GetResult()
	{
		return (k1+2*k2+2*k3+k4)/6;
	}
};

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}


void Physics::step( real_t dt )
{
    // TODO step the world forward by dt. Need to detect collisions, apply
    // forces, and integrate positions and orientations.
    //
    // Note: put RK4 here, not in any of the physics bodies
    //
    // Must use the functions that you implemented
    //
    // Note, when you change the position/orientation of a physics object,
    // change the position/orientation of the graphical object that represents
    // it

	for (size_t i = 0; i < spheres.size(); i++)
	{
		bool isCollide = false;

		
		//printf("%f\n", length(spheres[i]->velocity));
		// detect the sphere to triangle collision
		for (size_t triangleCount = 0; triangleCount < triangles.size(); triangleCount++)
		{
			if(collides(*spheres[i], *triangles[triangleCount], collision_damping))
			{
				isCollide = true;
			}
		}
		// detect the sphere to sphere collision
		for (size_t sphereCount = 0; sphereCount < spheres.size(); sphereCount++)
		{
			if (i != sphereCount)
			{
				if(collides(*spheres[i], *spheres[sphereCount], collision_damping))
				{
					isCollide = true;
				}
			}
		}
		// detect the sphere to plane collision
		for (size_t planeCount = 0; planeCount < planes.size(); planeCount++)
		{
			if(collides(*spheres[i], *planes[planeCount], collision_damping))
			{
				isCollide = true;
			}
		}


		if (length(spheres[i]->velocity)<=EPCILON && isCollide)
		{
			continue;
			//spheres[i]->apply_force(Vector3::Zero(), Vector3::Zero());
		}
			
		else
		{
		
			RK4Object positionRk4Obj = RK4Object();
			RK4Object orientationRk4Obj = RK4Object();
			RK4Object velocityRk4Obj = RK4Object();

			Vector3 positionOrigin = spheres[i]->position;
			Quaternion orientationOrigin = spheres[i]->orientation;
			Vector3 velocityOrigin = spheres[i]->velocity;

			// RK4 K1 caculation
			spheres[i]->apply_force(spheres[i]->mass*gravity, Vector3::Zero());
			for (size_t springCount = 0; springCount < springs.size(); springCount++)
			{
				springs[springCount]->step(0.0);
			}
			positionRk4Obj.k1 = dt*spheres[i]->step_position(0.0, 0.0);
			orientationRk4Obj.k1 = dt*spheres[i]->step_orientation(0.0, 0.0);
			velocityRk4Obj.k1 = dt*spheres[i]->force/spheres[i]->mass;
			spheres[i]->force = Vector3::Zero();

			// RK4 k2 calculation
			spheres[i]->position += positionRk4Obj.k1/2.0;
			spheres[i]->apply_force(spheres[i]->mass*gravity, Vector3::Zero());
			for (size_t springCount = 0; springCount < springs.size(); springCount++)
			{
				springs[springCount]->step(dt/2.0);
			}
			positionRk4Obj.k2 = dt*spheres[i]->step_position(dt/2.0, 0.0);
			orientationRk4Obj.k2 = dt*spheres[i]->step_orientation(dt/2.0, 0.0);
			velocityRk4Obj.k2 = dt*spheres[i]->force/spheres[i]->mass;
			spheres[i]->force = Vector3::Zero();

			// RK4 k3 calculation
			spheres[i]->position += positionRk4Obj.k2/2.0;
			spheres[i]->apply_force(spheres[i]->mass*gravity, Vector3::Zero());
			for (size_t springCount = 0; springCount < springs.size(); springCount++)
			{
				springs[springCount]->step(dt/2.0);
			}
			positionRk4Obj.k3 = dt*spheres[i]->step_position(dt/2.0, 0.0);
			orientationRk4Obj.k3 = dt*spheres[i]->step_orientation(dt/2.0, 0.0);
			velocityRk4Obj.k3 = dt*spheres[i]->force/spheres[i]->mass;
			spheres[i]->force = Vector3::Zero();

			// Rk4 K4 calculation
			spheres[i]->position += positionRk4Obj.k3;
			spheres[i]->apply_force(spheres[i]->mass*gravity, Vector3::Zero());
			for (size_t springCount = 0; springCount < springs.size(); springCount++)
			{
				springs[springCount]->step(dt);
			}
			positionRk4Obj.k4 = dt*spheres[i]->step_position(dt, 0.0);
			orientationRk4Obj.k4 = dt*spheres[i]->step_orientation(dt, 0.0);
			velocityRk4Obj.k4 = dt*spheres[i]->force/spheres[i]->mass;
			spheres[i]->force = Vector3::Zero();

			//Update position and orientation
			Vector3 newRotationEuler = orientationRk4Obj.GetResult();
			Quaternion newRotation = spheres[i]->orientation;
			newRotation = normalize( Quaternion( newRotation*Vector3::UnitX(), newRotationEuler.x ) * newRotation );
			newRotation = normalize( Quaternion( newRotation*Vector3::UnitZ(), newRotationEuler.z ) * newRotation );
			newRotation = normalize( Quaternion( newRotation*Vector3::UnitY(), newRotationEuler.y ) * newRotation );

			spheres[i]->orientation = newRotation;
			spheres[i]->sphere->orientation = spheres[i]->orientation;

			spheres[i]->position = positionOrigin + positionRk4Obj.GetResult();
			spheres[i]->sphere->position =spheres[i]->position;
		}
	}
}

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    
	gravity = Vector3::Zero();
	collision_damping = 0.0;
}

}
