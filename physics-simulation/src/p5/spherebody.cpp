#include "p5/spherebody.hpp"
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "scene/sphere.hpp"
#include <iostream>
#include <exception>
#include <algorithm>

namespace _462 {

SphereBody::SphereBody( Sphere* geom )
{
    sphere = geom;
    position = sphere->position;
    radius = sphere->radius;
    orientation = sphere->orientation;
    mass = 0.0;
    velocity = Vector3::Zero();
	acceleration = Vector3::Zero();
    angular_velocity = Vector3::Zero();
    force = Vector3::Zero();
    torque = Vector3::Zero();
}

Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in position dt in the future

	acceleration = force/mass;
	velocity += (force*dt)/mass;
	return velocity;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in orientation dt in the future
    // vec.x = rotation along x axis
    // vec.y = rotation along y axis
    // vec.z = rotation along z axis

	Vector3 angularAcceleration = torque/(2*mass*radius*radius/5);
	angular_velocity += angularAcceleration*dt;

    return angular_velocity;
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset )
{
    // TODO apply force/torque to sphere
	if (offset == Vector3::Zero())
	{
		force += f;
	}
	else
	{
		force += dot(f,normalize(offset)) * normalize(offset);
		torque = cross(offset, f);
	}
}

}
