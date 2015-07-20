#include "p5/collisions.hpp"


namespace _462 {

#define EPCILON 0.1

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	
	real_t d = distance(body1.position , body2.position);

	if (d <= body1.radius + body2.radius)
	{
		if(dot(body2.position - body1.position, body1.velocity - body2.velocity) > 0)
		{
			Vector3 v1 = body1.velocity - body2.velocity;
			Vector3 v2 = Vector3::Zero();

			Vector3 d = (body2.position - body1.position)/length(body2.position - body1.position);

			Vector3 v22 = 2*d*(body1.mass/(body1.mass + body2.mass))*dot(v1,d);

			Vector3 u2 = body2.velocity + v22;
			Vector3 u1 = (body1.mass*body1.velocity + body2.mass*body2.velocity - body2.mass*u2)/body1.mass;

			body1.velocity = u1;
			body2.velocity = u2;

			body1.velocity = body1.velocity - collision_damping*body1.velocity;
			body2.velocity = body2.velocity - collision_damping*body2.velocity;

			if (length(body1.velocity) <= EPCILON)
			{
				body1.velocity = Vector3::Zero();
			}
			if (length(body2.velocity) <= EPCILON)
			{
				body2.velocity = Vector3::Zero();
			}

			return true;	
		}	
	}
    return false;

}



bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	
	real_t dis = abs(dot((body1.position - body2.position), body2.normal));
	Vector3 projectedPosition = body1.position - abs(dis)*body2.normal;
	if (dis > body1.radius)
	{
		return false;
	}
	if (length(body1.velocity) <= EPCILON)
	{
		body1.velocity = Vector3::Zero();
		return true;
	}
	// calculate all beta, gamma, theta
	real_t a,b,c,d,e,f,g,h,i,j,k,l;
	a = body2.vertices[0].x - body2.vertices[1].x;
	b = body2.vertices[0].y - body2.vertices[1].y;
	c = body2.vertices[0].z - body2.vertices[1].z;

	d = body2.vertices[0].x - body2.vertices[2].x;
	e = body2.vertices[0].y - body2.vertices[2].y;
	f = body2.vertices[0].z - body2.vertices[2].z;

	g = -body2.normal.x;
	h = -body2.normal.y;
	i = -body2.normal.z;

	j = body2.vertices[0].x - body1.position.x;
	k = body2.vertices[0].y - body1.position.y;
	l = body2.vertices[0].z - body1.position.z;

	real_t m;
	m = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
	
	real_t beta, gamma, theta;
	beta = (j*(e*i - h*f) + k*(g*f - d*i) + l*(d*h - e*g))/m;
	gamma = (i*(a*k - j*b) + h*(j*c - a*l) + g*(b*l - k*c))/m;
	theta = -(f*(a*k - j*b) + e*(j*c - a*l) + d*(b*l - k*c))/m;

	// outside triangle
	if(gamma < 0 || gamma >1||beta < 0 || beta > (1-gamma))
	{
		return false;
	}
	// inside triangle
	else
	{
		if(dot(-body2.normal, body1.velocity)<=0) 
		{
			
			body1.velocity = body1.velocity - 2*dot(body1.velocity, body2.normal)*body2.normal;
			body1.velocity = body1.velocity - collision_damping*body1.velocity;
			if (length(body1.velocity) <= EPCILON)
			{
				body1.velocity = Vector3::Zero();
			}
		}

		return true;
	}
	return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	
	Vector3 alpha;
	real_t d;
	Vector3 normalFlag;

	alpha = body1.sphere->position -  body2.position;
	d = abs(dot(alpha, body2.normal));

	if (d <= body1.radius)
	{
		if(dot(body2.position-body1.position, body2.normal)>=0)
			normalFlag = -body2.normal;
		else
			normalFlag = body2.normal;

		if(dot(normalFlag, body1.velocity)<=0) // Check if the velocity has already been changed.
		{
			body1.velocity = body1.velocity - 2*dot(body1.velocity, body2.normal)*body2.normal;
			body1.velocity = body1.velocity - collision_damping*body1.velocity;
			if (length(body1.velocity) <= EPCILON)
			{
				body1.velocity = Vector3::Zero();
			}
		}
		return true;
	}

	return false;
	
}

}
