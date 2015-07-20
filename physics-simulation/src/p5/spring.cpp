#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "p5/spring.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include <iostream>

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero();
    body2_offset = Vector3::Zero();
    damping = 0.0;
}

void Spring::step( real_t dt )
{
    // TODO apply forces to attached bodies
	
	Vector3 dir1 = normalize((body1->position + body1->orientation*body1_offset) - (body2->position + body2->orientation*body2_offset));
	Vector3 dir2 = normalize((body2->position + body2->orientation*body2_offset) - (body1->position + body1->orientation*body1_offset));
	real_t dis = length((body1->position + body1->orientation*body1_offset) - (body2->position + body2->orientation*body2_offset)) - equilibrium;

	Vector3 velocity1 = dot(body1->velocity, dir1)*dir1;
	Vector3 force1 = -constant*dis*dir1 - damping*velocity1;
	body1->apply_force(force1, body1->orientation*body1_offset);

	Vector3 velocity2 = dot(body2->velocity, dir2)*dir2;
	Vector3 force2 = -constant*dis*dir2 - damping*velocity2;
	body2->apply_force(force2, body2->orientation*body1_offset);

}

}


