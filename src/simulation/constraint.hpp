#ifndef UCLOTH_CONSTRAINT_H_
#define UCLOTH_CONSTRAINT_H_

#include <umath.hpp>
#include <uclothcommon.hpp>

namespace ucloth{
    namespace simulation{
        struct DistanceConstraint{
            Particle p1;
            Particle p2;
            umath::Real distance;
            umath::Real stiffness;
        };
    } // namespace simulation
} // namespace ucloth

#endif // !UCLOTH_CONSTRAINT_H_