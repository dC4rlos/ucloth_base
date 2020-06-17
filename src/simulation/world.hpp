#ifndef UCLOTH_WORLD_H_
#define UCLOTH_WORLD_H_

#include <umath/umath.hpp>
#include <simulation/uclothcommon.hpp>
#include <vector>

namespace ucloth{
    namespace simulation{
        struct World{
            //Particles as Components
            std::vector<umath::Position> positions;
            std::vector<umath::Vec3> velocities;
            std::vector<umath::Real> inverseMasses;

            //Meshes
            std::vector<Mesh> meshes;

            void addCloth(std::vector<umath::Position> const& pos, Mesh const& mesh, umath::Real mass/*, umath::Real elasticity, umath::Real bendingStiffness*/);

            private: 
                void reserveForNParticles(size_t const nParticles);
        };
    }
}

#endif // !UCLOTH_WORLD_H_