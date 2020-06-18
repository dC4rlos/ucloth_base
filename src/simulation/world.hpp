#ifndef UCLOTH_WORLD_H_
#define UCLOTH_WORLD_H_

#include <constraint.hpp>
#include <umath.hpp>
#include <uclothcommon.hpp>
#include <vector>

namespace ucloth{
    namespace simulation{
        struct World{
            //Particles as Components
            std::vector<umath::Position> positions;
            std::vector<umath::Vec3> velocities;
            std::vector<umath::Real> inverseMasses;

            // Relations between particles
            std::vector<Attachment> attachments;
            std::vector<DistanceConstraint> distanceConstraints;

            //Meshes
            std::vector<Mesh> meshes;

            // External forces
            std::vector<umath::Vec3> accelerations;

            void addCloth(std::vector<umath::Position> const& pos, Mesh const& mesh, umath::Real const mass, umath::Real const elasticity/*, umath::Real bendingStiffness*/);
            void attachParticle(Mesh const& mesh, Particle particle, umath::Position const& position);

            private: 
                void reserveForNParticles(size_t const nParticles);
        };
    } // namespace simulation
} // namespace ucloth

#endif // !UCLOTH_WORLD_H_