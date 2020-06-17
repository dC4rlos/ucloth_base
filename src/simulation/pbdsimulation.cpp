#include "pbdsimulation.hpp"
#include <numeric>

namespace ucloth{
    namespace simulation{
        void PBDSimulation::simulate(umath::Real const deltaTime, size_t const solverIterations, World& world){
            applyExternalAccelerations(world.accelerations, deltaTime, world.velocities);
            dampVelocities(world.meshes, world.positions, world.inverseMasses, world.velocities);
            createPositionEstimates(world.positions, world.velocities, deltaTime);
        }

        void PBDSimulation::applyExternalAccelerations(std::vector<umath::Vec3> const& externalAccelerations, umath::Real const deltaTime, std::vector<umath::Vec3>& velocities) const{
            for(auto const& acceleration : externalAccelerations){
                for(auto& velocity : velocities){
                    velocity += acceleration * deltaTime;
                }
            }
        }

        void PBDSimulation::dampVelocities(std::vector<Mesh> const& meshes, std::vector<umath::Position> const& positions, std::vector<umath::Real> const& inverseMasses, std::vector<umath::Vec3>& velocities) const{
            for(auto const& mesh : meshes){
                // TODO: optimaze mass calculations
                // Total mass
                umath::Real const totalMass = 
                    std::accumulate(inverseMasses.begin() + mesh.begin, 
                                    inverseMasses.begin() + mesh.end, 
                                    0.0f,
                                    [](umath::Real input, umath::Real const inverseMass) -> umath::Real { 
                                        return input + (1.0f / inverseMass); 
                                    });

                // Calculate center of mass and its velocity
                umath::Position xcm = {0.0f, 0.0f, 0.0f};
                umath::Vec3 vcm = {0.0f, 0.0f, 0.0f};
                for(Particle p = mesh.begin; p < mesh.end; ++p){
                    xcm += positions[p] / (inverseMasses[p]);
                    vcm += velocities[p] / (inverseMasses[p]);
                }
                xcm /= totalMass;
                vcm /= totalMass;

                umath::Vec3 L = {0.0f, 0.0f, 0.0f};
                for(Particle p = mesh.begin; p < mesh.end; ++p){
                    umath::Position const ri = positions[p] - xcm;
                    L += umath::cross(ri, velocities[p] / inverseMasses[p]);
                }
                
                //Calculate inertia matrix
                umath::Mat3x3 I = umath::Mat3x3{0.0f};
                for(Particle p = mesh.begin; p < mesh.end; ++p){
                    umath::Position const ri = positions[p] - xcm;
                    // Column major
                    umath::Mat3x3 const riPrime = {{0, -ri.z, ri.y}, {ri.z, 0, -ri.x}, {-ri.y, ri.x, 0}};
                    I += riPrime * umath::transpose(riPrime) / inverseMasses[p];
                }

                umath::Vec3 const angularVelocity = umath::inverse(I) * L;
                for(Particle p = mesh.begin; p < mesh.end; ++p){
                    umath::Position const ri = positions[p] - xcm;
                    umath::Vec3 const deltaV = vcm + umath::cross(angularVelocity, ri) - velocities[p];
                    velocities[p] += mesh.kVelocity * deltaV;
                }
            }
         }

        void PBDSimulation::createPositionEstimates(std::vector<umath::Position> const& positions, std::vector<umath::Vec3> const& velocities, umath::Real  const deltaTime){
            size_t const nParticles = positions.size();
            m_positionEstimates.resize(nParticles); // Me da error aqui****
            for(Particle p = 0; p < nParticles; ++p){
                m_positionEstimates[p] = positions[p] + velocities[p] * deltaTime;
            }
        }
    } // namespace simulation
} // namespace ucloth
