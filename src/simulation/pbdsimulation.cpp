#include "pbdsimulation.hpp"
#include <numeric>

namespace ucloth{
    namespace simulation{
        void PBDSimulation::simulate(umath::Real const deltaTime, size_t const solverIterations, World& world){

        }

        void PBDSimulation::applyExternalAccelerations(std::vector<umath::Vec3> const& externalAccelerations, umath::Real const deltaTime, std::vector<umath::Vec3>& velocities){
            for(auto const& acceleration : externalAccelerations){
                for(auto& velocity : velocities){
                    velocity += acceleration * deltaTime;
                }
            }
        }

         void PBDSimulation::dampVelocities(std::vector<Mesh> const& meshes, std::vector<umath::Position> const& positions, std::vector<umath::Real> const& inverseMasses, std::vector<umath::Vec3>& velocities){
             for(auto const& mesh : meshes){
                umath::Real const total_mass = 
                    std::accumulate(inverseMasses.begin() + mesh.begin, 
                                    inverseMasses.begin() + mesh.end, 
                                    0.0f,
                                    [](umath::Real input, umath::Real const inverseMass) -> umath::Real { 
                                        return input + (1.0f / inverseMass); 
                                    });
             }
         }
    } // namespace simulation
} // namespace ucloth
