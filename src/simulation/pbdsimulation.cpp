#include "pbdsimulation.hpp"

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
    } // namespace simulation
} // namespace ucloth
