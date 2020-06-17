#ifndef UCLOTH_PBD_SIMULATION_H_
#define UCLOTH_PBD_SIMULATION_H_

#include  <umath.hpp>
#include <world.hpp>

namespace ucloth{
    namespace simulation{
        class PBDSimulation{
            public:
                void simulate(umath::Real const deltaTime, size_t const solverIterations, World& world);
            private:
                void applyExternalAccelerations(std::vector<umath::Vec3> const& externalAccelerations, umath::Real const deltaTime, std::vector<umath::Vec3>& velocities);
                

            private:     
                std::vector<umath::Position> m_positionEstimates;
        };
    } // namespace simulation
} // namespace ucloth

#endif // !UCLOTH_PBD_SIMULATION_H_