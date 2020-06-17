#ifndef UCLOTH_PBD_SIMULATION_H_
#define UCLOTH_PBD_SIMULATION_H_

#include <constraint.hpp>
#include <uclothcommon.hpp>
#include <umath.hpp>
#include <world.hpp>


namespace ucloth{
    namespace simulation{
        class PBDSimulation{
            public:
                void simulate(umath::Real const deltaTime, size_t const solverIterations, World& world);
                
            private:
                void applyExternalAccelerations(std::vector<umath::Vec3> const& externalAccelerations, umath::Real const deltaTime, std::vector<umath::Vec3>& velocities) const;
                void dampVelocities(std::vector<Mesh> const& meshes, std::vector<umath::Position> const& positions, std::vector<umath::Real> const& inverseMasses, std::vector<umath::Vec3>& velocities) const ;
                void createPositionEstimates(std::vector<umath::Position> const& positions, std::vector<umath::Vec3> const& velocities, umath::Real  const deltaTime);
                void solveAttachments(std::vector<Attachment> const& attachments);
                void ignoreAttachmentMasses(std::vector<Attachment> const& attachments, std::vector<umath::Real>& inverseMasses) const;
                void restoreAttachmentMasses(std::vector<Attachment> const& attachments, std::vector<umath::Real>& inverseMasses) const;
                void returnResultsToWorld(umath::Real const deltaTime, std::vector<umath::Position>& positions, std::vector<umath::Vec3>& velocities) const;
                void projectDistanceConstraints(std::vector<DistanceConstraint> const& constraints, std::vector<umath::Real> const& inverseMasses, size_t const solverIterations);

            private:
                std::vector<umath::Position> m_positionEstimates;
        };
    } // namespace simulation
} // namespace ucloth

#endif // !UCLOTH_PBD_SIMULATION_H_