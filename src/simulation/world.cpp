#include "world.hpp"

namespace ucloth{
    namespace simulation{
        void World::reserveForNParticles(size_t const nParticles){
            positions.reserve(nParticles);
            velocities.reserve(nParticles);
            inverseMasses.reserve(nParticles);
        }

        void World::addCloth(std::vector<umath::Position> const& pos, Mesh const& mesh, umath::Real mass/*, umath::Real elasticity, umath::Real bendingStiffness*/){
            size_t const nNewParticles = pos.size();
            size_t const currentSize = positions.size();
            reserveForNParticles(currentSize + nNewParticles);
            positions.insert(positions.end(), pos.begin(), pos.end());
            velocities.resize(currentSize + nNewParticles, umath::Vec3(0.0f, 0.0f, 0.0f));
            umath::Real const invMassParicle = static_cast<umath::Real> (currentSize + nNewParticles) / mass;
            inverseMasses.resize(currentSize + nNewParticles, invMassParicle);

            ///currentSize ==  lastest aviable
            size_t const indexShift = currentSize;
            Mesh copy = mesh;
            copy.begin += indexShift;
            copy.end += indexShift;
            for (auto& face : copy.faces){
                face[0] += indexShift;
                face[1] += indexShift;
                face[2] += indexShift;
            }
            copy.type = MeshType::Cloth;
            meshes.push_back(std::move(copy));
            // TODO: add constraints to mesh.
        }
    } //namespace simulation
} //namespace ucloth