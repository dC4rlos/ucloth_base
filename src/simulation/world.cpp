#include "world.hpp"

namespace ucloth{
    namespace simulation{
        void World::reserveForNParticles(size_t const nParticles){
            positions.reserve(nParticles);
            velocities.reserve(nParticles);
            inverseMasses.reserve(nParticles);
        }

        Mesh const& World::addCloth(std::vector<umath::Position> const& pos, Mesh const& mesh, umath::Real const mass, umath::Real const elasticity/*, umath::Real bendingStiffness*/){
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
            auto const& emplaceMesh = meshes.emplace_back(std::move(copy));
            

            // TODO: add constraints to mesh.
            //Distance constraint
            std::vector<Edge> edges;
            edges.reserve((mesh.end - mesh.begin) * 3);
            for(auto const& face : mesh.faces){
                 Edge const e0 = {face[0], face[1]};
                 Edge const e0i = {face[1], face[0]};
                 Edge const e1 = {face[1], face[2]};
                 Edge const e1i = {face[2], face[1]};
                 Edge const e2 = {face[2], face[0]};
                 Edge const e2i = {face[0], face[2]};

                auto e0_it = std::find_if(edges.begin(), edges.end(), [&e0, &e0i](Edge const& e) -> bool { return e == e0 || e == e0i; });
                if (e0_it == edges.end()){
                    edges.push_back(e0);
                }

                auto e1_it = std::find_if(edges.begin(), edges.end(), [&e1, &e1i](Edge const& e) -> bool { return e == e1 || e == e1i; });
                if (e1_it == edges.end()){
                    edges.push_back(e1);
                }

                auto e2_it = std::find_if(edges.begin(), edges.end(), [&e2, &e2i](Edge const& e) -> bool { return e == e2 || e == e2i; });
                if (e2_it == edges.end()){
                    edges.push_back(e2);
                }
            }
            for (auto const& edge : edges){
                umath::Real const distance = umath::length(pos[edge.first] - pos[edge.second]);
                distanceConstraints.push_back(DistanceConstraint{edge.first, edge.second, distance, elasticity});
            }
            
            //Bending


            //return
            return emplaceMesh;
        }

        void World::attachParticle(Mesh const& mesh, Particle particle, umath::Position const& position){
            Particle pInWorld = mesh.begin + particle;
            attachments.push_back(std::move(Attachment{pInWorld, inverseMasses[pInWorld], position}));
        }
    } //namespace simulation
} //namespace ucloth