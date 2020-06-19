#include "uclothinterface.h"

#include <world.hpp>
#include <pbdsimulation.hpp>
#include <uclothcommon.hpp>
#include <umath.hpp>

WorldHandle ucloth_createWorld(){
    auto* world = new ucloth::simulation::World();
    return reinterpret_cast<WorldHandle>(world);
}

void ucloth_deleteWorld(WorldHandle world){
    auto* worldPtr = reinterpret_cast<ucloth::simulation::World*>(world);
    delete worldPtr;
}

void ucloth_addAcceleration(WorldHandle world, UclothVector3f acceleration){
    auto* worldPtr = reinterpret_cast<ucloth::simulation::World*>(world);
    worldPtr -> accelerations.push_back({acceleration.x_, acceleration.y_, acceleration.z_});
}

PBDSystemHandle ucloth_createPBDSimulation(){
    auto* system = new ucloth::simulation::PBDSimulation();
    return reinterpret_cast<PBDSystemHandle>(system);
}

void ucloth_deletePBDSimulation(PBDSystemHandle system){
    auto* systemPtr = reinterpret_cast<ucloth::simulation::PBDSimulation*>(system);
    delete systemPtr;
}

void ucloth_simulate(PBDSystemHandle simulationHandle, WorldHandle worldHandle, int solverIterations, float deltaTime){
    auto* pbdSystemPtr = reinterpret_cast<ucloth::simulation::PBDSimulation*>(simulationHandle);
    auto* worldPtr = reinterpret_cast<ucloth::simulation::World*>(worldHandle);
    pbdSystemPtr->simulate(deltaTime, solverIterations, *worldPtr);
}

ClothHandle ucloth_addCloth(WorldHandle handle, UclothVector3f* positions, size_t posSize, int* faces, size_t facesSize, float mass, float elasticity, float damping){
    auto* worldPtr = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> posVector{pos, pos + posSize};

    ucloth::simulation::Mesh mesh;
    mesh.faces.reserve(facesSize / 3);

    mesh.begin = worldPtr -> positions.size();
    mesh.end = mesh.begin + posSize;
    mesh.kClothThickness = 0; // Para colisiones
    mesh.kVelocity = damping;
    mesh.type = ucloth::simulation::MeshType::Cloth;

    std::vector<int> facesVector{faces, faces + facesSize};
    for (int f = 0; f < facesSize; f += 3){
        ucloth::simulation::Particle const p1 = faces[f + 0];
        ucloth::simulation::Particle const p2 = faces[f + 1];
        ucloth::simulation::Particle const p3 = faces[f + 2];
        mesh.faces.emplace_back(ucloth::simulation::Face{p1, p2, p3});
    }

    ucloth::simulation::Mesh const* cloth = &worldPtr -> addCloth(posVector, mesh, mass, elasticity);

    return reinterpret_cast<ClothHandle>(cloth);
}

void ucloth_retrieveClothInfo(ClothHandle clothHandle, WorldHandle worldHandle, UclothVector3f*& positions, size_t& posSize, int*& faces, size_t& facesSize){
    auto* worldPtr = reinterpret_cast<ucloth::simulation::World*>(worldHandle);
    auto const* clothPtr = reinterpret_cast<ucloth::simulation::Mesh const*>(clothHandle);
    positions = reinterpret_cast<UclothVector3f*>(&worldPtr->positions[clothPtr->begin]);
    posSize = clothPtr->end - clothPtr->begin;
    faces = const_cast<int*>(reinterpret_cast<const int*>(clothPtr->faces.data()));
    facesSize = clothPtr->faces.size() * 3;
}

void ucloth_attachmentParticleToPosition(WorldHandle worldHandle, ClothHandle clothHandle, unsigned int index, UclothVector3f position){
    auto* worldPtr = reinterpret_cast<ucloth::simulation::World*>(worldHandle);
    auto const* clothPtr = reinterpret_cast<ucloth::simulation::Mesh const*>(clothHandle);
    ucloth::umath::Position pos = {position.x_, position.y_, position.z_};
    worldPtr->attachParticle(*clothPtr, index, pos);
}