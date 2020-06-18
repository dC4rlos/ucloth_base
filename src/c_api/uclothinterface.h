#ifndef UCLOTH_DLL_INTERFACE_H_
#define UCLOTH_DLL_INTERFACE_H_

#include "uclothexport.h"
#include "uclothhandles.h"
#include "uclothstructures.h"

extern "C"
{
    ucloth_export WorldHandle ucloth_createWorld(void);
    ucloth_export void ucloth_deleteWorld(WorldHandle world);
    ucloth_export void ucloth_addAcceleration(WorldHandle handle, UclothVector3f acceleration);

    ucloth_export PBDSystemHandle ucloth_createPBDSimulation(void);
    ucloth_export void ucloth_deletePBDSimulation(PBDSystemHandle pbdSimulation);
    ucloth_export void ucloth_simulate(PBDSystemHandle simulationHandle, WorldHandle worldHandle, int solverIterations, float deltaTime);

    ucloth_export ClothHandle ucloth_addCloth(WorldHandle handle, UclothVector3f* positions, size_t posSize, int* faces, size_t facesSize, float mass, float elasticity, float damping);

    ucloth_export void ucloth_retrieveClothInfo(ClothHandle clothHandle, WorldHandle worldHandle, UclothVector3f*& positions, size_t& posSize, int*& faces, size_t& facesSize);

    ucloth_export void ucloth_attachmentParticleToPosition(WorldHandle worldHandle, ClothHandle clothHandle, unsigned int index, UclothVector3f position);
}


#endif // !UCLOTH_DLL_INTERFACE_H_