#ifndef UCLOTH_UCLOTH_COMMON_H_
#define UCLOTH_UCLOTH_COMMON_H_

#include <array>
#include <cstdint>
#include <umath/umath.hpp>
#include <utility>
#include <vector>

namespace ucloth{
    namespace simulation{
        // 
        using Particle = int;
        using Face = std::array<Particle, 3>;
        using Edge = std::pair<Particle, Particle>;

        enum class MeshType : uint32_t{
            StaticMesh,
            RigidBody,
            Cloth
        };

        struct Mesh{
            std::vector<Face> faces;
            Particle begin; //< defines the range in the world vectors.
            Particle end;
            umath::Real kVelocity;
            umath::Real kClothThickness;
            MeshType type;
        };
    }
}

#endif // !UCLOTH_UCLOTH_COMMON_H_
