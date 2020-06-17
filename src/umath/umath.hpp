#ifndef UCLOTH_MATH_H_
#define UCLOTH_MATH_H_

#include <glm/glm.hpp>
#include <cmath>

namespace ucloth
{
namespace umath
{
using Real = float;
constexpr Real kEpsilon = 1e-8;
constexpr Real kDivisionByZeroGuard = 1e-8;

using Vec3 = glm::vec3;
using Position = Vec3;
using Quaternion = glm::vec4;
using Mat3x3 = glm::mat3;
struct Ray
{
    Position origin;
    Vec3 direction;
};

inline Real clamp(Real val, Real min, Real max)
{
    return glm::clamp(val, min, max);
}
inline Real dot(Vec3 a, Vec3 b)
{
    return glm::dot(a, b);
}
inline Vec3 cross(Vec3 a, Vec3 b)
{
    return glm::cross(a, b);
}
inline Mat3x3 inverse(Mat3x3 const& mat)
{
    return glm::inverse(mat);
}
inline Mat3x3 transpose(Mat3x3 const& mat)
{
    return glm::transpose(mat);
}
inline Vec3 normalize(Vec3 const& v)
{
    return glm::normalize(v);
}
inline Real length(Vec3 const& v)
{
    return glm::length(v);
}

}  // namespace umath
}  // namespace ucloth

#endif  // !UCLOTH_MATH_H_