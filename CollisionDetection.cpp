#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "Window.h"
#include "Maths.h"
#include "Debug.h"
#include "Plane.h"
#include "Quaternion.h"

#include <cfloat>
#include <algorithm>
#include <cmath>

using namespace NCL;
using namespace NCL::Maths;

namespace
{
    inline Vector3 ClampVec3(const Vector3 &v, const Vector3 &mins, const Vector3 &maxs)
    {
        return Vector3(
            std::clamp(v.x, mins.x, maxs.x),
            std::clamp(v.y, mins.y, maxs.y),
            std::clamp(v.z, mins.z, maxs.z));
    }

    inline Vector3 SafeNormalise(const Vector3 &v)
    {
        float lenSq = Vector::LengthSquared(v);
        if (lenSq <= 1e-8f)
        {
            return Vector3(0, 1, 0);
        }
        return v / std::sqrt(lenSq);
    }

    inline void GetCapsuleSegment(const CapsuleVolume &volume, const Transform &worldTransform, Vector3 &a, Vector3 &b)
    {
        Vector3 up = worldTransform.GetOrientation() * Vector3(0, 1, 0);
        up = SafeNormalise(up);
        float halfHeight = volume.GetHalfHeight();
        Vector3 centre = worldTransform.GetPosition();
        a = centre + up * halfHeight;
        b = centre - up * halfHeight;
    }

    inline bool SolveRayBox(const Vector3 &rayPos, const Vector3 &rayDir, const Vector3 &boxPos, const Vector3 &boxHalfSize, float &outDist)
    {
        Vector3 minPoint = boxPos - boxHalfSize;
        Vector3 maxPoint = boxPos + boxHalfSize;

        float tMin = 0.0f;
        float tMax = FLT_MAX;

        for (int i = 0; i < 3; ++i)
        {
            float direction = rayDir[i];
            float origin = rayPos[i];
            if (std::abs(direction) < 1e-6f)
            {
                if (origin < minPoint[i] || origin > maxPoint[i])
                {
                    return false;
                }
                continue;
            }
            float invDir = 1.0f / direction;
            float t1 = (minPoint[i] - origin) * invDir;
            float t2 = (maxPoint[i] - origin) * invDir;
            if (t1 > t2)
            {
                std::swap(t1, t2);
            }
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            if (tMax < tMin)
            {
                return false;
            }
        }

        outDist = tMin;
        if (outDist < 0.0f)
        {
            outDist = tMax;
            if (outDist < 0.0f)
            {
                return false;
            }
        }
        return true;
    }

    inline void ClosestPointsRaySegment(const Vector3 &rayOrigin, const Vector3 &rayDir, const Vector3 &segA, const Vector3 &segB, float &rayT, float &segT, Vector3 &pointRay, Vector3 &pointSeg)
    {
        Vector3 segDir = segB - segA;
        Vector3 diff = rayOrigin - segA;
        float a = Vector::Dot(segDir, segDir);
        float e = Vector::Dot(rayDir, rayDir);
        float b = Vector::Dot(segDir, rayDir);
        float c = Vector::Dot(segDir, diff);
        float f = Vector::Dot(rayDir, diff);
        float denom = a * e - b * b;

        if (denom != 0.0f)
        {
            rayT = (b * c - a * f) / denom;
        }
        else
        {
            rayT = 0.0f;
        }

        segT = (b * rayT + c) / a;

        if (rayT < 0.0f)
        {
            rayT = 0.0f;
            segT = std::clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            segT = std::clamp(segT, 0.0f, 1.0f);
        }

        pointRay = rayOrigin + rayDir * rayT;
        pointSeg = segA + segDir * segT;
    }
}

bool CollisionDetection::RayPlaneIntersection(const Ray &r, const Plane &p, RayCollision &collisions)
{
    float ln = Vector::Dot(p.GetNormal(), r.GetDirection());

    if (ln == 0.0f)
    {
        return false; // direction vectors are perpendicular!
    }

    Vector3 planePoint = p.GetPointOnPlane();

    Vector3 pointDir = planePoint - r.GetPosition();

    float d = Vector::Dot(pointDir, p.GetNormal()) / ln;

    collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

    return true;
}

bool CollisionDetection::RayIntersection(const Ray &r, GameObject &object, RayCollision &collision)
{
    bool hasCollided = false;

    const Transform &worldTransform = object.GetTransform();
    const CollisionVolume *volume = object.GetBoundingVolume();

    if (!volume)
    {
        return false;
    }

    switch (volume->type)
    {
    case VolumeType::AABB:
        hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume &)*volume, collision);
        break;
    case VolumeType::OBB:
        hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume &)*volume, collision);
        break;
    case VolumeType::Sphere:
        hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume &)*volume, collision);
        break;

    case VolumeType::Capsule:
        hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume &)*volume, collision);
        break;
    }

    return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray &r, const Vector3 &boxPos, const Vector3 &boxSize, RayCollision &collision)
{
    Vector3 dir = Vector::Normalise(r.GetDirection());
    float distance = 0.0f;
    if (!SolveRayBox(r.GetPosition(), dir, boxPos, boxSize, distance))
    {
        return false;
    }

    collision.rayDistance = distance;
    collision.collidedAt = r.GetPosition() + dir * distance;
    return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray &r, const Transform &worldTransform, const AABBVolume &volume, RayCollision &collision)
{
    return RayBoxIntersection(r, worldTransform.GetPosition(), volume.GetHalfDimensions(), collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray &r, const Transform &worldTransform, const OBBVolume &volume, RayCollision &collision)
{
    Matrix3 orientation = Quaternion::RotationMatrix<Matrix3>(worldTransform.GetOrientation());
    Matrix3 invOrientation = Matrix::Transpose(orientation);
    Vector3 localOrigin = invOrientation * (r.GetPosition() - worldTransform.GetPosition());
    Vector3 localDir = Vector::Normalise(invOrientation * r.GetDirection());
    Ray localRay(localOrigin, localDir);
    RayCollision localCollision;
    if (!RayBoxIntersection(localRay, Vector3(), volume.GetHalfDimensions(), localCollision))
    {
        return false;
    }
    collision.rayDistance = localCollision.rayDistance;
    collision.collidedAt = worldTransform.GetPosition() + orientation * localCollision.collidedAt;
    return true;
}

bool CollisionDetection::RaySphereIntersection(const Ray &r, const Transform &worldTransform, const SphereVolume &volume, RayCollision &collision)
{
    Vector3 spherePos = worldTransform.GetPosition();
    float radius = volume.GetRadius();
    Vector3 dir = Vector::Normalise(r.GetDirection());
    Vector3 originToCentre = spherePos - r.GetPosition();
    float projection = Vector::Dot(originToCentre, dir);
    float centreDistSq = Vector::LengthSquared(originToCentre) - projection * projection;
    float radiusSq = radius * radius;
    if (centreDistSq > radiusSq)
    {
        return false;
    }
    float thc = std::sqrt(std::max(radiusSq - centreDistSq, 0.0f));
    float t0 = projection - thc;
    float t1 = projection + thc;
    float intersection = t0;
    if (intersection < 0.0f)
    {
        intersection = t1;
        if (intersection < 0.0f)
        {
            return false;
        }
    }
    collision.rayDistance = intersection;
    collision.collidedAt = r.GetPosition() + dir * intersection;
    return true;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray &r, const Transform &worldTransform, const CapsuleVolume &volume, RayCollision &collision)
{
    Vector3 segA, segB;
    GetCapsuleSegment(volume, worldTransform, segA, segB);
    Vector3 dir = Vector::Normalise(r.GetDirection());
    float rayT = 0.0f;
    float segT = 0.0f;
    Vector3 pointRay;
    Vector3 pointSeg;
    ClosestPointsRaySegment(r.GetPosition(), dir, segA, segB, rayT, segT, pointRay, pointSeg);
    Vector3 delta = pointRay - pointSeg;
    float distSq = Vector::LengthSquared(delta);
    float radius = volume.GetRadius();
    if (distSq <= radius * radius && rayT >= 0.0f)
    {
        collision.rayDistance = rayT;
        collision.collidedAt = pointRay;
        return true;
    }
    // Check end caps as fallback
    SphereVolume sphere(radius);
    RayCollision temp;
    Transform endTransform;
    endTransform.SetPosition(segA);
    if (RaySphereIntersection(r, endTransform, sphere, temp))
    {
        collision = temp;
        return true;
    }
    endTransform.SetPosition(segB);
    if (RaySphereIntersection(r, endTransform, sphere, temp))
    {
        collision = temp;
        return true;
    }
    return false;
}

bool CollisionDetection::ObjectIntersection(GameObject *a, GameObject *b, CollisionInfo &collisionInfo)
{
    const CollisionVolume *volA = a->GetBoundingVolume();
    const CollisionVolume *volB = b->GetBoundingVolume();

    if (!volA || !volB)
    {
        return false;
    }

    collisionInfo.a = a;
    collisionInfo.b = b;

    Transform &transformA = a->GetTransform();
    Transform &transformB = b->GetTransform();

    VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

    // Two AABBs
    if (pairType == VolumeType::AABB)
    {
        return AABBIntersection((AABBVolume &)*volA, transformA, (AABBVolume &)*volB, transformB, collisionInfo);
    }
    // Two Spheres
    if (pairType == VolumeType::Sphere)
    {
        return SphereIntersection((SphereVolume &)*volA, transformA, (SphereVolume &)*volB, transformB, collisionInfo);
    }
    // Two OBBs
    if (pairType == VolumeType::OBB)
    {
        return OBBIntersection((OBBVolume &)*volA, transformA, (OBBVolume &)*volB, transformB, collisionInfo);
    }
    // Two Capsules

    // AABB vs Sphere pairs
    if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere)
    {
        return AABBSphereIntersection((AABBVolume &)*volA, transformA, (SphereVolume &)*volB, transformB, collisionInfo);
    }
    if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB)
    {
        collisionInfo.a = b;
        collisionInfo.b = a;
        return AABBSphereIntersection((AABBVolume &)*volB, transformB, (SphereVolume &)*volA, transformA, collisionInfo);
    }

    // OBB vs sphere pairs
    if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere)
    {
        return OBBSphereIntersection((OBBVolume &)*volA, transformA, (SphereVolume &)*volB, transformB, collisionInfo);
    }
    if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB)
    {
        collisionInfo.a = b;
        collisionInfo.b = a;
        return OBBSphereIntersection((OBBVolume &)*volB, transformB, (SphereVolume &)*volA, transformA, collisionInfo);
    }

    // Capsule vs other interactions
    if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere)
    {
        return SphereCapsuleIntersection((CapsuleVolume &)*volA, transformA, (SphereVolume &)*volB, transformB, collisionInfo);
    }
    if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule)
    {
        collisionInfo.a = b;
        collisionInfo.b = a;
        return SphereCapsuleIntersection((CapsuleVolume &)*volB, transformB, (SphereVolume &)*volA, transformA, collisionInfo);
    }

    if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB)
    {
        return AABBCapsuleIntersection((CapsuleVolume &)*volA, transformA, (AABBVolume &)*volB, transformB, collisionInfo);
    }
    if (volB->type == VolumeType::Capsule && volA->type == VolumeType::AABB)
    {
        collisionInfo.a = b;
        collisionInfo.b = a;
        return AABBCapsuleIntersection((CapsuleVolume &)*volB, transformB, (AABBVolume &)*volA, transformA, collisionInfo);
    }

    return false;
}

bool CollisionDetection::AABBTest(const Vector3 &posA, const Vector3 &posB, const Vector3 &halfSizeA, const Vector3 &halfSizeB)
{
    Vector3 delta = posB - posA;
    Vector3 totalSize = halfSizeA + halfSizeB;

    if (abs(delta.x) < totalSize.x &&
        abs(delta.y) < totalSize.y &&
        abs(delta.z) < totalSize.z)
    {
        return true;
    }
    return false;
}

// AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume &volumeA, const Transform &worldTransformA,
                                          const AABBVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Vector3 posA = worldTransformA.GetPosition();
    Vector3 posB = worldTransformB.GetPosition();
    Vector3 halfA = volumeA.GetHalfDimensions();
    Vector3 halfB = volumeB.GetHalfDimensions();

    if (!AABBTest(posA, posB, halfA, halfB))
    {
        return false;
    }

    Vector3 delta = posB - posA;
    Vector3 penetration = (halfA + halfB) - Vector3(std::abs(delta.x), std::abs(delta.y), std::abs(delta.z));
    float minDepth = penetration.x;
    Vector3 axis = Vector3(delta.x < 0 ? -1.0f : 1.0f, 0, 0);
    if (penetration.y < minDepth)
    {
        minDepth = penetration.y;
        axis = Vector3(0, delta.y < 0 ? -1.0f : 1.0f, 0);
    }
    if (penetration.z < minDepth)
    {
        minDepth = penetration.z;
        axis = Vector3(0, 0, delta.z < 0 ? -1.0f : 1.0f);
    }

    Vector3 pointOnA = posA + axis * halfA;
    Vector3 pointOnB = posB - axis * halfB;
    collisionInfo.AddContactPoint(pointOnA - posA, pointOnB - posB, axis, minDepth);
    return true;
}

// Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume &volumeA, const Transform &worldTransformA,
                                            const SphereVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Vector3 posA = worldTransformA.GetPosition();
    Vector3 posB = worldTransformB.GetPosition();
    Vector3 delta = posB - posA;
    float radiusA = volumeA.GetRadius();
    float radiusB = volumeB.GetRadius();
    float distSq = Vector::LengthSquared(delta);
    float totalRadius = radiusA + radiusB;
    if (distSq > totalRadius * totalRadius)
    {
        return false;
    }
    float dist = std::sqrt(std::max(distSq, 0.0001f));
    Vector3 normal = delta / dist;
    float penetration = totalRadius - dist;
    Vector3 pointOnA = posA + normal * radiusA;
    Vector3 pointOnB = posB - normal * radiusB;
    collisionInfo.AddContactPoint(pointOnA - posA, pointOnB - posB, normal, penetration);
    return true;
}

// AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume &volumeA, const Transform &worldTransformA,
                                                const SphereVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Vector3 boxPos = worldTransformA.GetPosition();
    Vector3 halfA = volumeA.GetHalfDimensions();
    Vector3 spherePos = worldTransformB.GetPosition();
    Vector3 closestPoint = ClampVec3(spherePos, boxPos - halfA, boxPos + halfA);
    Vector3 delta = spherePos - closestPoint;
    float distSq = Vector::LengthSquared(delta);
    float radius = volumeB.GetRadius();
    if (distSq > radius * radius)
    {
        return false;
    }
    float dist = std::sqrt(std::max(distSq, 0.0001f));
    Vector3 normal = (dist > 0.0f) ? delta / dist : Vector3(0, 1, 0);
    float penetration = radius - dist;
    Vector3 pointOnBox = closestPoint;
    Vector3 pointOnSphere = spherePos - normal * radius;
    collisionInfo.AddContactPoint(pointOnBox - boxPos, pointOnSphere - spherePos, normal, penetration);
    return true;
}

bool CollisionDetection::OBBSphereIntersection(const OBBVolume &volumeA, const Transform &worldTransformA,
                                               const SphereVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Matrix3 orientation = Quaternion::RotationMatrix<Matrix3>(worldTransformA.GetOrientation());
    Matrix3 invOrientation = Matrix::Transpose(orientation);
    Vector3 localSphere = invOrientation * (worldTransformB.GetPosition() - worldTransformA.GetPosition());
    Vector3 halfSize = volumeA.GetHalfDimensions();
    Vector3 closestPoint = ClampVec3(localSphere, -halfSize, halfSize);
    Vector3 delta = localSphere - closestPoint;
    float distSq = Vector::LengthSquared(delta);
    float radius = volumeB.GetRadius();
    if (distSq > radius * radius)
    {
        return false;
    }
    float dist = std::sqrt(std::max(distSq, 0.0001f));
    Vector3 normalLocal = (dist > 0.0f) ? delta / dist : Vector3(0, 1, 0);
    Vector3 normalWorld = orientation * normalLocal;
    Vector3 pointOnBox = worldTransformA.GetPosition() + orientation * closestPoint;
    Vector3 pointOnSphere = worldTransformB.GetPosition() - normalWorld * radius;
    float penetration = radius - dist;
    collisionInfo.AddContactPoint(pointOnBox - worldTransformA.GetPosition(), pointOnSphere - worldTransformB.GetPosition(), normalWorld, penetration);
    return true;
}

bool CollisionDetection::AABBCapsuleIntersection(
    const CapsuleVolume &volumeA, const Transform &worldTransformA,
    const AABBVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Vector3 segA, segB;
    GetCapsuleSegment(volumeA, worldTransformA, segA, segB);
    Vector3 boxPos = worldTransformB.GetPosition();
    Vector3 halfSize = volumeB.GetHalfDimensions();
    Vector3 bestBoxPoint = segA;
    Vector3 bestSegPoint = segA;
    float bestDist = FLT_MAX;
    const int samples = 10;
    for (int i = 0; i <= samples; ++i)
    {
        float t = i / (float)samples;
        Vector3 segPoint = segA + (segB - segA) * t;
        Vector3 clamped = ClampVec3(segPoint, boxPos - halfSize, boxPos + halfSize);
        float distSq = Vector::LengthSquared(segPoint - clamped);
        if (distSq < bestDist)
        {
            bestDist = distSq;
            bestBoxPoint = clamped;
            bestSegPoint = segPoint;
        }
    }
    float radius = volumeA.GetRadius();
    if (bestDist > radius * radius)
    {
        return false;
    }
    float dist = std::sqrt(std::max(bestDist, 0.0001f));
    Vector3 normal = SafeNormalise(bestBoxPoint - bestSegPoint);
    float penetration = radius - dist;
    Vector3 pointOnCapsule = bestSegPoint + normal * radius;
    collisionInfo.AddContactPoint(pointOnCapsule - worldTransformA.GetPosition(), bestBoxPoint - worldTransformB.GetPosition(), normal, penetration);
    return true;
}

bool CollisionDetection::SphereCapsuleIntersection(
    const CapsuleVolume &volumeA, const Transform &worldTransformA,
    const SphereVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Vector3 segA, segB;
    GetCapsuleSegment(volumeA, worldTransformA, segA, segB);
    Vector3 spherePos = worldTransformB.GetPosition();
    float radiusCapsule = volumeA.GetRadius();
    float radiusSphere = volumeB.GetRadius();
    Vector3 segDir = segB - segA;
    float segLengthSq = Vector::LengthSquared(segDir);
    float t = 0.0f;
    if (segLengthSq > 0.0f)
    {
        t = Vector::Dot(spherePos - segA, segDir) / segLengthSq;
        t = std::clamp(t, 0.0f, 1.0f);
    }
    Vector3 closest = segA + segDir * t;
    Vector3 delta = spherePos - closest;
    float distSq = Vector::LengthSquared(delta);
    float totalRadius = radiusCapsule + radiusSphere;
    if (distSq > totalRadius * totalRadius)
    {
        return false;
    }
    float dist = std::sqrt(std::max(distSq, 0.0001f));
    Vector3 normal = (dist > 0.0f) ? delta / dist : Vector3(0, 1, 0);
    float penetration = totalRadius - dist;
    Vector3 pointOnCapsule = closest + normal * radiusCapsule;
    Vector3 pointOnSphere = spherePos - normal * radiusSphere;
    collisionInfo.AddContactPoint(pointOnCapsule - worldTransformA.GetPosition(), pointOnSphere - worldTransformB.GetPosition(), normal, penetration);
    return true;
}

bool CollisionDetection::OBBIntersection(const OBBVolume &volumeA, const Transform &worldTransformA,
                                         const OBBVolume &volumeB, const Transform &worldTransformB, CollisionInfo &collisionInfo)
{
    Matrix3 orientationA = Quaternion::RotationMatrix<Matrix3>(worldTransformA.GetOrientation());
    Matrix3 orientationB = Quaternion::RotationMatrix<Matrix3>(worldTransformB.GetOrientation());
    Vector3 axesA[3] = {
        orientationA.GetColumn(0),
        orientationA.GetColumn(1),
        orientationA.GetColumn(2)};
    Vector3 axesB[3] = {
        orientationB.GetColumn(0),
        orientationB.GetColumn(1),
        orientationB.GetColumn(2)};
    float R[3][3];
    float AbsR[3][3];
    const float EPSILON = 1e-5f;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R[i][j] = Vector::Dot(axesA[i], axesB[j]);
            AbsR[i][j] = std::abs(R[i][j]) + EPSILON;
        }
    }
    Vector3 translation = worldTransformB.GetPosition() - worldTransformA.GetPosition();
    Vector3 t = Vector3(Vector::Dot(translation, axesA[0]), Vector::Dot(translation, axesA[1]), Vector::Dot(translation, axesA[2]));
    Vector3 halfA = volumeA.GetHalfDimensions();
    Vector3 halfB = volumeB.GetHalfDimensions();
    for (int i = 0; i < 3; ++i)
    {
        float ra = halfA[i];
        float rb = halfB.x * AbsR[i][0] + halfB.y * AbsR[i][1] + halfB.z * AbsR[i][2];
        if (std::abs(t[i]) > ra + rb)
        {
            return false;
        }
    }
    for (int j = 0; j < 3; ++j)
    {
        float ra = halfA.x * AbsR[0][j] + halfA.y * AbsR[1][j] + halfA.z * AbsR[2][j];
        float rb = halfB[j];
        if (std::abs(Vector::Dot(translation, axesB[j])) > ra + rb)
        {
            return false;
        }
    }
    float minPenetration = FLT_MAX;
    Vector3 bestAxis = axesA[0];
    for (int i = 0; i < 3; ++i)
    {
        float ra = halfA[i];
        float rb = halfB.x * AbsR[i][0] + halfB.y * AbsR[i][1] + halfB.z * AbsR[i][2];
        float depth = ra + rb - std::abs(t[i]);
        if (depth < minPenetration)
        {
            minPenetration = depth;
            bestAxis = axesA[i] * (t[i] < 0 ? -1.0f : 1.0f);
        }
    }
    for (int j = 0; j < 3; ++j)
    {
        float ra = halfA.x * AbsR[0][j] + halfA.y * AbsR[1][j] + halfA.z * AbsR[2][j];
        float rb = halfB[j];
        float depth = ra + rb - std::abs(Vector::Dot(translation, axesB[j]));
        if (depth < minPenetration)
        {
            minPenetration = depth;
            bestAxis = axesB[j] * (Vector::Dot(translation, axesB[j]) < 0 ? -1.0f : 1.0f);
        }
    }
    Vector3 contactPoint = worldTransformA.GetPosition() + bestAxis * minPenetration * 0.5f;
    collisionInfo.AddContactPoint(contactPoint - worldTransformA.GetPosition(), contactPoint - worldTransformB.GetPosition(), bestAxis, minPenetration);
    return true;
}

Matrix4 GenerateInverseView(const Camera &c)
{
    float pitch = c.GetPitch();
    float yaw = c.GetYaw();
    Vector3 position = c.GetPosition();

    Matrix4 iview =
        Matrix::Translation(position) *
        Matrix::Rotation(-yaw, Vector3(0, -1, 0)) *
        Matrix::Rotation(-pitch, Vector3(-1, 0, 0));

    return iview;
}

Matrix4 GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane)
{
    float negDepth = nearPlane - farPlane;

    float invNegDepth = negDepth / (2 * (farPlane * nearPlane));

    Matrix4 m;

    float h = 1.0f / tan(fov * PI_OVER_360);

    m.array[0][0] = aspect / h;
    m.array[1][1] = tan(fov * PI_OVER_360);
    m.array[2][2] = 0.0f;

    m.array[2][3] = invNegDepth; //// +PI_OVER_360;
    m.array[3][2] = -1.0f;
    m.array[3][3] = (0.5f / nearPlane) + (0.5f / farPlane);

    return m;
}

Vector3 CollisionDetection::Unproject(const Vector3 &screenPos, const PerspectiveCamera &cam)
{
    Vector2i screenSize = Window::GetWindow()->GetScreenSize();

    float aspect = Window::GetWindow()->GetScreenAspect();
    float fov = cam.GetFieldOfVision();
    float nearPlane = cam.GetNearPlane();
    float farPlane = cam.GetFarPlane();

    // Create our inverted matrix! Note how that to get a correct inverse matrix,
    // the order of matrices used to form it are inverted, too.
    Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

    Matrix4 proj = cam.BuildProjectionMatrix(aspect);

    // Our mouse position x and y values are in 0 to screen dimensions range,
    // so we need to turn them into the -1 to 1 axis range of clip space.
    // We can do that by dividing the mouse values by the width and height of the
    // screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
    // and then subtracting 1 (-1.0 to 1.0).
    Vector4 clipSpace = Vector4(
        (screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
        (screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
        (screenPos.z),
        1.0f);

    // Then, we multiply our clipspace coordinate by our inverted matrix
    Vector4 transformed = invVP * clipSpace;

    // our transformed w coordinate is now the 'inverse' perspective divide, so
    // we can reconstruct the final world space by dividing x,y,and z by w.
    return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const PerspectiveCamera &cam)
{
    Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
    Vector2i screenSize = Window::GetWindow()->GetScreenSize();

    // We remove the y axis mouse position from height as OpenGL is 'upside down',
    // and thinks the bottom left is the origin, instead of the top left!
    Vector3 nearPos = Vector3(screenMouse.x,
                              screenSize.y - screenMouse.y,
                              -0.99999f);

    // We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
    // causes the unproject function to go a bit weird.
    Vector3 farPos = Vector3(screenMouse.x,
                             screenSize.y - screenMouse.y,
                             0.99999f);

    Vector3 a = Unproject(nearPos, cam);
    Vector3 b = Unproject(farPos, cam);
    Vector3 c = b - a;

    c = Vector::Normalise(c);

    return Ray(cam.GetPosition(), c);
}

// http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane)
{
    Matrix4 m;

    float t = tan(fov * PI_OVER_360);

    float neg_depth = nearPlane - farPlane;

    const float h = 1.0f / t;

    float c = (farPlane + nearPlane) / neg_depth;
    float e = -1.0f;
    float d = 2.0f * (nearPlane * farPlane) / neg_depth;

    m.array[0][0] = aspect / h;
    m.array[1][1] = tan(fov * PI_OVER_360);
    m.array[2][2] = 0.0f;

    m.array[2][3] = 1.0f / d;

    m.array[3][2] = 1.0f / e;
    m.array[3][3] = -c / (d * e);

    return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c)
{
    float pitch = c.GetPitch();
    float yaw = c.GetYaw();
    Vector3 position = c.GetPosition();

    Matrix4 iview =
        Matrix::Translation(position) *
        Matrix::Rotation(yaw, Vector3(0, 1, 0)) *
        Matrix::Rotation(pitch, Vector3(1, 0, 0));

    return iview;
}

/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3 CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const PerspectiveCamera &c)
{
    // Create our inverted matrix! Note how that to get a correct inverse matrix,
    // the order of matrices used to form it are inverted, too.
    Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());

    Vector2i screenSize = Window::GetWindow()->GetScreenSize();

    // Our mouse position x and y values are in 0 to screen dimensions range,
    // so we need to turn them into the -1 to 1 axis range of clip space.
    // We can do that by dividing the mouse values by the width and height of the
    // screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
    // and then subtracting 1 (-1.0 to 1.0).
    Vector4 clipSpace = Vector4(
        (position.x / (float)screenSize.x) * 2.0f - 1.0f,
        (position.y / (float)screenSize.y) * 2.0f - 1.0f,
        (position.z) - 1.0f,
        1.0f);

    // Then, we multiply our clipspace coordinate by our inverted matrix
    Vector4 transformed = invVP * clipSpace;

    // our transformed w coordinate is now the 'inverse' perspective divide, so
    // we can reconstruct the final world space by dividing x,y,and z by w.
    return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}
