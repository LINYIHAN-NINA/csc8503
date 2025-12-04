#include "PhysicsSystem.h"
#include "PhysicsObject.h"
#include "GameObject.h"
#include "CollisionDetection.h"
#include "Quaternion.h"

#include "Constraint.h"

#include "Debug.h"
#include "Window.h"
#include <functional>
#include <algorithm>
#include <cmath>
using namespace NCL;
using namespace CSC8503;

PhysicsSystem::PhysicsSystem(GameWorld &g) : gameWorld(g)
{
    applyGravity = false;
    useBroadPhase = false;
    dTOffset = 0.0f;
    globalDamping = 0.995f;
    SetGravity(Vector3(0.0f, -9.8f, 0.0f));
}

PhysicsSystem::~PhysicsSystem()
{
}

void PhysicsSystem::SetGravity(const Vector3 &g)
{
    gravity = g;
}

/*

If the 'game' is ever reset, the PhysicsSystem must be
'cleared' to remove any old collisions that might still
be hanging around in the collision list. If your engine
is expanded to allow objects to be removed from the world,
you'll need to iterate through this collisions list to remove
any collisions they are in.

*/
void PhysicsSystem::Clear()
{
    allCollisions.clear();
}

/*

This is the core of the physics engine update

*/

bool useSimpleContainer = false;

int constraintIterationCount = 10;

// This is the fixed timestep we'd LIKE to have
const int idealHZ = 120;
const float idealDT = 1.0f / idealHZ;

/*
This is the fixed update we actually have...
If physics takes too long it starts to kill the framerate, it'll drop the
iteration count down until the FPS stabilises, even if that ends up
being at a low rate.
*/
int realHZ = idealHZ;
float realDT = idealDT;

void PhysicsSystem::Update(float dt)
{
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::B))
    {
        useBroadPhase = !useBroadPhase;
        std::cout << "Setting broadphase to " << useBroadPhase << std::endl;
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::N))
    {
        useSimpleContainer = !useSimpleContainer;
        std::cout << "Setting broad container to " << useSimpleContainer << std::endl;
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::I))
    {
        constraintIterationCount--;
        std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::O))
    {
        constraintIterationCount++;
        std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
    }

    dTOffset += dt; // We accumulate time delta here - there might be remainders from previous frame!

    GameTimer t;
    t.GetTimeDeltaSeconds();

    if (useBroadPhase)
    {
        UpdateObjectAABBs();
    }
    int iteratorCount = 0;
    while (dTOffset > realDT)
    {
        IntegrateAccel(realDT); // Update accelerations from external forces
        if (useBroadPhase)
        {
            BroadPhase();
            NarrowPhase();
        }
        else
        {
            BasicCollisionDetection();
        }

        // This is our simple iterative solver -
        // we just run things multiple times, slowly moving things forward
        // and then rechecking that the constraints have been met
        float constraintDt = realDT / (float)constraintIterationCount;
        for (int i = 0; i < constraintIterationCount; ++i)
        {
            UpdateConstraints(constraintDt);
        }
        IntegrateVelocity(realDT); // update positions from new velocity changes

        dTOffset -= realDT;
        iteratorCount++;
    }

    ClearForces(); // Once we've finished with the forces, reset them to zero

    UpdateCollisionList(); // Remove any old collisions

    t.Tick();
    float updateTime = t.GetTimeDeltaSeconds();

    // Uh oh, physics is taking too long...
    if (updateTime > realDT)
    {
        realHZ /= 2;
        realDT *= 2;
        std::cout << "Dropping iteration count due to long physics time...(now " << realHZ << ")\n";
    }
    else if (dt * 2 < realDT)
    { // we have plenty of room to increase iteration count!
        int temp = realHZ;
        realHZ *= 2;
        realDT /= 2;

        if (realHZ > idealHZ)
        {
            realHZ = idealHZ;
            realDT = idealDT;
        }
        if (temp != realHZ)
        {
            std::cout << "Raising iteration count due to short physics time...(now " << realHZ << ")\n";
        }
    }
}

/*
Later on we're going to need to keep track of collisions
across multiple frames, so we store them in a set.

The first time they are added, we tell the objects they are colliding.
The frame they are to be removed, we tell them they're no longer colliding.

From this simple mechanism, we we build up gameplay interactions inside the
OnCollisionBegin / OnCollisionEnd functions (removing health when hit by a
rocket launcher, gaining a point when the player hits the gold coin, and so on).
*/
void PhysicsSystem::UpdateCollisionList()
{
    for (std::set<CollisionDetection::CollisionInfo>::iterator i = allCollisions.begin(); i != allCollisions.end();)
    {
        if ((*i).framesLeft == numCollisionFrames)
        {
            i->a->OnCollisionBegin(i->b);
            i->b->OnCollisionBegin(i->a);
        }

        CollisionDetection::CollisionInfo &in = const_cast<CollisionDetection::CollisionInfo &>(*i);
        in.framesLeft--;

        if ((*i).framesLeft < 0)
        {
            i->a->OnCollisionEnd(i->b);
            i->b->OnCollisionEnd(i->a);
            i = allCollisions.erase(i);
        }
        else
        {
            ++i;
        }
    }
}

void PhysicsSystem::UpdateObjectAABBs()
{
    gameWorld.OperateOnContents(
        [](GameObject *g)
        {
            g->UpdateBroadphaseAABB();
        });
}

/*

This is how we'll be doing collision detection in tutorial 4.
We step thorugh every pair of objects once (the inner for loop offset
ensures this), and determine whether they collide, and if so, add them
to the collision set for later processing. The set will guarantee that
a particular pair will only be added once, so objects colliding for
multiple frames won't flood the set with duplicates.
*/
void PhysicsSystem::BasicCollisionDetection()
{
    std::vector<GameObject *>::const_iterator first;
    std::vector<GameObject *>::const_iterator last;
    gameWorld.GetObjectIterators(first, last);

    for (auto i = first; i != last; ++i)
    {
        for (auto j = i + 1; j != last; ++j)
        {
            GameObject *a = *i;
            GameObject *b = *j;
            if (!a->GetBoundingVolume() || !b->GetBoundingVolume())
            {
                continue;
            }
            if (!a->IsCollisionEnabled() || !b->IsCollisionEnabled())
            {
                continue;
            }

            CollisionDetection::CollisionInfo info;
            if (CollisionDetection::ObjectIntersection(a, b, info))
            {
                info.framesLeft = numCollisionFrames;
                auto insertResult = allCollisions.insert(info);
                if (!insertResult.second)
                {
                    CollisionDetection::CollisionInfo &existing = const_cast<CollisionDetection::CollisionInfo &>(*insertResult.first);
                    existing = info;
                }
                ImpulseResolveCollision(*a, *b, info.point);
            }
        }
    }
}

/*

In tutorial 5, we start determining the correct response to a collision,
so that objects separate back out.

*/
void PhysicsSystem::ImpulseResolveCollision(GameObject &a, GameObject &b, CollisionDetection::ContactPoint &p) const
{
    PhysicsObject *physA = a.GetPhysicsObject();
    PhysicsObject *physB = b.GetPhysicsObject();
    if (!physA || !physB)
    {
        return;
    }
    float invMassA = physA->GetInverseMass();
    float invMassB = physB->GetInverseMass();
    if (invMassA + invMassB <= 0.0f)
    {
        return;
    }

    Vector3 relativeA = p.localA;
    Vector3 relativeB = p.localB;
    Vector3 velocityA = physA->GetLinearVelocity();
    Vector3 velocityB = physB->GetLinearVelocity();
    Vector3 angVelA = physA->GetAngularVelocity();
    Vector3 angVelB = physB->GetAngularVelocity();

    Vector3 fullVelA = velocityA + Vector::Cross(angVelA, relativeA);
    Vector3 fullVelB = velocityB + Vector::Cross(angVelB, relativeB);
    Vector3 relativeVelocity = fullVelB - fullVelA;
    float contactVelocity = Vector::Dot(relativeVelocity, p.normal);
    if (contactVelocity > 0.0f)
    {
        return;
    }

    float restitution = 0.5f;
    Vector3 angularA = Vector::Cross(physA->GetInertiaTensor() * Vector::Cross(relativeA, p.normal), relativeA);
    Vector3 angularB = Vector::Cross(physB->GetInertiaTensor() * Vector::Cross(relativeB, p.normal), relativeB);
    float angularEffect = Vector::Dot(angularA + angularB, p.normal);
    float invMassSum = invMassA + invMassB + angularEffect;
    float impulseMagnitude = -(1.0f + restitution) * contactVelocity / invMassSum;
    Vector3 impulse = p.normal * impulseMagnitude;

    physA->ApplyLinearImpulse(-impulse);
    physB->ApplyLinearImpulse(impulse);
    physA->ApplyAngularImpulse(Vector::Cross(relativeA, -impulse));
    physB->ApplyAngularImpulse(Vector::Cross(relativeB, impulse));

    Vector3 tangent = relativeVelocity - p.normal * contactVelocity;
    float tangentialLenSq = Vector::LengthSquared(tangent);
    if (tangentialLenSq > 1e-6f)
    {
        tangent = tangent / std::sqrt(tangentialLenSq);
        float frictionCoeff = 0.5f;
        float tangentialVel = Vector::Dot(relativeVelocity, tangent);
        float frictionImpulseMag = -tangentialVel / invMassSum;
        frictionImpulseMag = std::clamp(frictionImpulseMag, -impulseMagnitude * frictionCoeff, impulseMagnitude * frictionCoeff);
        Vector3 frictionImpulse = tangent * frictionImpulseMag;
        physA->ApplyLinearImpulse(-frictionImpulse);
        physB->ApplyLinearImpulse(frictionImpulse);
        physA->ApplyAngularImpulse(Vector::Cross(relativeA, -frictionImpulse));
        physB->ApplyAngularImpulse(Vector::Cross(relativeB, frictionImpulse));
    }

    float penetration = std::max(p.penetration - 0.01f, 0.0f);
    if (penetration > 0.0f)
    {
        float totalInvMass = invMassA + invMassB;
        Vector3 correction = p.normal * (penetration / std::max(totalInvMass, 1e-6f));
        if (invMassA > 0.0f)
        {
            Transform &tA = a.GetTransform();
            tA.SetPosition(tA.GetPosition() - correction * invMassA);
        }
        if (invMassB > 0.0f)
        {
            Transform &tB = b.GetTransform();
            tB.SetPosition(tB.GetPosition() + correction * invMassB);
        }
    }
}

/*

Later, we replace the BasicCollisionDetection method with a broadphase
and a narrowphase collision detection method. In the broad phase, we
split the world up using an acceleration structure, so that we can only
compare the collisions that we absolutely need to.

*/
void PhysicsSystem::BroadPhase()
{
    broadphaseCollisions.clear();
    broadphaseCollisionsVec.clear();

    std::vector<GameObject *>::const_iterator first;
    std::vector<GameObject *>::const_iterator last;
    gameWorld.GetObjectIterators(first, last);

    for (auto i = first; i != last; ++i)
    {
        Vector3 halfSizeA;
        if (!(*i)->IsCollisionEnabled() || !(*i)->GetBroadphaseAABB(halfSizeA))
        {
            continue;
        }
        for (auto j = i + 1; j != last; ++j)
        {
            Vector3 halfSizeB;
            if (!(*j)->IsCollisionEnabled() || !(*j)->GetBroadphaseAABB(halfSizeB))
            {
                continue;
            }
            Vector3 posA = (*i)->GetTransform().GetPosition();
            Vector3 posB = (*j)->GetTransform().GetPosition();
            if (CollisionDetection::AABBTest(posA, posB, halfSizeA, halfSizeB))
            {
                CollisionDetection::CollisionInfo info;
                info.a = *i;
                info.b = *j;
                info.framesLeft = numCollisionFrames;
                broadphaseCollisions.insert(info);
                broadphaseCollisionsVec.emplace_back(info);
            }
        }
    }
}

/*

The broadphase will now only give us likely collisions, so we can now go through them,
and work out if they are truly colliding, and if so, add them into the main collision list
*/
void PhysicsSystem::NarrowPhase()
{
    for (auto &info : broadphaseCollisionsVec)
    {
        CollisionDetection::CollisionInfo collisionInfo;
        collisionInfo.a = info.a;
        collisionInfo.b = info.b;
        if (CollisionDetection::ObjectIntersection(info.a, info.b, collisionInfo))
        {
            collisionInfo.framesLeft = numCollisionFrames;
            auto insertResult = allCollisions.insert(collisionInfo);
            if (!insertResult.second)
            {
                CollisionDetection::CollisionInfo &existing = const_cast<CollisionDetection::CollisionInfo &>(*insertResult.first);
                existing = collisionInfo;
            }
            ImpulseResolveCollision(*collisionInfo.a, *collisionInfo.b, collisionInfo.point);
        }
    }
}

/*
Integration of acceleration and velocity is split up, so that we can
move objects multiple times during the course of a PhysicsUpdate,
without worrying about repeated forces accumulating etc.

This function will update both linear and angular acceleration,
based on any forces that have been accumulated in the objects during
the course of the previous game frame.
*/
void PhysicsSystem::IntegrateAccel(float dt)
{
    gameWorld.OperateOnContents(
        [this, dt](GameObject *g)
        {
            PhysicsObject *phys = g->GetPhysicsObject();
            if (!phys)
            {
                return;
            }
            float invMass = phys->GetInverseMass();
            if (invMass <= 0.0f)
            {
                return;
            }
            Vector3 accel = phys->GetForce() * invMass;
            if (applyGravity)
            {
                accel += gravity;
            }
            Vector3 velocity = phys->GetLinearVelocity() + accel * dt;
            phys->SetLinearVelocity(velocity);
            Vector3 angularAccel = phys->GetInertiaTensor() * phys->GetTorque();
            Vector3 angularVelocity = phys->GetAngularVelocity() + angularAccel * dt;
            phys->SetAngularVelocity(angularVelocity);
        });
}

/*
This function integrates linear and angular velocity into
position and orientation. It may be called multiple times
throughout a physics update, to slowly move the objects through
the world, looking for collisions.
*/
void PhysicsSystem::IntegrateVelocity(float dt)
{
    gameWorld.OperateOnContents(
        [this, dt](GameObject *g)
        {
            PhysicsObject *phys = g->GetPhysicsObject();
            if (!phys)
            {
                return;
            }
            if (phys->GetInverseMass() <= 0.0f)
            {
                return;
            }
            Transform &transform = g->GetTransform();
            transform.SetPosition(transform.GetPosition() + phys->GetLinearVelocity() * dt);
            Vector3 angVel = phys->GetAngularVelocity();
            float angSpeedSq = Vector::LengthSquared(angVel);
            if (angSpeedSq > 1e-6f)
            {
                Quaternion orientation = transform.GetOrientation();
                Quaternion spin(angVel.x * dt * 0.5f, angVel.y * dt * 0.5f, angVel.z * dt * 0.5f, 0);
                Quaternion newOrientation = orientation + spin * orientation;
                newOrientation.Normalise();
                transform.SetOrientation(newOrientation);
            }
            phys->SetLinearVelocity(phys->GetLinearVelocity() * globalDamping);
            phys->SetAngularVelocity(phys->GetAngularVelocity() * globalDamping);
            phys->UpdateInertiaTensor();
        });
}

/*
Once we're finished with a physics update, we have to
clear out any accumulated forces, ready to receive new
ones in the next 'game' frame.
*/
void PhysicsSystem::ClearForces()
{
    gameWorld.OperateOnContents(
        [](GameObject *o)
        {
            PhysicsObject *phys = o->GetPhysicsObject();
            if (phys)
            {
                phys->ClearForces();
            }
        });
}

/*

As part of the final physics tutorials, we add in the ability
to constrain objects based on some extra calculation, allowing
us to model springs and ropes etc.

*/
void PhysicsSystem::UpdateConstraints(float dt)
{
    std::vector<Constraint *>::const_iterator first;
    std::vector<Constraint *>::const_iterator last;
    gameWorld.GetConstraintIterators(first, last);

    for (auto i = first; i != last; ++i)
    {
        (*i)->UpdateConstraint(dt);
    }
}