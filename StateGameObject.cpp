#include "StateGameObject.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace CSC8503;

StateGameObject::StateGameObject()
{
    stateMachine = new StateMachine();
    counter = 0.0f;
}

StateGameObject::~StateGameObject()
{
    delete stateMachine;
}

void StateGameObject::Update(float dt)
{
    if (stateMachine)
    {
        stateMachine->Update(dt);
    }
}

void StateGameObject::MoveLeft(float dt)
{
    if (physicsObject)
    {
        physicsObject->AddForce(Vector3(-100.0f, 0.0f, 0.0f));
    }
    counter += dt;
}

void StateGameObject::MoveRight(float dt)
{
    if (physicsObject)
    {
        physicsObject->AddForce(Vector3(100.0f, 0.0f, 0.0f));
    }
    counter += dt;
}