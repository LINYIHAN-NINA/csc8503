#include "CourierGameObjects.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace CSC8503;

FragileParcel::FragileParcel(float breakVelocityThreshold)
{
    breakVelocity = breakVelocityThreshold;
    broken = false;
    delivered = false;
}

void FragileParcel::SetBreakCallback(const std::function<void(FragileParcel &)> &callback)
{
    onBreak = callback;
}

void FragileParcel::SetDeliveryCallback(const std::function<void(FragileParcel &)> &callback)
{
    onDelivered = callback;
}

void FragileParcel::ResetParcel()
{
    broken = false;
    delivered = false;
}

void FragileParcel::MarkDelivered()
{
    if (delivered || broken)
    {
        return;
    }
    delivered = true;
    if (onDelivered)
    {
        onDelivered(*this);
    }
}

void FragileParcel::OnCollisionBegin(GameObject *otherObject)
{
    if (broken || delivered)
    {
        return;
    }
    if (!physicsObject)
    {
        return;
    }
    float speed = Vector::Length(physicsObject->GetLinearVelocity());
    if (speed >= breakVelocity)
    {
        broken = true;
        if (onBreak)
        {
            onBreak(*this);
        }
    }
}

PressurePlate::PressurePlate(const std::function<void(bool)> &callback)
{
    onStateChanged = callback;
    pressed = false;
}

void PressurePlate::OnCollisionBegin(GameObject *otherObject)
{
    contacts.insert(otherObject);
    if (!pressed)
    {
        pressed = true;
        if (onStateChanged)
        {
            onStateChanged(true);
        }
    }
}

void PressurePlate::OnCollisionEnd(GameObject *otherObject)
{
    contacts.erase(otherObject);
    if (contacts.empty() && pressed)
    {
        pressed = false;
        if (onStateChanged)
        {
            onStateChanged(false);
        }
    }
}

SlidingDoor::SlidingDoor(const Vector3 &closedPos, const Vector3 &openOffset)
{
    closedPosition = closedPos;
    openPosition = closedPos + openOffset;
    isOpen = false;
}

void SlidingDoor::Open()
{
    if (isOpen)
    {
        return;
    }
    isOpen = true;
    transform.SetPosition(openPosition);
}

void SlidingDoor::Close()
{
    if (!isOpen)
    {
        return;
    }
    isOpen = false;
    transform.SetPosition(closedPosition);
}

void DeliveryZone::OnCollisionBegin(GameObject *otherObject)
{
    auto parcel = dynamic_cast<FragileParcel *>(otherObject);
    if (!parcel)
    {
        return;
    }
    if (!parcel->IsDelivered() && !parcel->IsBroken())
    {
        parcel->MarkDelivered();
    }
}
