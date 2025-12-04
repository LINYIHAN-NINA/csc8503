#pragma once

#include "GameObject.h"

#include <functional>
#include <set>

namespace NCL
{
    namespace CSC8503
    {

        class FragileParcel : public GameObject
        {
        public:
            FragileParcel(float breakVelocityThreshold = 15.0f);

            void SetBreakCallback(const std::function<void(FragileParcel &)> &callback);
            void SetDeliveryCallback(const std::function<void(FragileParcel &)> &callback);

            bool IsBroken() const { return broken; }
            bool IsDelivered() const { return delivered; }

            void ResetParcel();
            void MarkDelivered();

            float GetBreakVelocity() const { return breakVelocity; }

            void OnCollisionBegin(GameObject *otherObject) override;

        private:
            float breakVelocity;
            bool broken;
            bool delivered;
            std::function<void(FragileParcel &)> onBreak;
            std::function<void(FragileParcel &)> onDelivered;
        };

        class PressurePlate : public GameObject
        {
        public:
            PressurePlate(const std::function<void(bool)> &callback);

            void OnCollisionBegin(GameObject *otherObject) override;
            void OnCollisionEnd(GameObject *otherObject) override;

            bool IsPressed() const { return pressed; }

        private:
            std::function<void(bool)> onStateChanged;
            std::set<GameObject *> contacts;
            bool pressed;
        };

        class SlidingDoor : public GameObject
        {
        public:
            SlidingDoor(const Vector3 &closedPos, const Vector3 &openOffset);

            void Open();
            void Close();
            bool IsOpen() const { return isOpen; }

        private:
            Vector3 closedPosition;
            Vector3 openPosition;
            bool isOpen;
        };

        class DeliveryZone : public GameObject
        {
        public:
            DeliveryZone() = default;

            void OnCollisionBegin(GameObject *otherObject) override;
        };

    }
}
