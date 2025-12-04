#pragma once
#include "RenderObject.h"
#include "Vector.h"
#include "ScoreboardManager.h"
#include "NavigationGrid.h"
#include <vector>
#include <functional>
#include <string>
namespace NCL
{
    class Controller;

    namespace Rendering
    {
        class Mesh;
        class Texture;
        class Shader;
    }
    namespace CSC8503
    {
        class GameTechRendererInterface;
        class PhysicsSystem;
        class GameWorld;
        class GameObject;
        class FragileParcel;
        class PressurePlate;
        class SlidingDoor;
        class DeliveryZone;
        class StateGameObject;

        class TutorialGame
        {
        public:
            TutorialGame(GameWorld &gameWorld, GameTechRendererInterface &renderer, PhysicsSystem &physics);
            ~TutorialGame();

            virtual void UpdateGame(float dt);
            void StartNewRun();

        protected:
            void InitCamera();

            void InitWorld();
            void BuildMazeEnvironment();

            /*
            These are some of the world/object creation functions I created when testing the functionality
            in the module. Feel free to mess around with them to see different objects being created in different
            test scenarios (constraints, collision types, and so on).
            */
            void InitGameExamples();

            void CreateSphereGrid(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
            void CreatedMixedGrid(int numRows, int numCols, float rowSpacing, float colSpacing);
            void CreateAABBGrid(int numRows, int numCols, float rowSpacing, float colSpacing, const NCL::Maths::Vector3 &cubeDims);

            bool SelectObject();
            void MoveSelectedObject();
            void DebugObjectMovement();
            void LockedObjectMovement();

            GameObject *AddFloorToWorld(const NCL::Maths::Vector3 &position);
            GameObject *AddSphereToWorld(const NCL::Maths::Vector3 &position, float radius, float inverseMass = 10.0f);
            GameObject *AddCubeToWorld(const NCL::Maths::Vector3 &position, NCL::Maths::Vector3 dimensions, float inverseMass = 10.0f);

            GameObject *AddPlayerToWorld(const NCL::Maths::Vector3 &position);
            StateGameObject *AddEnemyToWorld(const NCL::Maths::Vector3 &position);
            GameObject *AddBonusToWorld(const NCL::Maths::Vector3 &position);
            FragileParcel *AddFragileParcelToWorld(const NCL::Maths::Vector3 &position);
            PressurePlate *AddPressurePlateToWorld(const NCL::Maths::Vector3 &position, const NCL::Maths::Vector3 &dimensions, const std::function<void(bool)> &callback);
            SlidingDoor *AddSlidingDoorToWorld(const NCL::Maths::Vector3 &position, const NCL::Maths::Vector3 &dimensions, const NCL::Maths::Vector3 &openOffset);
            DeliveryZone *AddDeliveryZoneToWorld(const NCL::Maths::Vector3 &position, const NCL::Maths::Vector3 &dimensions);

            void InitGameplay();
            void BuildEnvironment();
            void SpawnParcels();
            void SpawnEnemies();
            void UpdatePlayerInput(float dt);
            void StabiliseDynamicCharacters(float dt);
            void AlignObjectToDirection(GameObject *object, const NCL::Maths::Vector3 &direction, float dt, float turnSpeed);
            void UpdateHUD(float dt);
            void UpdateDeliveryIndicator(float dt);
            void UpdateThirdPersonCamera(float dt);
            void UpdateParcelDeliveryChecks();
            void UpdateGhostCamera(float dt);
            void UpdateNavigationPath(float dt);
            bool IsInsideDeliveryZone(const NCL::Maths::Vector3 &point, float verticalTolerance = 1.5f) const;
            void UpdateGameState(float dt);
            void ResetGameplay();
            void HandleRayInteractions();
            void SetMouseLookActive(bool enabled);
            void UpdateCarriedParcel(float dt);
            FragileParcel *FindNearestParcel(float radius) const;
            void PickupParcel(FragileParcel *parcel);
            void DropCarriedParcel(bool immediate = false);
            void SetupEnemyStateMachine(StateGameObject *enemy, const NCL::Maths::Vector3 &patrolStart,
                                        const NCL::Maths::Vector3 &patrolEnd, float alertRadius);

            void OnParcelBroken(FragileParcel &parcel);
            void OnParcelDelivered(FragileParcel &parcel);

            void StabiliseCharacter(GameObject *object, float dt, float alignStrength, float turnSpeed);
            void UpdateScoreboardSystem(float dt);
            void SubmitScoreIfNeeded();
            void DrawHighScorePanel(const NCL::Maths::Vector2 &origin, float fontSize = 20.0f) const;
            void SetScoreboardStatus(const std::string &message);

            std::vector<FragileParcel *> parcels;
            std::vector<NCL::Maths::Vector3> parcelSpawnPoints;
            std::vector<PressurePlate *> plates;
            std::vector<SlidingDoor *> doors;
            std::vector<StateGameObject *> enemies;
            DeliveryZone *deliveryZone = nullptr;
            GameObject *deliveryMarker = nullptr;
            GameObject *deliveryPad = nullptr;
            GameObject *playerObject = nullptr;
            FragileParcel *carriedParcel = nullptr;
            NCL::Maths::Vector3 deliveryZoneHalfSize = NCL::Maths::Vector3();
            NCL::Maths::Vector3 mazeCentre = NCL::Maths::Vector3();
            float godViewHeight = 520.0f;

            int totalParcels = 0;
            int deliveredParcels = 0;
            int playerScore = 0;
            float gameTimer = 0.0f;
            float timeLimit = 300.0f;
            bool gameWon = false;
            bool gameLost = false;
            bool showMenu = true;

            GameWorld &world;
            GameTechRendererInterface &renderer;
            PhysicsSystem &physics;
            Controller *controller;

            bool useGravity;
            bool inSelectionMode;
            bool mouseLookActive = false;
            float rayCastCooldown = 0.0f;
            float playerMoveForce = 32.0f;
            float playerVerticalForce = 18.0f;
            float playerJumpImpulse = 14.0f;
            float playerTurnSpeed = 12.0f;
            int jumpCount = 0;
            float parcelPickupCooldown = 0.0f;
            float parcelPickupRange = 3.5f;
            float parcelDefaultInverseMass = 0.5f;
            NCL::Maths::Vector3 parcelCarryOffset = NCL::Maths::Vector3(0.0f, 1.4f, 1.1f);
            NCL::Maths::Vector3 playerSpawnPosition = NCL::Maths::Vector3(-40.0f, 5.0f, 0.0f);
            float deliveryIndicatorTimer = 0.0f;
            float enemyTurnSpeed = 6.0f;
            float firstPersonLookSensitivity = 0.42f;
            bool showHighScorePanel = false;
            bool scoreSubmittedThisRun = false;
            std::string scoreboardStatusMessage = "Scoreboard offline";
            std::string scoreboardJoinAddress = "127.0.0.1";
            std::string playerProfileName = "Courier";
            float scoreboardStatusTimer = 0.0f;
            ScoreboardManager scoreboardManager;

            float forceMagnitude;

            GameObject *selectionObject = nullptr;

            Rendering::Mesh *capsuleMesh = nullptr;
            Rendering::Mesh *cubeMesh = nullptr;
            Rendering::Mesh *sphereMesh = nullptr;

            Rendering::Texture *defaultTex = nullptr;
            Rendering::Texture *checkerTex = nullptr;
            Rendering::Texture *glassTex = nullptr;

            // Coursework Meshes
            Rendering::Mesh *catMesh = nullptr;
            Rendering::Mesh *kittenMesh = nullptr;
            Rendering::Mesh *enemyMesh = nullptr;
            Rendering::Mesh *bonusMesh = nullptr;
            Rendering::Mesh *playerMesh = nullptr;

            GameTechMaterial checkerMaterial;
            GameTechMaterial glassMaterial;
            GameTechMaterial notexMaterial;

            // Coursework Additional functionality
            GameObject *lockedObject = nullptr;
            NCL::Maths::Vector3 lockedOffset = NCL::Maths::Vector3(0, 14, 20);
            void LockCameraToObject(GameObject *o)
            {
                lockedObject = o;
            }

            GameObject *objClosest = nullptr;
            NavigationGrid *navGrid = nullptr;
            NCL::Maths::Vector3 mazeOffset;
        };
    }
}