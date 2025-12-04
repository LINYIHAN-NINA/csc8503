#include "TutorialGame.h"
#include "GameWorld.h"
#include "PhysicsSystem.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"
#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"
#include "CourierGameObjects.h"

#include "Window.h"
#include "Texture.h"
#include "Shader.h"
#include "Mesh.h"

#include "Debug.h"

#include "KeyboardMouseController.h"

#include "GameTechRendererInterface.h"

#include "Ray.h"
#include "Quaternion.h"
#include "Maths.h"

#include "Assets.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <utility>

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame(GameWorld &inWorld, GameTechRendererInterface &inRenderer, PhysicsSystem &inPhysics)
    : world(inWorld),
      renderer(inRenderer),
      physics(inPhysics)
{

    forceMagnitude = 10.0f;
    useGravity = true;
    inSelectionMode = false;

    controller = new KeyboardMouseController(*Window::GetWindow()->GetKeyboard(), *Window::GetWindow()->GetMouse());

    world.GetMainCamera().SetController(*controller);

    world.SetSunPosition({-200.0f, 60.0f, -200.0f});
    world.SetSunColour({0.8f, 0.8f, 0.5f});

    controller->MapAxis(0, "Sidestep");
    controller->MapAxis(1, "UpDown");
    controller->MapAxis(2, "Forward");

    controller->MapAxis(3, "XLook");
    controller->MapAxis(4, "YLook");

    cubeMesh = renderer.LoadMesh("cube.msh");
    sphereMesh = renderer.LoadMesh("sphere.msh");
    catMesh = renderer.LoadMesh("ORIGAMI_Chat.msh");
    kittenMesh = renderer.LoadMesh("Kitten.msh");

    playerMesh = renderer.LoadMesh("Keeper.msh");
    enemyMesh = renderer.LoadMesh("goose.msh");

    bonusMesh = renderer.LoadMesh("19463_Kitten_Head_v1.msh");
    capsuleMesh = renderer.LoadMesh("capsule.msh");

    defaultTex = renderer.LoadTexture("Default.png");
    checkerTex = renderer.LoadTexture("checkerboard.png");
    glassTex = renderer.LoadTexture("stainedglass.tga");

    checkerMaterial.type = MaterialType::Opaque;
    checkerMaterial.diffuseTex = checkerTex;

    glassMaterial.type = MaterialType::Transparent;
    glassMaterial.diffuseTex = glassTex;

    notexMaterial.type = MaterialType::Opaque;
    notexMaterial.diffuseTex = defaultTex;

    physics.UseGravity(useGravity);

    InitCamera();
    InitWorld();
    showMenu = true;
    Window::GetWindow()->ShowOSPointer(true);
    Window::GetWindow()->LockMouseToWindow(false);
}

TutorialGame::~TutorialGame()
{
}

void TutorialGame::UpdateGame(float dt)
{
    UpdateScoreboardSystem(dt);
    if (showMenu)
    {
        Debug::ClearPrints(); // Clear previous debug prints
        mouseLookActive = false;
        Window::GetWindow()->ShowOSPointer(true);
        Window::GetWindow()->LockMouseToWindow(false);
        SubmitScoreIfNeeded();

        Vector4 titleColour = Debug::WHITE;
        std::string title = "Courier HQ";
        if (gameWon)
        {
            title = "All parcels delivered!";
            titleColour = Debug::GREEN;
        }
        else if (gameLost)
        {
            title = "Delivery failed";
            titleColour = Debug::RED;
        }

        std::string scoreboardState = "Scoreboard: Offline";
        auto sbState = scoreboardManager.GetState();
        if (sbState == ScoreboardConnectionState::Hosting)
        {
            scoreboardState = "Scoreboard: Hosting on port " + std::to_string(ScoreboardServicePort);
        }
        else if (sbState == ScoreboardConnectionState::Client)
        {
            scoreboardState = "Scoreboard: Connected to remote";
        }

        const bool showingScores = showHighScorePanel;
        const float spacing = showingScores ? 4.0f : 4.5f;
        const float horizontalOffset = showingScores ? 40.0f : 48.0f;
        const float menuFontSize = 16.0f;

        std::vector<std::pair<std::string, Vector4>> menuLines;
        if (showingScores)
        {
            menuLines.emplace_back(title, titleColour);
            menuLines.emplace_back("Press TAB to return to mission briefing", Debug::WHITE);
            menuLines.emplace_back(scoreboardState, Debug::GREEN);
            menuLines.emplace_back(scoreboardStatusMessage, Debug::WHITE);
        }
        else
        {
            menuLines = {
                {title, titleColour},
                {"Press ENTER or 1 to deploy", Debug::WHITE},
                {"Press ESC to exit", Debug::WHITE},
                {"Last score: " + std::to_string(playerScore), Debug::WHITE},
                {"[H] Host scoreboard  [J] Join  [R] Refresh  [O] Disconnect", Debug::WHITE},
                {"[TAB] Toggle high scores  (default join: " + scoreboardJoinAddress + ")", Debug::WHITE},
                {scoreboardState, Debug::GREEN},
                {scoreboardStatusMessage, Debug::WHITE}};
        }

        const float totalHeight = spacing * static_cast<float>(std::max<int>(static_cast<int>(menuLines.size()) - 1, 1));
        const Vector2 menuOrigin(50.0f - horizontalOffset, 50.0f - totalHeight * 0.5f);
        for (size_t i = 0; i < menuLines.size(); ++i)
        {
            Debug::Print(menuLines[i].first, menuOrigin + Vector2(0.0f, spacing * static_cast<float>(i)), menuLines[i].second, menuFontSize);
        }

        if (showingScores)
        {
            const Vector2 panelOrigin = menuOrigin + Vector2(0.0f, spacing * static_cast<float>(menuLines.size()));
            DrawHighScorePanel(panelOrigin, menuFontSize);
        }
        else
        {
            const Vector2 panelOrigin = menuOrigin + Vector2(20.0f, 0.0f);
            Debug::Print("Press TAB to view the latest courier rankings", panelOrigin, Debug::WHITE, menuFontSize);
        }

        auto *keyboard = Window::GetKeyboard();
        if (keyboard->KeyPressed(KeyCodes::RETURN) || keyboard->KeyPressed(KeyCodes::NUM1))
        {
            ResetGameplay();
            return;
        }
        if (keyboard->KeyPressed(KeyCodes::H))
        {
            bool ok = scoreboardManager.Host();
            SetScoreboardStatus(ok ? "Hosting scoreboard" : "Unable to host scoreboard");
            if (ok)
            {
                scoreboardManager.RequestScores();
            }
        }
        if (keyboard->KeyPressed(KeyCodes::J))
        {
            bool ok = scoreboardManager.Connect(scoreboardJoinAddress);
            SetScoreboardStatus(ok ? "Connected to scoreboard" : "Failed to connect to scoreboard");
        }
        if (keyboard->KeyPressed(KeyCodes::R))
        {
            scoreboardManager.RequestScores();
            SetScoreboardStatus("Refreshing scoreboard data");
        }
        if (keyboard->KeyPressed(KeyCodes::O))
        {
            scoreboardManager.Disconnect();
            SetScoreboardStatus("Scoreboard offline");
        }
        if (keyboard->KeyPressed(KeyCodes::TAB))
        {
            showHighScorePanel = !showHighScorePanel;
        }
        if (keyboard->KeyPressed(KeyCodes::ESCAPE))
        {
            return;
        }
        return;
    }

    rayCastCooldown = std::max(0.0f, rayCastCooldown - dt);

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::Q))
    {
        SetMouseLookActive(!mouseLookActive);
    }

    controller->Update(dt);

	//摄像机控制 Camera control
    if (mouseLookActive)
    {
        UpdateThirdPersonCamera(dt); //第三人称视角跟随
    }
    else
    {
		UpdateGhostCamera(dt); //幽灵视角自由移动
    }
    if (lockedObject != nullptr && !mouseLookActive)
    {
        // Disable locked object camera movement in Ghost View
        // Vector3 objPos = lockedObject->GetTransform().GetPosition();
        // Vector3 desiredPos = objPos + lockedOffset;
        // Vector3 currentPos = world.GetMainCamera().GetPosition();
        // float followLerp = std::clamp(dt * 5.0f, 0.0f, 1.0f);
        // Vector3 smoothedPos = currentPos + (desiredPos - currentPos) * followLerp;
        // world.GetMainCamera().SetPosition(smoothedPos);
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F1))
    {
        InitWorld(); // We can reset the simulation at any time with F1
        selectionObject = nullptr;
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F2))
    {
        InitCamera(); // F2 will reset the camera to a specific default place
    }

    // Running certain physics updates in a consistent order might cause some
    // bias in the calculations - the same objects might keep 'winning' the constraint
    // allowing the other one to stretch too much etc. Shuffling the order so that it
    // is random every frame can help reduce such bias.
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F9))
    {
        world.ShuffleConstraints(true);
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10))
    {
        world.ShuffleConstraints(false);
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F7))
    {
        world.ShuffleObjects(true);
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F8))
    {
        world.ShuffleObjects(false);
    }

    if (lockedObject)
    {
        LockedObjectMovement();
    }
    else
    {
        DebugObjectMovement();
    }

	//读取WASD按键 Read WASD key inputs
    UpdatePlayerInput(dt);
    StabiliseDynamicCharacters(dt);
	//如果手里拿着快递，让它跟着玩家走 If you are carrying a parcel, let it follow the player
    UpdateCarriedParcel(dt);
    if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::Middle) && rayCastCooldown <= 0.0f)
    {
        HandleRayInteractions();
        rayCastCooldown = 0.6f;
    }

    UpdateHUD(dt);
    UpdateDeliveryIndicator(dt);
    UpdateParcelDeliveryChecks(); //检查快递是否进圈
    UpdateGameState(dt); //检查时间是否到了或者快递是否送完 
    UpdateNavigationPath(dt);

    RayCollision closestCollision;
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::K) && selectionObject)
    {
        Vector3 rayPos;
        Vector3 rayDir;

        rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

        rayPos = selectionObject->GetTransform().GetPosition();

        Ray r = Ray(rayPos, rayDir);

        if (world.Raycast(r, closestCollision, true, selectionObject))
        {
            if (objClosest)
            {
                objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
            }
            objClosest = (GameObject *)closestCollision.node;

            objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));
        }
    }

    // This year we can draw debug textures as well!
    Debug::DrawTex(*defaultTex, Vector2(10, 10), Vector2(5, 5), Debug::WHITE);
    Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));
    if (useGravity)
    {
        Debug::Print("(G)ravity on", Vector2(5, 95), Debug::RED);
    }
    else
    {
        Debug::Print("(G)ravity off", Vector2(5, 95), Debug::RED);
    }

    SelectObject();
    MoveSelectedObject();

    world.OperateOnContents(
        [dt](GameObject *o)
        {
            o->Update(dt);
        });
}

void TutorialGame::InitCamera()
{
    world.GetMainCamera().SetNearPlane(0.1f);
    world.GetMainCamera().SetFarPlane(500.0f);
    world.GetMainCamera().SetPitch(-15.0f);
    world.GetMainCamera().SetYaw(315.0f);
    world.GetMainCamera().SetPosition(Vector3(-60, 40, 60));
    world.GetMainCamera().SetSpeed(25.0f);
    lockedObject = nullptr;
}

void TutorialGame::InitWorld() 
{
	world.ClearAndErase(); 
    physics.Clear();

    parcels.clear();
    parcelSpawnPoints.clear();
    plates.clear();
    doors.clear();
    enemies.clear();
    deliveryZone = nullptr;
    deliveryPad = nullptr;
    playerObject = nullptr;
    objClosest = nullptr;
    carriedParcel = nullptr;
    deliveryMarker = nullptr;
    deliveryZoneHalfSize = Vector3();
    scoreSubmittedThisRun = false;

    gameTimer = timeLimit;
    deliveredParcels = 0;
    totalParcels = 0;
    playerScore = 0;
    gameWon = false;
    gameLost = false;
    rayCastCooldown = 0.0f;
    parcelPickupCooldown = 0.0f;
    deliveryIndicatorTimer = 0.0f;

    BuildEnvironment();
    InitGameplay();
}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject *TutorialGame::AddFloorToWorld(const Vector3 &position)
{
    GameObject *floor = new GameObject();

    Vector3 floorSize = Vector3(200, 2, 200);
    AABBVolume *volume = new AABBVolume(floorSize);
    floor->SetBoundingVolume(volume);
    floor->GetTransform()
        .SetScale(floorSize * 2.0f)
        .SetPosition(position);

    floor->SetRenderObject(new RenderObject(floor->GetTransform(), cubeMesh, checkerMaterial));
    floor->SetPhysicsObject(new PhysicsObject(floor->GetTransform(), floor->GetBoundingVolume()));

    floor->GetPhysicsObject()->SetInverseMass(0);
    floor->GetPhysicsObject()->InitCubeInertia();

    world.AddGameObject(floor);

    return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple'
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject *TutorialGame::AddSphereToWorld(const Vector3 &position, float radius, float inverseMass)
{
    GameObject *sphere = new GameObject();

    Vector3 sphereSize = Vector3(radius, radius, radius);
    SphereVolume *volume = new SphereVolume(radius);
    sphere->SetBoundingVolume(volume);

    sphere->GetTransform()
        .SetScale(sphereSize)
        .SetPosition(position);

    sphere->SetRenderObject(new RenderObject(sphere->GetTransform(), sphereMesh, checkerMaterial));
    sphere->SetPhysicsObject(new PhysicsObject(sphere->GetTransform(), sphere->GetBoundingVolume()));

    sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
    sphere->GetPhysicsObject()->InitSphereInertia();

    world.AddGameObject(sphere);

    return sphere;
}

GameObject *TutorialGame::AddCubeToWorld(const Vector3 &position, Vector3 dimensions, float inverseMass)
{
    GameObject *cube = new GameObject();

    AABBVolume *volume = new AABBVolume(dimensions);
    cube->SetBoundingVolume(volume);

    cube->GetTransform()
        .SetPosition(position)
        .SetScale(dimensions * 2.0f);

    cube->SetRenderObject(new RenderObject(cube->GetTransform(), cubeMesh, checkerMaterial));
    cube->SetPhysicsObject(new PhysicsObject(cube->GetTransform(), cube->GetBoundingVolume()));

    cube->GetPhysicsObject()->SetInverseMass(inverseMass);
    cube->GetPhysicsObject()->InitCubeInertia();

    world.AddGameObject(cube);

    return cube;
}

GameObject *TutorialGame::AddPlayerToWorld(const Vector3 &position)
{
    float meshSize = 3.0f;
    float inverseMass = 0.5f;

    GameObject *character = new GameObject();
    Vector3 bodyHalfDims = Vector3(0.3f, 0.9f, 0.3f) * meshSize;
    AABBVolume *volume = new AABBVolume(bodyHalfDims);

    character->SetBoundingVolume(volume);

    character->GetTransform()
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    Rendering::Mesh *courierMesh = playerMesh ? playerMesh : (capsuleMesh ? capsuleMesh : cubeMesh);
    character->SetRenderObject(new RenderObject(character->GetTransform(), courierMesh, notexMaterial));
    character->SetPhysicsObject(new PhysicsObject(character->GetTransform(), character->GetBoundingVolume()));

    character->GetPhysicsObject()->SetInverseMass(inverseMass);
    character->GetPhysicsObject()->InitCubeInertia();

    world.AddGameObject(character);

    return character;
}

StateGameObject *TutorialGame::AddEnemyToWorld(const Vector3 &position)
{
    float meshSize = 3.0f;
    float inverseMass = 0.5f;

    StateGameObject *character = new StateGameObject();

    AABBVolume *volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
    character->SetBoundingVolume(volume);

    character->GetTransform()
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    character->SetRenderObject(new RenderObject(character->GetTransform(), enemyMesh, notexMaterial));
    character->SetPhysicsObject(new PhysicsObject(character->GetTransform(), character->GetBoundingVolume()));

    character->GetPhysicsObject()->SetInverseMass(inverseMass);
    character->GetPhysicsObject()->InitSphereInertia();

    world.AddGameObject(character);

    return character;
}

GameObject *TutorialGame::AddBonusToWorld(const Vector3 &position)
{
    GameObject *apple = new GameObject();

    SphereVolume *volume = new SphereVolume(0.5f);
    apple->SetBoundingVolume(volume);
    apple->GetTransform()
        .SetScale(Vector3(2, 2, 2))
        .SetPosition(position);

    apple->SetRenderObject(new RenderObject(apple->GetTransform(), bonusMesh, glassMaterial));
    apple->SetPhysicsObject(new PhysicsObject(apple->GetTransform(), apple->GetBoundingVolume()));

    apple->GetPhysicsObject()->SetInverseMass(1.0f);
    apple->GetPhysicsObject()->InitSphereInertia();

    world.AddGameObject(apple);

    return apple;
}

FragileParcel *TutorialGame::AddFragileParcelToWorld(const Vector3 &position)
{
    Vector3 parcelHalfDims = Vector3(0.5f, 0.5f, 0.5f);
    FragileParcel *parcel = new FragileParcel(18.0f);
    AABBVolume *volume = new AABBVolume(parcelHalfDims);
    parcel->SetBoundingVolume(volume);
    parcel->GetTransform()
        .SetScale(parcelHalfDims * 2.0f)
        .SetPosition(position);

    parcel->SetRenderObject(new RenderObject(parcel->GetTransform(), cubeMesh, glassMaterial));
    parcel->SetPhysicsObject(new PhysicsObject(parcel->GetTransform(), parcel->GetBoundingVolume()));
    parcel->GetPhysicsObject()->SetInverseMass(parcelDefaultInverseMass);
    parcel->GetPhysicsObject()->InitCubeInertia();

    parcel->SetBreakCallback([this](FragileParcel &p)
                             { OnParcelBroken(p); });
    parcel->SetDeliveryCallback([this](FragileParcel &p)
                                { OnParcelDelivered(p); });

    world.AddGameObject(parcel);
    parcels.emplace_back(parcel);
    return parcel;
}

PressurePlate *TutorialGame::AddPressurePlateToWorld(const Vector3 &position, const Vector3 &dimensions, const std::function<void(bool)> &callback)
{
    PressurePlate *plate = new PressurePlate(callback);
    AABBVolume *volume = new AABBVolume(dimensions);
    plate->SetBoundingVolume(volume);
    plate->GetTransform()
        .SetScale(dimensions * 2.0f)
        .SetPosition(position);

    plate->SetRenderObject(new RenderObject(plate->GetTransform(), cubeMesh, checkerMaterial));
    plate->SetPhysicsObject(new PhysicsObject(plate->GetTransform(), plate->GetBoundingVolume()));
    plate->GetPhysicsObject()->SetInverseMass(0.0f);
    plate->GetPhysicsObject()->InitCubeInertia();

    plates.emplace_back(plate);
    world.AddGameObject(plate);
    return plate;
}

SlidingDoor *TutorialGame::AddSlidingDoorToWorld(const Vector3 &position, const Vector3 &dimensions, const Vector3 &openOffset)
{
    SlidingDoor *door = new SlidingDoor(position, openOffset);
    AABBVolume *volume = new AABBVolume(dimensions);
    door->SetBoundingVolume(volume);
    door->GetTransform()
        .SetScale(dimensions * 2.0f)
        .SetPosition(position);

    door->SetRenderObject(new RenderObject(door->GetTransform(), cubeMesh, glassMaterial));
    door->SetPhysicsObject(new PhysicsObject(door->GetTransform(), door->GetBoundingVolume()));
    door->GetPhysicsObject()->SetInverseMass(0.0f);
    door->GetPhysicsObject()->InitCubeInertia();

    doors.emplace_back(door);
    world.AddGameObject(door);
    return door;
}

DeliveryZone *TutorialGame::AddDeliveryZoneToWorld(const Vector3 &position, const Vector3 &dimensions)
{
    DeliveryZone *zone = new DeliveryZone();
    AABBVolume *volume = new AABBVolume(dimensions);
    zone->SetBoundingVolume(volume);
    zone->GetTransform()
        .SetScale(dimensions * 2.0f)
        .SetPosition(position);

    zone->SetRenderObject(new RenderObject(zone->GetTransform(), cubeMesh, glassMaterial));
    zone->SetPhysicsObject(new PhysicsObject(zone->GetTransform(), zone->GetBoundingVolume()));
    zone->GetPhysicsObject()->SetInverseMass(0.0f);
    zone->GetPhysicsObject()->InitCubeInertia();
    zone->GetRenderObject()->SetColour(Vector4(0.3f, 1.0f, 0.4f, 0.6f));

    world.AddGameObject(zone);
    deliveryZone = zone;
    deliveryZoneHalfSize = dimensions;
    return zone;
}

void TutorialGame::BuildEnvironment()
{
    BuildMazeEnvironment();
}

void TutorialGame::BuildMazeEnvironment()
{
    AddFloorToWorld(Vector3(0.0f, -8.0f, 0.0f));
    parcelSpawnPoints.clear();

    using GridPos = std::pair<int, int>;
    const int mazeRows = 21;
    const int mazeCols = 21;
    std::vector<std::string> mazeLayout(mazeRows, std::string(mazeCols, 'X'));

    const float cellSize = 12.0f;
    const float corridorHeight = -6.0f;
    const float wallHeight = 8.0f;
    const Vector3 mazeOrigin(-(static_cast<float>(mazeCols) * cellSize) * 0.5f,
                             corridorHeight,
                             -(static_cast<float>(mazeRows) * cellSize) * 0.5f);
    mazeCentre = mazeOrigin + Vector3(static_cast<float>(mazeCols) * cellSize * 0.5f,
                                      0.0f,
                                      static_cast<float>(mazeRows) * cellSize * 0.5f);
    mazeCentre.y = corridorHeight;

    std::mt19937 rng(1337u);
    const std::array<GridPos, 4> carveDirections = {GridPos{0, 1}, GridPos{0, -1}, GridPos{1, 0}, GridPos{-1, 0}};

    std::function<void(int, int)> carveMaze = [&](int r, int c)
    {
        mazeLayout[r][c] = '.';
        auto dirs = carveDirections;
        std::shuffle(dirs.begin(), dirs.end(), rng);
        for (const auto &dir : dirs)
        {
            int nr = r + dir.first * 2;
            int nc = c + dir.second * 2;
            if (nr <= 0 || nc <= 0 || nr >= mazeRows - 1 || nc >= mazeCols - 1)
            {
                continue;
            }
            if (mazeLayout[nr][nc] == 'X')
            {
                mazeLayout[r + dir.first][c + dir.second] = '.';
                carveMaze(nr, nc);
            }
        }
    };

    carveMaze(1, 1);

    std::ofstream mapFile(Assets::DATADIR + "maze.map");
    mapFile << (int)cellSize << "\n";
    mapFile << mazeCols << "\n";
    mapFile << mazeRows << "\n";
    for (int r = 0; r < mazeRows; ++r)
    {
        for (int c = 0; c < mazeCols; ++c)
        {
            char tile = mazeLayout[r][c];
            char mapChar = '.';
            if (tile == 'X' || tile == 'B')
            {
                mapChar = 'x';
            }
            mapFile << mapChar;
        }
        mapFile << "\n";
    }
    mapFile.close();

    if (navGrid)
    {
        delete navGrid;
    }
    navGrid = new NavigationGrid("maze.map");
    mazeOffset = mazeOrigin;

    auto cellIndex = [&](int r, int c)
    { return r * mazeCols + c; };

    std::vector<int> distances(mazeRows * mazeCols, -1);
    std::queue<GridPos> frontier;
    GridPos startCell = {1, 1};
    frontier.push(startCell);
    distances[cellIndex(startCell.first, startCell.second)] = 0;
    GridPos farthestCell = startCell;

    while (!frontier.empty())
    {
        GridPos cell = frontier.front();
        frontier.pop();
        int baseIdx = cellIndex(cell.first, cell.second);
        for (const auto &dir : carveDirections)
        {
            int nr = cell.first + dir.first;
            int nc = cell.second + dir.second;
            if (nr <= 0 || nc <= 0 || nr >= mazeRows - 1 || nc >= mazeCols - 1)
            {
                continue;
            }
            if (mazeLayout[nr][nc] == 'X')
            {
                continue;
            }
            int neighborIdx = cellIndex(nr, nc);
            if (distances[neighborIdx] != -1)
            {
                continue;
            }
            distances[neighborIdx] = distances[baseIdx] + 1;
            frontier.emplace(nr, nc);
            if (distances[neighborIdx] > distances[cellIndex(farthestCell.first, farthestCell.second)])
            {
                farthestCell = {nr, nc};
            }
        }
    }

    GridPos destCell = farthestCell;

    std::vector<std::vector<bool>> onSolution(mazeRows, std::vector<bool>(mazeCols, false));
    GridPos cursor = destCell;
    int cursorIdx = cellIndex(cursor.first, cursor.second);
    while (!(cursor == startCell))
    {
        onSolution[cursor.first][cursor.second] = true;
        bool advanced = false;
        for (const auto &dir : carveDirections)
        {
            int nr = cursor.first + dir.first;
            int nc = cursor.second + dir.second;
            if (nr < 0 || nc < 0 || nr >= mazeRows || nc >= mazeCols)
            {
                continue;
            }
            int neighborIdx = cellIndex(nr, nc);
            if (distances[neighborIdx] == -1)
            {
                continue;
            }
            if (distances[neighborIdx] == distances[cursorIdx] - 1)
            {
                cursor = {nr, nc};
                cursorIdx = neighborIdx;
                advanced = true;
                break;
            }
        }
        if (!advanced)
        {
            break;
        }
    }
    onSolution[startCell.first][startCell.second] = true;

    std::vector<std::pair<int, GridPos>> solutionCells;
    for (int r = 0; r < mazeRows; ++r)
    {
        for (int c = 0; c < mazeCols; ++c)
        {
            if (!onSolution[r][c])
            {
                continue;
            }
            if ((r == startCell.first && c == startCell.second) || (r == destCell.first && c == destCell.second))
            {
                continue;
            }
            int idx = cellIndex(r, c);
            if (distances[idx] < 0)
            {
                continue;
            }
            solutionCells.emplace_back(distances[idx], GridPos{r, c});
        }
    }

    std::sort(solutionCells.begin(), solutionCells.end(), [](const auto &a, const auto &b)
              { return a.first < b.first; });

    std::vector<GridPos> parcelCells;
    if (!solutionCells.empty())
    {
        const std::array<float, 3> fractions = {0.2f, 0.5f, 0.8f};
        for (float fraction : fractions)
        {
            int target = static_cast<int>(std::round(fraction * static_cast<float>(solutionCells.size() - 1)));
            target = std::clamp(target, 0, static_cast<int>(solutionCells.size() - 1));
            GridPos candidate = solutionCells[target].second;
            if (candidate == startCell || candidate == destCell)
            {
                continue;
            }
            if (std::find(parcelCells.begin(), parcelCells.end(), candidate) == parcelCells.end())
            {
                parcelCells.push_back(candidate);
            }
        }
    }
    if (parcelCells.empty() && !solutionCells.empty())
    {
        parcelCells.push_back(solutionCells[solutionCells.size() / 2].second);
    }

    for (const GridPos &cell : parcelCells)
    {
        mazeLayout[cell.first][cell.second] = 'P';
    }

    auto isWalkable = [&](int r, int c)
    {
        if (r < 0 || c < 0 || r >= mazeRows || c >= mazeCols)
        {
            return false;
        }
        return mazeLayout[r][c] != 'X';
    };

    std::vector<GridPos> shortcutWalls;
    for (int r = 2; r < mazeRows - 2; ++r)
    {
        for (int c = 2; c < mazeCols - 2; ++c)
        {
            if (mazeLayout[r][c] != 'X')
            {
                continue;
            }
            bool northOpen = isWalkable(r - 1, c);
            bool southOpen = isWalkable(r + 1, c);
            bool eastOpen = isWalkable(r, c + 1);
            bool westOpen = isWalkable(r, c - 1);
            bool connectsOpposite = (northOpen && southOpen) || (eastOpen && westOpen);
            if (!connectsOpposite)
            {
                continue;
            }
            bool touchesSolution = onSolution[r - 1][c] || onSolution[r + 1][c] || onSolution[r][c - 1] || onSolution[r][c + 1];
            if (!touchesSolution)
            {
                continue;
            }
            shortcutWalls.emplace_back(r, c);
        }
    }

    std::shuffle(shortcutWalls.begin(), shortcutWalls.end(), rng);
    const int shortcutQuota = 4;
    for (int i = 0; i < std::min(shortcutQuota, static_cast<int>(shortcutWalls.size())); ++i)
    {
        const auto &pos = shortcutWalls[i];
        mazeLayout[pos.first][pos.second] = 'B';
    }

    std::vector<GridPos> crateCandidates;
    for (int r = 1; r < mazeRows - 1; ++r)
    {
        for (int c = 1; c < mazeCols - 1; ++c)
        {
            if (mazeLayout[r][c] != '.' || onSolution[r][c])
            {
                continue;
            }
            int neighborCount = 0;
            for (const auto &dir : carveDirections)
            {
                int nr = r + dir.first;
                int nc = c + dir.second;
                if (mazeLayout[nr][nc] != 'X')
                {
                    neighborCount++;
                }
            }
            if (neighborCount >= 3)
            {
                crateCandidates.emplace_back(r, c);
            }
        }
    }
    std::shuffle(crateCandidates.begin(), crateCandidates.end(), rng);
    const int maxCrates = 6;
    for (int i = 0; i < std::min(maxCrates, static_cast<int>(crateCandidates.size())); ++i)
    {
        GridPos cell = crateCandidates[i];
        mazeLayout[cell.first][cell.second] = 'B';
    }

    mazeLayout[startCell.first][startCell.second] = 'S';
    mazeLayout[destCell.first][destCell.second] = 'D';

    auto gridToWorld = [&](const GridPos &cell)
    {
        Vector3 cellCentre = mazeOrigin + Vector3(static_cast<float>(cell.second) * cellSize, 0.0f, static_cast<float>(cell.first) * cellSize);
        return Vector3(cellCentre.x, corridorHeight, cellCentre.z);
    };

    std::vector<Vector3> crateCells;
    Vector3 deliveryCell;
    bool deliveryCellDefined = false;

    for (int row = 0; row < mazeRows; ++row)
    {
        for (int col = 0; col < mazeCols; ++col)
        {
            char tile = mazeLayout[row][col];
            Vector3 worldCell = gridToWorld(GridPos{row, col});

            switch (tile)
            {
            case 'X':
            {
                Vector3 dims(cellSize * 0.5f, wallHeight * 0.5f, cellSize * 0.5f);
                Vector3 position = worldCell + Vector3(0.0f, wallHeight * 0.5f, 0.0f);
                GameObject *wall = AddCubeToWorld(position, dims, 0.0f);
                if (wall && wall->GetRenderObject())
                {
                    wall->GetRenderObject()->SetColour(Vector4(0.35f, 0.35f, 0.42f, 1.0f));
                }
            }
            break;
            case 'B':
                crateCells.emplace_back(worldCell);
                break;
            case 'P':
                parcelSpawnPoints.emplace_back(worldCell + Vector3(0.0f, 0.5f, 0.0f));
                break;
            case 'S':
                playerSpawnPosition = worldCell + Vector3(0.0f, 2.0f, 0.0f);
                break;
            case 'D':
                deliveryCell = worldCell;
                deliveryCellDefined = true;
                break;
            default:
                break;
            }
        }
    }

    for (const Vector3 &crateCentre : crateCells)
    {
        Vector3 dims(2.5f, 2.5f, 2.5f);
        Vector3 position = crateCentre + Vector3(0.0f, dims.y, 0.0f);
        GameObject *crate = AddCubeToWorld(position, dims, 0.5f);
        if (crate && crate->GetRenderObject())
        {
            crate->GetRenderObject()->SetColour(Vector4(0.82f, 0.52f, 0.25f, 1.0f));
        }
    }

    if (!solutionCells.empty())
    {
        auto placeBonusAt = [&](const GridPos &cell)
        {
            Vector3 bonusPos = gridToWorld(cell) + Vector3(0.0f, 1.0f, 0.0f);
            AddBonusToWorld(bonusPos);
        };
        placeBonusAt(solutionCells[solutionCells.size() / 4].second);
        placeBonusAt(solutionCells[solutionCells.size() * 3 / 4].second);
    }

    if (deliveryCellDefined)
    {
        deliveryZone = AddDeliveryZoneToWorld(deliveryCell + Vector3(0.0f, -1.0f, 0.0f), Vector3(4.0f, 0.25f, 4.0f));
    }

    if (deliveryZone)
    {
        Vector3 zonePos = deliveryZone->GetTransform().GetPosition();
        if (deliveryZoneHalfSize.x > 0.0f && deliveryZoneHalfSize.z > 0.0f)
        {
            deliveryPad = new GameObject();
            Vector3 padScale = Vector3(deliveryZoneHalfSize.x * 2.4f, 0.2f, deliveryZoneHalfSize.z * 2.4f);
            Vector3 padPos = zonePos - Vector3(0.0f, deliveryZoneHalfSize.y - 0.1f, 0.0f);
            deliveryPad->GetTransform()
                .SetScale(padScale)
                .SetPosition(padPos);
            deliveryPad->SetRenderObject(new RenderObject(deliveryPad->GetTransform(), cubeMesh, checkerMaterial));
            deliveryPad->GetRenderObject()->SetColour(Vector4(0.05f, 0.4f, 0.08f, 0.85f));
            world.AddGameObject(deliveryPad);
        }
        deliveryMarker = new GameObject();
        deliveryMarker->GetTransform()
            .SetScale(Vector3(2.0f, 12.0f, 2.0f))
            .SetPosition(zonePos + Vector3(0.0f, 7.0f, 0.0f));
        deliveryMarker->SetRenderObject(new RenderObject(deliveryMarker->GetTransform(), cubeMesh, glassMaterial));
        deliveryMarker->GetRenderObject()->SetColour(Vector4(0.2f, 1.0f, 0.4f, 0.65f));
        world.AddGameObject(deliveryMarker);
    }
}

void TutorialGame::InitGameplay()
{
    playerObject = AddPlayerToWorld(playerSpawnPosition);
    if (playerObject)
    {
        playerObject->GetRenderObject()->SetColour(Vector4(1.0f, 0.85f, 0.85f, 1.0f));
        LockCameraToObject(playerObject);
    }
    AddCubeToWorld(playerSpawnPosition + Vector3(5, 0, 0), Vector3(2, 2, 2), 0.0f);
    AddCubeToWorld(playerSpawnPosition + Vector3(5, 2, 0), Vector3(2, 2, 2), 0.0f);
    AddCubeToWorld(playerSpawnPosition + Vector3(5, 0, 2), Vector3(2, 2, 2), 0.0f);
    AddCubeToWorld(playerSpawnPosition + Vector3(5, 0, -2), Vector3(2, 2, 2), 0.0f);
    SpawnParcels();
    SpawnEnemies();
}

void TutorialGame::SpawnParcels()
{
    parcels.clear();

    if (parcelSpawnPoints.empty())
    {
        parcelSpawnPoints = {
            Vector3(-70.0f, -6.0f, -25.0f),
            Vector3(-20.0f, -6.0f, -10.0f),
            Vector3(15.0f, -6.0f, 15.0f)};
    }

    for (const auto &pos : parcelSpawnPoints)
    {
        AddFragileParcelToWorld(pos);
    }

    totalParcels = static_cast<int>(parcels.size());
    deliveredParcels = 0;
}

void TutorialGame::SpawnEnemies()
{
    enemies.clear();
    StateGameObject *northPatrol = AddEnemyToWorld(Vector3(-20.0f, -4.0f, -40.0f));
    SetupEnemyStateMachine(northPatrol, Vector3(-60.0f, -4.0f, -40.0f), Vector3(10.0f, -4.0f, -40.0f), 35.0f);
    if (northPatrol)
    {
        enemies.emplace_back(northPatrol);
    }

    StateGameObject *plazaGuard = AddEnemyToWorld(Vector3(25.0f, -4.0f, 5.0f));
    SetupEnemyStateMachine(plazaGuard, Vector3(10.0f, -4.0f, -5.0f), Vector3(55.0f, -4.0f, 25.0f), 30.0f);
    if (plazaGuard)
    {
        enemies.emplace_back(plazaGuard);
    }
}

void TutorialGame::SetupEnemyStateMachine(StateGameObject *enemy, const Vector3 &patrolStart, const Vector3 &patrolEnd, float alertRadius)
{
    if (!enemy || !enemy->GetStateMachine())
    {
        return;
    }

    struct EnemyAIContext
    {
        Vector3 start;
        Vector3 end;
        bool toEnd = true;
        float timeSinceSeen = 10.0f;
        float chaseMemory = 1.5f;
        float patrolSpeed = 14.0f;
        float chaseSpeed = 26.0f;
        float steeringGain = 1.2f;
    };
    auto context = std::make_shared<EnemyAIContext>();
    context->start = patrolStart;
    context->end = patrolEnd;

    State *patrolState = new State([this, enemy, context](float dt)
                                   {
        context->timeSinceSeen += dt;
        if (!enemy->GetPhysicsObject()) {
            return;
        }
        Vector3 target = context->toEnd ? context->end : context->start;
        Vector3 direction = target - enemy->GetTransform().GetPosition();
        direction.y = 0.0f;
        if (Vector::Length(direction) < 1.0f) {
            context->toEnd = !context->toEnd;
            return;
        }
        direction = Vector::Normalise(direction);
        Vector3 desiredVel = direction * context->patrolSpeed;
        Vector3 steering = desiredVel - enemy->GetPhysicsObject()->GetLinearVelocity();
        enemy->GetPhysicsObject()->AddForce(steering * context->steeringGain); });

    State *chaseState = new State([this, enemy, context](float dt)
                                  {
        context->timeSinceSeen += dt;
        if (!playerObject || !enemy->GetPhysicsObject()) {
            return;
        }
        Vector3 toPlayer = playerObject->GetTransform().GetPosition() - enemy->GetTransform().GetPosition();
        toPlayer.y = 0.0f;
        if (Vector::LengthSquared(toPlayer) < 0.0001f) {
            return;
        }
        toPlayer = Vector::Normalise(toPlayer);
        Vector3 desiredVel = toPlayer * context->chaseSpeed;
        Vector3 steering = desiredVel - enemy->GetPhysicsObject()->GetLinearVelocity();
        enemy->GetPhysicsObject()->AddForce(steering * (context->steeringGain * 1.3f)); });

    std::function<bool()> canSeePlayer = [this, enemy, alertRadius, context]()
    {
        if (!playerObject)
        {
            return false;
        }
        Vector3 toPlayer = playerObject->GetTransform().GetPosition() - enemy->GetTransform().GetPosition();
        float distance = Vector::Length(toPlayer);
        if (distance > alertRadius)
        {
            return false;
        }
        toPlayer = Vector::Normalise(toPlayer);
        Ray sightRay(enemy->GetTransform().GetPosition(), toPlayer);
        RayCollision sightHit;
        if (world.Raycast(sightRay, sightHit, true, enemy))
        {
            if (sightHit.node == playerObject)
            {
                context->timeSinceSeen = 0.0f;
                return true;
            }
        }
        return false;
    };

    enemy->GetStateMachine()->AddState(patrolState);
    enemy->GetStateMachine()->AddState(chaseState);
    std::function<bool()> lostSight = [context]()
    {
        return context->timeSinceSeen > context->chaseMemory;
    };

    enemy->GetStateMachine()->AddTransition(new StateTransition(patrolState, chaseState, canSeePlayer));
    enemy->GetStateMachine()->AddTransition(new StateTransition(chaseState, patrolState, lostSight));
}

void TutorialGame::InitGameExamples()
{
    AddPlayerToWorld(Vector3(0, 5, 0));
    AddEnemyToWorld(Vector3(5, 5, 0));
    AddBonusToWorld(Vector3(10, 5, 0));
}

void TutorialGame::CreateSphereGrid(int numRows, int numCols, float rowSpacing, float colSpacing, float radius)
{
    for (int x = 0; x < numCols; ++x)
    {
        for (int z = 0; z < numRows; ++z)
        {
            Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
            AddSphereToWorld(position, radius, 1.0f);
        }
    }
    AddFloorToWorld(Vector3(0, -2, 0));
}

void TutorialGame::CreatedMixedGrid(int numRows, int numCols, float rowSpacing, float colSpacing)
{
    float sphereRadius = 1.0f;
    Vector3 cubeDims = Vector3(1, 1, 1);

    for (int x = 0; x < numCols; ++x)
    {
        for (int z = 0; z < numRows; ++z)
        {
            Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

            if (rand() % 2)
            {
                AddCubeToWorld(position, cubeDims);
            }
            else
            {
                AddSphereToWorld(position, sphereRadius);
            }
        }
    }
}

void TutorialGame::CreateAABBGrid(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3 &cubeDims)
{
    for (int x = 1; x < numCols + 1; ++x)
    {
        for (int z = 1; z < numRows + 1; ++z)
        {
            Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
            AddCubeToWorld(position, cubeDims, 1.0f);
        }
    }
}

/*
Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around.

*/
bool TutorialGame::SelectObject()
{
    if (mouseLookActive)
    {
        Debug::Print("Press Q to exit first-person before interacting", Vector2(5, 80), Debug::WHITE);
        return false;
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::Z))
    {
        inSelectionMode = !inSelectionMode;
        if (inSelectionMode)
        {
            Window::GetWindow()->ShowOSPointer(true);
            Window::GetWindow()->LockMouseToWindow(false);
        }
        else
        {
            Window::GetWindow()->ShowOSPointer(false);
            Window::GetWindow()->LockMouseToWindow(true);
        }
    }
    if (inSelectionMode)
    {
        Debug::Print("Press Z to change to camera mode!", Vector2(5, 85));

        if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::Left))
        {
            if (selectionObject)
            { // set colour to deselected;
                if (selectionObject->GetRenderObject())
                {
                    selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
                }
                selectionObject = nullptr;
            }

            Ray ray = CollisionDetection::BuildRayFromMouse(world.GetMainCamera());

            RayCollision closestCollision;
            if (world.Raycast(ray, closestCollision, true))
            {
                GameObject *candidate = reinterpret_cast<GameObject *>(closestCollision.node);
                if (candidate && candidate->GetPhysicsObject())
                {
                    selectionObject = candidate;
                    if (selectionObject->GetRenderObject())
                    {
                        selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
                    }
                }
                return selectionObject != nullptr;
            }
            else
            {
                return false;
            }
        }
        if (Window::GetKeyboard()->KeyPressed(NCL::KeyCodes::L))
        {
            if (selectionObject)
            {
                if (lockedObject == selectionObject)
                {
                    lockedObject = nullptr;
                }
                else
                {
                    lockedObject = selectionObject;
                }
            }
        }
    }
    else
    {
        Debug::Print("Press Z to change to select mode!", Vector2(5, 85));
    }
    return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/

void TutorialGame::MoveSelectedObject()
{
    Debug::Print("Click Force:" + std::to_string(forceMagnitude), Vector2(5, 90));
    forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

    if (!selectionObject || !selectionObject->GetPhysicsObject())
    {
        return; // we haven't selected anything!
    }
    // Push the selected object!
    if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::Right))
    {
        Ray ray = CollisionDetection::BuildRayFromMouse(world.GetMainCamera());

        RayCollision closestCollision;
        if (world.Raycast(ray, closestCollision, true))
        {
            if (closestCollision.node == selectionObject)
            {
                selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
            }
        }
    }
}

void TutorialGame::LockedObjectMovement()
{
    if (!lockedObject || !lockedObject->GetPhysicsObject())
    {
        return;
    }
    Matrix4 view = world.GetMainCamera().BuildViewMatrix();
    Matrix4 camWorld = Matrix::Inverse(view);

    Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); // view is inverse of model!

    // forward is more tricky -  camera forward is 'into' the screen...
    // so we can take a guess, and use the cross of straight up, and
    // the right axis, to hopefully get a vector that's good enough!

    Vector3 fwdAxis = Vector::Cross(Vector3(0, 1, 0), rightAxis);
    fwdAxis.y = 0.0f;
    fwdAxis = Vector::Normalise(fwdAxis);

    if (Window::GetKeyboard()->KeyDown(KeyCodes::UP))
    {
        lockedObject->GetPhysicsObject()->AddForce(fwdAxis);
    }

    if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN))
    {
        lockedObject->GetPhysicsObject()->AddForce(-fwdAxis);
    }

    if (Window::GetKeyboard()->KeyDown(KeyCodes::NEXT))
    {
        lockedObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
    }
}

void TutorialGame::DebugObjectMovement()
{
    // If we've selected an object, we can manipulate it with some key presses
    if (inSelectionMode && selectionObject && selectionObject->GetPhysicsObject())
    {
        // Twist the selected object!
        if (Window::GetKeyboard()->KeyDown(KeyCodes::LEFT))
        {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT))
        {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM7))
        {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM8))
        {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT))
        {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::UP))
        {
            selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN))
        {
            selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM5))
        {
            selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
        }
    }
}

void TutorialGame::UpdatePlayerInput(float dt)
{
    if (!playerObject || !playerObject->GetPhysicsObject())
    {
        return;
    }

    if (!mouseLookActive)
    {
        return;
    }

	float forwardInput = controller->GetNamedAxis("Forward"); //对应W和S键
	float strafeInput = controller->GetNamedAxis("Sidestep"); //对应A和D键
    float verticalInput = controller->GetNamedAxis("UpDown");
    Vector3 planarInput = Vector3(strafeInput, 0.0f, forwardInput);
    float planarLength = Vector::Length(planarInput);
    if (planarLength > 1.0f)
    {
        planarInput /= planarLength;
    }

    Matrix4 view = world.GetMainCamera().BuildViewMatrix();
    Matrix4 camWorld = Matrix::Inverse(view);
    Vector3 rightAxis = Vector3(camWorld.GetColumn(0));
    rightAxis.y = 0.0f;
    rightAxis = Vector::Normalise(rightAxis);
    Vector3 forwardAxis = Vector::Cross(Vector3(0, 1, 0), rightAxis);
    forwardAxis = Vector::Normalise(forwardAxis);

    Vector3 moveForce = (forwardAxis * planarInput.z) + (rightAxis * planarInput.x);
    moveForce *= playerMoveForce;
    if (Vector::LengthSquared(moveForce) > 0.0f)
    {
        playerObject->GetPhysicsObject()->AddForce(moveForce);
    }

    float verticalForce = verticalInput * playerVerticalForce;
    if (std::abs(verticalForce) > 0.0f)
    {
        playerObject->GetPhysicsObject()->AddForce(Vector3(0.0f, verticalForce, 0.0f));
    }

    Ray ray(playerObject->GetTransform().GetPosition(), Vector3(0, -1, 0));
    RayCollision collision;
    if (world.Raycast(ray, collision, true, playerObject))
    {
        if (collision.rayDistance < 3.0f)
        {
            jumpCount = 0;
        }
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE))
    {
        if (jumpCount < 2)
        {
            playerObject->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0.0f, playerJumpImpulse, 0.0f));
            jumpCount++;
        }
    }
}

void TutorialGame::StabiliseDynamicCharacters(float dt)
{
    // StabiliseCharacter(playerObject, dt, 10.0f, playerTurnSpeed);
    if (playerObject && playerObject->GetPhysicsObject())
    {
        PhysicsObject *phys = playerObject->GetPhysicsObject();
        Vector3 angVel = phys->GetAngularVelocity();
        bool trimmedTilt = false;
        if (std::abs(angVel.x) > 0.01f)
        {
            angVel.x = 0.0f;
            trimmedTilt = true;
        }
        if (std::abs(angVel.z) > 0.01f)
        {
            angVel.z = 0.0f;
            trimmedTilt = true;
        }
        angVel.y *= 0.75f;
        if (trimmedTilt)
        {
            phys->SetAngularVelocity(angVel);
        }

        if (!mouseLookActive)
        {
            Quaternion current = playerObject->GetTransform().GetOrientation();
            Vector3 euler = current.ToEuler();
            Quaternion target = Quaternion::EulerAnglesToQuaternion(0.0f, euler.y, 0.0f);
            float alignRate = std::clamp(dt * 10.0f, 0.0f, 1.0f);
            Quaternion stabilised = Quaternion::Slerp(current, target, alignRate);
            playerObject->GetTransform().SetOrientation(stabilised);
        }
    }

    for (auto *enemy : enemies)
    {
        StabiliseCharacter(enemy, dt, 7.5f, enemyTurnSpeed);
    }
}

void TutorialGame::StabiliseCharacter(GameObject *object, float dt, float alignStrength, float turnSpeed)
{
    if (!object)
    {
        return;
    }
    PhysicsObject *phys = object->GetPhysicsObject();
    if (!phys)
    {
        return;
    }
    Vector3 angVel = phys->GetAngularVelocity();
    bool trimmedTilt = false;
    if (std::abs(angVel.x) > 0.01f)
    {
        angVel.x = 0.0f;
        trimmedTilt = true;
    }
    if (std::abs(angVel.z) > 0.01f)
    {
        angVel.z = 0.0f;
        trimmedTilt = true;
    }
    angVel.y *= 0.75f; // slightly damp spin
    if (trimmedTilt)
    {
        phys->SetAngularVelocity(angVel);
    }
    Quaternion current = object->GetTransform().GetOrientation();
    Vector3 euler = current.ToEuler();
    Quaternion target = Quaternion::EulerAnglesToQuaternion(0.0f, euler.y, 0.0f);
    float alignRate = std::clamp(dt * alignStrength, 0.0f, 1.0f);
    Quaternion stabilised = Quaternion::Slerp(current, target, alignRate);
    object->GetTransform().SetOrientation(stabilised);

    Vector3 planarVelocity = phys->GetLinearVelocity();
    planarVelocity.y = 0.0f;
    if (Vector::LengthSquared(planarVelocity) > 0.05f)
    {
        AlignObjectToDirection(object, planarVelocity, dt, turnSpeed);
    }
}

void TutorialGame::AlignObjectToDirection(GameObject *object, const Vector3 &direction, float dt, float turnSpeed)
{
    if (!object)
    {
        return;
    }
    Vector3 planarDir = direction;
    planarDir.y = 0.0f;
    if (Vector::LengthSquared(planarDir) < 0.0001f)
    {
        return;
    }
    planarDir = Vector::Normalise(planarDir);
    float yawRad = std::atan2(planarDir.x, planarDir.z);
    float yawDeg = Maths::RadiansToDegrees(yawRad);
    yawDeg += 180.0f;
    if (yawDeg > 180.0f)
    {
        yawDeg -= 360.0f;
    }
    Quaternion current = object->GetTransform().GetOrientation();
    Quaternion target = Quaternion::EulerAnglesToQuaternion(0.0f, yawDeg, 0.0f);
    float blend = std::clamp(dt * turnSpeed, 0.0f, 1.0f);
    Quaternion rotated = Quaternion::Slerp(current, target, blend);
    object->GetTransform().SetOrientation(rotated);
}

void TutorialGame::HandleRayInteractions()
{
    if (!playerObject)
    {
        return;
    }
    Matrix4 view = world.GetMainCamera().BuildViewMatrix();
    Matrix4 camWorld = Matrix::Inverse(view);
    Vector3 camPos = world.GetMainCamera().GetPosition();
    Vector3 forward = Vector3(-camWorld.GetColumn(2));
    forward = Vector::Normalise(forward);

    Ray scanRay(camPos, forward);
    RayCollision hit;
    if (world.Raycast(scanRay, hit, true, playerObject))
    {
        float drawDist = std::min(hit.rayDistance, 120.0f);
        Vector3 hitPos = camPos + forward * drawDist;
        Debug::DrawLine(camPos, hitPos, Vector4(1, 1, 0, 1), 0.5f);
        GameObject *hitObj = (GameObject *)hit.node;
        if (!hitObj)
        {
            return;
        }
        if (auto parcel = dynamic_cast<FragileParcel *>(hitObj))
        {
            if (parcel->GetPhysicsObject())
            {
                parcel->GetPhysicsObject()->AddForce(Vector3(0.0f, 150.0f, 0.0f));
            }
            Debug::Print("Parcel scanned", Vector2(60, 10), Debug::WHITE);
        }
        else if (auto enemy = dynamic_cast<StateGameObject *>(hitObj))
        {
            if (enemy->GetPhysicsObject())
            {
                enemy->GetPhysicsObject()->AddForce(-forward * 500.0f);
            }
            Debug::Print("Enemy distracted", Vector2(60, 15), Debug::YELLOW);
        }
    }
}

void TutorialGame::UpdateCarriedParcel(float dt)
{
    parcelPickupCooldown = std::max(0.0f, parcelPickupCooldown - dt);
    if (!playerObject)
    {
        return;
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::E) && parcelPickupCooldown <= 0.0f)
    {
        if (carriedParcel)
        {
            DropCarriedParcel();
        }
        else
        {
            PickupParcel(FindNearestParcel(parcelPickupRange));
        }
    }

    if (!carriedParcel)
    {
        return;
    }

    Matrix4 camWorld = Matrix::Inverse(world.GetMainCamera().BuildViewMatrix());
    Vector3 right = Vector3(camWorld.GetColumn(0));
    Vector3 forward = Vector3(-camWorld.GetColumn(2));
    Vector3 up = Vector3(0, 1, 0);
    right.y = 0.0f;
    forward.y = 0.0f;
    if (Vector::LengthSquared(right) > 0.0f)
    {
        right = Vector::Normalise(right);
    }
    if (Vector::LengthSquared(forward) > 0.0f)
    {
        forward = Vector::Normalise(forward);
    }
    Vector3 basePos = playerObject->GetTransform().GetPosition();
    Vector3 target = basePos + (right * parcelCarryOffset.x) + (up * parcelCarryOffset.y) + (forward * parcelCarryOffset.z);
    carriedParcel->GetTransform().SetPosition(target);
    if (auto phys = carriedParcel->GetPhysicsObject())
    {
        phys->SetLinearVelocity(Vector3());
        phys->ClearForces();
    }
}

FragileParcel *TutorialGame::FindNearestParcel(float radius) const
{
    if (parcels.empty() || !playerObject)
    {
        return nullptr;
    }
    FragileParcel *result = nullptr;
    float bestDistSq = radius * radius;
    Vector3 playerPos = playerObject ? playerObject->GetTransform().GetPosition() : Vector3();
    for (auto *parcel : parcels)
    {
        if (!parcel || parcel->IsBroken() || parcel->IsDelivered())
        {
            continue;
        }
        Vector3 diff = parcel->GetTransform().GetPosition() - playerPos;
        float distSq = Vector::LengthSquared(diff);
        if (distSq <= bestDistSq)
        {
            bestDistSq = distSq;
            result = parcel;
        }
    }
    return result;
}

void TutorialGame::PickupParcel(FragileParcel *parcel)
{
    if (!parcel || carriedParcel)
    {
        return;
    }
    if (parcel->IsBroken() || parcel->IsDelivered())
    {
        return;
    }
    if (auto phys = parcel->GetPhysicsObject())
    {
        phys->SetInverseMass(0.0f);
        phys->SetLinearVelocity(Vector3());
        phys->ClearForces();
    }
    parcel->SetCollisionEnabled(false);
    carriedParcel = parcel;
    parcelPickupCooldown = 0.25f;
}

void TutorialGame::DropCarriedParcel(bool immediate)
{
    if (!carriedParcel)
    {
        return;
    }
    carriedParcel->SetCollisionEnabled(true);
    if (auto phys = carriedParcel->GetPhysicsObject())
    {
        phys->SetInverseMass(parcelDefaultInverseMass);
        if (!immediate)
        {
            Matrix4 camWorld = Matrix::Inverse(world.GetMainCamera().BuildViewMatrix());
            Vector3 forward = Vector3(-camWorld.GetColumn(2));
            if (Vector::LengthSquared(forward) > 0.0f)
            {
                forward = Vector::Normalise(forward);
            }
            phys->ApplyLinearImpulse((forward * 12.0f) + Vector3(0.0f, 9.0f, 0.0f));
        }
    }
    carriedParcel = nullptr;
    parcelPickupCooldown = 0.35f;
}

void TutorialGame::UpdateHUD(float dt)
{
    (void)dt;
    std::string timerText = "Time: " + std::to_string((int)std::ceil(std::max(0.0f, gameTimer)));
    std::string parcelText = "Parcels delivered: " + std::to_string(deliveredParcels) + "/" + std::to_string(std::max(1, totalParcels));
    Debug::Print("Score: " + std::to_string(playerScore), Vector2(5, 5), Debug::WHITE);
    Debug::Print(parcelText, Vector2(5, 10), Debug::WHITE);
    Debug::Print(timerText, Vector2(5, 15), Debug::WHITE);
    Debug::Print("Middle mouse: Courier scan", Vector2(5, 20), Debug::WHITE);
    Debug::Print("[Q] Toggle Ghost / first-person view", Vector2(5, 25), Debug::WHITE);
    Debug::Print("Orange crates can be pushed to open paths", Vector2(5, 30), Debug::YELLOW);
}

void TutorialGame::UpdateDeliveryIndicator(float dt)
{
    deliveryIndicatorTimer += dt;
    if (!deliveryZone)
    {
        return;
    }

    float pulse = 0.5f + 0.5f * std::sin(deliveryIndicatorTimer * 2.5f);
    Vector3 zonePos = deliveryZone->GetTransform().GetPosition();
    Vector3 base = zonePos;
    Vector3 top = zonePos + Vector3(0.0f, 12.0f + 4.0f * pulse, 0.0f);
    Vector4 beamColour(0.2f + 0.5f * pulse, 0.95f, 0.4f + 0.4f * pulse, 1.0f);

    Debug::DrawLine(base, top, beamColour, 0.0f);
    Debug::DrawLine(top + Vector3(-2.0f, 0.0f, 0.0f), top + Vector3(2.0f, 0.0f, 0.0f), beamColour, 0.0f);
    Debug::DrawLine(top + Vector3(0.0f, 0.0f, -2.0f), top + Vector3(0.0f, 0.0f, 2.0f), beamColour, 0.0f);

    if (deliveryZoneHalfSize.x > 0.0f && deliveryZoneHalfSize.z > 0.0f)
    {
        float outlineScale = 1.3f;
        Vector3 halfOutline = Vector3(deliveryZoneHalfSize.x * outlineScale, 0.0f, deliveryZoneHalfSize.z * outlineScale);
        Vector3 corners[4] = {
            zonePos + Vector3(-halfOutline.x, 0.05f, -halfOutline.z),
            zonePos + Vector3(halfOutline.x, 0.05f, -halfOutline.z),
            zonePos + Vector3(halfOutline.x, 0.05f, halfOutline.z),
            zonePos + Vector3(-halfOutline.x, 0.05f, halfOutline.z)};
        for (int i = 0; i < 4; ++i)
        {
            Vector3 start = corners[i];
            Vector3 end = corners[(i + 1) % 4];
            Debug::DrawLine(start, end, beamColour, 0.0f);
        }
    }

    if (deliveryMarker && deliveryMarker->GetRenderObject())
    {
        Vector4 markerColour(0.2f + 0.4f * pulse, 0.9f + 0.1f * pulse, 0.45f, 0.7f);
        deliveryMarker->GetRenderObject()->SetColour(markerColour);
    }
    if (deliveryPad && deliveryPad->GetRenderObject())
    {
        Vector4 padColour(0.05f, 0.3f + 0.4f * pulse, 0.1f + 0.25f * pulse, 0.85f);
        deliveryPad->GetRenderObject()->SetColour(padColour);
    }
}

void TutorialGame::UpdateGhostCamera(float dt)
{
    auto &camera = world.GetMainCamera();
    const float sensitivity = firstPersonLookSensitivity;
    float targetPitch = camera.GetPitch() - controller->GetNamedAxis("YLook") * sensitivity;
    targetPitch = std::clamp(targetPitch, -85.0f, 85.0f);
    float targetYaw = camera.GetYaw() - controller->GetNamedAxis("XLook") * sensitivity;
    if (targetYaw < 0.0f)
    {
        targetYaw += 360.0f;
    }
    else if (targetYaw > 360.0f)
    {
        targetYaw -= 360.0f;
    }
    camera.SetPitch(targetPitch);
    camera.SetYaw(targetYaw);

    float forwardInput = controller->GetNamedAxis("Forward");
    float strafeInput = controller->GetNamedAxis("Sidestep");
    float verticalInput = controller->GetNamedAxis("UpDown");

    float speed = 30.0f * dt;

    Matrix4 view = camera.BuildViewMatrix();
    Matrix4 camWorld = Matrix::Inverse(view);

    Vector3 rightAxis = Vector3(camWorld.GetColumn(0));
    Vector3 forwardAxis = Vector3(camWorld.GetColumn(2));

    Vector3 moveDir = (forwardAxis * -forwardInput) + (rightAxis * strafeInput);
    moveDir.y += verticalInput;

    camera.SetPosition(camera.GetPosition() + moveDir * speed);
}

void TutorialGame::SetMouseLookActive(bool enabled)
{
    if (mouseLookActive == enabled)
    {
        return;
    }
    mouseLookActive = enabled;
    if (mouseLookActive)
    {
        inSelectionMode = false;
        Window::GetWindow()->ShowOSPointer(false);
        Window::GetWindow()->LockMouseToWindow(true);
        if (playerObject)
        {
            Vector3 euler = playerObject->GetTransform().GetOrientation().ToEuler();
            world.GetMainCamera().SetYaw(euler.y);
            world.GetMainCamera().SetPitch(0.0f);
        }
    }
    else if (!showMenu && !inSelectionMode)
    {
        Window::GetWindow()->ShowOSPointer(false);
        Window::GetWindow()->LockMouseToWindow(true);
    }
}

void TutorialGame::DrawHighScorePanel(const Vector2 &origin, float fontSize) const
{
    Debug::Print("=== Courier Leaderboard ===", origin, Debug::YELLOW, fontSize);
    const auto &entries = scoreboardManager.GetDisplayScores();
    if (entries.empty())
    {
        Debug::Print("No deliveries recorded", Vector2(origin.x, origin.y + 5.0f), Debug::WHITE, fontSize);
        return;
    }
    int limit = std::min<int>(static_cast<int>(entries.size()), 8);
    for (int i = 0; i < limit; ++i)
    {
        const auto &entry = entries[i];
        std::string line = std::to_string(i + 1) + ". " + entry.name + " - " + std::to_string(entry.score);
        Debug::Print(line, Vector2(origin.x, origin.y + 5.0f * (i + 1)), Debug::WHITE, fontSize);
    }
}

void TutorialGame::UpdateScoreboardSystem(float dt)
{
    scoreboardManager.Update(dt);
    if (scoreboardStatusTimer > 0.0f)
    {
        scoreboardStatusTimer = std::max(0.0f, scoreboardStatusTimer - dt);
        if (scoreboardStatusTimer <= 0.0f && scoreboardManager.GetState() == ScoreboardConnectionState::Offline)
        {
            scoreboardStatusMessage = "Scoreboard offline";
        }
    }
}

void TutorialGame::SetScoreboardStatus(const std::string &message)
{
    scoreboardStatusMessage = message;
    scoreboardStatusTimer = 5.0f;
}

void TutorialGame::SubmitScoreIfNeeded()
{
    if (scoreSubmittedThisRun)
    {
        return;
    }
    if (!gameWon && !gameLost)
    {
        return;
    }
    scoreboardManager.SubmitScore(playerProfileName, playerScore);
    scoreboardManager.RequestScores();
    scoreSubmittedThisRun = true;
    SetScoreboardStatus("Score submitted: " + std::to_string(playerScore) + " pts");
}

bool TutorialGame::IsInsideDeliveryZone(const Vector3 &point, float verticalTolerance) const
{
    if (!deliveryZone || deliveryZoneHalfSize.x <= 0.0f || deliveryZoneHalfSize.z <= 0.0f)
    {
        return false;
    }
    Vector3 zonePos = deliveryZone->GetTransform().GetPosition();
    Vector3 diff = point - zonePos;
    bool insideHorizontal = std::abs(diff.x) <= deliveryZoneHalfSize.x && std::abs(diff.z) <= deliveryZoneHalfSize.z;
    bool insideVertical = diff.y >= -verticalTolerance && diff.y <= deliveryZoneHalfSize.y + verticalTolerance;
    return insideHorizontal && insideVertical;
}

void TutorialGame::UpdateParcelDeliveryChecks()
{
    if (!deliveryZone)
    {
        return;
    }
    for (auto *parcel : parcels)
    {
        if (!parcel || parcel == carriedParcel)
        {
            continue;
        }
        if (parcel->IsBroken() || parcel->IsDelivered())
        {
            continue;
        }
        if (IsInsideDeliveryZone(parcel->GetTransform().GetPosition(), 2.0f))
        {
            parcel->MarkDelivered();
        }
    }
}

void TutorialGame::UpdateGameState(float dt)
{
    if (!playerObject)
    {
        return;
    }

    if (!gameWon && !gameLost)
    {
        gameTimer -= dt;
        if (gameTimer <= 0.0f)
        {
            gameTimer = 0.0f;
            gameLost = true;
            showMenu = true;
        }
    }

    Vector3 playerPos = playerObject->GetTransform().GetPosition();
    if (playerPos.y < -40.0f)
    {
        playerObject->GetTransform().SetPosition(playerSpawnPosition);
        if (playerObject->GetPhysicsObject())
        {
            playerObject->GetPhysicsObject()->SetLinearVelocity(Vector3());
            playerObject->GetPhysicsObject()->ClearForces();
        }
        if (carriedParcel)
        {
            DropCarriedParcel(true);
        }
        playerPos = playerSpawnPosition;
    }

    for (auto *enemy : enemies)
    {
        if (!enemy)
        {
            continue;
        }
        Vector3 diff = enemy->GetTransform().GetPosition() - playerPos;
        float distance = Vector::Length(diff);
        if (distance < 3.0f)
        {
            playerObject->GetTransform().SetPosition(playerSpawnPosition);
            if (playerObject->GetPhysicsObject())
            {
                playerObject->GetPhysicsObject()->SetLinearVelocity(Vector3());
                playerObject->GetPhysicsObject()->ClearForces();
            }
            playerScore = std::max(0, playerScore - 25);
            playerPos = playerSpawnPosition;
            if (carriedParcel)
            {
                DropCarriedParcel(true);
            }
        }
    }
}

void TutorialGame::ResetGameplay()
{
    InitWorld();
    showMenu = false;
    inSelectionMode = false;
    SetMouseLookActive(false);
    selectionObject = nullptr;
    scoreSubmittedThisRun = false;
    showHighScorePanel = false;
    Window::GetWindow()->ShowOSPointer(false);
    Window::GetWindow()->LockMouseToWindow(true);
    jumpCount = 0;
}

void TutorialGame::StartNewRun()
{
    ResetGameplay();
}

void TutorialGame::OnParcelBroken(FragileParcel &parcel)
{
    (void)parcel;
    if (carriedParcel == &parcel)
    {
        DropCarriedParcel(true);
    }
    playerScore = std::max(0, playerScore - 100);
    gameLost = true;
    showMenu = true;
}

void TutorialGame::OnParcelDelivered(FragileParcel &parcel)
{
    (void)parcel;
    if (carriedParcel == &parcel)
    {
        DropCarriedParcel(true);
    }
    ++deliveredParcels;
    playerScore += 150;
    if (deliveredParcels >= totalParcels && totalParcels > 0)
    {
        gameWon = true;
        showMenu = true;
    }
}

void TutorialGame::UpdateThirdPersonCamera(float dt)
{
    if (!playerObject)
    {
        return;
    }

    auto &camera = world.GetMainCamera();
    const float sensitivity = firstPersonLookSensitivity;
    float targetPitch = camera.GetPitch() - controller->GetNamedAxis("YLook") * sensitivity;
    targetPitch = std::clamp(targetPitch, -85.0f, 85.0f);
    float targetYaw = camera.GetYaw() - controller->GetNamedAxis("XLook") * sensitivity;
    if (targetYaw < 0.0f)
    {
        targetYaw += 360.0f;
    }
    else if (targetYaw > 360.0f)
    {
        targetYaw -= 360.0f;
    }
    camera.SetPitch(targetPitch);
    camera.SetYaw(targetYaw);

    float distance = 20.0f;
    Vector3 playerPos = playerObject->GetTransform().GetPosition();
    Vector3 camOffset = Quaternion::EulerAnglesToQuaternion(targetPitch, targetYaw, 0.0f) * Vector3(0, 0, distance);
    Vector3 camPos = playerPos + Vector3(0, 5, 0) + camOffset;

    camera.SetPosition(camPos);

    // Force upright orientation and smooth yaw interpolation
    float currentYaw = playerObject->GetTransform().GetOrientation().ToEuler().y;
    float diff = targetYaw - currentYaw;
    if (diff > 180.0f)
        diff -= 360.0f;
    if (diff < -180.0f)
        diff += 360.0f;

    float lerpFactor = std::clamp(dt * 10.0f, 0.0f, 1.0f);
    float newYaw = currentYaw + diff * lerpFactor;

    playerObject->GetTransform().SetOrientation(Quaternion::EulerAnglesToQuaternion(0.0f, newYaw, 0.0f));

    // Zero out angular velocity to prevent physics from fighting the orientation
    if (playerObject->GetPhysicsObject())
    {
        Vector3 angVel = playerObject->GetPhysicsObject()->GetAngularVelocity();
        angVel.x = 0;
        angVel.z = 0;
        playerObject->GetPhysicsObject()->SetAngularVelocity(angVel);
    }
}

void TutorialGame::UpdateNavigationPath(float dt)
{
    if (!navGrid || !playerObject)
    {
        return;
    }

    Vector3 startPos = playerObject->GetTransform().GetPosition();
    Vector3 endPos;

    if (carriedParcel)
    {
        if (deliveryZone)
        {
            endPos = deliveryZone->GetTransform().GetPosition();
        }
        else
        {
            return;
        }
    }
    else
    {
        FragileParcel *nearest = FindNearestParcel(9999.0f);
        if (nearest)
        {
            endPos = nearest->GetTransform().GetPosition();
        }
        else
        {
            return;
        }
    }

    Vector3 localStart = startPos - mazeOffset;
    Vector3 localEnd = endPos - mazeOffset;

    NavigationPath outPath;
    if (navGrid->FindPath(localStart, localEnd, outPath))
    {
        Vector3 pos;
        std::vector<Vector3> waypoints;
        while (outPath.PopWaypoint(pos))
        {
            waypoints.push_back(pos + mazeOffset + Vector3(0, 2.0f, 0));
        }

        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            Debug::DrawLine(waypoints[i - 1], waypoints[i], Vector4(0, 1, 0, 1));
        }
    }
}
