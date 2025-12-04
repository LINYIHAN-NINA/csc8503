#include "Window.h"

#include "Debug.h"

#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"

#include "GameServer.h"
#include "GameClient.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "TutorialGame.h"
#include "NetworkedGame.h"

#include "PushdownMachine.h"

#include "PushdownState.h"

#include "BehaviourNode.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "BehaviourAction.h"

#include "PhysicsSystem.h"

#ifdef USEOPENGL
#include "GameTechRenderer.h"
#define CAN_COMPILE
#endif
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#define CAN_COMPILE
#endif

using namespace NCL;
using namespace CSC8503;

#include <chrono>
#include <thread>
#include <sstream>

void TestPathfinding()
{
}

void DisplayPathfinding()
{
}

/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead.

This time, we've added some extra functionality to the window class - we can
hide or show the

*/
int main()
{
    WindowInitialisation initInfo;
    initInfo.width = 1280;
    initInfo.height = 720;
    initInfo.fullScreen = true;
    initInfo.windowTitle = "CSC8503 Game technology!";

	// Create a window 创建窗口
    Window *w = Window::CreateGameWindow(initInfo);

    if (!w->HasInitialised())
    {
        return -1;
    }

	//鼠标设置 Mouse settings
    w->ShowOSPointer(false);
    w->LockMouseToWindow(true);

	GameWorld* world = new GameWorld(); // Create a game world 创建一个游戏世界
	PhysicsSystem* physics = new PhysicsSystem(*world); // Create a physics system 创建一个物理系统

#ifdef USEVULKAN
    GameTechVulkanRenderer *renderer = new GameTechVulkanRenderer(*world);
#elif USEOPENGL
	GameTechRenderer* renderer = new GameTechRenderer(*world); // Create a game renderer 创建一个游戏渲染器
#endif

    TutorialGame *g = new TutorialGame(*world, *renderer, *physics); 

    w->GetTimer().GetTimeDeltaSeconds(); // Clear the timer so we don't get a larget first dt!
	while (w->UpdateWindow() && !Window::GetKeyboard()->KeyDown(KeyCodes::ESCAPE)) // Main game loop 主游戏循环
    {
        float dt = w->GetTimer().GetTimeDeltaSeconds();
        if (dt > 0.1f)
        {
            std::cout << "Skipping large time delta" << std::endl;
            continue; // must have hit a breakpoint or something to have a 1 second frame time!
        }
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::PRIOR))
        {
            w->ShowConsole(true);
        }
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::NEXT))
        {
            w->ShowConsole(false);
        }

        if (Window::GetKeyboard()->KeyPressed(KeyCodes::T))
        {
            w->SetWindowPosition(0, 0);
        }

        if (Window::GetKeyboard()->KeyPressed(KeyCodes::F11))
        {
            w->SetFullScreen(true);
        }
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10))
        {
            w->SetFullScreen(false);
        }

        w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));

        g->UpdateGame(dt);

        world->UpdateWorld(dt);
        physics->Update(dt);
        renderer->Update(dt);
        renderer->Render();

        Debug::UpdateRenderables(dt);
    }
    Window::DestroyGameWindow();
}