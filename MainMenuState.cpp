#include "MainMenuState.h"
#include "TutorialGame.h"
#include "ScoreboardManager.h"
#include "Debug.h"
#include "Window.h"

using namespace NCL::CSC8503;

namespace
{
    const char *kMenuItems[] = {
        "Start Solo",
        "Host Coop",
        "Join Coop",
        "High Scores",
        "Quit"};
    constexpr int kMenuItemCount = sizeof(kMenuItems) / sizeof(kMenuItems[0]);
}

MainMenuState::MainMenuState(TutorialGame &inGame, ScoreboardManager &inScoreboard)
    : game(inGame), scoreboard(inScoreboard) {}

void MainMenuState::OnAwake()
{
    selectedIndex = 0;
}

PushdownState::PushdownResult MainMenuState::OnUpdate(float dt, PushdownState **pushFunc)
{
    DrawMenu();
    HandleInput(pushFunc);
    return PushdownResult::NoChange;
}

void MainMenuState::DrawMenu() const
{
    Debug::ClearPrints();
    const float spacing = 5.0f;
    const float horizontalOffset = 40.0f;
    const float menuFontSize = 16.0f;
    const float totalRows = static_cast<float>(kMenuItemCount + 3);
    const float totalHeight = spacing * totalRows;
    const Vector2 menuOrigin(50.0f - horizontalOffset, 50.0f - totalHeight * 0.5f);

    Debug::Print("Courier HQ", menuOrigin, Debug::YELLOW, menuFontSize);
    for (int i = 0; i < kMenuItemCount; ++i)
    {
        Vector4 colour = (i == selectedIndex) ? Debug::GREEN : Debug::WHITE;
        Debug::Print(kMenuItems[i], menuOrigin + Vector2(0.0f, spacing * (i + 1)), colour, menuFontSize);
    }
    Debug::Print("Use Arrow Keys + Enter", menuOrigin + Vector2(0.0f, spacing * (kMenuItemCount + 2)), Debug::WHITE, menuFontSize);
}

void MainMenuState::HandleInput(PushdownState **pushFunc)
{
    auto *keyboard = Window::GetKeyboard();
    if (keyboard->KeyPressed(KeyCodes::UP))
    {
        selectedIndex = (selectedIndex + kMenuItemCount - 1) % kMenuItemCount;
    }
    if (keyboard->KeyPressed(KeyCodes::DOWN))
    {
        selectedIndex = (selectedIndex + 1) % kMenuItemCount;
    }
    if (keyboard->KeyPressed(KeyCodes::RETURN))
    {
        switch (selectedIndex)
        {
        case 0:
            game.StartNewRun();
            break;
        case 1:
            scoreboard.Host(NetworkBase::GetDefaultPort() + 1);
            break;
        case 2:
            scoreboard.Connect(hostAddress, NetworkBase::GetDefaultPort() + 1);
            break;
        case 3:
            scoreboard.RequestScores();
            break;
        case 4:
            *pushFunc = nullptr;
            break;
        default:
            break;
        }
    }
}
