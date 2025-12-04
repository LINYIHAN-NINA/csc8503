#include "MenuState.h"
#include "TutorialGame.h"
#include "ScoreboardManager.h"
#include "GameWorld.h"
#include "GameTechRendererInterface.h"
#include "PhysicsSystem.h"
#include "Debug.h"
#include "Window.h"

using namespace NCL::CSC8503;

MenuState::MenuState(std::unique_ptr<TutorialGame> singlePlayerGame,
                     std::unique_ptr<GameWorld> inWorld,
                     std::unique_ptr<GameTechRendererInterface> inRenderer,
                     std::unique_ptr<PhysicsSystem> inPhysics,
                     ScoreboardManager &inScoreboard)
    : gameInstance(std::move(singlePlayerGame)),
      world(std::move(inWorld)),
      renderer(std::move(inRenderer)),
      physics(std::move(inPhysics)),
      scoreboard(inScoreboard) {}

MenuState::~MenuState() = default;

void MenuState::OnAwake()
{
    showScores = false;
}

PushdownState::PushdownResult MenuState::OnUpdate(float dt, PushdownState **pushFunc)
{
    RenderMenu(dt);
    HandleMenuInput(pushFunc);
    return PushdownResult::NoChange;
}

void MenuState::RenderMenu(float /*dt*/) const
{
    Debug::ClearPrints();
    const float spacing = 4.5f;
    const float horizontalOffset = 42.0f;
    const float menuFontSize = 16.0f;
    const float totalRows = 7.0f; // title + entries + hint space
    const float totalHeight = spacing * totalRows;
    const Vector2 menuOrigin(50.0f - horizontalOffset, 50.0f - totalHeight * 0.5f);
    Debug::Print("Courier Command", menuOrigin, Debug::YELLOW, menuFontSize);
    Debug::Print("1 - Start Delivery", menuOrigin + Vector2(0.0f, spacing * 1), Debug::WHITE, menuFontSize);
    Debug::Print("2 - Host Coop", menuOrigin + Vector2(0.0f, spacing * 2), Debug::WHITE, menuFontSize);
    Debug::Print("3 - Join Coop", menuOrigin + Vector2(0.0f, spacing * 3), Debug::WHITE, menuFontSize);
    Debug::Print("4 - Show High Scores", menuOrigin + Vector2(0.0f, spacing * 4), Debug::WHITE, menuFontSize);
    Debug::Print("ESC - Quit", menuOrigin + Vector2(0.0f, spacing * 5), Debug::WHITE, menuFontSize);

    if (showScores)
    {
        const auto &scores = scoreboard.GetDisplayScores();
        const Vector2 scoreOrigin(menuOrigin + Vector2(16.0f, 0.0f));
        Debug::Print("-- High Scores --", scoreOrigin, Debug::GREEN, menuFontSize);
        int row = 0;
        for (const auto &entry : scores)
        {
            std::string line = entry.name + " : " + std::to_string(entry.score);
            Debug::Print(line, scoreOrigin + Vector2(0.0f, 3.0f * (row + 1)), Debug::WHITE, menuFontSize);
            row++;
        }
    }
}

void MenuState::HandleMenuInput(PushdownState ** /*newState*/)
{
    auto *keyboard = Window::GetKeyboard();
    if (keyboard->KeyPressed(KeyCodes::NUM1))
    {
        if (gameInstance)
        {
            gameInstance->StartNewRun();
        }
    }
    if (keyboard->KeyPressed(KeyCodes::NUM4))
    {
        scoreboard.RequestScores();
        showScores = !showScores;
    }
}
