#pragma once

#include "PushdownState.h"
#include <memory>

namespace NCL::CSC8503
{
    class GameWorld;
    class GameTechRendererInterface;
    class PhysicsSystem;
    class TutorialGame;
    class ScoreboardManager;

    class MenuState : public PushdownState
    {
    public:
        MenuState(std::unique_ptr<TutorialGame> singlePlayerGame,
                  std::unique_ptr<GameWorld> world,
                  std::unique_ptr<GameTechRendererInterface> renderer,
                  std::unique_ptr<PhysicsSystem> physics,
                  ScoreboardManager &scoreboard);
        ~MenuState();

        PushdownResult OnUpdate(float dt, PushdownState **pushFunc) override;
        void OnAwake() override;

    private:
        void RenderMenu(float dt) const;
        void HandleMenuInput(PushdownState **newState);

        std::unique_ptr<TutorialGame> gameInstance;
        std::unique_ptr<GameWorld> world;
        std::unique_ptr<GameTechRendererInterface> renderer;
        std::unique_ptr<PhysicsSystem> physics;
        ScoreboardManager &scoreboard;

        bool showScores = false;
    };
}
