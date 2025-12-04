#pragma once

#include "PushdownState.h"
#include <memory>
#include <string>

namespace NCL::CSC8503
{
    class TutorialGame;
    class ScoreboardManager;

    class MainMenuState : public PushdownState
    {
    public:
        MainMenuState(TutorialGame &game, ScoreboardManager &scoreboard);

        PushdownResult OnUpdate(float dt, PushdownState **pushFunc) override;
        void OnAwake() override;

    private:
        void DrawMenu() const;
        void HandleInput(PushdownState **pushFunc);

        TutorialGame &game;
        ScoreboardManager &scoreboard;
        int selectedIndex = 0;
        std::string hostAddress = "127.0.0.1";
    };
}
