#pragma once
#include "HighScoreTable.h"
#include "NetworkBase.h"
#include <memory>
#include <string>

namespace NCL::CSC8503
{
    class GameServer;
    class GameClient;

    enum class ScoreboardConnectionState
    {
        Offline,
        Hosting,
        Client
    };

    inline constexpr int ScoreboardServicePort = 1235;

    struct ScoreboardEntryPacket : public GamePacket
    {
        ScoreboardEntryPacket()
        {
            type = BasicNetworkMessages::Score_Submit;
            size = sizeof(data);
        }
        struct EntryData
        {
            char name[32];
            int score;
        } data;
    };

    struct ScoreboardRequestPacket : public GamePacket
    {
        ScoreboardRequestPacket()
        {
            type = BasicNetworkMessages::Score_Request;
            size = 0;
        }
    };

    struct ScoreboardDataPacket : public GamePacket
    {
        static constexpr int MaxEntries = 10;
        ScoreboardDataPacket()
        {
            type = BasicNetworkMessages::Score_Data;
            size = sizeof(entryCount) + sizeof(entries);
            entryCount = 0;
        }
        int entryCount;
        ScoreboardEntryPacket::EntryData entries[MaxEntries];
    };

    class ScoreboardManager : public PacketReceiver
    {
    public:
        ScoreboardManager();
        ~ScoreboardManager();

        void Update(float dt);

        bool Host(int port = ScoreboardServicePort);
        bool Connect(const std::string &address, int port = ScoreboardServicePort);
        void Disconnect();

        void SubmitScore(const std::string &name, int score);
        void RequestScores();

        ScoreboardConnectionState GetState() const { return connectionState; }
        bool HasNetworkScores() const { return hasNetworkScores; }
        bool IsHosting() const { return connectionState == ScoreboardConnectionState::Hosting; }
        bool HasClient() const { return client != nullptr; }
        const std::vector<HighScoreEntry> &GetDisplayScores() const;
        const std::vector<HighScoreEntry> &GetLocalScores() const { return localScores.GetEntries(); }

        void ReceivePacket(int type, GamePacket *payload, int source) override;

        void SetStoragePath(const std::string &path) { storagePath = path; }

    private:
        void BroadcastScores();
        void SaveLocalScores();
        void LoadLocalScores();

        std::unique_ptr<GameServer> server;
        std::unique_ptr<GameClient> client;
        ScoreboardConnectionState connectionState = ScoreboardConnectionState::Offline;

        HighScoreTable localScores;
        std::vector<HighScoreEntry> networkScores;
        bool hasNetworkScores = false;
        std::string storagePath;
    };
}
