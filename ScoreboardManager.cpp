#include "ScoreboardManager.h"
#include "GameServer.h"
#include "GameClient.h"

#include <algorithm>
#include <cstring>
#include <cstdio>

using namespace NCL::CSC8503;

namespace
{
    void CopyName(char *destination, size_t destSize, const std::string &source)
    {
        if (!destination || destSize == 0)
        {
            return;
        }
        std::snprintf(destination, destSize, "%s", source.c_str());
        destination[destSize - 1] = '\0';
    }
}

ScoreboardManager::ScoreboardManager()
{
    NetworkBase::Initialise();
    storagePath = "highscores.txt";
    LoadLocalScores();
}

ScoreboardManager::~ScoreboardManager()
{
    Disconnect();
}

void ScoreboardManager::Update(float /*dt*/)
{
    if (server)
    {
        server->UpdateServer();
    }
    if (client)
    {
        client->UpdateClient();
    }
}

bool ScoreboardManager::Host(int port)
{
    if (connectionState == ScoreboardConnectionState::Hosting)
    {
        return true;
    }
    server = std::make_unique<GameServer>(port, 8);
    if (!server || !server->IsActive())
    {
        server.reset();
        return false;
    }
    server->RegisterPacketHandler(BasicNetworkMessages::Score_Submit, this);
    server->RegisterPacketHandler(BasicNetworkMessages::Score_Request, this);
    connectionState = ScoreboardConnectionState::Hosting;
    BroadcastScores();
    return true;
}

bool ScoreboardManager::Connect(const std::string &address, int port)
{
    client = std::make_unique<GameClient>();
    int a, b, c, d;
    if (sscanf_s(address.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4)
    {
        return false;
    }
    if (!client->Connect(static_cast<uint8_t>(a), static_cast<uint8_t>(b), static_cast<uint8_t>(c), static_cast<uint8_t>(d), port))
    {
        client.reset();
        return false;
    }
    client->RegisterPacketHandler(BasicNetworkMessages::Score_Data, this);
    if (connectionState != ScoreboardConnectionState::Hosting)
    {
        connectionState = ScoreboardConnectionState::Client;
    }
    hasNetworkScores = false;
    RequestScores();
    return true;
}

void ScoreboardManager::Disconnect()
{
    if (client)
    {
        client.reset();
    }
    if (server)
    {
        server.reset();
    }
    connectionState = ScoreboardConnectionState::Offline;
    hasNetworkScores = false;
    networkScores.clear();
}

void ScoreboardManager::SubmitScore(const std::string &name, int score)
{
    HighScoreEntry entry{name, score};
    if (client)
    {
        ScoreboardEntryPacket packet;
        packet.data.score = score;
        CopyName(packet.data.name, sizeof(packet.data.name), name);
        client->SendPacket(packet);
    }
    else
    {
        localScores.AddScore(entry);
        SaveLocalScores();
        if (server)
        {
            BroadcastScores();
        }
    }
}

void ScoreboardManager::RequestScores()
{
    if (client)
    {
        ScoreboardRequestPacket request;
        client->SendPacket(request);
    }
    else if (server)
    {
        BroadcastScores();
    }
}

const std::vector<HighScoreEntry> &ScoreboardManager::GetDisplayScores() const
{
    if (hasNetworkScores)
    {
        return networkScores;
    }
    return localScores.GetEntries();
}

void ScoreboardManager::ReceivePacket(int type, GamePacket *payload, int /*source*/)
{
    switch (type)
    {
    case BasicNetworkMessages::Score_Submit:
    {
        if (!server)
        {
            break;
        }
        auto *packet = reinterpret_cast<ScoreboardEntryPacket *>(payload);
        HighScoreEntry entry;
        entry.name = packet->data.name;
        entry.score = packet->data.score;
        localScores.AddScore(entry);
        SaveLocalScores();
        BroadcastScores();
        break;
    }
    case BasicNetworkMessages::Score_Request:
    {
        if (server)
        {
            BroadcastScores();
        }
        break;
    }
    case BasicNetworkMessages::Score_Data:
    {
        auto *packet = reinterpret_cast<ScoreboardDataPacket *>(payload);
        networkScores.clear();
        for (int i = 0; i < packet->entryCount && i < ScoreboardDataPacket::MaxEntries; ++i)
        {
            HighScoreEntry entry;
            entry.name = packet->entries[i].name;
            entry.score = packet->entries[i].score;
            networkScores.emplace_back(entry);
        }
        hasNetworkScores = true;
        break;
    }
    default:
        break;
    }
}

void ScoreboardManager::BroadcastScores()
{
    if (!server)
    {
        return;
    }
    ScoreboardDataPacket packet;
    const auto &source = localScores.GetEntries();
    packet.entryCount = static_cast<int>(std::min<std::size_t>(source.size(), ScoreboardDataPacket::MaxEntries));
    for (int i = 0; i < packet.entryCount; ++i)
    {
        packet.entries[i].score = source[i].score;
        CopyName(packet.entries[i].name, sizeof(packet.entries[i].name), source[i].name);
    }
    server->SendGlobalPacket(packet);
}

void ScoreboardManager::SaveLocalScores()
{
    localScores.Save(storagePath);
}

void ScoreboardManager::LoadLocalScores()
{
    localScores.Load(storagePath);
}
