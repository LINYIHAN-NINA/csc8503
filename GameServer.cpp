#include "GameServer.h"
#include "GameWorld.h"
#include "./enet/enet.h"
#include <algorithm>
using namespace NCL;
using namespace CSC8503;

GameServer::GameServer(int onPort, int maxClients)
{
    port = onPort;
    clientMax = maxClients;
    clientCount = 0;
    netHandle = nullptr;
    Initialise();
}

GameServer::~GameServer()
{
    Shutdown();
}

void GameServer::Shutdown()
{
    if (!netHandle)
    {
        return;
    }
    SendGlobalPacket(BasicNetworkMessages::Shutdown);
    enet_host_destroy(netHandle);
    netHandle = nullptr;
}

bool GameServer::Initialise()
{
    ENetAddress address;
    address.host = ENET_HOST_ANY;
    address.port = port;
    netHandle = enet_host_create(&address, clientMax, 1, 0, 0);
    return netHandle != nullptr;
}

bool GameServer::SendGlobalPacket(int msgID)
{
    GamePacket packet(static_cast<short>(msgID));
    return SendGlobalPacket(packet);
}

bool GameServer::SendGlobalPacket(GamePacket &packet)
{
    if (!netHandle)
    {
        return false;
    }
    ENetPacket *dataPacket = enet_packet_create(&packet, packet.GetTotalSize(), ENET_PACKET_FLAG_RELIABLE);
    enet_host_broadcast(netHandle, 0, dataPacket);
    enet_host_flush(netHandle);
    return true;
}

void GameServer::UpdateServer()
{
    if (!netHandle)
    {
        return;
    }
    ENetEvent event;
    while (enet_host_service(netHandle, &event, 0) > 0)
    {
        switch (event.type)
        {
        case ENET_EVENT_TYPE_CONNECT:
            clientCount++;
            break;
        case ENET_EVENT_TYPE_DISCONNECT:
            clientCount = std::max(0, clientCount - 1);
            break;
        case ENET_EVENT_TYPE_RECEIVE:
            ProcessPacket(reinterpret_cast<GamePacket *>(event.packet->data), event.peer ? event.peer->incomingPeerID : -1);
            enet_packet_destroy(event.packet);
            break;
        default:
            break;
        }
    }
}

void GameServer::SetGameWorld(GameWorld &g)
{
    gameWorld = &g;
}