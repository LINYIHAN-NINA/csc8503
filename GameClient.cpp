#include "GameClient.h"
#include "./enet/enet.h"
#include <cstdint>
using namespace NCL;
using namespace CSC8503;

GameClient::GameClient()
{
    netHandle = enet_host_create(nullptr, 1, 1, 0, 0);
    netPeer = nullptr;
}

GameClient::~GameClient()
{
    if (netPeer)
    {
        enet_peer_disconnect(netPeer, 0);
        netPeer = nullptr;
    }
    if (netHandle)
    {
        enet_host_destroy(netHandle);
        netHandle = nullptr;
    }
}

bool GameClient::Connect(uint8_t a, uint8_t b, uint8_t c, uint8_t d, int portNum)
{
    if (!netHandle)
    {
        return false;
    }
    ENetAddress address;
    address.port = portNum;
    address.host = (static_cast<uint32_t>(a) << 24) |
                   (static_cast<uint32_t>(b) << 16) |
                   (static_cast<uint32_t>(c) << 8) |
                   static_cast<uint32_t>(d);
    netPeer = enet_host_connect(netHandle, &address, 1, 0);
    if (!netPeer)
    {
        return false;
    }
    ENetEvent event;
    if (enet_host_service(netHandle, &event, 5000) > 0 && event.type == ENET_EVENT_TYPE_CONNECT)
    {
        return true;
    }
    enet_peer_reset(netPeer);
    netPeer = nullptr;
    return false;
}

void GameClient::UpdateClient()
{
    if (!netHandle)
    {
        return;
    }
    ENetEvent event;
    while (enet_host_service(netHandle, &event, 0) > 0)
    {
        if (event.type == ENET_EVENT_TYPE_RECEIVE)
        {
            ProcessPacket(reinterpret_cast<GamePacket *>(event.packet->data), event.peer ? event.peer->incomingPeerID : -1);
            enet_packet_destroy(event.packet);
        }
    }
}

void GameClient::SendPacket(GamePacket &payload)
{
    if (!netPeer)
    {
        return;
    }
    ENetPacket *packet = enet_packet_create(&payload, payload.GetTotalSize(), ENET_PACKET_FLAG_RELIABLE);
    enet_peer_send(netPeer, 0, packet);
    enet_host_flush(netHandle);
}
