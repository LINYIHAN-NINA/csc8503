#include "NetworkBase.h"
#include "./enet/enet.h"
#include <map>
NetworkBase::NetworkBase()
{
    netHandle = nullptr;
}

NetworkBase::~NetworkBase()
{
    if (netHandle)
    {
        enet_host_destroy(netHandle);
    }
}

void NetworkBase::Initialise()
{
    enet_initialize();
}

void NetworkBase::Destroy()
{
    enet_deinitialize();
}

bool NetworkBase::ProcessPacket(GamePacket *packet, int peerID)
{
    if (!packet)
    {
        return false;
    }
    std::multimap<int, PacketReceiver *>::const_iterator first;
    std::multimap<int, PacketReceiver *>::const_iterator last;
    if (!GetPacketHandlers(packet->type, first, last))
    {
        return false;
    }
    for (auto i = first; i != last; ++i)
    {
        i->second->ReceivePacket(packet->type, packet, peerID);
    }
    return true;
}