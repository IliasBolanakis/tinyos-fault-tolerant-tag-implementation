/**
 * @file SRTreeAppC.nc
 * @brief Top-level TEP-3 configuration wiring the modular, fault-tolerant
 *        TAG sensor network application (`SRTreeC`, `TreeRoutingP`,
 *        `BackupTableP`, and `DupFilterP`).
 */
#include "SimpleRoutingTree.h"

configuration SRTreeAppC @safe() {
}
implementation {
    // Modular TEP-3 application components
    components SRTreeC, TreeRoutingP, BackupTableP, DupFilterP;

#if defined(DELUGE)
    components DelugeC;
#endif

#ifdef PRINTFDBG_MODE
    components PrintfC;
#endif

    // Basic platform services: boot, radio control, LEDs, and PRNG
    components MainC, ActiveMessageC, LedsC, RandomC;

    // Radio communication channels
    // AM_ROUTINGMSG (22): Tree construction, query dissemination, and local repair
    components new AMSenderC(AM_ROUTINGMSG) as RoutingSenderC;
    components new AMReceiverC(AM_ROUTINGMSG) as RoutingReceiverC;

    // AM_MYMSG (23): Bottom-up data aggregation with MAC-layer ACK support
    components new AMSenderC(AM_MYMSG) as MySenderC;
    components new AMReceiverC(AM_MYMSG) as MyReceiverC;

    // FIFO packet queues for inbound/outbound traffic
    components new PacketQueueC(SENDER_QUEUE_SIZE) as RoutingSendQueueC;
    components new PacketQueueC(RECEIVER_QUEUE_SIZE) as RoutingReceiveQueueC;

    components new PacketQueueC(SENDER_QUEUE_SIZE) as MySendQueueC;
    components new PacketQueueC(RECEIVER_QUEUE_SIZE) as MyReceiveQueueC;

    // Demultiplexed millisecond timers: dissemination, orphan pull, Root election, and TAG epoch slotting
    components new TimerMilliC() as DisseminationTimerC;
    components new TimerMilliC() as OrphanTimerC;
    components new TimerMilliC() as ElectionTimerC;
    components new TimerMilliC() as MyTimerC;

    // Wire platform and PRNG interfaces to data-plane coordinator
    SRTreeC.Boot -> MainC.Boot;
    SRTreeC.RadioControl -> ActiveMessageC;
    SRTreeC.Leds -> LedsC;
    SRTreeC.Random -> RandomC;
    SRTreeC.SeedInit -> RandomC.SeedInit;

    // Wire TAG epoch timer and data-plane AM 23 stack + synchronous MAC ACKs
    SRTreeC.MyTimer -> MyTimerC;
    SRTreeC.MyPacket -> MySenderC.Packet;
    SRTreeC.MyAMPacket -> MySenderC.AMPacket;
    SRTreeC.MyAMSend -> MySenderC.AMSend;
    SRTreeC.MyReceive -> MyReceiverC.Receive;
    SRTreeC.MyPacketAcknowledgements -> ActiveMessageC;
    SRTreeC.MySendQueue -> MySendQueueC;
    SRTreeC.MyReceiveQueue -> MyReceiveQueueC;

    // Wire extracted control-plane and helper modules
    SRTreeC.TreeTopology -> TreeRoutingP.TreeTopology;
    SRTreeC.BackupTable -> BackupTableP.BackupTable;
    SRTreeC.DupFilter -> DupFilterP.DupFilter;

    // Wire control-plane routing module (AM 22, demultiplexed timers, and BackupTable)
    TreeRoutingP.RoutingPacket -> RoutingSenderC.Packet;
    TreeRoutingP.RoutingAMPacket -> RoutingSenderC.AMPacket;
    TreeRoutingP.RoutingAMSend -> RoutingSenderC.AMSend;
    TreeRoutingP.RoutingReceive -> RoutingReceiverC.Receive;
    TreeRoutingP.RoutingSendQueue -> RoutingSendQueueC;
    TreeRoutingP.RoutingReceiveQueue -> RoutingReceiveQueueC;
    TreeRoutingP.DisseminationTimer -> DisseminationTimerC;
    TreeRoutingP.OrphanTimer -> OrphanTimerC;
    TreeRoutingP.ElectionTimer -> ElectionTimerC;
    TreeRoutingP.BackupTable -> BackupTableP.BackupTable;
}
