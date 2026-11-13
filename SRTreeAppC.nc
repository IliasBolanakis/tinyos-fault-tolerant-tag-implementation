/**
 * @file SRTreeAppC.nc
 * @brief Top-level configuration wiring the fault-tolerant TAG sensor network application.
 */
#include "SimpleRoutingTree.h"

configuration SRTreeAppC @safe() {
}
implementation {
    // Core application logic module
    components SRTreeC;

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

    // Timers: routing propagation / election delay and TAG epoch slotting
    components new TimerMilliC() as RoutingMsgTimerC;
    components new TimerMilliC() as MyTimerC;

    // Wire platform and PRNG interfaces
    SRTreeC.Boot -> MainC.Boot;
    SRTreeC.RadioControl -> ActiveMessageC;
    SRTreeC.Leds -> LedsC;
    SRTreeC.Random -> RandomC;
    SRTreeC.SeedInit -> RandomC.SeedInit;

    // Wire timers
    SRTreeC.RoutingMsgTimer -> RoutingMsgTimerC;
    SRTreeC.MyTimer -> MyTimerC;

    // Wire routing message stack (AM 22)
    SRTreeC.RoutingPacket -> RoutingSenderC.Packet;
    SRTreeC.RoutingAMPacket -> RoutingSenderC.AMPacket;
    SRTreeC.RoutingAMSend -> RoutingSenderC.AMSend;
    SRTreeC.RoutingReceive -> RoutingReceiverC.Receive;

    // Wire data aggregation message stack (AM 23) and synchronous MAC ACKs
    SRTreeC.MyPacket -> MySenderC.Packet;
    SRTreeC.MyAMPacket -> MySenderC.AMPacket;
    SRTreeC.MyAMSend -> MySenderC.AMSend;
    SRTreeC.MyReceive -> MyReceiverC.Receive;
    SRTreeC.MyPacketAcknowledgements -> ActiveMessageC;

    // Wire packet queue interfaces
    SRTreeC.RoutingSendQueue -> RoutingSendQueueC;
    SRTreeC.RoutingReceiveQueue -> RoutingReceiveQueueC;
    SRTreeC.MySendQueue -> MySendQueueC;
    SRTreeC.MyReceiveQueue -> MyReceiveQueueC;
}