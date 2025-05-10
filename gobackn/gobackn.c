#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "gobackn.h"
#include "protocol.h"

#define DATA_TIMER 2000
#define ACK_TIMER 300
#define MAX_SEQ 7
#define INC(num) ((num) = ((num + 1) & MAX_SEQ))

typedef unsigned char SeqNr;

typedef struct {
    unsigned char Kind;  /* FRAME_DATA, FRAME_ACK, FRAME_NAK */
    unsigned char Ack;
    unsigned char Seq;
    unsigned char Data[PKT_LEN];
    unsigned int Padding;
} Frame;

static SeqNr FrameToSend = 0;
static SeqNr AckExpected = 0;
static SeqNr FrameExpected = 0;

static bool NoNAK = true;
static bool PhlReady = false;

static int FrameLength = 0;

static unsigned char Buffer[MAX_SEQ + 1][PKT_LEN];
static int PacketLength[MAX_SEQ + 1];

static SeqNr NBuffered = 0;

// 发送帧到物理层
static void PutFrame(unsigned char* Frame, int Len) {
    *(unsigned int*)(Frame + Len) = crc32(Frame, Len);
    send_frame(Frame, Len + 4);
}

// 发送数据帧
static void SendData(SeqNr FrameNr, SeqNr FrameExpected, unsigned char* Packet, size_t Len) {
    Frame S;

    S.Kind = FRAME_DATA;
    S.Seq = FrameNr;
    S.Ack = (FrameExpected + MAX_SEQ) % (MAX_SEQ + 1);

    if (Len > PKT_LEN) {
        dbg_frame("Error while sending packet %d with ack %d: Length too large.\n",
                  S.Seq, S.Ack, Len);
        return;
    }
    
    memcpy(S.Data, Packet, PKT_LEN);

    dbg_frame("Packet sent: seq = %d, ack = %d, data id = %d\n", 
              S.Seq, S.Ack, *(short*)S.Data);

    // 发送帧: kind + ack + seq + 数据
    PutFrame((unsigned char*)&S, 3 + PKT_LEN);
}

// 发送ACK帧
static void SendACK(unsigned char FrameExpected) {
    Frame S;

    S.Kind = FRAME_ACK;
    S.Ack = (FrameExpected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("Send ACK %d\n", S.Ack);
    PutFrame((unsigned char*)&S, 2);
}

// 发送NAK帧
static void SendNAK(unsigned char FrameExpected) {
    Frame S;

    S.Kind = FRAME_NAK;
    S.Ack = (FrameExpected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("Send NAK %d\n", S.Ack);
    PutFrame((unsigned char*)&S, 2);
}

// 判断序号是否在窗口范围内
static inline bool Between(unsigned char A, unsigned char B, unsigned char C) {
    return ((A <= B) && (B < C)) || ((C < A) && (A <= B)) || ((B < C) && (C < A));
}

// 处理网络层就绪事件
static void NetworkReadyHandler() {
    // 从网络层获取数据包
    PacketLength[FrameToSend] = get_packet(Buffer[FrameToSend]);
    
    // 增加窗口大小
    NBuffered++;
    
    // 发送数据帧
    SendData(FrameToSend, FrameExpected, Buffer[FrameToSend], PacketLength[FrameToSend]);
    start_timer(FrameToSend, DATA_TIMER);
    stop_ack_timer();

    INC(FrameToSend);
    PhlReady = false;
}

// 处理物理层就绪事件
static void PhysicalReadyHandler() {
    PhlReady = true;
}

// 处理帧接收事件
static void FrameReceivedHandler() {
    // 从物理层接收帧
    Frame F;
    FrameLength = recv_frame((unsigned char*)(&F), sizeof(F));
    
    // 检查帧CRC校验
    if (FrameLength < 5 || crc32((unsigned char*)&F, FrameLength) != 0) {
        dbg_event("Bad CRC Checksum, Receive Error!!!\n");
        
        if (NoNAK) {
            SendNAK(FrameExpected);
            NoNAK = false;
            stop_ack_timer();
        }

        return;
    }
    
    // 记录接收到的帧
    if (F.Kind == FRAME_ACK) 
        dbg_frame("Recv ACK %d\n", F.Ack);
        
    if (F.Kind == FRAME_NAK) 
        dbg_frame("Recv NAK %d\n", F.Ack);
        
    if (F.Kind == FRAME_DATA) {
        dbg_frame("Recv DATA %d %d, ID %d\n", F.Seq, F.Ack, *(short*)F.Data);
        
        // 处理按序到达的数据帧
        if (F.Seq == FrameExpected) {
            put_packet(F.Data, FrameLength - 7);
            NoNAK = true;
            INC(FrameExpected);
            start_ack_timer(ACK_TIMER);
        } 
        // 发送NAK请求重传
        else if (NoNAK) {
            SendNAK(FrameExpected);
            NoNAK = false;
            stop_ack_timer();
        }
    }

    // 滑动发送窗口，确认已接收的帧
    while (Between(AckExpected, F.Ack, FrameToSend)) {
        NBuffered--;
        stop_timer(AckExpected);
        INC(AckExpected);
    }

    // 处理NAK，重传指定帧
    if (F.Kind == FRAME_NAK) {
        stop_timer(AckExpected);
        SeqNr ResendStart = AckExpected;
        
        // 回退N步，重传所有已发送但未确认的帧
        for (SeqNr i = 0; i < NBuffered; i++) {
            SendData(ResendStart, FrameExpected, Buffer[ResendStart], PacketLength[ResendStart]);
            start_timer(ResendStart, DATA_TIMER);
            stop_ack_timer();
            INC(ResendStart);
        }

        PhlReady = false;
    }
}

// 处理数据超时事件
static void DataTimeoutHandler(int* Arg) {
    dbg_event("---- DATA %d timeout\n", *Arg);
    
    // 回退N步，重传所有已发送但未确认的帧
    SeqNr ResendStart = AckExpected;
    
    for (SeqNr i = 0; i < NBuffered; i++) {
        SendData(ResendStart, FrameExpected, Buffer[ResendStart], PacketLength[ResendStart]);
        start_timer(ResendStart, DATA_TIMER);
        stop_ack_timer();
        INC(ResendStart);
    }

    PhlReady = false;
}

// 处理ACK超时事件
static void ACKTimeoutHandler() {
    SendACK(FrameExpected);
    stop_ack_timer();
}

// 事件处理函数表
void (*EventHandler[])(int*) = {
    [NETWORK_LAYER_READY] = NetworkReadyHandler,
    [PHYSICAL_LAYER_READY] = PhysicalReadyHandler,
    [FRAME_RECEIVED] = FrameReceivedHandler,
    [DATA_TIMEOUT] = DataTimeoutHandler,
    [ACK_TIMEOUT] = ACKTimeoutHandler,
};

int main(int argc, char** argv) {
    int Event, Arg;

    protocol_init(argc, argv);
    disable_network_layer();
    
    for (;;) {
        Event = wait_for_event(&Arg);

        // 调用对应的事件处理函数
        EventHandler[Event](&Arg);

        // 根据窗口状态启用/禁用网络层
        if (NBuffered < MAX_SEQ && PhlReady)
            enable_network_layer();
        else
            disable_network_layer();
    }

    return 0;
}    