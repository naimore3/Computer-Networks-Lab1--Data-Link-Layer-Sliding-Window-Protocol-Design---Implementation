#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "selective.h"
#include "protocol.h"

#define MAX_SEQ 31
#define NR_BUFS ((MAX_SEQ + 1) / 2)
#define DATA_TIMER 2000

typedef unsigned char SeqNr;

typedef struct {
    unsigned char Kind;     /* FRAME_DATA, FRAME_ACK */
    SeqNr AckSeq;           /* 确认号或序列号 */
    unsigned char Data[PKT_LEN];
    unsigned int Padding;
} Frame;

typedef struct {
    unsigned char Buf[PKT_LEN];
    int Len;
} Buffer;

static Buffer InBuf[NR_BUFS], OutBuf[NR_BUFS];
static bool PhlReady = false;
static SeqNr RecvBase = 0;
static SeqNr SendBase = 0, NextSeqNr = 0;
static bool Acked[MAX_SEQ + 1] = {false};
static bool Cached[MAX_SEQ + 1] = {false};

// 发送帧到物理层
static void PutFrame(unsigned char* Frame, int Len) {
    *(unsigned int*)(Frame + Len) = crc32(Frame, Len);
    send_frame(Frame, Len + 4);
    PhlReady = false;
}

// 判断序号是否在窗口范围内
static bool Between(SeqNr A, SeqNr B, SeqNr C) {
    A %= (MAX_SEQ + 1);
    B %= (MAX_SEQ + 1);
    C %= (MAX_SEQ + 1);
    
    return (((A <= B) && (B <= C)) || 
            ((C <= A) && (A <= B)) || 
            ((B <= C) && (C <= A)));
}

// 处理网络层就绪事件
void NetworkLayerReadyHandler(int* Arg) {
    // 获取网络层数据包并缓存
    int Len = get_packet(OutBuf[NextSeqNr % NR_BUFS].Buf);
    OutBuf[NextSeqNr % NR_BUFS].Len = Len;

    // 立即发送数据帧并启动计时器
    Frame S;
    S.Kind = FRAME_DATA;
    S.AckSeq = NextSeqNr;  // 序列号
    
    memcpy(S.Data, OutBuf[NextSeqNr % NR_BUFS].Buf, PKT_LEN);

    dbg_frame("Send DATA %d, ID %d\n", S.AckSeq, *(short*)S.Data);
    PutFrame((unsigned char*)&S, 2 + Len);
    start_timer(S.AckSeq, DATA_TIMER);

    // 更新下一个要发送的序列号
    NextSeqNr = (NextSeqNr + 1) % (MAX_SEQ + 1);
}

// 处理物理层就绪事件
void PhysicalLayerReadyHandler(int* Arg) {
    PhlReady = true;
}

// 处理帧接收事件
void FrameReceivedHandler(int* Arg) {
    Frame F;
    int Len = recv_frame((unsigned char*)&F, sizeof(F));
    
    // 检查帧CRC校验
    if (Len > 6 && crc32((unsigned char*)&F, Len) != 0) {
        dbg_event("**** Receiver Error, Bad CRC Checksum\n");
        return; // 忽略错误帧
    }

    // 处理ACK帧
    if (F.Kind == FRAME_ACK && Between(SendBase, F.AckSeq, (SendBase + NR_BUFS - 1))) {
        Acked[F.AckSeq] = true;  // 标记为已确认
        dbg_frame("Recv ACK %d\n", F.AckSeq);
        stop_timer(F.AckSeq);

        // 滑动发送窗口
        while (Acked[SendBase]) {
            Acked[SendBase] = false;
            SendBase = (SendBase + 1) % (MAX_SEQ + 1);
            
            if (SendBase == NextSeqNr)
                break;
        }
    }

    // 处理数据帧
    if (F.Kind == FRAME_DATA) {
        dbg_frame("Recv DATA %d, ID %d\n", F.AckSeq, *(short*)F.Data);

        // 发送ACK确认
        Frame S;
        S.Kind = FRAME_ACK;
        S.AckSeq = F.AckSeq;  // 确认号
        
        dbg_frame("Send ACK %d\n", S.AckSeq);
        PutFrame((unsigned char*)&S, 2);

        // 如果在接收窗口内
        if (Between(RecvBase, F.AckSeq, (RecvBase + NR_BUFS - 1))) {
            // 缓存帧数据
            if (!Cached[F.AckSeq]) {
                memcpy(InBuf[F.AckSeq % NR_BUFS].Buf, F.Data, Len - 6);
                InBuf[F.AckSeq % NR_BUFS].Len = Len - 6;
                Cached[F.AckSeq] = true;
            }
            
            // 向上层传递按序到达的数据
            while (Cached[RecvBase]) {
                Cached[RecvBase] = false;
                put_packet(InBuf[RecvBase % NR_BUFS].Buf, InBuf[RecvBase % NR_BUFS].Len);
                RecvBase = (RecvBase + 1) % (MAX_SEQ + 1);
            }
        }
    }
}

// 处理数据超时事件
void DataTimeoutHandler(int* Arg) {
    SeqNr Num = *Arg;

    // 重传超时的数据帧
    Frame S;
    S.Kind = FRAME_DATA;
    S.AckSeq = Num;
    
    memcpy(S.Data, OutBuf[Num % NR_BUFS].Buf, PKT_LEN);

    PutFrame((unsigned char*)&S, 2 + OutBuf[Num % NR_BUFS].Len);
    start_timer(Num, DATA_TIMER);

    dbg_frame("Timeout, ReSend DATA %d, ID %d\n", S.AckSeq, *(short*)S.Data);
}

// 事件处理函数表
void (*EventHandler[])(int*) = {
    [NETWORK_LAYER_READY] = NetworkLayerReadyHandler,
    [PHYSICAL_LAYER_READY] = PhysicalLayerReadyHandler,
    [FRAME_RECEIVED] = FrameReceivedHandler,
    [DATA_TIMEOUT] = DataTimeoutHandler,
};

int main(int argc, char** argv) {
    int Event, Arg;

    protocol_init(argc, argv);
    lprintf("Designed by Jiang Yanjun, build: " __DATE__ "  "__TIME__ "\n");

    disable_network_layer();

    for (;;) {
        Event = wait_for_event(&Arg);

        // 调用对应的事件处理函数
        EventHandler[Event](&Arg);

        // 根据窗口状态启用/禁用网络层
        if (Between(SendBase, NextSeqNr, (SendBase + NR_BUFS - 1)) && PhlReady)
            enable_network_layer();
        else
            disable_network_layer();
    }

    return 0;
}    