#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "selective.h"

#define MAX_SEQ 31
#define CACHE_SIZE ((MAX_SEQ+1)/2)
#define ACK_TIMEOUT_LIMIT 300
#define DATA_TIMEOUT_LIMIT 2800

// 定义布尔类型
typedef enum {
    False,
    True
} Bool;

typedef unsigned char Byte; // 字节定义

typedef struct {
    Byte Info[PKT_LEN];
} Packet;

typedef struct {
    Byte Kind;     // FRAME_DATA, FRAME_ACK, FRAME_NAK
    Byte Ack;
    Byte Seq;
    Byte Data[PKT_LEN];
    unsigned int Crc32;
} Frame;

static Bool PhlReady = False;    // 物理层状态
static Bool NoNAK = True;        
static Byte NBuffered = 0;       // 当前窗口

// 计算CRC校验
static void PutFrame(Byte* Frame, int Len) {
    *(unsigned int*)(Frame + Len) = crc32(Frame, Len); // CRC编码
    send_frame(Frame, Len + 4);
    PhlReady = False;
}

// 判断是否在窗口内
static int Between(Byte A, Byte B, Byte C) {
    return ((A <= B) && (B < C)) || ((C < A) && (A <= B)) || ((B < C) && (C < A));
}

// 发送帧函数
static void SendFrame(Byte Kind, Byte Seq, Byte ExpFrame, Packet Buffer[]) {
    Frame Frame;
    Frame.Kind = Kind;
    Frame.Ack = (ExpFrame + MAX_SEQ) % (MAX_SEQ + 1);

    switch (Frame.Kind) {
        case FRAME_DATA: // 发送数据帧
            Frame.Seq = Seq;
            memcpy(Frame.Data, Buffer[Seq % CACHE_SIZE].Info, PKT_LEN);
            dbg_frame("Send DATA %d %d, ID %d\n", Frame.Seq, Frame.Ack, *(short*)Frame.Data);
            PutFrame((Byte*)&Frame, 3 + PKT_LEN);
            start_timer(Seq % CACHE_SIZE, DATA_TIMEOUT_LIMIT); // 启动数据帧重传计时器
            break;
            
        case FRAME_ACK: // 发送ACK应答帧
            dbg_frame("Send ACK %d\n", Frame.Ack);
            PutFrame((Byte*)&Frame, 2);
            break;
            
        case FRAME_NAK: // 发送NAK重传指示帧
            NoNAK = False;
            dbg_frame("Send NAK %d\n", Frame.Ack);
            PutFrame((Byte*)&Frame, 2);
            break;
    }
    
    stop_ack_timer();
}

int main(int argc, char* argv[]) {
    // 数据定义
    static Byte ToSendFrame = 0;     // 发送窗口上界
    static Byte ExpACK = 0;          // 发送窗口下界
    static Byte TooFar = CACHE_SIZE; // 接收窗口上界
    static Byte ExpFrame = 0;        // 接收窗口下界
    
    static Packet OutputBuffer[CACHE_SIZE]; // 输出缓冲区
    static Packet InputBuffer[CACHE_SIZE];  // 输入缓冲区
    
    Bool Arrived[CACHE_SIZE] = { False };   // 缓冲区标志位
    
    int Event, Arg;
    Frame Frame;
    size_t Length = 0;

    // 协议初始化
    protocol_init(argc, argv);
    lprintf("build: " __DATE__ " "__TIME__"\n");
    enable_network_layer();

    for (;;) {
        Event = wait_for_event(&Arg);
        
        switch (Event) {
            case NETWORK_LAYER_READY: // 网络层就绪
                NBuffered++;
                get_packet(OutputBuffer[ToSendFrame % CACHE_SIZE].Info);
                SendFrame(FRAME_DATA, ToSendFrame, ExpFrame, OutputBuffer);
                
                // 移动发送窗口
                if (ToSendFrame < MAX_SEQ)
                    ToSendFrame++;
                else
                    ToSendFrame = 0;
                break;
                
            case PHYSICAL_LAYER_READY: // 物理层就绪
                PhlReady = True;
                break;
                
            case FRAME_RECEIVED: // 接收帧
                Length = recv_frame((Byte*)&Frame, sizeof Frame);
                
                // 检查帧完整性
                if (Length < 5 || crc32((Byte*)&Frame, Length) != 0) {
                    if (NoNAK)
                        SendFrame(FRAME_NAK, 0, ExpFrame, OutputBuffer);
                    dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                    break;
                }
                
                // 处理接收到的帧
                switch (Frame.Kind) {
                    case FRAME_ACK: // 处理ACK帧
                        dbg_frame("Recv ACK %d\n", Frame.Ack);
                        break;
                        
                    case FRAME_DATA: // 处理数据帧
                        if ((Frame.Seq != ExpFrame) && NoNAK) {
                            SendFrame(FRAME_NAK, 0, ExpFrame, OutputBuffer);
                        } else {
                            start_ack_timer(ACK_TIMEOUT_LIMIT);
                        }
                        
                        // 缓存接收到的数据帧
                        if (Between(ExpFrame, Frame.Seq, TooFar) && 
                            (Arrived[Frame.Seq % CACHE_SIZE] == False)) {
                            
                            dbg_frame("Recv DATA %d %d, ID %d\n", Frame.Seq, Frame.Ack, *(short*)Frame.Data);
                            Arrived[Frame.Seq % CACHE_SIZE] = True;
                            memcpy(InputBuffer[Frame.Seq % CACHE_SIZE].Info, Frame.Data, Length - 7);
                            
                            // 按序提交数据
                            while (Arrived[ExpFrame % CACHE_SIZE]) {
                                put_packet(InputBuffer[ExpFrame % CACHE_SIZE].Info, Length - 7);
                                NoNAK = True;
                                Arrived[ExpFrame % CACHE_SIZE] = False;
                                
                                // 移动接收窗口
                                if (ExpFrame < MAX_SEQ)
                                    ExpFrame++;
                                else
                                    ExpFrame = 0;
                                    
                                if (TooFar < MAX_SEQ)
                                    TooFar++;
                                else
                                    TooFar = 0;
                                    
                                start_ack_timer(ACK_TIMEOUT_LIMIT);
                            }
                        }
                        break;
                        
                    case FRAME_NAK: // 处理NAK帧
                        if (Between(ExpACK, (Frame.Ack + 1) % (MAX_SEQ + 1), ToSendFrame)) {
                            dbg_frame("Recv NAK %d\n", Frame.Ack);
                            // 选择重传
                            SendFrame(FRAME_DATA, (Frame.Ack + 1) % (MAX_SEQ + 1), ExpFrame, OutputBuffer);
                        }
                        break;
                }
                
                // 滑动发送窗口
                while (Between(ExpACK, Frame.Ack, ToSendFrame)) {
                    NBuffered--;
                    stop_timer(ExpACK % CACHE_SIZE);
                    
                    if (ExpACK < MAX_SEQ)
                        ExpACK++;
                    else
                        ExpACK = 0;
                }
                break;
                
            case DATA_TIMEOUT: // 数据超时重传
                dbg_event("---- DATA %d timeout\n", Arg);
                SendFrame(FRAME_DATA, ExpACK, ExpFrame, OutputBuffer);
                break;
                
            case ACK_TIMEOUT: // ACK超时重传
                dbg_event("---- ACK %d timeout\n", Arg);
                SendFrame(FRAME_ACK, 0, ExpFrame, OutputBuffer);
                break;
        }
        
        // 根据缓冲区状态启用/禁用网络层
        if (NBuffered < CACHE_SIZE && PhlReady) {
            enable_network_layer();
        } else {
            disable_network_layer();
        }
    }
}    