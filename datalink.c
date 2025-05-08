#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define MAX_SEQ 31
#define CACHE_SIZE ((MAX_SEQ + 1) / 2)
#define ACK_TIMEOUT_LMT 300
#define DATA_TIMEOUT_LMT 2800

// 定义判断物理层状态及 NAK 传输标志的枚举类型 bool
typedef enum{
    false,
    true
} bool;

typedef unsigned char byte; // 字节定义

typedef struct packet{
    byte info[PKT_LEN];
} PACKET;

typedef struct{
    byte kind; /* FRAME_DATA, FRAME_ACK, FRAME_NAK */
    byte ack;
    byte seq;
    byte data[PKT_LEN];
    unsigned int crc32;
} FRAME;

static bool phl_ready = false; // 物理层状态
static bool noNAK = true;
static byte nbuffered = 0; // 当前窗口

// crc 译码并发送帧，同时标记物理层未就绪
static void putFrame(byte *frame, int len){
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = false;
}

// 判断是否在窗口内
static int between(byte a, byte b, byte c){
    return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

// 根据帧类型发送不同的帧，并停止 ACK 计时器
static void sendFrame(byte kind, byte seq, byte expFrame, PACKET buffer[]){
    FRAME frame;
    frame.kind = kind;
    frame.ack = (expFrame + MAX_SEQ) % (MAX_SEQ + 1);

    switch (frame.kind){
		case FRAME_DATA: // 发送数据帧，数据帧长度为 2
			frame.seq = seq;
			memcpy(frame.data, buffer[seq % CACHE_SIZE].info, PKT_LEN);
			dbg_frame("Send DATA %d %d, ID %d\n", frame.seq, frame.ack, *(short *)frame.data);
			putFrame((byte *)&frame, 3 + PKT_LEN);
			start_timer(seq % CACHE_SIZE, DATA_TIMEOUT_LMT); // 发送数据帧后的重传计时
			break;
		case FRAME_ACK: // 发送 ACK 应答帧，ACK 帧长度为 2
			dbg_frame("Send ACK %d\n", frame.ack);
			putFrame((byte *)&frame, 2);
			break;
		case FRAME_NAK: // 发送 NAK 重传指示帧，NAK 帧长度为 2
			noNAK = false;
			dbg_frame("Send NAK %d\n", frame.ack);
			putFrame((byte *)&frame, 2);
			break;
    }
    stop_ack_timer();
}

int main(int argc, char *argv[]){
    // 数据定义
    static byte to_send_frame = 0;    // 发送窗口上界
    static byte expACK = 0;           // 发送窗口下界
    static byte too_Far = CACHE_SIZE; // 接收窗口上界
    static byte expFrame = 0;         // 接收窗口下界
    static PACKET oBuf[CACHE_SIZE];   // 输出暂存区
    static PACKET iBuf[CACHE_SIZE];   // 输入暂存区
    bool arrived[CACHE_SIZE] = {false}; // 缓冲区标志位
    int event, arg;
    FRAME frame;
    size_t length = 0;

    // 协议事件实现
    protocol_init(argc, argv);
    lprintf("build: " __DATE__ " " __TIME__ "\n");
    enable_network_layer();

    for (;;){
        event = wait_for_event(&arg);
        switch (event){
			case NETWORK_LAYER_READY: // 网络层就绪，接受包并将数据帧传递至返回区
				nbuffered++;
				get_packet(oBuf[to_send_frame % CACHE_SIZE].info);
				sendFrame(FRAME_DATA, to_send_frame, expFrame, oBuf);
				if (to_send_frame < MAX_SEQ)
					to_send_frame++;
				else
					to_send_frame = 0; // 移动发送窗口
				break;
			case PHYSICAL_LAYER_READY: // 物理层就绪
				phl_ready = true;
				break;
			case FRAME_RECEIVED: // 接受帧
				length = recv_frame((byte *)&frame, sizeof frame);
				// 先行接受帧
				if (length < 5 || crc32((byte *)&frame, length) != 0){
					if (noNAK)
						sendFrame(FRAME_NAK, 0, expFrame, oBuf);
					dbg_event("**** Receiver Error, Bad CRC Checksum\n");
					break;
				} // 检测是否为正常帧，否则发送 NAK
				switch (frame.kind){ // 判断接受帧的类型
					case FRAME_ACK:
						dbg_frame("Recv ACK %d\n", frame.ack); // 接受 ACK
						break;
					case FRAME_DATA:
						if ((frame.seq != expFrame) && noNAK){
							sendFrame(FRAME_NAK, 0, expFrame, oBuf);
						} // 判断数据帧序列号,若不接收且未发送过 NAK 则发送 NAK
						else{
							start_ack_timer(ACK_TIMEOUT_LMT); // 启动计时器
						}
						// 若接收的数据帧未到达过，则将数据写入缓存并移动接收窗口
						if (between(expFrame, frame.seq, too_Far) && (arrived[frame.seq % CACHE_SIZE] == 0)){
							dbg_frame("Recv DATA %d %d, ID %d\n", frame.seq, frame.ack, *(short *)frame.data);
							arrived[frame.seq % CACHE_SIZE] = 1;
							memcpy(iBuf[frame.seq % CACHE_SIZE].info, frame.data, length - 7);
							while (arrived[expFrame % CACHE_SIZE]){
								put_packet(iBuf[expFrame % CACHE_SIZE].info, length - 7);
								noNAK = true; // 重置 NAK 发送检测
								arrived[expFrame % CACHE_SIZE] = 0;
								// 移动窗口
								if (expFrame < MAX_SEQ){
									expFrame++;
								}
								else{
									expFrame = 0;
								}
								if (too_Far < MAX_SEQ){
									too_Far++;
								}
								else{
									too_Far = 0;
								}
								start_ack_timer(ACK_TIMEOUT_LMT);
							}
						}
						break;
					case FRAME_NAK:
						if (between(expACK, (frame.ack + 1) % (MAX_SEQ + 1), to_send_frame)){
							dbg_frame("Recv NAK %d\n", frame.ack);
							// 选择重传
							sendFrame(FRAME_DATA, (frame.ack + 1) % (MAX_SEQ + 1), expFrame, oBuf);
						}
						break;
				}
				while (between(expACK, frame.ack, to_send_frame)){
					nbuffered--;
					stop_timer(expACK % CACHE_SIZE);
					if (expACK < MAX_SEQ){
						expACK++;
					}
					else{
						expACK = 0;
					}
				}
				break;
			case DATA_TIMEOUT: // 数据超时重传
				dbg_event("---- DATA %d timeout\n", arg);
				sendFrame(FRAME_DATA, expACK, expFrame, oBuf);
				break;
			case ACK_TIMEOUT: // ACK 超时重传
				dbg_event("---- ACK %d timeout\n", arg);
				sendFrame(FRAME_ACK, 0, expFrame, oBuf);
				break;
		}
        if (nbuffered < CACHE_SIZE && phl_ready){
            enable_network_layer();
        }
        else{
            disable_network_layer();
        }
    }
}
