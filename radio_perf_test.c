/**
 * Radio performane test module.
 *
 * Copyright Thinnect Inc. 2021
 * @license <PROPRIETARY>
 */
#include "loglevels.h"
#define __MODUUL__ "Perf"
#define __LOG_LEVEL__ ( LOG_LEVEL_performance & BASE_LOG_LEVEL )
#include "log.h"

#include <string.h>
#include "mist_comm_am.h"
#include "platform_mutex.h"
#include "platform.h"

#define AM_ID_UC_MSG 0x70
#define SEND_RETRY_COUNT 0
#define MAX_PACKET_LEN 100
#define MAX_PACKET_COUNT 1000
#define PCKT_SENT_LED 0x04

// Thread flag definitions
#define SM_FLG_START_TEST       (1 << 0)
#define SM_FLG_SEND_SUCCESS     (1 << 1)
#define SM_FLG_SEND_FAIL        (1 << 2)
#define SM_FLG_SEND_DONE        (1 << 3)
//#define SM_FLG_PCKT_RCVD        (1 << 4)
//#define SM_FLG_FINISH_TEST      (1 << 5)

#define SM_FLGS_ALL             (0x0000000F)

enum sm_states
{
    SM_STATE_START,
    SM_STATE_SEND_DATA_PCKT,
    SM_STATE_WAIT_SEND_DONE,
    SM_STATE_FINISH_TEST
};

static platform_mutex_t m_send_mutex;
static bool m_sending = false;
static comms_layer_t* m_p_radio;
static comms_receiver_t m_receiver_uc;

static comms_msg_t m_msg_to_send;
static am_addr_t m_node_addr;
static am_addr_t m_partner_addr;
static uint32_t m_sent_pckt_cnt = 0;
static uint32_t m_rcvd_ack_cnt = 0;
static uint32_t m_pckt_id;
static uint32_t m_rcvd_pckt_cnt = 0;
static uint32_t m_test_start_time;
static uint32_t m_test_end_time;
static bool m_test_started = false;
static osThreadId_t m_thread_id;

typedef struct data_packet
{
	uint32_t id;
    uint8_t dummy[MAX_PACKET_LEN];
} __attribute__((packed)) data_packet_t;

static void radio_send_done (comms_layer_t* p_radio, comms_msg_t* msg, comms_error_t result, void* p_user)
{
    platform_mutex_acquire(m_send_mutex);

    m_sending = false;
    osThreadFlagsSet(m_thread_id, SM_FLG_SEND_DONE);

    if (COMMS_SUCCESS == result)
    {
        debug2("Snt:%u id:%u", result, m_pckt_id - 1);
        ++m_sent_pckt_cnt;
        if ((comms_is_ack_required(p_radio, msg) == true) && (comms_ack_received(p_radio, msg) == true))
        {
            ++m_rcvd_ack_cnt;
        }
    }
    else
    {
        warn1("!snt:%d", result);
        if (COMMS_ENOACK == result)
        {
            // no ACK but packet was sent
            ++m_sent_pckt_cnt;
        }
    }
    platform_mutex_release(m_send_mutex);
}

static comms_error_t send_packet (uint16_t dest, am_id_t am_id, void* p_msg, uint8_t msg_size)
{
    void* p_payload;
    comms_error_t err;

    comms_init_message(m_p_radio, &m_msg_to_send);
    comms_set_packet_type(m_p_radio, &m_msg_to_send, am_id);
    comms_am_set_destination(m_p_radio, &m_msg_to_send, dest);
    comms_am_set_source(m_p_radio, &m_msg_to_send, m_node_addr);
    comms_set_retries(m_p_radio, &m_msg_to_send, SEND_RETRY_COUNT);
    comms_set_ack_required(m_p_radio, &m_msg_to_send, true);

    p_payload = comms_get_payload(m_p_radio, &m_msg_to_send, msg_size);
    if (p_payload == NULL)
    {
        err1("!Payload");
        return COMMS_UNINITIALIZED;
    }
    memcpy(p_payload, p_msg, msg_size);
    comms_set_payload_length(m_p_radio, &m_msg_to_send, msg_size);

    err = comms_send(m_p_radio, &m_msg_to_send, &radio_send_done, NULL);
    debug2("Snd->%04X e:%d", comms_am_get_destination(m_p_radio, &m_msg_to_send), err);

    return err;
}

static void receive_uc (comms_layer_t* p_comms, const comms_msg_t* p_msg, void* user)
{
    data_packet_t* data_pckt;

    data_pckt = (data_packet_t*)comms_get_payload(p_comms, p_msg, comms_get_payload_length(p_comms, p_msg));
    ++m_rcvd_pckt_cnt;
    debug2("RcvUC<-%04X id:%u, cnt:%u", comms_am_get_source(p_comms, p_msg), data_pckt->id, m_rcvd_pckt_cnt);

    // osThreadFlagsSet(m_thread_id, SM_FLG_PCKT_RCVD);

    if ((false == m_test_started) && (m_node_addr < m_partner_addr))
    {
        m_test_started = true;
        osThreadFlagsSet(m_thread_id, SM_FLG_START_TEST);
        debug1("Start!");
    }
}

static void send_data_packet (void)
{
    data_packet_t data_packet;
    comms_error_t res;

    platform_mutex_acquire(m_send_mutex);

    if (true == m_sending)
    {
        warn1("busy!");
        platform_mutex_release(m_send_mutex);
        return;
    }
    data_packet.id = m_pckt_id++;
    res = send_packet(m_partner_addr, AM_ID_UC_MSG, &data_packet, sizeof(data_packet));
    if (COMMS_SUCCESS == res)
    {
        m_sending = true;
        osThreadFlagsSet(m_thread_id, SM_FLG_SEND_SUCCESS);
    }
    else
    {
        osThreadFlagsSet(m_thread_id, SM_FLG_SEND_FAIL);
    }
    platform_mutex_release(m_send_mutex);
}

#if 0
static void start_performance_test (void)
{
    float test_duration;
    float thruput;
    data_packet_t data_packet;
    comms_error_t res;
    m_pckt_id = 1;
    
    m_test_start_time = osKernelGetTickCount();
    debug1("Test start:%u", m_test_start_time);
    for (m_pckt_cnt = 0; m_pckt_cnt < MAX_PACKET_COUNT; ++m_pckt_cnt)
    {
        while (m_sending == true);
        
        data_packet.id = m_pckt_id++;
        res = send_packet(m_partner_addr, AM_ID_UC_MSG, &data_packet, sizeof(data_packet));
        if (COMMS_SUCCESS == res)
        {
            platform_mutex_acquire(m_send_mutex);
            m_sending = true;
            platform_mutex_release(m_send_mutex);
        }
    }
    osDelay(1000);
    test_duration = (float)(m_test_end_time - m_test_start_time) / (float)1000.0;
    if (test_duration > 0)
    {
        thruput = (float)m_sent_pckt_cnt / test_duration;
    }
    else
    {
        warn1("Increase MAX_PACKET_COUNT to get results!");
    }
    debug1("start:%u end:%u dur:%.2f", m_test_start_time, m_test_end_time, test_duration);
    info1("Test finished. %u pckts sent in %.2f seconds, thruput:%.2f pckt/s, pckts rcvd:%u", m_sent_pckt_cnt, test_duration, thruput, m_rcvd_pckt_cnt);
}
#endif

static void finish_test (void)
{
    float test_duration;
    float thruput;
    float packets_sent_loss;
    float packets_rcvd_loss;
    float akcs_rcvd_loss;
    
    m_test_end_time = osKernelGetTickCount();

#if USE_LEDS == 1
    // clear sent LED
    PLATFORM_LedsSet(PLATFORM_LedsGet() & ~PCKT_SENT_LED);
#endif
    
    debug1("End time:%u", m_test_end_time);
    test_duration = (float)(m_test_end_time - m_test_start_time) / (float)1000.0;
    if (test_duration > 0)
    {
        thruput = (float)m_sent_pckt_cnt / test_duration;
    }
    else
    {
        warn1("Increase MAX_PACKET_COUNT to get results!");
    }
    packets_sent_loss = (1.0 - (float)m_sent_pckt_cnt / (float)MAX_PACKET_COUNT) * 100.0;
    packets_rcvd_loss = (1.0 - (float)m_rcvd_pckt_cnt / (float)MAX_PACKET_COUNT) * 100.0;
    akcs_rcvd_loss = (1.0 - (float)m_rcvd_ack_cnt / (float)MAX_PACKET_COUNT) * 100.0;
    
    debug1("start:%u end:%u dur:%.2f", m_test_start_time, m_test_end_time, test_duration);
    info1("Test finished");
    info1("Packets sent: %u", m_sent_pckt_cnt);
    info1("Send failed: %u", MAX_PACKET_COUNT - m_sent_pckt_cnt);
    info1("Test duration: %.2f seconds", test_duration);
    info1("Thruput:%.2f pckt/s", thruput);
    info1("Packets received: %u", m_rcvd_pckt_cnt);
    info1("ACK-s received: %u", m_rcvd_ack_cnt);
    info1("Packets sent loss: %.2f%%", packets_sent_loss);
    info1("Packets received loss: %.2f%%", packets_rcvd_loss);
    info1("ACK-s received loss: %.2f%%", akcs_rcvd_loss);
}
    
static void state_machine_1_thread (void* arg)
{
    uint32_t state = SM_STATE_START;
    uint32_t flags;

    flags = osThreadFlagsClear(SM_FLGS_ALL);
    
    debug1("Thrd starts");
    
    for (;;)
    {
        flags = osThreadFlagsWait(SM_FLGS_ALL, osFlagsWaitAny, osWaitForever);
        flags &= SM_FLGS_ALL;
        
        debug2("st:%X flgs:%X", state, flags);
#if USE_LEDS == 1
        // blink sent LED for every 100 packets
        if ((m_sent_pckt_cnt % 100) == 0)
        {
            PLATFORM_LedsSet(PLATFORM_LedsGet() ^ PCKT_SENT_LED);
        }
#endif
        
        switch (state)
        {
            case SM_STATE_START:
                if (flags & SM_FLG_START_TEST)
                {
                    state = SM_STATE_SEND_DATA_PCKT;
                    send_data_packet();
                }
                if (flags & SM_FLG_SEND_DONE)
                {
                    send_data_packet();
                }
            break;

            case SM_STATE_SEND_DATA_PCKT:
                if (flags & SM_FLG_SEND_SUCCESS)
                {
                    state = SM_STATE_WAIT_SEND_DONE;
                }
                if (flags & SM_FLG_SEND_FAIL)
                {
                    send_data_packet();
                }
                if (flags & SM_FLG_SEND_DONE)
                {
                    if ((MAX_PACKET_COUNT + 1) == m_pckt_id)
                    {
                        finish_test();
                    }
                    else
                    {
                        send_data_packet();
                    }
                }
            break;

            case SM_STATE_WAIT_SEND_DONE:
                if (flags & SM_FLG_SEND_DONE)
                {
                    if ((MAX_PACKET_COUNT + 1) == m_pckt_id)
                    {
                        finish_test();
                    }
                    else
                    {
                        state = SM_STATE_SEND_DATA_PCKT;
                        send_data_packet();
                    }
                }
            break;
                
            default:
                err1("Unknown state!");
        }
    }
}
    
void init_performance_test (comms_layer_t* p_radio, am_addr_t my_addr, am_addr_t partner_addr)
{
    comms_error_t res;
    
    m_p_radio = p_radio;
    m_node_addr = my_addr;
    m_partner_addr = partner_addr;
    m_pckt_id = 1;

    m_send_mutex = platform_mutex_new("send");

    res = comms_register_recv(m_p_radio, &m_receiver_uc, &receive_uc, NULL, AM_ID_UC_MSG);
    if (res != COMMS_SUCCESS)
    {
        err1("!Reg pckt rcv");
        while (1);
    }
    
    const osThreadAttr_t sm_thread_attr = { .name = "sm_thrd", .priority = osPriorityLow, .stack_size = 1024 };
    m_thread_id = osThreadNew(state_machine_1_thread, NULL, &sm_thread_attr);
    if (NULL == m_thread_id)
    {
        err1("!Thrd");
        while (1);
    }
    osDelay(1000);
    
    if (m_node_addr > m_partner_addr)
    {
        debug1("Set start flg");
        info1("Test starts");
        info1("Sending %u packets, ACK required", MAX_PACKET_COUNT);
#if USE_LEDS == 1
        PLATFORM_LedsSet(PLATFORM_LedsGet() & ~PCKT_SENT_LED);
#endif
        osThreadFlagsSet(m_thread_id, SM_FLG_START_TEST);
        //start_performance_test();
    }
    else
    {
        debug1("Wait for test start");
//        while (false == m_test_started)
//        {
//            osDelay(100);
//        }
//        start_performance_test();
    }
}

