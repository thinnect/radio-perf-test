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
#define AM_ID_ADDR_MSG 0x71
#define SEND_RETRY_COUNT 0
#define MAX_PACKET_LEN 100
#define MAX_PACKET_COUNT 10000
#define PCKT_SENT_LED 0x04
#define WAIT_DATA_PCKT_TIMEOUT 100
#define WAIT_MASTER_PCKT_TIMEOUT 1000
#define SEND_ADDR_TIMEOUT 500

enum role
{
    ROLE_MASTER = 1,
    ROLE_SERVANT,
    ROLE_UNKNOWN
};

// Thread flag definitions
#define SM_FLG_START_TEST           (1 << 0)
#define SM_FLG_SEND_SUCCESS         (1 << 1)
#define SM_FLG_SEND_FAIL            (1 << 2)
#define SM_FLG_SEND_DONE_OK         (1 << 3)
#define SM_FLG_SEND_DONE_NOACK      (1 << 4)
#define SM_FLG_SEND_DONE_FAIL       (1 << 5)
#define SM_FLG_PCKT_RCVD            (1 << 6)
#define SM_FLG_WAIT_PCKT_TIMEOUT    (1 << 7)
#define SM_FLG_SEND_ID              (1 << 8)
#define SM_FLG_ID_RCVD              (1 << 9)

#define SM_FLGS_ALL (0x000003FF)

enum sm_states
{
    SM_STATE_CHOOSE_ROLE = 1,
    SM_STATE_START,
    SM_STATE_SEND_DATA_PCKT,
    SM_STATE_WAIT_SEND_DONE,
    SM_STATE_WAIT_DATA_PCKT,
    SM_STATE_FINISH_TEST
};

static platform_mutex_t m_send_mutex;
static bool m_sending = false;
static comms_layer_t* m_p_radio;
static comms_receiver_t m_receiver_uc;
static comms_receiver_t m_receiver_id;

static comms_msg_t m_msg_to_send;
static am_addr_t m_node_addr;
static am_addr_t m_partner_addr;
static uint32_t m_sent_pckt_cnt = 0;
static uint32_t m_rcvd_ack_cnt = 0;
static uint32_t m_pckt_id;
static uint32_t m_rcvd_pckt_cnt = 0;
static uint32_t m_rcvd_pckt_id;
static uint32_t m_test_start_time;
static uint32_t m_test_end_time;
static bool m_test_started = false;
static osThreadId_t m_thread_id;
static uint8_t m_node_role;
static osTimerId_t m_tmr_wait_pckt;
static osTimerId_t m_tmr_send_id;

typedef struct data_packet
{
	uint32_t id;
    uint8_t dummy[MAX_PACKET_LEN];
} __attribute__((packed)) data_packet_t;

typedef struct id_packet
{
	am_addr_t node_addr;
    am_addr_t partner_addr;
} __attribute__((packed)) id_packet_t;

static void radio_send_done (comms_layer_t* p_radio, comms_msg_t* msg, comms_error_t result, void* p_user)
{
    platform_mutex_acquire(m_send_mutex);

    m_sending = false;

#if USE_LEDS == 1
        // blink sent LED for every 100 packets
        if ((m_sent_pckt_cnt % 100) == 0)
        {
            PLATFORM_LedsSet(PLATFORM_LedsGet() ^ PCKT_SENT_LED);
        }
#endif

    if (comms_get_packet_type(m_p_radio, msg) == AM_ID_UC_MSG)
    {
        if (COMMS_SUCCESS == result)
        {
            debug2("Snt:%u id:%u", result, m_pckt_id - 1);
            ++m_sent_pckt_cnt;
            if ((comms_is_ack_required(p_radio, msg) == true) && (comms_ack_received(p_radio, msg) == true))
            {
                ++m_rcvd_ack_cnt;
            }
            osThreadFlagsSet(m_thread_id, SM_FLG_SEND_DONE_OK);
        }
        else
        {
            if (COMMS_ENOACK == result)
            {
                // no ACK but packet was sent
                warn2("No ACK:%d", result);
                osThreadFlagsSet(m_thread_id, SM_FLG_SEND_DONE_NOACK);
            }
            else
            {
                err1("Failed to send a packet!");
                osThreadFlagsSet(m_thread_id, SM_FLG_SEND_DONE_FAIL);
            }
        }
    }
    else if (comms_get_packet_type(m_p_radio, msg) == AM_ID_ADDR_MSG)
    {
        if (COMMS_SUCCESS == result)
        {
            debug2("Snt:%u", result);
            osThreadFlagsSet(m_thread_id, SM_FLG_SEND_DONE_OK);
        }
        else
        {
            err1("Failed to send a packet!");
            osThreadFlagsSet(m_thread_id, SM_FLG_SEND_DONE_FAIL);
        }
    }
    else
    {
        err1("Rcvd UNKNOWN packet!");
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
    m_rcvd_pckt_id = data_pckt->id;
    debug2("RcvUC<-%04X id:%u, cnt:%u", comms_am_get_source(p_comms, p_msg), data_pckt->id, m_rcvd_pckt_cnt);

    osThreadFlagsSet(m_thread_id, SM_FLG_PCKT_RCVD);

    if (false == m_test_started)
    {
        if (0 == m_partner_addr)
        {
            m_partner_addr = comms_am_get_source(p_comms, p_msg);
        }
        m_test_started = true;
        osThreadFlagsSet(m_thread_id, SM_FLG_START_TEST);
        debug1("Start!");
    }
}

static void receive_id (comms_layer_t* p_comms, const comms_msg_t* p_msg, void* user)
{
    id_packet_t* id_pckt;

    id_pckt = (id_packet_t*)comms_get_payload(p_comms, p_msg, comms_get_payload_length(p_comms, p_msg));
    m_partner_addr = id_pckt->node_addr;
    debug2("RcvID<-%04X id:%04X prt:%04X", comms_am_get_source(p_comms, p_msg), id_pckt->node_addr, id_pckt->partner_addr);

    // signal that my partner has received my ID packet when it's packet contains my address
    // or when my address is greater
    if ((id_pckt->partner_addr == m_node_addr) || (m_node_addr > id_pckt->node_addr))
    {
        osThreadFlagsSet(m_thread_id, SM_FLG_ID_RCVD);
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

static void send_id_packet (void)
{
    id_packet_t id_packet;
    comms_error_t res;

    platform_mutex_acquire(m_send_mutex);

    if (true == m_sending)
    {
        warn1("busy!");
        platform_mutex_release(m_send_mutex);
        return;
    }
    id_packet.node_addr = m_node_addr;
    id_packet.partner_addr = m_partner_addr;
    res = send_packet(AM_BROADCAST_ADDR, AM_ID_ADDR_MSG, &id_packet, sizeof(id_packet));
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

static void finish_test (void)
{
    float test_duration;
    float thruput;
    float packets_sent_loss;
    float packets_rcvd_loss;
    float akcs_rcvd_loss;
    
    osTimerStop(m_tmr_wait_pckt);
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
    info1("Total packets to send: %u", MAX_PACKET_COUNT);
    info1("Packets successfully sent: %u", m_sent_pckt_cnt);
    info1("Send failed (no ACK): %u", MAX_PACKET_COUNT - m_sent_pckt_cnt);
    info1("Test duration: %.2f seconds", test_duration);
    info1("Thruput:%.2f pckt/s", thruput);
    info1("Packets received: %u", m_rcvd_pckt_cnt);
    info1("ACK-s received: %u", m_rcvd_ack_cnt);
    info1("Packets sent loss: %.2f%%", packets_sent_loss);
    info1("Packets received loss: %.2f%%", packets_rcvd_loss);
    info1("ACK-s received loss: %.2f%%", akcs_rcvd_loss);
}

void tmr_wait_pckt_callback (void* arg)
{
    osThreadFlagsSet(m_thread_id, SM_FLG_WAIT_PCKT_TIMEOUT);
}

void tmr_send_id_callback (void* arg)
{
    osThreadFlagsSet(m_thread_id, SM_FLG_SEND_ID);
}

static void state_machine_thread (void* arg)
{
    uint32_t state = SM_STATE_CHOOSE_ROLE;
    uint32_t flags;
    osStatus_t status;
    
    flags = osThreadFlagsClear(SM_FLGS_ALL);
    
    debug1("Thrd starts");
    
    for (;;)
    {
        flags = osThreadFlagsWait(SM_FLGS_ALL, osFlagsWaitAny, osWaitForever);
        flags &= SM_FLGS_ALL;
        
        debug2("st:%X flgs:%X", state, flags);
        
#if TEST_NR == 1
        switch (state)
        {
            case SM_STATE_START:
                if (flags & SM_FLG_START_TEST)
                {
                    state = SM_STATE_SEND_DATA_PCKT;
                    send_data_packet();
                }
                if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
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
                if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
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
                if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
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
                while (1);
        }
#else
#if TEST_NR == 2
        if (ROLE_MASTER == m_node_role)
        {
            switch (state)
            {
                case SM_STATE_START:
                    if (flags & SM_FLG_START_TEST)
                    {
                        m_test_started = 1;
                        m_test_start_time = osKernelGetTickCount();
                        state = SM_STATE_SEND_DATA_PCKT;
                        info1("Test started");
                        send_data_packet();
                    }
                    if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
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
                    if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
                    {
                        if ((MAX_PACKET_COUNT + 1) == m_pckt_id)
                        {
                            finish_test();
                        }
                        else
                        {
                            state = SM_STATE_WAIT_DATA_PCKT;
                            // start timeout timer
                            status = osTimerStart(m_tmr_wait_pckt, WAIT_DATA_PCKT_TIMEOUT);
                            if (osOK != status)
                            {
                                err1("!Tmr");
                                while (1);
                            }
                        }
                    }
                break;

                case SM_STATE_WAIT_SEND_DONE:
                    if (flags & SM_FLG_SEND_DONE_OK)
                    {
                        state = SM_STATE_WAIT_DATA_PCKT;
                        // start timeout timer
                        status = osTimerStart(m_tmr_wait_pckt, WAIT_DATA_PCKT_TIMEOUT);
                        if (osOK != status)
                        {
                            err1("!Tmr");
                            while (1);
                        }
                    }
                    else if ((flags & SM_FLG_SEND_DONE_NOACK) || (flags & SM_FLG_SEND_DONE_FAIL))
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

                case SM_STATE_WAIT_DATA_PCKT:
                    if ((flags & SM_FLG_WAIT_PCKT_TIMEOUT) || (flags & SM_FLG_PCKT_RCVD))
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
            }
        }
        else if (ROLE_SERVANT == m_node_role)
        {
            switch (state)
            {
                case SM_STATE_START:
                    if (flags & SM_FLG_START_TEST)
                    {
                        m_test_started = 1;
                        m_test_start_time = osKernelGetTickCount();
                        state = SM_STATE_SEND_DATA_PCKT;
                        info1("Test started");
                        send_data_packet();
                    }
                    if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
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
                    if ((flags & SM_FLG_SEND_DONE_OK) || (flags & SM_FLG_SEND_DONE_NOACK))
                    {
                        //if ((MAX_PACKET_COUNT + 1) == m_pckt_id)
                        if (((MAX_PACKET_COUNT) == m_rcvd_pckt_id) || ((MAX_PACKET_COUNT + 1) == m_pckt_id))
                        {
                            finish_test();
                        }
                        else
                        {
                            state = SM_STATE_WAIT_DATA_PCKT;
                            // start timeout timer
                            status = osTimerStart(m_tmr_wait_pckt, WAIT_MASTER_PCKT_TIMEOUT);
                            if (osOK != status)
                            {
                                err1("!Tmr");
                                while (1);
                            }
                        }
                    }
                break;

                case SM_STATE_WAIT_SEND_DONE:
                    if (flags & SM_FLG_SEND_DONE_OK)
                    {
                        state = SM_STATE_WAIT_DATA_PCKT;
                        // start timeout timer
                        status = osTimerStart(m_tmr_wait_pckt, WAIT_MASTER_PCKT_TIMEOUT);
                        if (osOK != status)
                        {
                            err1("!Tmr");
                            while (1);
                        }
                    }
                    else if ((flags & SM_FLG_SEND_DONE_NOACK) || (flags & SM_FLG_SEND_DONE_FAIL))
                    {
                        if (((MAX_PACKET_COUNT) == m_rcvd_pckt_id) || ((MAX_PACKET_COUNT + 1) == m_pckt_id))
                        {
                            finish_test();
                        }
                        else
                        {
                            state = SM_STATE_WAIT_DATA_PCKT;
                        }
                    }
                break;

                case SM_STATE_WAIT_DATA_PCKT:
                    if (flags & SM_FLG_WAIT_PCKT_TIMEOUT)
                    {
                            warn1("Timeout waiting for data packet");
                            finish_test();
                            osThreadSuspend(m_thread_id);
                    }
                    else if (flags & SM_FLG_PCKT_RCVD)
                    {
                        state = SM_STATE_SEND_DATA_PCKT;
                        send_data_packet();
                    }
                break;
            }
        }
        else if (ROLE_UNKNOWN == m_node_role)
        {
            switch (state)
            {
                case SM_STATE_CHOOSE_ROLE:
                    if (flags & SM_FLG_SEND_ID)
                    {
                        info1("Sending ID");
                        send_id_packet();
                    }
                    if (flags & SM_FLG_ID_RCVD)
                    {
                        state = SM_STATE_START;
                        osTimerStop(m_tmr_send_id);

                        if (m_node_addr > m_partner_addr)
                        {
                            m_node_role = ROLE_MASTER;
                            debug1("Role: master");
                            osThreadFlagsSet(m_thread_id, SM_FLG_START_TEST);
                        }
                        else
                        {
                            m_node_role = ROLE_SERVANT;
                            debug1("Role: servant");
                            // do not set start flag here, wait for the master!
                        }
                    }
                    if (flags & SM_FLG_PCKT_RCVD)
                    {
                        // gues i have to be in servant mode!
                        m_node_role = ROLE_SERVANT;
                        state = SM_STATE_START;
                        osTimerStop(m_tmr_send_id);
                        debug1("Role: servant");
                        osThreadFlagsSet(m_thread_id, SM_FLG_START_TEST);
                    }
                break;
                
                default:
                    err1("Unknown state");
            }
        }
        else
        {
            err1("!Role");
        }


#else
        err1("TEST_NR must be 1 or 2");
        while (1);
#endif
#endif
    }
}
    
void init_performance_test (comms_layer_t* p_radio, am_addr_t my_addr)
{
    comms_error_t res;
    
    m_p_radio = p_radio;
    m_node_addr = my_addr;
    m_partner_addr = 0;
    m_pckt_id = 1;
    m_node_role = ROLE_UNKNOWN;

    const osThreadAttr_t sm_thread_attr = { .name = "sm_thrd", .priority = osPriorityLow, .stack_size = 1024 };
    m_thread_id = osThreadNew(state_machine_thread, NULL, &sm_thread_attr);
    if (NULL == m_thread_id)
    {
        err1("!Thrd");
        while (1);
    }

    osDelay(1000);
    
    m_tmr_wait_pckt = osTimerNew(tmr_wait_pckt_callback, osTimerOnce, NULL, NULL);
    m_tmr_send_id = osTimerNew(tmr_send_id_callback, osTimerPeriodic, NULL, NULL);
    
    m_send_mutex = platform_mutex_new("send");

    res = comms_register_recv(m_p_radio, &m_receiver_uc, &receive_uc, NULL, AM_ID_UC_MSG);
    if (res != COMMS_SUCCESS)
    {
        err1("!Reg pckt rcv");
        while (1);
    }
    res = comms_register_recv(m_p_radio, &m_receiver_id, &receive_id, NULL, AM_ID_ADDR_MSG);
    if (res != COMMS_SUCCESS)
    {
        err1("!Reg pckt rcv");
        while (1);
    }
    
    osTimerStart(m_tmr_send_id, SEND_ADDR_TIMEOUT);
    info1("Searching for partner...");
    
#if 0
    if (m_node_addr > m_partner_addr)
    {
        info1("Test #%u starts", TEST_NR);
        info1("Sending %u packets, ACK required", MAX_PACKET_COUNT);
#if USE_LEDS == 1
        PLATFORM_LedsSet(PLATFORM_LedsGet() & ~PCKT_SENT_LED);
#endif
        m_node_role = ROLE_MASTER;
        osThreadFlagsSet(m_thread_id, SM_FLG_START_TEST);
    }
    else
    {
        m_node_role = ROLE_SERVANT;
        m_test_started = 0;
        info1("Waiting for the master to initiate the test");
    }
#endif
}

