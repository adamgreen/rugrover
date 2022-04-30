/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Test bed for verifying that MRI can be made to work over BLE.

   This program is heavily influenced by Nordic's BLE UART Service (ble_app_uart) SDK sample.
*/
#include <strings.h>
#include <ctype.h>
#include <app_timer.h>
#include <app_simple_timer.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_hci.h>
#include <ble_nus.h>
#include <ble_conn_state.h>
#include <peer_manager.h>
#include <fds.h>
#include <fstorage.h>
#include <nrf_adc.h>
#include <nrf_assert.h>
#include <nrf_atomic.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#define NRF_LOG_MODULE_NAME "APP"
#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <softdevice_handler_appsh.h>
#include <bsp.h>
#include <core/mri.h>


// UNDONE: Currently using switch 4 which is also connected to RMOTOR_DIAG by my shield.
// The pin connected to a switch to be pressed to enable pairing and during reset to
// erase previous bonding information from Peer Manager.
#define BONDING_SWITCH_PIN      NRF_GPIO_PIN_MAP(0, 16)

// The name of this device.
#define DEVICE_NAME             "mriblue"

// The unique portion of the serivce UUID advertised by this device.
#define MRIBLUE_ADVERTISE                0xADA4

// If haven't received any more data from MRI in this amount of bit times, then send what we have already received.
// Will run timer with this period. This value is selected because there are ~558us between the start of a full
// BLE packet and the time an ACK comes back.
#define NAGLE_TIME_MICROSECONDS    500

// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
// UNDONE: Does this really need to be larger than 1?
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         4

// The service database for this device can't be changed at runtime. Must be non-zero for DFU.
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

#if (NRF_SD_BLE_API_VERSION == 3)
// MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT
#endif

// Reply when unsupported features are requested.
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2

// This application doesn't need to act as a central device.
#define CENTRAL_LINK_COUNT              0
// This application is a peripheral and only hosts 1 peripheral link back to a central device.
#define PERIPHERAL_LINK_COUNT           1

// The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).
#define APP_ADV_FAST_INTERVAL           64
// The advertising timeout (in units of seconds).
#define APP_ADV_FAST_TIMEOUT            180
// Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds).
#define APP_ADV_SLOW_INTERVAL           3200
// The advertising timeout in units of seconds. 0 for infinite.
#define APP_ADV_SLOW_TIMEOUT            0

// Value of the RTC1 PRESCALER register used by application timer.
#define APP_TIMER_PRESCALER             0
// Size of timer operation queues. Includes room for BSP specific timers and ones used by this application.
#define APP_TIMER_OP_QUEUE_SIZE         4

// Timings for how often peripheral and central should communicate over the link. The shorter the interval, the lower
// the link latency but at the cost of higher power usage.
// Apple's Accessory Design Guidelines places some extra constraints on these parameters:
// * Peripheral Latency of up to 30 connection intervals.
// * Supervision Timeout from 2 seconds to 6 seconds.
// * Interval Min of at least 15 ms.
// * Interval Min is a multiple of 15 ms.
// * One of the following:
//   * Interval Max at least 15 ms greater than Interval Min.
//    * Interval Max and Interval Min both set to 15 ms.
// * Interval Max * (Peripheral Latency + 1) of 2 seconds or less.
// * Supervision Timeout greater than Interval Max * (Peripheral Latency + 1) * 3.
// Minimum acceptable connection interval in ms (7.5 ms is smallest allowed). Connection interval uses 1.25 ms units.
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)
// Maximum acceptable connection interval in ms. Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)
// The number of the connection interval events that the peripheral can ignore before response is required.
#define SLAVE_LATENCY                   0

// Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
// Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
// Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
// Number of attempts before giving up the connection parameter negotiation.
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

// Security GAP parameters.
// Perform bonding.
#define SEC_PARAM_BOND                  1
// Man In The Middle protection required (applicable when display module is detected).
#define SEC_PARAM_MITM                  1
// LE Secure Connections not enabled.
#define SEC_PARAM_LESC                  0
// Keypress notifications not enabled.
#define SEC_PARAM_KEYPRESS              0
// Display I/O capabilities.
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY
// Out Of Band data not available.
#define SEC_PARAM_OOB                   0
/// Minimum encryption key size.
#define SEC_PARAM_MIN_KEY_SIZE          7
// Maximum encryption key size.
#define SEC_PARAM_MAX_KEY_SIZE          16

// How often timer should fire to check for bonding button presses.
#define BUTTON_SAMPLE_INTERVAL          APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

// Length of numeric pass-key received by the stack for display.
#define PASSKEY_LENGTH                  6

// This program uses a static passkey as it has no I/O.
#ifndef STATIC_PASSKEY
#define STATIC_PASSKEY                  "123456"
#endif

// Nordic UART service.
static ble_nus_t                    g_nordicUartService;

// Current BLE connection.
static uint16_t                     g_currBleConnection = BLE_CONN_HANDLE_INVALID;

// UUIDS returned in advertising scan response.
static ble_uuid_t                   g_advertiseUuids[] = {{MRIBLUE_ADVERTISE, BLE_UUID_TYPE_VENDOR_BEGIN}};

// Used to track peers that didn't use MITM and should be deleted from Peer Manager on disconnect.
static pm_peer_id_t                 g_peerToDelete = PM_PEER_ID_INVALID;

// Peer Manager handle to the currently bonded central.
static pm_peer_id_t                 g_peerId = PM_PEER_ID_INVALID;

// Flag to indicate whether the timer routine should still check for bond button presses.
static bool         g_checkForBondButton = true;

// Whitelist of peers.
static pm_peer_id_t g_whitelistPeers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
// Number of peers currently in g_whitelistPeers.
static uint32_t     g_whitelistPeerCount;
// Has the whitelist been updated since last updated in the Peer Manager.
static bool         g_whitelistModified;



// Circular queue used to communicate between BLE stack and MRI.
#define CIRCULAR_QUEUE_SIZE 256

typedef struct CircularQueue {
    uint32_t write;
    uint32_t read;
    uint32_t peek;
    uint32_t peekSize;
    volatile uint32_t count;
    volatile uint8_t  queue[CIRCULAR_QUEUE_SIZE];
} CircularQueue;

static uint32_t incrementIndex(uint32_t index);

void CircularQueue_Init(CircularQueue* pThis)
{
    ASSERT ( (sizeof(pThis->queue) & (sizeof(pThis->queue)-1)) == 0 );

    pThis->write = 0;
    pThis->read = 0;
    pThis->peek = 0;
    pThis->peekSize = 0;
    pThis->count = 0;
}

uint32_t CircularQueue_Write(CircularQueue* pThis, const uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );

    uint32_t bytesLeft = sizeof(pThis->queue) - pThis->count;
    uint32_t bytesToWrite = (bytesLeft < dataSize) ? bytesLeft : dataSize;
    for (uint32_t i = 0 ; i < bytesToWrite ; i++)
    {
        pThis->queue[pThis->write] = *pData++;
        pThis->write = incrementIndex(pThis->write);
    }
    nrf_atomic_u32_add(&pThis->count, bytesToWrite);
    return bytesToWrite;
}

static uint32_t incrementIndex(uint32_t index)
{
    return (index + 1) & (CIRCULAR_QUEUE_SIZE-1);
}

uint32_t CircularQueue_BytesToRead(CircularQueue* pThis)
{
    return pThis->count;
}

uint32_t CircularQueue_IsFull(CircularQueue* pThis)
{
    return CircularQueue_BytesToRead(pThis) == CIRCULAR_QUEUE_SIZE;
}

uint32_t CircularQueue_IsEmpty(CircularQueue* pThis)
{
    return CircularQueue_BytesToRead(pThis) == 0;
}

uint32_t CircularQueue_Read(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );

    uint32_t bytesToRead = (dataSize > pThis->count) ? pThis->count : dataSize;
    for (uint32_t i = 0 ; i < bytesToRead ; i++)
    {
        *pData++ = pThis->queue[pThis->read];
        pThis->read = incrementIndex(pThis->read);
    }
    nrf_atomic_u32_sub(&pThis->count, bytesToRead);
    return bytesToRead;
}

uint32_t CircularQueue_Peek(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );
    ASSERT ( pThis->peekSize == 0 );

    pThis->peek = pThis->read;
    uint32_t bytesToRead = (dataSize > pThis->count) ? pThis->count : dataSize;
    for (uint32_t i = 0 ; i < bytesToRead ; i++)
    {
        *pData++ = pThis->queue[pThis->peek];
        pThis->peek = incrementIndex(pThis->peek);
    }
    pThis->peekSize = bytesToRead;
    return bytesToRead;
}

void CircularQueue_CommitPeek(CircularQueue* pThis)
{
    ASSERT ( pThis->peekSize > 0 );

    pThis->read = pThis->peek;
    nrf_atomic_u32_sub(&pThis->count, pThis->peekSize);
    pThis->peekSize = 0;
}

void CircularQueue_RollbackPeek(CircularQueue* pThis)
{
    ASSERT ( pThis->peekSize > 0 );

    pThis->peekSize = 0;
}

CircularQueue g_mriToBleQueue;
CircularQueue g_bleToMriQueue;



// Forward Function Declarations
static void initLogging(void);
static void initTimers(void);
static void initButtonsAndLeds(bool * pEraseBonds);
static void initBleStack(void);
static void handleBleEvent(ble_evt_t * p_ble_evt);
static void handleBleEventsForApplication(ble_evt_t * pBleEvent);
static void handleSysEvent(uint32_t sysEvent);
static void initPeerManager(bool eraseBonds);
static void handlePeerManagerEvent(pm_evt_t const * pEvent);
static void initGapParams(void);
static void initBleUartService(void);
static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length);
static void initBleAdvertising(void);
static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent);
static void handleServiceError(uint32_t errorCode);
static void enterDeepSleep(void);
static void initConnectionParameters(void);
static void connectionParameterEventHandler(ble_conn_params_evt_t * pEvent);
static void startTimers(void);
static void timeoutHandler(void* pvContext);
static void checkForDataToSendOverBLE();
static uint32_t sendBlePacket(uint32_t bytesToSend);
static void checkForButtonPress();
static void startAdvertising(void);
static void getPeerList(pm_peer_id_t* pPeers, uint32_t* pCount);
static void enterLowPowerModeUntilNextEvent(void);



int main(void)
{
    initLogging();
    initTimers();

    bool eraseBonds = false;
    initButtonsAndLeds(&eraseBonds);
    initBleStack();
    initPeerManager(eraseBonds);
    initGapParams();
    initBleUartService();
    initBleAdvertising();
    initConnectionParameters();

    startTimers();
    startAdvertising();


    // Communicates events from Radio/BLE stack up to application code.
    NVIC_SetPriority(SWI1_EGU1_IRQn, 5);
    NVIC_SetPriority(SWI2_EGU2_IRQn, 5);
    // Timer1 is used to send MRI data to BLE stack (implements naggling algorithm as well.)
    NVIC_SetPriority(TIMER1_IRQn, 5);

    // Used by BSP code.
//    NVIC_SetPriority(SWI0_EGU0_IRQn, 5);
//    NVIC_SetPriority(RTC1_IRQn, 5);
//    NVIC_SetPriority(GPIOTE_IRQn, 5);

    mriInit("MRI_PRIORITY=6");
    __debugbreak();

    /* Toggle LEDs. */
    while (true)
    {
#ifdef UNDONE
        for (int i = 2; i < LEDS_NUMBER; i++)
        {
            bsp_board_led_invert(i);
            nrf_delay_ms(500);
        }
#endif // UNDONE
        __debugbreak();
    }

    // Enter main loop.
    while (true)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            enterLowPowerModeUntilNextEvent();
        }
    }
}

static void initLogging(void)
{
    uint32_t errorCode = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(errorCode);
}


static void initTimers(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    app_simple_timer_init();
}

static void initButtonsAndLeds(bool* pEraseBonds)
{
    // Configure the pin used to enable bonding and erasing of old bonds during reset.
    nrf_gpio_cfg_input(BONDING_SWITCH_PIN, NRF_GPIO_PIN_PULLUP);

    // Calling nrf_gpio_pin_read() right after nrf_gpio_cfg_input() on nRF52 device can result in an incorrect
    // read because the CPU runs 4 times faster than the GPIO block. Reading back GPIO pin configuration forces
    // sync.
    NRF_P0->PIN_CNF[BONDING_SWITCH_PIN];
    // Add an extra millisecond just to be safe.
    nrf_delay_ms(1);

    // If this button is pressed during reset then the user wants to delete the bonds.
    *pEraseBonds = false;
    if (nrf_gpio_pin_read(BONDING_SWITCH_PIN) == 0)
    {
        *pEraseBonds = true;
    }
    g_checkForBondButton = true;
}

static void initBleStack(void)
{
    uint32_t errorCode;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t defaultBleParams;
    errorCode = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                     PERIPHERAL_LINK_COUNT,
                                                     &defaultBleParams);
    APP_ERROR_CHECK(errorCode);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    defaultBleParams.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    errorCode = softdevice_enable(&defaultBleParams);
    APP_ERROR_CHECK(errorCode);

    // Subscribe for BLE events.
    errorCode = softdevice_ble_evt_handler_set(handleBleEvent);
    APP_ERROR_CHECK(errorCode);

    // Register with the SoftDevice handler module for System events.
    errorCode = softdevice_sys_evt_handler_set(handleSysEvent);
    APP_ERROR_CHECK(errorCode);
}

static void handleBleEvent(ble_evt_t * pBleEvent)
{
    ble_conn_state_on_ble_evt(pBleEvent);
    pm_on_ble_evt(pBleEvent);
    ble_conn_params_on_ble_evt(pBleEvent);
    ble_nus_on_ble_evt(&g_nordicUartService, pBleEvent);
    handleBleEventsForApplication(pBleEvent);
    ble_advertising_on_ble_evt(pBleEvent);
}

static void handleBleEventsForApplication(ble_evt_t * pBleEvent)
{
    uint32_t errorCode;

    switch (pBleEvent->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected\r\n");
            g_currBleConnection = BLE_CONN_HANDLE_INVALID;
            g_checkForBondButton = true;
            // Delete this peer from database if it didn't use MITM encryption.
            if (g_peerToDelete != PM_PEER_ID_INVALID)
            {
                ret_code_t returnCode = pm_peer_delete(g_peerToDelete);
                APP_ERROR_CHECK(returnCode);
                NRF_LOG_DEBUG("Collector's bond deleted\r\n");
                g_peerToDelete = PM_PEER_ID_INVALID;
            }
            if (g_whitelistModified)
            {
                // The whitelist has been modified for this last connection, update it in the Peer Manager.
                errorCode = pm_whitelist_set(g_whitelistPeers, g_whitelistPeerCount);
                APP_ERROR_CHECK(errorCode);

                errorCode = pm_device_identities_list_set(g_whitelistPeers, g_whitelistPeerCount);
                if (errorCode != NRF_ERROR_NOT_SUPPORTED)
                {
                    APP_ERROR_CHECK(errorCode);
                }

                g_whitelistModified = false;
            }
            break;
        }

        case BLE_GAP_EVT_CONNECTED:
        {
            g_peerToDelete = PM_PEER_ID_INVALID;
            NRF_LOG_INFO("Connected\r\n");
            g_currBleConnection = pBleEvent->evt.gap_evt.conn_handle;
            break;
        }

        case BLE_EVT_TX_COMPLETE:
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            errorCode = sd_ble_gatts_sys_attr_set(g_currBleConnection, NULL, 0, 0);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            char passkey[PASSKEY_LENGTH + 1];
            memcpy(passkey, pBleEvent->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;
            NRF_LOG_INFO("Passkey: %s\r\n", nrf_log_push(passkey));
            break;
        }

        case BLE_EVT_USER_MEM_REQUEST:
            errorCode = sd_ble_user_mem_reply(pBleEvent->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = pBleEvent->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    errorCode = sd_ble_gatts_rw_authorize_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(errorCode);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            errorCode = sd_ble_gatts_exchange_mtu_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

static void handleSysEvent(uint32_t sysEvent)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sysEvent);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sysEvent);
}

static void initPeerManager(bool eraseBonds)
{
    ble_gap_sec_params_t securityParam;
    ret_code_t           errorCode;

    errorCode = pm_init();
    APP_ERROR_CHECK(errorCode);

    if (eraseBonds)
    {
        NRF_LOG_INFO("Erasing bond information by user request.\r\n");
        errorCode = pm_peers_delete();
        APP_ERROR_CHECK(errorCode);
    }

    memset(&securityParam, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    securityParam.bond           = SEC_PARAM_BOND;
    securityParam.mitm           = SEC_PARAM_MITM;
    securityParam.lesc           = SEC_PARAM_LESC;
    securityParam.keypress       = SEC_PARAM_KEYPRESS;
    securityParam.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    securityParam.oob            = SEC_PARAM_OOB;
    securityParam.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    securityParam.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    securityParam.kdist_own.enc  = 1;
    securityParam.kdist_own.id   = 1;
    securityParam.kdist_peer.enc = 1;
    securityParam.kdist_peer.id  = 1;

    errorCode = pm_sec_params_set(&securityParam);
    APP_ERROR_CHECK(errorCode);

    errorCode = pm_register(handlePeerManagerEvent);
    APP_ERROR_CHECK(errorCode);
}

static void handlePeerManagerEvent(pm_evt_t const * pEvent)
{
    ret_code_t errorCode;

    switch (pEvent->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            pm_conn_sec_status_t connectionSecurityStatus;

            // Check if the link is authenticated (meaning at least MITM).
            errorCode = pm_conn_sec_status_get(pEvent->conn_handle, &connectionSecurityStatus);
            APP_ERROR_CHECK(errorCode);

            if (connectionSecurityStatus.mitm_protected)
            {
                NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                             ble_conn_state_role(pEvent->conn_handle),
                             pEvent->conn_handle,
                             pEvent->params.conn_sec_succeeded.procedure);
                g_peerId = pEvent->peer_id;

                if (pEvent->params.conn_sec_succeeded.procedure == PM_LINK_SECURED_PROCEDURE_BONDING)
                {
                    NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible\r\n");
                    NRF_LOG_INFO("\tg_whiteListPeerCount: %d   MAX_PEERS_WLIST: %d\r\n",
                                g_whitelistPeerCount + 1,
                                BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
                    if (g_whitelistPeerCount < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                    {
                        // Bonded to a new peer, add it to the whitelist.
                        g_whitelistPeers[g_whitelistPeerCount++] = g_peerId;
                        g_whitelistModified = true;
                    }
                }
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting\r\n");
                errorCode = pm_peer_id_get(g_currBleConnection, &g_peerToDelete);
                APP_ERROR_CHECK(errorCode);
                errorCode = sd_ble_gap_disconnect(g_currBleConnection,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(errorCode);
            }
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            NRF_LOG_INFO("Failed to secure connection. Disconnecting.\r\n");
            errorCode = sd_ble_gap_disconnect(g_currBleConnection,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t connectionSecurityConfig = {.allow_repairing = false};
            pm_conn_sec_config_reply(pEvent->conn_handle, &connectionSecurityConfig);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            errorCode = fds_gc();
            if (errorCode == FDS_ERR_BUSY || errorCode == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(errorCode);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            startAdvertising();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(pEvent->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(pEvent->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(pEvent->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(pEvent->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

static void initGapParams(void)
{
    uint32_t                errorCode;
    ble_gap_conn_params_t   gapPreferredConnectionParams;
    ble_gap_conn_sec_mode_t securityMode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&securityMode);

    errorCode = sd_ble_gap_device_name_set(&securityMode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(errorCode);

    memset(&gapPreferredConnectionParams, 0, sizeof(gapPreferredConnectionParams));

    gapPreferredConnectionParams.min_conn_interval = MIN_CONN_INTERVAL;
    gapPreferredConnectionParams.max_conn_interval = MAX_CONN_INTERVAL;
    gapPreferredConnectionParams.slave_latency     = SLAVE_LATENCY;
    gapPreferredConnectionParams.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    errorCode = sd_ble_gap_ppcp_set(&gapPreferredConnectionParams);
    APP_ERROR_CHECK(errorCode);

    static uint8_t passkey[] = STATIC_PASSKEY;
    ble_opt_t bleOption;
	bleOption.gap_opt.passkey.p_passkey = passkey;
	errorCode =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &bleOption);
	APP_ERROR_CHECK(errorCode);
}

static void initBleUartService(void)
{
    ble_nus_init_t nordicUartServiceParams;

    memset(&nordicUartServiceParams, 0, sizeof(nordicUartServiceParams));

    nordicUartServiceParams.data_handler = nordicUartServiceHandler;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&nordicUartServiceParams.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&nordicUartServiceParams.rx_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&nordicUartServiceParams.tx_write_perm);

    uint32_t errorCode = ble_nus_init(&g_nordicUartService, &nordicUartServiceParams);
    APP_ERROR_CHECK(errorCode);
}

static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length)
{
    uint32_t bytesWritten = CircularQueue_Write(&g_bleToMriQueue, pData, length);
    ASSERT ( bytesWritten == length );
}

static void initBleAdvertising(void)
{
    uint32_t               errorCode;
    ble_advdata_t          advertisingData;
    ble_advdata_t          scanResponse;
    ble_adv_modes_config_t options;

    // Data to be sent with each advertising cycle.
    // Place UUIDs in the main advertising packet and the name in the scan response which should only be
    // returned to devices on the whitelist.
    memset(&advertisingData, 0, sizeof(advertisingData));
    advertisingData.uuids_complete.uuid_cnt = sizeof(g_advertiseUuids) / sizeof(g_advertiseUuids[0]);
    advertisingData.uuids_complete.p_uuids  = g_advertiseUuids;
    advertisingData.include_appearance = false;
    advertisingData.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    // Data to be sent back to central device if it sends a scan request.
    memset(&scanResponse, 0, sizeof(scanResponse));
    scanResponse.name_type          = BLE_ADVDATA_FULL_NAME;

    memset(&options, 0, sizeof(options));
    options.ble_adv_whitelist_enabled   = true;
    options.ble_adv_fast_enabled        = true;
    options.ble_adv_fast_interval       = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout        = APP_ADV_FAST_TIMEOUT;
    options.ble_adv_slow_enabled        = true;
    options.ble_adv_slow_interval       = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout        = APP_ADV_SLOW_TIMEOUT;
    errorCode = ble_advertising_init(&advertisingData,
                                    &scanResponse,
                                    &options,
                                    bleAdvertisingEventHandler,
                                    handleServiceError);
    APP_ERROR_CHECK(errorCode);
}

static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent)
{
    switch (bleAdvertisingEvent)
    {
        case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED\r\n");
            break; //BLE_ADV_EVT_DIRECTED

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("BLE_ADV_EVT_FAST\r\n");
            break; //BLE_ADV_EVT_FAST

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("BLE_ADV_EVT_SLOW\r\n");
            break; //BLE_ADV_EVT_SLOW

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("BLE_ADV_EVT_FAST_WHITELIST\r\n");
            break; //BLE_ADV_EVT_FAST_WHITELIST

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("BLE_ADV_EVT_SLOW_WHITELIST\r\n");
            break; //BLE_ADV_EVT_SLOW_WHITELIST

        case BLE_ADV_EVT_IDLE:
            enterDeepSleep();
            break; //BLE_ADV_EVT_IDLE

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            NRF_LOG_INFO("BLE_ADV_EVT_WHITELIST_REQUEST\r\n");

            ble_gap_addr_t whitelistAddrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelistIrks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addrCount = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irkCount  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            uint32_t errorCode = pm_whitelist_get(whitelistAddrs, &addrCount, whitelistIrks, &irkCount);
            APP_ERROR_CHECK(errorCode);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist\r\n",
                           addrCount, irkCount);

            // Apply the whitelist.
            errorCode = ble_advertising_whitelist_reply(whitelistAddrs, addrCount, whitelistIrks, irkCount);
            APP_ERROR_CHECK(errorCode);
        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            NRF_LOG_INFO("BLE_ADV_EVT_PEER_ADDR_REQUEST\r\n");

            // Only Give peer address if we have a handle to the bonded peer.
            if (g_peerId != PM_PEER_ID_INVALID)
            {
                pm_peer_data_bonding_t peerBondingData;
                uint32_t errorCode = pm_peer_data_bonding_load(g_peerId, &peerBondingData);
                if (errorCode != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(errorCode);

                    ble_gap_addr_t * pPeerAddr = &(peerBondingData.peer_ble_id.id_addr_info);
                    errorCode = ble_advertising_peer_addr_reply(pPeerAddr);
                    APP_ERROR_CHECK(errorCode);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

        default:
            break;
    }
}

static void enterDeepSleep(void)
{
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint32_t errorCode = sd_power_system_off();
    APP_ERROR_CHECK(errorCode);
}

static void handleServiceError(uint32_t errorCode)
{
    APP_ERROR_HANDLER(errorCode);
}

static void initConnectionParameters(void)
{
    uint32_t               errorCode;
    ble_conn_params_init_t initParams;

    memset(&initParams, 0, sizeof(initParams));

    initParams.p_conn_params                  = NULL;
    initParams.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    initParams.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    initParams.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    initParams.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    initParams.disconnect_on_fail             = false;
    initParams.evt_handler                    = connectionParameterEventHandler;
    initParams.error_handler                  = handleServiceError;

    errorCode = ble_conn_params_init(&initParams);
    APP_ERROR_CHECK(errorCode);
}

static void connectionParameterEventHandler(ble_conn_params_evt_t * pEvent)
{
    uint32_t errorCode;

    if (pEvent->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        errorCode = sd_ble_gap_disconnect(g_currBleConnection, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(errorCode);
    }
}

static void startTimers(void)
{
    // Start the timer used to check for button presses.
    app_simple_timer_start(APP_SIMPLE_TIMER_MODE_REPEATED,
                            timeoutHandler,
                            NAGLE_TIME_MICROSECONDS,
                            NULL);
}

static void timeoutHandler(void* pvContext)
{
    checkForDataToSendOverBLE();
    checkForButtonPress();
}

static void checkForDataToSendOverBLE()
{
    static uint32_t lastCount = 0;

    // Exit early if there is no BLE connection over which to send data.
    if (g_currBleConnection == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    uint32_t count = CircularQueue_BytesToRead(&g_mriToBleQueue);
    if (count == 0)
    {
        // Nothing to send so just return.
        lastCount = 0;
        return;
    }
    else if (count >= BLE_NUS_MAX_DATA_LEN)
    {
        // Can fill up 1 or more BLE packets.
        while (count >= BLE_NUS_MAX_DATA_LEN)
        {
            uint32_t bytesRead = sendBlePacket(BLE_NUS_MAX_DATA_LEN);
            count -= bytesRead;
            if (bytesRead == 0)
            {
                // BLE stack couldn't send packet at this time so exit loop early.
                break;
            }
        }
    }
    else if (count == lastCount)
    {
        // No new data has been added to the queue by MRI since the last tick so send what we have so far.
        uint32_t bytesRead = sendBlePacket(count);
        count -= bytesRead;
    }
    ASSERT ( count == CircularQueue_BytesToRead(&g_mriToBleQueue) );
    lastCount = count;
}

static uint32_t sendBlePacket(uint32_t bytesToSend)
{
    uint8_t data[BLE_NUS_MAX_DATA_LEN];

    // Copy the specified number of bytes out of the queue and only update the read index in the queue if the BLE send
    // succeeds. That is why the Peek method is used.
    ASSERT ( bytesToSend <= sizeof(data) );
    uint32_t bytesRead = CircularQueue_Peek(&g_mriToBleQueue, data, bytesToSend);
    ASSERT ( bytesRead == bytesToSend );

    // Attempt to send the MRI data to the PC via BLE.
    uint32_t errorCode  = ble_nus_string_send(&g_nordicUartService, data, bytesRead);
    if (errorCode == NRF_SUCCESS)
    {
        // Data has been successfully queued up to the BLE stack so advance the read index in the queue.
        CircularQueue_CommitPeek(&g_mriToBleQueue);
        return bytesRead;
    }

    // Data was not successfully queued up to the BLE stack.
    CircularQueue_RollbackPeek(&g_mriToBleQueue);
    if (errorCode != NRF_ERROR_INVALID_STATE && errorCode != BLE_ERROR_NO_TX_PACKETS)
    {
        APP_ERROR_CHECK(errorCode);
    }
    return 0;
}

static void checkForButtonPress()
{
    if (g_checkForBondButton &&
        g_currBleConnection == BLE_CONN_HANDLE_INVALID &&
        nrf_gpio_pin_read(BONDING_SWITCH_PIN) == 0)
    {
        g_checkForBondButton = false;
        uint32_t errorCode = ble_advertising_restart_without_whitelist();
        if (errorCode != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(errorCode);
        }
    }
}

static void startAdvertising(void)
{
    memset(g_whitelistPeers, PM_PEER_ID_INVALID, ARRAY_SIZE(g_whitelistPeers));
    g_whitelistPeerCount = ARRAY_SIZE(g_whitelistPeers);

    getPeerList(g_whitelistPeers, &g_whitelistPeerCount);

    ret_code_t errorCode = pm_whitelist_set(g_whitelistPeers, g_whitelistPeerCount);
    APP_ERROR_CHECK(errorCode);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    errorCode = pm_device_identities_list_set(g_whitelistPeers, g_whitelistPeerCount);
    if (errorCode != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(errorCode);
    }

    g_whitelistModified = false;

    errorCode = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(errorCode);
}

static void getPeerList(pm_peer_id_t* pPeers, uint32_t* pCount)
{
    uint32_t     count = *pCount;
    pm_peer_id_t peerId;
    uint32_t     peersToCopy;

    peersToCopy = (count < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ? count : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peerId = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    count = 0;

    while ((peerId != PM_PEER_ID_INVALID) && (peersToCopy--))
    {
        pPeers[count++] = peerId;
        peerId = pm_next_peer_id_get(peerId);
    }
    *pCount = count;
}

static void enterLowPowerModeUntilNextEvent(void)
{
    uint32_t errorCode = sd_app_evt_wait();
    APP_ERROR_CHECK(errorCode);
}



// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}




void mriNRF52Uart_Init(void* pParameterTokens, uint32_t debugMonPriorityLevel)
{
}

uint32_t mriPlatform_CommHasReceiveData(void)
{
    return !CircularQueue_IsEmpty(&g_bleToMriQueue);
}


uint32_t  mriPlatform_CommHasTransmitCompleted(void)
{
    return CircularQueue_IsEmpty(&g_mriToBleQueue);
}


static void waitToReceiveData(void);
int mriPlatform_CommReceiveChar(void)
{
    waitToReceiveData();

    uint8_t byte = 0;
    uint32_t bytesRead = CircularQueue_Read(&g_bleToMriQueue, &byte, sizeof(byte));
    ASSERT ( bytesRead == sizeof(byte) );

    return (int)byte;
}

static void waitToReceiveData(void)
{
    while (!mriPlatform_CommHasReceiveData())
    {
    }
}

static void waitForSpaceInQueue();
void mriPlatform_CommSendChar(int Character)
{
    waitForSpaceInQueue();

    uint8_t byte = (uint8_t)Character;
    uint32_t bytesAdded = CircularQueue_Write(&g_mriToBleQueue, &byte, sizeof(byte));
    ASSERT ( bytesAdded == sizeof(byte) );
}

static void waitForSpaceInQueue()
{
    while (CircularQueue_IsFull(&g_mriToBleQueue))
    {
    }
}
