/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* mriblue_boot - Bootloader to enable MRI debugging and programming wirelessly over BLE.

   This program is heavily influenced by Nordic's BLE UART Service (ble_app_uart) SDK sample.
*/
#include <app_timer.h>
#include <app_simple_timer.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_conn_state.h>
#include <ble_nus.h>
#include <bsp.h>
#include <fds.h>
#include <fstorage.h>
#include <nrf_atomic.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#define NRF_LOG_MODULE_NAME "APP"
#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <peer_manager.h>
#include <softdevice_handler.h>
#include <architectures/armv7-m/debug_cm3.h>
#include <architectures/armv7-m/armv7-m.h>
#include <core/mri.h>
#include <core/cmd_common.h>
#include <core/core.h>
#include "app.h"
#include "CircularQueue.h"



// UNDONE: Currently using switch 4 which is also connected to RMOTOR_DIAG by my shield.
// The pin connected to a switch to be pressed to enable pairing and during reset to
// erase previous bonding information from Peer Manager.
#define BONDING_SWITCH_PIN              NRF_GPIO_PIN_MAP(0, 16)

// The name of this device.
#define DEVICE_NAME                     "mriblue"

// The unique portion of the serivce UUID advertised by this device.
#define MRIBLUE_ADVERTISE               0xADA4

// If haven't received any more data from MRI in this amount of bit times, then send what we have already received.
// Will run timer with this period. This value is selected because there are ~558us between the start of a full
// BLE packet and the time an ACK comes back.
#define NAGLE_TIME_MICROSECONDS         500

// Priority level to run MRI debug monitor at.
#define MRIBLUE_PRIORITY                6

// Macro to convert priority level to MRI parameter string.
#define MRI_PRIORITY_STRING(PRIO)       MRI_PRIORITY_STRING2(PRIO)
#define MRI_PRIORITY_STRING2(PRIO)      "MRI_PRIORITY=" #PRIO

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
static ble_nus_t            g_nordicUartService;

// Current BLE connection.
static uint16_t             g_currBleConnection = BLE_CONN_HANDLE_INVALID;

// UUIDS returned in advertising scan response.
static ble_uuid_t           g_advertiseUuids[] = {{MRIBLUE_ADVERTISE, BLE_UUID_TYPE_VENDOR_BEGIN}};

// Used to track peers that didn't use MITM and should be deleted from Peer Manager on disconnect.
static pm_peer_id_t         g_peerToDelete = PM_PEER_ID_INVALID;

// Peer Manager handle to the currently bonded central.
static pm_peer_id_t         g_peerId = PM_PEER_ID_INVALID;

// Flag to indicate whether the timer routine should still check for bond button presses.
static bool                 g_checkForBondButton = true;

// Whitelist of peers.
static pm_peer_id_t         g_whitelistPeers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
// Number of peers currently in g_whitelistPeers.
static uint32_t             g_whitelistPeerCount;
// Has the whitelist been updated since last updated in the Peer Manager.
static bool                 g_whitelistModified;

// The number of BLE packets in flight.
static volatile uint32_t    g_packetsInFlight;

// Circular queues used to move bytes between MRI and BLE interrupt levels.
static CircularQueue        g_mriToBleQueue;
static CircularQueue        g_bleToMriQueue;

// Used to let main MRI thread know when SoftDevice has completed FLASH commands.
typedef enum FlashState
{
    FLASH_STATE_IDLE,
    FLASH_STATE_STARTED,
    FLASH_STATE_COMPLETED_SUCCESS,
    FLASH_STATE_COMPLETED_ERROR
} FlashState;
static volatile FlashState  g_flashState = FLASH_STATE_IDLE;



// Forward Function Declarations
static void initLogging(void);
static void initTimers(void);
static void initButtonsAndLeds(bool * pEraseBonds);
static void initBleStack(void);
static void handleBleEvent(ble_evt_t * p_ble_evt);
static void handleBleEventsForApplication(ble_evt_t * pBleEvent);
static void handleSysEvent(uint32_t sysEvent);
static void handleFlashEvent(uint32_t sysEvent);
static void initPeerManager(bool eraseBonds);
static void handlePeerManagerEvent(pm_evt_t const * pEvent);
static void initGapParams(void);
static void initBleUartService(void);
static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length);
static bool isAlreadyDebugging(void);
static void setControlCFlag(void);
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
static void elevateInterruptPriorities(void);
static int enteredResetHandlerCallback(void* pvContext);



// *********************************************************************************************************************
//  Code to setup Bluetooth low energy stack, initialize the MRI debug monitor, and then jump to the application's
//  Reset_Handler.
// *********************************************************************************************************************
int main(void)
{
    initLogging();
    initTimers();

    // Initialize BLE stack.
    bool eraseBonds = false;
    initButtonsAndLeds(&eraseBonds);
    initBleStack();
    initPeerManager(eraseBonds);
    initGapParams();
    initBleUartService();
    initBleAdvertising();
    initConnectionParameters();

    // Initialize the MRI debug monitor and prepare it to communicate to GDB over BLE.
    elevateInterruptPriorities();
    mriInit(MRI_PRIORITY_STRING(MRIBLUE_PRIORITY));

    // Let the BLE stack advertising that it is available for connections.
    startTimers();
    startAdvertising();

    // Find the location of the Reset_Handler for the main application which occurs a bit higher in the FLASH.
    uint32_t* pAppIsrVectors = (uint32_t*)APP_START;
    uint32_t resetHandler = pAppIsrVectors[1];
    if (resetHandler == 0xFFFFFFFF)
    {
        // The Reset_Handler is 0xFFFFFFFF which means that the first page of the application code has been erased
        // and contains no valid data so just stop here.
        __debugbreak();
        while (1)
        {
        }
    }

    // Set the initial breakpoint at the beginning of the app's Reset_Handler, allowing the developer to set breakpoints
    // of interest. This is early enough to set breakpoints in global constructors and is the only address this
    // bootloader really knows about in the application. It doesn't know the location of main() for example.
    mriCore_SetTempBreakpoint(resetHandler, enteredResetHandlerCallback, NULL);

    // Jump to the application's Reset_Handler.
    void (*pResetHandler)(void) = (void (*)(void))resetHandler;
    pResetHandler();

    // Should never return here but if we do, halt.
    __debugbreak();
    while (1)
    {
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
            g_packetsInFlight = 0;
            break;
        }

        case BLE_GAP_EVT_CONNECTED:
        {
            g_peerToDelete = PM_PEER_ID_INVALID;
            NRF_LOG_INFO("Connected\r\n");
            g_currBleConnection = pBleEvent->evt.gap_evt.conn_handle;
            g_packetsInFlight = 0;
            break;
        }

        case BLE_EVT_TX_COMPLETE:
            nrf_atomic_u32_sub(&g_packetsInFlight, pBleEvent->evt.common_evt.params.tx_complete.count);
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

    handleFlashEvent(sysEvent);
}

static void handleFlashEvent(uint32_t sysEvent)
{
    if (g_flashState != FLASH_STATE_STARTED)
    {
        return;
    }

    switch (sysEvent)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            g_flashState =  FLASH_STATE_COMPLETED_SUCCESS;
            break;
        case NRF_EVT_FLASH_OPERATION_ERROR:
            g_flashState =  FLASH_STATE_COMPLETED_ERROR;
            break;
    }

    return;
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
    if (!isAlreadyDebugging())
    {
        // GDB is sending a command, probably a CTRL+C, so pend entry into DebugMon handler.
        setControlCFlag();
        setMonitorPending();
    }
}

static bool isAlreadyDebugging(void)
{
    return mriCortexMFlags & CORTEXM_FLAGS_ACTIVE_DEBUG;
}

static void setControlCFlag(void)
{
    mriCortexMFlags |= CORTEXM_FLAGS_CTRL_C;
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
        nrf_atomic_u32_add(&g_packetsInFlight, 1);
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

static void elevateInterruptPriorities(void)
{
    // Make sure that all interrupts required for BLE communications are running at a priority higher than MRI's
    // DebugMon handler.
    //  Communicates events from Radio/BLE stack up to application code via SWI1 and SWI2.
    NVIC_SetPriority(SWI1_EGU1_IRQn, MRIBLUE_PRIORITY-1);
    NVIC_SetPriority(SWI2_EGU2_IRQn, MRIBLUE_PRIORITY-1);
    //  Timer1 is used to send MRI data to BLE stack (implements naggling algorithm as well.)
    NVIC_SetPriority(TIMER1_IRQn, MRIBLUE_PRIORITY-1);
}

static int enteredResetHandlerCallback(void* pvContext)
{
    // We want MRI to stop at this breakpoint so return 0 to not resume execution.
    return 0;
}



// *********************************************************************************************************************
//  Nordic SDK will call the following routine if it encounters an unrecoverable error.
// *********************************************************************************************************************
// Break into debugger if any errors/asserts are detected at runtime.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __debugbreak();
    while (1)
    {
    }
}



// *********************************************************************************************************************
//  Implementation of the mriPlatform_Comm* functions.
//  The MRI debug monitor library calls these routines to communicate with GDB. The implementations below use the BLE
//  code from above to communicate with GDB wirelessly.
// *********************************************************************************************************************
void mriNRF52Uart_Init(void* pParameterTokens, uint32_t debugMonPriorityLevel)
{
    // Nothing to do here as the comm channel was already initialized by the time mriInit() was called.
    // Needs to be here though so that everything from the nrf52_uart module is redefined here so that nothing from
    // the mri library version gets linked in.
}

uint32_t mriPlatform_CommHasReceiveData(void)
{
    return !CircularQueue_IsEmpty(&g_bleToMriQueue);
}


uint32_t  mriPlatform_CommHasTransmitCompleted(void)
{
    return (CircularQueue_IsEmpty(&g_mriToBleQueue) && g_packetsInFlight == 0);
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



// *********************************************************************************************************************
//  Implementation of the mriPlatform_HandleGDBComand() function.
//  The MRI debug monitor library calls this routine to allow us to handle GDB remote commands that aren't already
//  handled by the MRI core code. For mriblue, we are going to add in the ability to handle the commands related to
//  programming the FLASH. This will add support for GDB's "load" command.
// *********************************************************************************************************************
static uint32_t  handleFlashEraseCommand(Buffer* pBuffer);
static __throws void throwIfAddressAndLengthNot4kAligned(AddressLength* pAddressLength);
static bool is4kAligned(uint32_t value);
static __throws void throwIfAttemptingToFlashSoftDeviceOrBootloader(AddressLength* pAddressLength);
static uint32_t eraseFlashPages(uint32_t startPage, uint32_t pageCount);
static uint32_t eraseFlashPage(uint32_t page);
static uint32_t waitForFlashOperationToCompleteAndCheckForError(void);
static uint32_t  handleFlashWriteCommand(Buffer* pBuffer);
static uint32_t writeToFlash(Buffer* pBuffer, AddressLength* pAddressLength);
static uint32_t alignStartOfWriteByCopyingExistingFlashData(uint8_t* pDest, uint8_t* pSrc, uint32_t unalignedStart);
static uint32_t copyEscapedBytes(void* pvDest, size_t destSize, Buffer* pBuffer);
static char unescapeCharIfNecessary(Buffer* pBuffer, char currentChar);
static int isEscapePrefixChar(char charToCheck);
static char readNextCharAndUnescape(Buffer* pBuffer);
static char unescapeByte(char charToUnescape);
static uint32_t  handleFlashDoneCommand(Buffer* pBuffer);

/* Handle the 'vFlash' commands used by gdb.

    Command Format: vFlashSSS
    Where SSS is a variable length string indicating which Flash command is being sent to the stub.
*/
uint32_t  mriPlatform_HandleGDBCommand(Buffer* pBuffer)
{
    const char   vFlashEraseCommand[] = "vFlashErase";
    const char   vFlashWriteCommand[] = "vFlashWrite";
    const char   vFlashDoneCommand[] = "vFlashDone";

    Buffer_Reset(pBuffer);
    if (Buffer_MatchesString(pBuffer, vFlashEraseCommand, sizeof(vFlashEraseCommand)-1))
    {
        return handleFlashEraseCommand(pBuffer);
    }
    else if (Buffer_MatchesString(pBuffer, vFlashWriteCommand, sizeof(vFlashWriteCommand)-1))
    {
        return handleFlashWriteCommand(pBuffer);
    }
    else if (Buffer_MatchesString(pBuffer, vFlashDoneCommand, sizeof(vFlashDoneCommand)-1))
    {
        return handleFlashDoneCommand(pBuffer);
    }
    else
    {
        // Returning 0 means that MRI should handle this command instead.
        return 0;
    }
}

/* Handle the 'vFlashErase' command which erases the specified pages in FLASH.

    Command Format:     vFlashErase:AAAAAAAA,LLLLLLLL
    Response Format:    OK

    Where AAAAAAAA is the hexadecimal representation of the address where the erase is to start.
          LLLLLLLL is the hexadecimal representation of the length (in bytes) of the erase to be conducted.
*/
static uint32_t  handleFlashEraseCommand(Buffer* pBuffer)
{
    AddressLength  addressLength;

    __try
    {
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ':') );
        __throwing_func( ReadAddressAndLengthArguments(pBuffer, &addressLength) );
        __throwing_func( throwIfAddressAndLengthNot4kAligned(&addressLength) );
        __throwing_func( throwIfAttemptingToFlashSoftDeviceOrBootloader(&addressLength) );
    }
    __catch
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }

    uint32_t startPage = addressLength.address / 4096;
    uint32_t pageCount = addressLength.length / 4096;
    uint32_t errorCode = eraseFlashPages(startPage, pageCount);
    if (errorCode != NRF_SUCCESS)
    {
        PrepareStringResponse(MRI_ERROR_MEMORY_ACCESS_FAILURE);
        return HANDLER_RETURN_HANDLED;
    }

    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}

static __throws void throwIfAddressAndLengthNot4kAligned(AddressLength* pAddressLength)
{
    if (!is4kAligned(pAddressLength->address) || !is4kAligned(pAddressLength->length))
    {
        __throw(invalidArgumentException);
    }
}

static bool is4kAligned(uint32_t value)
{
    return (value & (4096-1)) == 0;
}

static __throws void throwIfAttemptingToFlashSoftDeviceOrBootloader(AddressLength* pAddressLength)
{
    if (pAddressLength->address < APP_START)
    {
        __throw(invalidArgumentException);
    }
}

static uint32_t eraseFlashPages(uint32_t startPage, uint32_t pageCount)
{
    for (uint32_t i = 0 ; i < pageCount ; i++)
    {
        uint32_t errorCode = eraseFlashPage(startPage+i);
        if (errorCode != NRF_SUCCESS)
        {
            return errorCode;
        }
    }

    return NRF_SUCCESS;
}

static uint32_t eraseFlashPage(uint32_t page)
{
    g_flashState = FLASH_STATE_STARTED;

    uint32_t errorCode = sd_flash_page_erase(page);
    if (errorCode != NRF_SUCCESS)
    {
        g_flashState = FLASH_STATE_IDLE;
        return errorCode;
    }
    errorCode = waitForFlashOperationToCompleteAndCheckForError();

    g_flashState = FLASH_STATE_IDLE;

    return errorCode;
}

static uint32_t waitForFlashOperationToCompleteAndCheckForError(void)
{
    while (g_flashState == FLASH_STATE_STARTED)
    {
        // Waiting for FLASH command to complete down in the SoftDevice.
    }
    if (g_flashState == FLASH_STATE_COMPLETED_ERROR)
    {
        return NRF_ERROR_BUSY;
    }

    return NRF_SUCCESS;
}

/* Handle the 'vFlashWrite' command which writes to the specified location in FLASH.

    Command Format:     vFlashWrite:AAAAAAAA:XX...
    Response Format:    OK

    Where AAAAAAAA is the hexadecimal representation of the address where the write is to start.
          xx is the byte in escaped binary format of the first byte to be written to the specified location.
          ... continue returning the rest of the bytes in escaped binary format.
*/
static uint32_t  handleFlashWriteCommand(Buffer* pBuffer)
{
    AddressLength  addressLength = {0, 0};

    __try
    {
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ':') );
        __throwing_func( addressLength.address = ReadUIntegerArgument(pBuffer) );
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ':') );
        __throwing_func( throwIfAttemptingToFlashSoftDeviceOrBootloader(&addressLength) );
    }
    __catch
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }

    uint32_t errorCode = writeToFlash(pBuffer, &addressLength);
    if (errorCode != NRF_SUCCESS)
    {
        PrepareStringResponse(MRI_ERROR_MEMORY_ACCESS_FAILURE);
        return HANDLER_RETURN_HANDLED;
    }

    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}

static uint32_t writeToFlash(Buffer* pBuffer, AddressLength* pAddressLength)
{
    // Need to align both the address and length to 4-bytes.
    uint32_t startAddress = pAddressLength->address;
    uint32_t alignedStartAddress = startAddress & ~(sizeof(uint32_t)-1);

    // Copy bytes into 4-byte aligned buffer on the stack.
    uint32_t wordBuffer[CORTEXM_PACKET_BUFFER_SIZE/sizeof(uint32_t)+2];

    // Align start of FLASH write to word boundary by padding with existing bytes from beginning of first word in FLASH.
    uint32_t bytesLeft = sizeof(wordBuffer);
    uint8_t* pDest = (uint8_t*)wordBuffer;
    uint32_t size = alignStartOfWriteByCopyingExistingFlashData(pDest, (uint8_t*)alignedStartAddress, startAddress);
    bytesLeft -= size;
    pDest += size;

    // Copy the escaped bytes provided by GDB into aligned buffer.
    size = copyEscapedBytes(pDest, bytesLeft, pBuffer);
    bytesLeft -= size;
    pDest += size;

    // Pad last few bytes with 0xFF to make the length of the write word aligned.
    uint32_t endAddress = startAddress + size;
    uint32_t alignedEndAddress = (endAddress+sizeof(uint32_t)-1) & ~(sizeof(uint32_t)-1);
    memset(pDest, 0xFF, alignedEndAddress-endAddress);
    uint32_t wordCount = (alignedEndAddress-alignedStartAddress)/sizeof(uint32_t);

    g_flashState = FLASH_STATE_STARTED;

    uint32_t errorCode = sd_flash_write((uint32_t*)alignedStartAddress, wordBuffer, wordCount);
    if (errorCode != NRF_SUCCESS)
    {
        g_flashState = FLASH_STATE_IDLE;
        return errorCode;
    }
    errorCode = waitForFlashOperationToCompleteAndCheckForError();

    g_flashState = FLASH_STATE_IDLE;

    return errorCode;
}

static uint32_t alignStartOfWriteByCopyingExistingFlashData(uint8_t* pDest, uint8_t* pSrc, uint32_t unalignedStart)
{
    uint8_t* pStart = pDest;
    while (pSrc < (uint8_t*)unalignedStart)
    {
        *pDest++ = *pSrc++;
    }

    return pDest - pStart;
}

static uint32_t copyEscapedBytes(void* pvDest, size_t destSize, Buffer* pBuffer)
{
    uint8_t* pStart = pvDest;
    uint8_t* pDest = pvDest;
    while (destSize-- > 0)
    {
        char currChar;

        // Keep reading bytes until we reach the end of the source buffer.
        __try
        {
            __throwing_func( currChar = Buffer_ReadChar(pBuffer) );
        }
        __catch
        {
            break;
        }

        // UNDONE: Could catch this error as well.
        currChar = unescapeCharIfNecessary(pBuffer, currChar);
        *pDest++ = currChar;
    }
    return pDest - pStart;
}

static char unescapeCharIfNecessary(Buffer* pBuffer, char currentChar)
{
    if (isEscapePrefixChar(currentChar))
        return readNextCharAndUnescape(pBuffer);

    return currentChar;
}

static int isEscapePrefixChar(char charToCheck)
{
    return charToCheck == '}';
}

static char readNextCharAndUnescape(Buffer* pBuffer)
{
    char nextChar;

    __try
        nextChar = Buffer_ReadChar(pBuffer);
    __catch
        __rethrow_and_return('\0');

    return unescapeByte(nextChar);
}

static char unescapeByte(char charToUnescape)
{
    return charToUnescape ^ 0x20;
}

/* Handle the 'vFlashDone' command which lets us know that the FLASH update is now complete.

    Command Format:     vFlashDone
    Response Format:    OK
*/
static uint32_t  handleFlashDoneCommand(Buffer* pBuffer)
{
    /* We do all of the work as the vFlashErase/vFlashWrite commands come in so there is nothing to do here. */
    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}



// *********************************************************************************************************************
//  Implementation of the mriFaultHandler() function. This routine is called from the assembly language based
//  fault handlers found in mriblue_asm.S when a fault is detected in the code. mriFaultHandler() actions depend on
//  whether the fault was detected in low or high priority code or caused by a memory read/write initiated by GDB.
// *********************************************************************************************************************
/* Lower nibble of EXC_RETURN in LR will have one of these values if interrupted code was running in thread mode.
   Using PSP. */
#define EXC_RETURN_THREADMODE_PROCESSSTACK  0xD
/*  Using MSP. */
#define EXC_RETURN_THREADMODE_MAINSTACK     0x9

/* This is the order that ARMv7-M stacks registers when an exception takes place. */
typedef struct ExceptionStack
{
    uint32_t    r0;
    uint32_t    r1;
    uint32_t    r2;
    uint32_t    r3;
    uint32_t    r12;
    uint32_t    lr;
    uint32_t    pc;
    uint32_t    xpsr;
    /* Need to check EXC_RETURN value in exception LR to see if these floating point registers have been stacked. */
    uint32_t    s0;
    uint32_t    s1;
    uint32_t    s2;
    uint32_t    s3;
    uint32_t    s4;
    uint32_t    s5;
    uint32_t    s6;
    uint32_t    s7;
    uint32_t    s8;
    uint32_t    s9;
    uint32_t    s10;
    uint32_t    s11;
    uint32_t    s12;
    uint32_t    s13;
    uint32_t    s14;
    uint32_t    s15;
    uint32_t    fpscr;
} ExceptionStack;


static ExceptionStack* getExceptionStack(uint32_t excReturn, uint32_t psp, uint32_t msp);
static bool isExceptionPriorityLowEnoughToDebug(uint32_t exceptionNumber);
static uint32_t getInterruptPriority(uint32_t exceptionNumber);
static void recordAndClearFaultStatusBits();
static void setPendedFromFaultBit(void);
static bool isDebugThreadActive();
static void setFaultDetectedFlag();
static bool isImpreciseBusFault();
static void advancePCToNextInstruction(ExceptionStack* pExceptionStack);
static int isInstruction32Bit(uint16_t firstWordOfInstruction);
static void clearFaultStatusBits();

// This handler will be called from the fault handlers (Hard Fault, etc.)
// If the fault occurred in low priority code that pend a DebugMon interrupt so that MRI can be used to debug it.
// Just infinite loops if it was in a high priority interrupt since MRI can't debug those type of faults.
void mriFaultHandler(uint32_t psp, uint32_t msp, uint32_t excReturn)
{
    const uint32_t debugMonExceptionNumber = (uint32_t)(DebugMonitor_IRQn + 16);
    ExceptionStack* pExceptionStack = getExceptionStack(excReturn, psp, msp);
    uint32_t exceptionNumber = pExceptionStack->xpsr & 0xFF;
    if (isDebugThreadActive() && exceptionNumber == debugMonExceptionNumber)
    {
        // Encountered memory fault when GDB attempted to access an invalid address.
        // Set flag to let MRI know that its access failed and advance past the faulting instruction
        // if it was a precise bus fault so that it doesn't just occur again on return.
        setFaultDetectedFlag();
        if (!isImpreciseBusFault())
        {
            advancePCToNextInstruction(pExceptionStack);
        }
        clearFaultStatusBits();
        return;
    }

    if (isExceptionPriorityLowEnoughToDebug(exceptionNumber))
    {
        // Pend DebugMon interrupt to debug the fault.
        recordAndClearFaultStatusBits();
        setPendedFromFaultBit();
        setMonitorPending();
    }
    else
    {
        // UNDONE: Come up with a better way to handle these.
        while (true)
        {
        }
    }
}

static ExceptionStack* getExceptionStack(uint32_t excReturn, uint32_t psp, uint32_t msp)
{
    uint32_t sp;
    if ((excReturn & 0xF) == EXC_RETURN_THREADMODE_PROCESSSTACK)
    {
        sp = psp;
    }
    else
    {
        sp = msp;
    }
    return (ExceptionStack*)sp;
}

static bool isDebugThreadActive()
{
    return (mriCortexMFlags & CORTEXM_FLAGS_ACTIVE_DEBUG);
}

static void setFaultDetectedFlag()
{
    mriCortexMFlags |= CORTEXM_FLAGS_FAULT_DURING_DEBUG;
}

static bool isImpreciseBusFault()
{
    return SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk;
}

static void advancePCToNextInstruction(ExceptionStack* pExceptionStack)
{
    uint32_t* pPC = &pExceptionStack->pc;
    uint16_t  currentInstruction = *(uint16_t*)*pPC;
    if (isInstruction32Bit(currentInstruction))
    {
        *pPC += sizeof(uint32_t);
    }
    else
    {
        *pPC += sizeof(uint16_t);
    }
}

static int isInstruction32Bit(uint16_t firstWordOfInstruction)
{
    uint16_t maskedOffUpper5BitsOfWord = firstWordOfInstruction & 0xF800;

    /* 32-bit instructions start with 0b11101, 0b11110, 0b11111 according to page A5-152 of the
       ARMv7-M Architecture Manual. */
    return  (maskedOffUpper5BitsOfWord == 0xE800 ||
             maskedOffUpper5BitsOfWord == 0xF000 ||
             maskedOffUpper5BitsOfWord == 0xF800);
}

static void clearFaultStatusBits()
{
    /* Clear fault status bits by writing 1s to bits that are already set. */
    SCB->DFSR = SCB->DFSR;
    SCB->HFSR = SCB->HFSR;
    SCB->CFSR = SCB->CFSR;
}

static bool isExceptionPriorityLowEnoughToDebug(uint32_t exceptionNumber)
{
    if (exceptionNumber == 0)
    {
        // Can always debug main thread as it has lowest priority.
        return true;
    }
    else if (exceptionNumber < 16)
    {
        // These are system exceptions that can't be debugged.
        return false;
    }
    else
    {
        // Look up priority level in NVIC to see if it is low enough to be debugged.
        return getInterruptPriority(exceptionNumber) > MRIBLUE_PRIORITY;
    }
}

static uint32_t getInterruptPriority(uint32_t exceptionNumber)
{
    return NVIC->IP[exceptionNumber - 16] >> mriCortexMState.subPriorityBitCount;
}

static void recordAndClearFaultStatusBits(void)
{
    mriCortexMState.exceptionNumber = getCurrentlyExecutingExceptionNumber();
    mriCortexMState.dfsr = SCB->DFSR;
    mriCortexMState.hfsr = SCB->HFSR;
    mriCortexMState.cfsr = SCB->CFSR;
    mriCortexMState.mmfar = SCB->MMFAR;
    mriCortexMState.bfar = SCB->BFAR;

    // Clear fault status bits by writing 1s to bits that are already set.
    SCB->DFSR = mriCortexMState.dfsr;
    SCB->HFSR = mriCortexMState.hfsr;
    SCB->CFSR = mriCortexMState.cfsr;
}

static void setPendedFromFaultBit(void)
{
    mriCortexMFlags |= CORTEXM_FLAGS_PEND_FROM_FAULT;
}