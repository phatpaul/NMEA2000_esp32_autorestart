/*
NMEA2000_esp32xx.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi
Copyright (c) 2023 Jaume Clarens "jiauka"
2024 - Improved with error handling by wellenvogel - see https://github.com/wellenvogel/esp32-nmea2000/issues/67
2024/10/18 - Improved with proper CAN bus recovery process. see https://github.com/phatpaul/NMEA2000_esp32xx


Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "NMEA2000_esp32xx.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG /* Enable this to show debug logging for this file only. */
#include "esp_log.h"
#include <inttypes.h>
#include "driver/twai.h"

#define LOGID(id) ((id >> 8) & 0x1ffff)

#define logDebug(level, fmt, args...)                     \
    do                                                    \
    {                                                     \
        int esp_level = ESP_LOG_VERBOSE;                  \
        if (level == LOG_ERR)                             \
        {                                                 \
            esp_level = ESP_LOG_ERROR;                    \
        }                                                 \
        else if (level == LOG_INFO)                       \
        {                                                 \
            esp_level = ESP_LOG_INFO;                     \
        }                                                 \
        else if (level == LOG_DEBUG)                      \
        {                                                 \
            esp_level = ESP_LOG_DEBUG;                    \
        }                                                 \
        ESP_LOG_LEVEL_LOCAL(esp_level, TAG, fmt, ##args); \
    } while (0)

const char *TAG = "N2K-drv"; // just a tag for logging

static const int TIMEOUT_OFFLINE = 16; // # of consecutive timeouts to consider driver frozen

const char *tNMEA2000_esp32xx::stateStr(const tNMEA2000_esp32xx::STATE &st)
{
    switch (st)
    {
    case ST_BUS_OFF:
        return "BUS_OFF";
    case ST_RECOVERING:
        return "RECOVERING";
    case ST_RUNNING:
        return "RUNNING";
    case ST_STOPPED:
        return "STOPPED";
    case ST_PROBE_BUS:
        return "RESTARTING";
    case ST_DISABLED:
        return "DISABLED";
    default:
        break;
    }
    return "ERROR";
}

const char *tNMEA2000_esp32xx::errStateStr(const tNMEA2000_esp32xx::ERR_STATE &st)
{
    switch (st)
    {
    case ERR_ACTIVE:
        return "ACTIVE";
    case ERR_WARNING:
        return "WARNING";
    case ERR_PASSIVE:
        return "PASSIVE";
    case ERR_BUS_OFF:
        return "BUS_OFF";
    default:
        break;
    }
    return "--";
}

tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::getStatus(const void *twai_status_ptr)
{
    tNMEA2000_esp32xx::Status rt;
    if (nullptr == twai_status_ptr)
    {
        rt.state = ST_ERROR;
        return rt;
    }
    // used a void pointer to avoid including twai.h in the header file, cast it here
    const twai_status_info_t &twai_status = *static_cast<const twai_status_info_t *>(twai_status_ptr);
    rt.rx_errors = twai_status.rx_error_counter;
    rt.tx_errors = twai_status.tx_error_counter;
    rt.tx_failed = twai_status.tx_failed_count;
    rt.rx_missed = twai_status.rx_missed_count;
    rt.rx_overrun = twai_status.rx_overrun_count;
    rt.tx_timeouts = txTimeouts;
    rt.tx_queued = twai_status.msgs_to_tx;
    rt.state = state;

    // Error Active: When both TEC and REC are less than 96, the node is in the active error state, meaning normal operation.
    // The node participates in bus communication and sends active error flags when errors are detected to actively report them.
    if ((twai_status.tx_error_counter < 96) && (twai_status.rx_error_counter < 96))
    {
        rt.err_state = ERR_ACTIVE;
    }
    // Error Warning: When either TEC or REC is greater than or equal to 96 but both are less than 128,
    // the node is in the warning error state. Errors may exist but the node behavior remains unchanged.
    else if ((twai_status.tx_error_counter < 128) && (twai_status.rx_error_counter < 128))
    {
        rt.err_state = ERR_WARNING;
    }
    // Error Passive: When either TEC or REC is greater than or equal to 128, the node enters the passive error state.
    // It can still communicate on the bus but sends only one passive error flag when detecting errors.
    else if ((twai_status.tx_error_counter < 256) && (twai_status.rx_error_counter < 256))
    {
        rt.err_state = ERR_PASSIVE;
    }
    // Bus Off: When TEC is greater than or equal to 256, the node enters the bus off (offline) state.
    // The node is effectively disconnected and does not affect the bus. It remains offline until recovery is triggered by software.
    else
    {
        rt.err_state = ERR_BUS_OFF;
    }

    return rt;
}

tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::getStatus()
{
    twai_status_info_t twai_status;
    if (ESP_OK != twai_get_status_info(&twai_status))
    {
        return getStatus(nullptr);
    }
    return getStatus(&twai_status);
}

void tNMEA2000_esp32xx::logStatus(const tNMEA2000_esp32xx::Status &canState)
{
    logDebug(LOG_INFO, "TWAI state %s, rxerr %u, txerr %u, txfail %u, rxmiss %u, rxoverrun %u, txqueued %u, txtimeouts %u, errstate %s",
        stateStr(canState.state),
        (unsigned)canState.rx_errors,
        (unsigned)canState.tx_errors,
        (unsigned)canState.tx_failed,
        (unsigned)canState.rx_missed,
        (unsigned)canState.rx_overrun,
        (unsigned)canState.tx_queued,
        (unsigned)txTimeouts,
        errStateStr(canState.err_state)
    );
}

/**
 * @brief Construct a new tNMEA2000 esp32xx::tNMEA2000 esp32xx object
 *
 * @param _TxPin CAN bus Tx pin #
 * @param _RxPin CAN bus Rx pin #
 * @param recoveryPeriod Interval in ms to wait before CAN bus recovery after BUS_OFF condition. Set 0 to disable auto recovery.
 * @param logPeriod Interval in ms for periodic logging of stats. Default: 0 <- disabled.
 */
tNMEA2000_esp32xx::tNMEA2000_esp32xx(int _TxPin, int _RxPin, unsigned long recoveryPeriod, unsigned long logPeriod)
    : tNMEA2000(), RxPin(_RxPin), TxPin(_TxPin)
{
    // Construct timers
    recoveryTimer = tN2kSyncScheduler(false, recoveryPeriod, 0);
    logTimer = tN2kSyncScheduler(false, logPeriod, 0);
}

bool tNMEA2000_esp32xx::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
    if (ST_RUNNING != state)
    {
        return false;
    }
    logDebug(LOG_MSG, "TWAI transmit id %lu, len %d", (unsigned long)LOGID(id), (int)len);

    twai_message_t message;
    memset(&message, 0, sizeof(message));
    message.identifier = id;
    message.extd = 1;
    message.data_length_code = len;
    memcpy(message.data, buf, len);
    esp_err_t rt = twai_transmit(&message, pdMS_TO_TICKS(10)); // 10ms timeout instead of 0
    if (rt == ESP_ERR_INVALID_STATE)
    {
        // Probably driver not in RUNNING state, don't spam the console here.
        return false;
    }
    if (rt == ESP_ERR_TIMEOUT)
    {
        txTimeouts++;
        logDebug(LOG_DEBUG, "TWAI tx timeout");
        return false;
    }
    if (ESP_OK != rt)
    {
        logDebug(LOG_ERR, "TWAI tx %lu failed: %x", (unsigned long)LOGID(id), (int)rt);
        return false;
    }

    txTimeouts = 0; // reset timeouts counter on a successful send
    return true;
}

bool tNMEA2000_esp32xx::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
    if (ST_RUNNING != state)
    {
        return false;
    }
    twai_message_t message;
    esp_err_t rt = twai_receive(&message, 0);
    if (rt != ESP_OK)
    {
        return false;
    }
    if (!message.extd)
    {
        return false;
    }
    id = message.identifier;
    len = message.data_length_code;
    if (len > 8)
    {
        logDebug(LOG_DEBUG, "TWAI: received invalid message %lu, len %d", (unsigned long)LOGID(id), len);
        len = 8;
    }
    logDebug(LOG_MSG, "TWAI rcv id=%lu,len=%d, ext=%d", (unsigned long)LOGID(message.identifier), message.data_length_code, message.extd);
    if (!message.rtr)
    {
        memcpy(buf, message.data, len);
    }
    return true;
}

bool tNMEA2000_esp32xx::CANOpen()
{
    if (ST_STOPPED != state) // CANOpen should only be called in STOPPED state
    {
        logDebug(LOG_ERR, "CANOpen invalid state");
        return true;
    }
    esp_err_t rt = twai_start();
    if (rt != ESP_OK)
    {
        logDebug(LOG_ERR, "CANOpen failed: %x", (int)rt);
        state = ST_ERROR;
        return false;
    }
    else
    {
        logDebug(LOG_DEBUG, "CANOpen ok");

        // Send a probe frame to detect other nodes on the bus
        // Use ISO Address Claim (PGN 60928) with invalid temporary address
        // This is safe because: 1) Standard NMEA2000 message, 2) Invalid claim won't cause conflicts
        twai_message_t probe_message;
        memset(&probe_message, 0, sizeof(probe_message));
        probe_message.identifier = 0x18EEFFFE;  // PGN 60928, src 254 (temporary), dest 255 (global)
        probe_message.extd = 1;
        probe_message.data_length_code = 8;
        // Invalid address claim data - ensures no real address conflict
        probe_message.data[0] = 0xFF;  // Unique Number (invalid)
        probe_message.data[1] = 0xFF;
        probe_message.data[2] = 0xFF;
        probe_message.data[3] = 0xFF;
        probe_message.data[4] = 0xFF;  // Device Instance + System Instance (invalid)
        probe_message.data[5] = 0xFF;  // Function + Device Class (invalid)
        probe_message.data[6] = 0xFF;  // Reserved + Industry Group (invalid)
        probe_message.data[7] = 0xFE;  // Reserved + Address (254 = temporary)

        logDebug(LOG_DEBUG, "Sending bus probe frame...");
        twai_transmit(&probe_message, pdMS_TO_TICKS(50)); // 50ms timeout
    }
    return true;
}

// Initialize ESP32 TWAI driver
void tNMEA2000_esp32xx::installEspCanDriver()
{
    if (RxPin < 0 || TxPin < 0)
    {
        state = ST_DISABLED;
        logDebug(LOG_INFO, "TWAI init - disabled");
    }
    else
    {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TxPin, (gpio_num_t)RxPin, TWAI_MODE_NORMAL);
        // Use configured buffer sizes, with reasonable defaults if not set
        g_config.tx_queue_len = (MaxCANSendFrames > 0) ? MaxCANSendFrames : 80;  // Default 80 for burst handling
        g_config.rx_queue_len = MaxCANReceiveFrames;  // Use configured buffer size instead of hardcoded 40
        g_config.intr_flags |= ESP_INTR_FLAG_LOWMED; // LOWMED might be needed if you run out of LEVEL1 interrupts.

        // Start with the default timing configuration, so we get sane values for new fields
        twai_timing_config_t t_config; // = TWAI_TIMING_CONFIG_250KBITS();

        // ESP-IDF Default Timing gives 80% sample point
        // But NMEA2000 requires sample point at 87%
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        // ESP-IDF 5.x requires explicit clock source configuration
        t_config.clk_src = TWAI_CLK_SRC_DEFAULT;
        t_config.quanta_resolution_hz = 0; // Let driver calculate from brp
#endif
        t_config.brp = 20; // divide 80MHz APB clock by 20 = 4MHz. At 250Kbps, 4MHz = 16 time quanta (including sync)
        // Want sample at 87%: sample_point = (0.875 * 16) = 14
        t_config.tseg_1 = 13; // tseg_1 = (sample_point - sync) = (14 - 1) = 13
        t_config.tseg_2 = 2;  // tseg_2 = (total - tseg_1) = (16 - 14) = 2
        t_config.sjw = 1; //synchronization jump width should be 1 time quanta (why?)
        /* triple_sampling
        * true: the bus is sampled three times; recommended for low/medium speed buses (class A and B) where filtering spikes on the bus line is beneficial
        * false: the bus is sampled once; recommended for high speed buses (SAE class C)*/
        t_config.triple_sampling = true;

        logDebug(LOG_DEBUG, "TWAI timing config: brp=%u tseg1=%u tseg2=%u sjw=%u triple_sampling=%d",
            (unsigned)t_config.brp, (unsigned)t_config.tseg_1, (unsigned)t_config.tseg_2, (unsigned)t_config.sjw, t_config.triple_sampling ? 1 : 0);

        // Filter config - accept all messages
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        txTimeouts = 0;

        // Install the driver
        esp_err_t rt = twai_driver_install(&g_config, &t_config, &f_config);
        if (rt == ESP_OK)
        {
            state = ST_STOPPED; // We start in STOPPED state, user must call CANOpen() to start

            // Debug logging of initial status
            if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG)
            {
                logDebug(LOG_DEBUG, "TWAI driver initialzed, rx=%d,tx=%d, tx_queue=%u, rx_queue=%u",
                    (int)RxPin, (int)TxPin, (unsigned)g_config.tx_queue_len, (unsigned)g_config.rx_queue_len);
                // Check initial status after driver install but before start
                twai_status_info_t initial_status;
                if (ESP_OK == twai_get_status_info(&initial_status)) {
                    logDebug(LOG_DEBUG, "Initial TWAI status: TXerr=%u RXerr=%u state=%d",
                        (unsigned)initial_status.tx_error_counter, (unsigned)initial_status.rx_error_counter, initial_status.state);
                }
            }
        }
        else
        {
            logDebug(LOG_ERR, "TWAI driver init failed: %x", (int)rt);
            state = ST_DISABLED;
        }
    }
}

// Uninstall the driver.  Undo what is done in installEspCanDriver()
void tNMEA2000_esp32xx::uninstallEspCanDriver()
{
    twai_stop();
    twai_driver_uninstall();
    state = ST_DISABLED;
    logDebug(LOG_INFO, "TWAI driver uninstalled");
}

// This will be called on Open() before any other initialization. Inherit this, if buffers can be set for the driver
// and you want to change size of library send frame buffer size. See e.g. NMEA2000_teensy.cpp.
void tNMEA2000_esp32xx::InitCANFrameBuffers()
{
    // Initialize ESP32 TWAI driver
    installEspCanDriver();

    // Start log timer now.
    logTimer.UpdateNextTime();

    // call parent function
    tNMEA2000::InitCANFrameBuffers();
}

// Uninstall the driver.  Undo what is done in InitCANFrameBuffers()
void tNMEA2000_esp32xx::DeinitCANFrameBuffers()
{
    uninstallEspCanDriver();

    // Stop log timer
    logTimer.Disable();
}

// Destructor to automatically uninstall the driver if the object goes out of scope or is deleted.
tNMEA2000_esp32xx::~tNMEA2000_esp32xx()
{
    DeinitCANFrameBuffers();
    // Base destructor automatically called after this
}

/**
 * @brief This must be called periodically from your task loop
 *
 */
void tNMEA2000_esp32xx::loop()
{
    if (ST_DISABLED == state)
    {
        return; // No way out of DISABLED state here
    }
    Status status;
    twai_status_info_t twai_status;
    if (ESP_OK != twai_get_status_info(&twai_status))
    {
        status = getStatus(nullptr);
    }
    else {
        status = getStatus(&twai_status);
    }

    if (ST_ERROR == status.state)
    {
        logDebug(LOG_ERR, "TWAI driver broken");
        state = ST_DISABLED; // TWAI driver is broken, no need to keep trying and spamming logs
        return;
    }

    tNMEA2000_esp32xx::STATE next_state = ST_INVALID;
    // Some state transition are unconditional of current state, so are moved out of the switch()
    if ((TWAI_STATE_STOPPED == twai_status.state) && (ST_STOPPED != state))
    {
        next_state = ST_STOPPED;
    }
    else if ((TWAI_STATE_BUS_OFF == twai_status.state) && (ST_BUS_OFF != state))
    {
        // reload timer for waiting in RESTARTING before restart the stack
        recoveryTimer.UpdateNextTime();
        next_state = ST_BUS_OFF;
    }
    else if ((TWAI_STATE_RECOVERING == twai_status.state) && (ST_RECOVERING != state))
    {
        next_state = ST_RECOVERING;
    }
    // Note: We use TWAI_STATE_RUNNING as a condition in our state machine
    else
    {
        const bool autoRecoveryEnabled = (0 != recoveryTimer.GetPeriod());
        switch (state)
        {
        case ST_STOPPED:
        {
            if (TWAI_STATE_RUNNING == twai_status.state)
            {
                // If stack or user called CANOpen(), keep track.
                next_state = ST_RUNNING;
                // reload timer for waiting in RUNNING before checking for errors
                recoveryTimer.UpdateNextTime();
                break;
            }
            else {
                // (Re)start CAN bus
                CANOpen();
                next_state = ST_PROBE_BUS;
                // reload timer for waiting in PROBE_BUS for other nodes before restarting the stack
                recoveryTimer.UpdateNextTime();
                break;
            }
        }
        break;
        case ST_PROBE_BUS:
        {
            // Wait for recovery timer to give time for probe frame to accumulate errors if no other nodes present
            if (autoRecoveryEnabled && recoveryTimer.IsTime())
            {
                recoveryTimer.Disable(); // one-shot
                // Check if transmission caused error increment (indicates no other nodes)
                bool other_nodes_present = (0 == twai_status.tx_error_counter);

                // Only restart NMEA2000 library if we detected other nodes during probe
                if (other_nodes_present)
                {
                    // Now the higher-level NMEA2000 library should be restarted!
                    tNMEA2000::Restart();
                    next_state = ST_RUNNING;
                }
                // if no other nodes detected, then go back to STOPPED and try again
                else
                {
                    // No other nodes detected - go back offline to try again
                    logDebug(LOG_DEBUG, "Bus probe indicates NO other nodes present (TXerr=%u)",
                        (unsigned)twai_status.tx_error_counter);
                    twai_stop();
                    next_state = ST_STOPPED;
                }
            }
        }
        break;
        case ST_RUNNING: // Normal running state
        {
            // TWAI_STATE_BUS_OFF is entered automatically when the hardware detects too many errors on the bus
            // Transition to BUS_OFF is handled in common.

            // Delay error checks in RUNNING state until recovery timer expires
            if (autoRecoveryEnabled && recoveryTimer.IsTime()) {
                // Check for Error Passive condition
                if (ERR_PASSIVE == status.err_state) {
                    logDebug(LOG_ERR, "Detected Error Passive (Disconnected or only node on bus) (TXerr=%u) -> Stopping",
                        (unsigned)twai_status.tx_error_counter);
                    twai_stop();
                    next_state = ST_STOPPED; // Transition to STOPPED, then will try to restart...
                    break;
                }
                // Check for TX timeouts indicating frozen driver
                if (txTimeouts >= TIMEOUT_OFFLINE) {
                    logDebug(LOG_ERR, "Detected Frozen Driver (TxTimeouts=%u) -> Reinstall Driver",
                        (unsigned)txTimeouts);
                    uninstallEspCanDriver();
                    installEspCanDriver();
                    next_state = ST_STOPPED; // Transition to STOPPED, then will try to restart...
                    break;
                }
            }
        }
        break;
        case ST_BUS_OFF:
        {
            if (autoRecoveryEnabled && recoveryTimer.IsTime())
            {
                // If auto recovery enabled, try to recover
                twai_initiate_recovery(); // Needs 128 occurrences of bus free signal
                next_state = ST_RECOVERING;
                break;
            }
        }
        break;
        case ST_RECOVERING:
        {
            // TWAI driver should handle recovery and set state to STOPPED when successful
            // Transition to STOPPED is handled in common
        }
        break;
        case ST_DISABLED:
            // No way out of DISABLED state here
            break;
        case ST_ERROR:
            // No way out of ERROR state here
            break;
        default:
            next_state = ST_ERROR;
            break;
        }
    }

    if (ST_INVALID != next_state)
    {
        // State changed. Do common stuff on state transitions.
        logDebug(LOG_DEBUG, "TWAI %s --> %s (TXerr=%u)", stateStr(state), stateStr(next_state), (unsigned)twai_status.tx_error_counter);
        state = next_state;
    }

    // Periodic logging
    if (logTimer.IsTime())
    {
        logTimer.UpdateNextTime();
        logStatus(status);
    }
}
