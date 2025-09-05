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
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG /* Enable this to show debug logging for this file only. */
#include "esp_log.h"

#include "NMEA2000_esp32xx.h"
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

static const int TIMEOUT_OFFLINE = 256; // # of consecutive timeouts to consider OFFLINE

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
    case ST_RESTARTING:
        return "RESTARTING";
    case ST_DISABLED:
        return "DISABLED";
    default:
        break;
    }
    return "ERROR";
}

tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::getStatus()
{
    Status rt;
    twai_status_info_t twai_status;
    if (ESP_OK != twai_get_status_info(&twai_status))
    {
        rt.state = ST_ERROR;
        return rt;
    }
    rt.rx_errors = twai_status.rx_error_counter;
    rt.tx_errors = twai_status.tx_error_counter;
    rt.tx_failed = twai_status.tx_failed_count;
    rt.rx_missed = twai_status.rx_missed_count;
    rt.rx_overrun = twai_status.rx_overrun_count;
    rt.tx_queued = twai_status.msgs_to_tx;
    rt.state = state;
    return rt;
}

tNMEA2000_esp32xx::Status tNMEA2000_esp32xx::logStatus()
{
    Status canState = getStatus();
    logDebug(LOG_INFO, "TWAI state %s, rxerr %d, txerr %d, txfail %d, rxmiss %d, rxoverrun %d, txqueued %d",
        stateStr(canState.state),
        canState.rx_errors,
        canState.tx_errors,
        canState.tx_failed,
        canState.rx_missed,
        canState.rx_overrun,
        canState.tx_queued);
    return canState;
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
    if (RxPin < 0 || TxPin < 0)
    {
        state = ST_DISABLED;
    }
    else
    {
        recoveryTimer = tN2kSyncScheduler(false, recoveryPeriod, 0);
        logTimer = tN2kSyncScheduler(false, logPeriod, 0);
    }
}

bool tNMEA2000_esp32xx::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
    if (ST_RUNNING != state)
    {
        return false;
    }
    logDebug(LOG_MSG, "TWAI transmit id %ld, len %d", LOGID(id), (int)len);

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
        logDebug(LOG_DEBUG, "TWAI tx timeout");
        return false;
    }
    if (ESP_OK != rt)
    {
        logDebug(LOG_ERR, "TWAI tx %ld failed: %x", LOGID(id), (int)rt);
        return false;
    }

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
        logDebug(LOG_DEBUG, "TWAI: received invalid message %ld, len %d", LOGID(id), len);
        len = 8;
    }
    logDebug(LOG_MSG, "TWAI rcv id=%d,len=%d, ext=%d", LOGID(message.identifier), message.data_length_code, message.extd);
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
    // Start timers now.
    logTimer.UpdateNextTime();
    return true;
}

// This will be called on Open() before any other initialization. Inherit this, if buffers can be set for the driver
// and you want to change size of library send frame buffer size. See e.g. NMEA2000_teensy.cpp.
void tNMEA2000_esp32xx::InitCANFrameBuffers()
{
    if (ST_DISABLED == state)
    {
        logDebug(LOG_INFO, "TWAI init - disabled");
    }
    else
    {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TxPin, (gpio_num_t)RxPin, TWAI_MODE_NORMAL);
        // Use configured buffer sizes, with reasonable defaults if not set
        g_config.tx_queue_len = (MaxCANSendFrames > 0) ? MaxCANSendFrames : 80;  // Default 80 for burst handling
        g_config.rx_queue_len = MaxCANReceiveFrames;  // Use configured buffer size instead of hardcoded 40
        g_config.intr_flags |= ESP_INTR_FLAG_LOWMED; // LOWMED might be needed if you run out of LEVEL1 interrupts.

        twai_timing_config_t t_config;
        // ESP_IDF DEFAULT TIMING CONFIGURATION gives 80% sample point
        // #define TWAI_TIMING_CONFIG_250KBITS()   {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
        // t_config = TWAI_TIMING_CONFIG_250KBITS();

        // But NMEA2000 requires sample point at 87%
        t_config.brp = 20; // divide 80MHz APB clock by 20 = 4MHz. At 250Kbps, 4MHz = 16 time quanta (including sync)
        // Want sample at 87%: sample_point = (0.875 * 16) = 14
        t_config.tseg_1 = 13; // tseg_1 = (sample_point - sync) = (14 - 1) = 13
        t_config.tseg_2 = 2;  // tseg_2 = (total - tseg_1) = (16 - 14) = 2
        t_config.sjw = 1; //synchronization jump width should be 1 time quanta (why?)
        /* triple_sampling
        * true: the bus is sampled three times; recommended for low/medium speed buses (class A and B) where filtering spikes on the bus line is beneficial
        * false: the bus is sampled once; recommended for high speed buses (SAE class C)*/
        t_config.triple_sampling = true;

        logDebug(LOG_DEBUG, "TWAI timing config: brp=%d tseg1=%d tseg2=%d sjw=%d triple_sampling=%d",
            t_config.brp, t_config.tseg_1, t_config.tseg_2, t_config.sjw, t_config.triple_sampling);

        // Filter config
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        esp_err_t rt = twai_driver_install(&g_config, &t_config, &f_config);
        if (rt == ESP_OK)
        {
            state = ST_STOPPED; // We start in STOPPED state, user must call CANOpen() to start
            if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG)
            {
                logDebug(LOG_DEBUG, "TWAI driver initialzed, rx=%d,tx=%d, tx_queue=%d, rx_queue=%d",
                    (int)RxPin, (int)TxPin, g_config.tx_queue_len, g_config.rx_queue_len);
                // Check initial status after driver install but before start
                twai_status_info_t initial_status;
                if (ESP_OK == twai_get_status_info(&initial_status)) {
                    logDebug(LOG_DEBUG, "Initial TWAI status: TXerr=%d RXerr=%d state=%d",
                        initial_status.tx_error_counter, initial_status.rx_error_counter, initial_status.state);
                }
            }
        }
        else
        {
            logDebug(LOG_ERR, "TWAI driver init failed: %x", (int)rt);
            state = ST_DISABLED;
        }
    }

    // call parent function
    tNMEA2000::InitCANFrameBuffers();
}

// Uninstall the driver.  Undo what is done in InitCANFrameBuffers()
void tNMEA2000_esp32xx::DeinitCANFrameBuffers()
{
    if (state != ST_DISABLED)
    {
        twai_stop();
        twai_driver_uninstall();
        state = ST_DISABLED;
    }
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

    // No way out of DISABLED state
    if (ST_DISABLED != state)
    {
        const bool autoRecoveryEnabled = (0 != recoveryTimer.GetPeriod());
        twai_status_info_t twai_status;
        if (ESP_OK != twai_get_status_info(&twai_status))
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
            next_state = ST_BUS_OFF;
        }
        else if ((TWAI_STATE_RECOVERING == twai_status.state) && (ST_RECOVERING != state))
        {
            next_state = ST_RECOVERING;
        }
        // Note: We use TWAI_STATE_RUNNING as a condition in our state machine
        else
        {
            switch (state)
            {
            case ST_STOPPED:
            {
                if (TWAI_STATE_RUNNING == twai_status.state)
                {
                    // If stack or user called CANOpen(), keep track.
                    next_state = ST_RUNNING;
                    break;
                }
                if (autoRecoveryEnabled && recoveryTimer.IsTime())
                {
                    recoveryTimer.Disable(); // one-shot
                    CANOpen();
                    next_state = ST_RESTARTING;
                    break;
                }
            }
            break;
            case ST_RESTARTING:
            {
                // Check if transmission caused error increment (indicates no other nodes)
                bool other_nodes_present = (0 == twai_status.tx_error_counter);

                // Only restart NMEA2000 library if we detected other nodes during probe
                // This prevents bus flooding when we're the only device
                if (other_nodes_present)
                {
                    // Now the higher-level NMEA2000 library should be restarted!
                    tNMEA2000::Restart();
                    next_state = ST_RUNNING;
                }
                else
                {
                    // No other nodes detected - go back offline to try again
                    logDebug(LOG_DEBUG, "Bus probe indicates NO other nodes present (TXerr: %d)",
                        twai_status.tx_error_counter);
                    twai_stop();
                    next_state = ST_STOPPED;
                }
            }
            break;
            case ST_RUNNING: // Normal running state
            {
                // TWAI_STATE_BUS_OFF is entered automatically when the hardware detects too many errors on the bus
                // Transition to BUS_OFF is handled in common.

                // Check for Error Passive condition with full queue - this causes the timeout cascade
                if (twai_status.tx_error_counter >= 128) {
                    logDebug(LOG_ERR, "Detected Error Passive (TXerr=%d, queue=%d) - (maybe disconnected or only node on bus)",
                        twai_status.tx_error_counter, twai_status.msgs_to_tx);
                    twai_stop();
                    next_state = ST_STOPPED;
                    break;
                }
            }
            break;
            case ST_BUS_OFF:
            {
                if (autoRecoveryEnabled)
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
                // No way out of DISABLED state
                break;
            case ST_ERROR:
                // No way out of ERROR state
                break;
            default:
                next_state = ST_ERROR;
                break;
            }
        }

        if (ST_INVALID != next_state)
        {
            // State changed. Do common stuff on state transitions.
            if (ST_STOPPED == next_state)
            {
                // reload timer for waiting in STOPPED before restarting
                recoveryTimer.UpdateNextTime();
            }
            logDebug(LOG_DEBUG, "TWAI %s --> %s", stateStr(state), stateStr(next_state));
            state = next_state;
        }
    }

    if (logTimer.IsTime())
    {
        logTimer.UpdateNextTime();
        logStatus();
    }
}
