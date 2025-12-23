#ifndef _NMEA2KTWAI_H
#define _NMEA2KTWAI_H
#include "NMEA2000.h"

class tNMEA2000_esp32xx : public tNMEA2000
{
public:
    tNMEA2000_esp32xx(int _TxPin, int _RxPin, unsigned long recoveryPeriod = 1000, unsigned long logPeriod = 0);

    // Auto-restart states. See State-Machine diagram in README.md
    typedef enum
    {
        ST_INVALID,
        ST_STOPPED,
        ST_RUNNING,
        ST_BUS_OFF,
        ST_RECOVERING,
        ST_PROBE_BUS,
        ST_DISABLED,
        ST_ERROR
    } STATE;
    typedef enum
    {
        ERR_ACTIVE,
        ERR_WARNING,
        ERR_PASSIVE,
        ERR_BUS_OFF,
    } ERR_STATE;
    typedef struct
    {
        // see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#_CPPv418twai_status_info_t
        uint32_t rx_errors = 0;
        uint32_t tx_errors = 0;
        uint32_t tx_failed = 0;
        uint32_t rx_missed = 0;
        uint32_t rx_overrun = 0;
        uint32_t tx_timeouts = 0;
        uint32_t tx_queued = 0;
        STATE state = ST_ERROR;
        ERR_STATE err_state = ERR_ACTIVE;
    } Status;
    static const char *stateStr(const STATE &st);
    static const char *errStateStr(const ERR_STATE &st);
    Status getStatus();

    virtual bool CANOpen();
    void loop();
    virtual ~tNMEA2000_esp32xx();
protected:
    // Virtual functions for different interfaces. Currently there are own classes
    // for Arduino due internal CAN (NMEA2000_due), external MCP2515 SPI CAN bus controller (NMEA2000_mcp),
    // Teensy FlexCAN (NMEA2000_Teensy), NMEA2000_avr for AVR, NMEA2000_mbed for MBED and NMEA2000_socketCAN for e.g. RPi.
    virtual bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true);
    virtual bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
    // This will be called on Open() before any other initialization. Inherit this, if buffers can be set for the driver
    // and you want to change size of library send frame buffer size. See e.g. NMEA2000_teensy.cpp.
    virtual void InitCANFrameBuffers();
    virtual void DeinitCANFrameBuffers();

private:
    void _logStatus(const tNMEA2000_esp32xx::Status &canState);
    Status _getStatus(const void *twai_status_ptr);
    void _installEspCanDriver();
    bool _startEspCanDriver();
    void _sendProbeFrame();
    void _uninstallEspCanDriver();
    int RxPin;
    int TxPin;
    tN2kSyncScheduler recoveryTimer;
    tN2kSyncScheduler logTimer;
    uint32_t txTimeouts;
    STATE state = ST_STOPPED;
    void* driverMutex = nullptr;  // Mutex handle (actually SemaphoreHandle_t) to protect driver operations
};

#endif
