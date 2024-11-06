#ifndef IP2366_H
#define IP2366_H

#include "IP236x.h"

class IP2366 : public IP236x {
public:
    using IP236x::IP236x;

    // Enumeration for selecting Vbus1 output power
    enum class Vbus1OutputPower
    {
        W30 = 0,
        W45 = 1,
        W60 = 2,
        W65 = 3,
        W100 = 4,
        W140 = 5
    };

    // Enumeration for selecting charging PDO mode
    enum class ChargingPDOmode
    {
        V5 = 0,
        V9 = 1,
        V12 = 2,
        V15 = 3,
        V20 = 4
    };

    ///////// IS? ////////

    // SYS_CTL9
    bool isStandby(uint8_t * errorCode = nullptr);

    // TypeC_CTL18

    bool isSrcPdoAdd10mA5VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdoAdd10mA9VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdoAdd10mA12VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdoAdd10mA15VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdoAdd10mA20VEnabled(uint8_t * errorCode = nullptr);

    // RECEIVED_PDO

    bool isReceives5VPdo(uint8_t * errorCode = nullptr);
    bool isReceives9VPdo(uint8_t * errorCode = nullptr);
    bool isReceives12VPdo(uint8_t * errorCode = nullptr);
    bool isReceives15VPdo(uint8_t * errorCode = nullptr);
    bool isReceives20VPdo(uint8_t * errorCode = nullptr);

    ///////// SET ////////

    // SYS_CTL2

    void setFullChargeVoltage(uint16_t voltage = 4400, uint8_t * errorCode = nullptr) override;

    // SYS_CTL3

    void setMaxInputPowerOrBatteryCurrent(uint16_t current_mA = 9700, uint8_t * errorCode = nullptr) override;

    // SYS_CTL6

    void setTrickleChargeCurrent(uint16_t current = 200, uint8_t * errorCode = nullptr) override;

    // SYS_CTL9

    void Standby(bool enable = true, uint8_t * errorCode = nullptr);

    // SYS_CTL10

    void setLowBatteryVoltage(uint16_t voltage_mV = 2700, uint8_t * errorCode = nullptr) override;

    // SYS_CTL12

    void setMaxOutputPower(Vbus1OutputPower power = Vbus1OutputPower::W140, uint8_t * errorCode = nullptr);

    // SELECT_PDO

    void setChargingPDOmode(ChargingPDOmode mode = ChargingPDOmode::V20, uint8_t * errorCode = nullptr);

    // TypeC_CTL18

    void enableSrcPdoAdd10mA(bool en5VPdoAdd10mA = true, bool en9VPdoAdd10mA = true, bool en12VPdoAdd10mA = true,
                             bool en15VPdoAdd10mA = true, bool en20VPdoAdd10mA = true, uint8_t * errorCode = nullptr);

    ///////// GET ////////

    // SYS_CTL2

    uint16_t getFullChargeVoltage(uint8_t * errorCode = nullptr) override;

    // SYS_CTL3

    uint16_t getMaxInputPowerOrBatteryCurrent(uint8_t * errorCode = nullptr) override;

    // SYS_CTL6

    uint16_t getTrickleChargeCurrent(uint8_t * errorCode = nullptr) override;

    // SYS_CTL10

    uint16_t getLowBatteryVoltage(uint8_t * errorCode = nullptr) override;

    // SYS_CTL12

    Vbus1OutputPower getMaxOutputPower(uint8_t * errorCode = nullptr);

    // SELECT_PDO

    ChargingPDOmode getChargingPDOmode(uint8_t * errorCode = nullptr);

    // TIMENODE

    void getTimenode(char timenode[5], uint8_t * errorCode = nullptr);
};

#endif
