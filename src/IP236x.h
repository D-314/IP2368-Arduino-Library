#ifndef IP236X_H
#define IP236X_H

#include <Wire.h>
#include <stdint.h>
class IP236x
{
public:
    uint8_t _address;
    TwoWire *_wire;
    IP236x(uint8_t address = 0x75, TwoWire &wire = Wire)
        : _address(address), _wire(&wire) {}

    void begin();

    // Enumeration for defining the charge state
    enum class ChargeState
    {
        STANDBY = 0,
        TRICKLE_CHARGE = 1,
        CONSTANT_CURRENT = 2,
        CONSTANT_VOLTAGE = 3,
        CHARGE_WAIT = 4,
        CHARGE_FULL = 5,
        CHARGE_TIMEOUT = 6
    };

    // Enumeration for defining the USB Type-C operating mode
    enum class TypeCMode
    {
        UFP = 0, // Upstream Facing Port (device connected as a consumer)
        DFP = 1, // Downstream Facing Port (device connected as a power source)
        DRP = 3  // Dual Role Port (device can act as both a source and a consumer)
    };

    ///////// IS? ////////

    // SYS_CTL0

    bool isChargerEnabled(uint8_t * errorCode = nullptr);
    bool isVbusSinkSCPEnabled(uint8_t * errorCode = nullptr);
    bool isVbusSinkPDEnabled(uint8_t * errorCode = nullptr);
    bool isVbusSinkDPdMEnabled(uint8_t * errorCode = nullptr);
    bool isINTLowEnabled(uint8_t * errorCode = nullptr);
    bool isLoadOTPEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL9
    bool isStandbyModeEnabled(uint8_t * errorCode = nullptr);
    bool isBATLowEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL11

    bool isDcDcOutputEnabled(uint8_t * errorCode = nullptr);
    bool isVbusSrcDPdMEnabled(uint8_t * errorCode = nullptr);
    bool isVbusSrcPdEnabled(uint8_t * errorCode = nullptr);
    bool isVbusSrcSCPEnabled(uint8_t * errorCode = nullptr);

    // TypeC_CTL9

    bool is5VPdo3AEnabled(uint8_t * errorCode = nullptr);
    bool isPps2PdoIsetEnabled(uint8_t * errorCode = nullptr);
    bool isPps1PdoIsetEnabled(uint8_t * errorCode = nullptr);
    bool is20VPdoIsetEnabled(uint8_t * errorCode = nullptr);
    bool is15VPdoIsetEnabled(uint8_t * errorCode = nullptr);
    bool is12VPdoIsetEnabled(uint8_t * errorCode = nullptr);
    bool is9VPdoIsetEnabled(uint8_t * errorCode = nullptr);
    bool is5VPdoIsetEnabled(uint8_t * errorCode = nullptr);

    // TypeC_CTL17

    bool isSrcPdo9VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdo12VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdo15VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPdo20VEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPps1PdoEnabled(uint8_t * errorCode = nullptr);
    bool isSrcPps2PdoEnabled(uint8_t * errorCode = nullptr);

    // STATE_CTL0

    bool isCharging(uint8_t * errorCode = nullptr);
    bool isChargeFull(uint8_t * errorCode = nullptr);
    bool isDischarging(uint8_t * errorCode = nullptr);

    // STATE_CTL1

    bool isFastCharge(uint8_t * errorCode = nullptr);

    // STATE_CTL2

    bool isVbusPresent(uint8_t * errorCode = nullptr);
    bool isVbusOvervoltage(uint8_t * errorCode = nullptr);

    // TypeC_STATE

    bool isTypeCSinkConnected(uint8_t * errorCode = nullptr);
    bool isTypeCSrcConnected(uint8_t * errorCode = nullptr);
    bool isTypeCSrcPdConnected(uint8_t * errorCode = nullptr);
    bool isTypeCSinkPdConnected(uint8_t * errorCode = nullptr);
    bool isVbusSinkQcActive(uint8_t * errorCode = nullptr);
    bool isVbusSrcQcActive(uint8_t * errorCode = nullptr);

    // STATE_CTL3

    bool isVsysOverCurrent(uint8_t * errorCode = nullptr);
    bool isVsysShortCircuitDt(uint8_t * errorCode = nullptr);

    // ADC
    bool isOverHeat(uint8_t * errorCode = nullptr);

    ///////// SET ////////

    // SYS_CTL0

    void enableCharger(bool enable = true, uint8_t * errorCode = nullptr);
    void enableVbusSinkSCP(bool enable = true, uint8_t * errorCode = nullptr);
    void enableVbusSinkPD(bool enable = true, uint8_t * errorCode = nullptr);
    void enableVbusSinkDPdM(bool enable = true, uint8_t * errorCode = nullptr);
    void enableINTLow(bool enable = true, uint8_t * errorCode = nullptr);
    void ResetMCU(bool enable = true, uint8_t * errorCode = nullptr);
    void enableLoadOTP(bool enable = true, uint8_t * errorCode = nullptr);

    // SYS_CTL2

    virtual void setFullChargeVoltage(uint16_t voltage, uint8_t * errorCode = nullptr);

    // SYS_CTL3

    virtual void setMaxInputPowerOrBatteryCurrent(uint16_t current_mA, uint8_t * errorCode = nullptr);

    // SYS_CTL6

    virtual void setTrickleChargeCurrent(uint16_t current_mA = 200, uint8_t * errorCode = nullptr);

    // SYS_CTL8

    void setChargeStopCurrent(uint16_t current = 100, uint8_t * errorCode = nullptr);
    void setCellRechargeThreshold(uint16_t voltageDrop_mV = 200, uint8_t * errorCode = nullptr);

    // SYS_CTL9

    void enableStandbyMode(bool enable = true, uint8_t * errorCode = nullptr);
    void enableBATLow(bool enable = true, uint8_t * errorCode = nullptr);

    // SYS_CTL10

    virtual void setLowBatteryVoltage(uint16_t voltage_mV, uint8_t * errorCode = nullptr);

    // SYS_CTL11

    void setOutputFeatures(bool enableDcDcOutput = true, bool enableVbusSrcDPdM = true, bool enableVbusSrcPd = true, bool enableVbusSrcSCP = true, uint8_t * errorCode = nullptr);

    // TypeC_CTL8
    void setTypeCMode(TypeCMode mode = TypeCMode::DRP, uint8_t * errorCode = nullptr);

    // TypeC_CTL9

    void enablePdoCurrentOutputSet(bool en5VPdoIset = true, bool en5VPdo3A = true, bool en9VPdoIset = true,
                                   bool en12VPdoIset = true, bool en15VPdoIset = true, bool en20VPdoIset = true,
                                   bool enPps1PdoIset = true, bool enPps2PdoIset = true, uint8_t * errorCode = nullptr);

    // TypeC_CTL10 - TypeC_CTL14

    void setPDOCurrent5V(uint16_t current_mA = 3000, uint8_t * errorCode = nullptr);
    void setPDOCurrent9V(uint16_t current_mA = 3000, uint8_t * errorCode = nullptr);
    void setPDOCurrent12V(uint16_t current_mA = 3000, uint8_t * errorCode = nullptr);
    void setPDOCurrent15V(uint16_t current_mA = 3000, uint8_t * errorCode = nullptr);
    void setPDOCurrent20V(uint16_t current_mA = 5000, uint8_t * errorCode = nullptr);

    // TypeC_CTL23 - TypeC_CTL24

    void setPDOCurrentPPS1(uint16_t current_mA = 3000, uint8_t * errorCode = nullptr);
    void setPDOCurrentPPS2(uint16_t current_mA = 3000, uint8_t * errorCode = nullptr);

    // TypeC_CTL17

    void enableSrcPdo(bool en9VPdo = true, bool en12VPdo = true, bool en15VPdo = true, bool en20VPdo = true, bool enPps1Pdo = true, bool enPps2Pdo = true, uint8_t * errorCode = nullptr);

    ///////// GET ////////

    // SYS_CTL2

    virtual uint16_t getFullChargeVoltage(uint8_t * errorCode = nullptr);

    // SYS_CTL3

    virtual uint16_t getMaxInputPowerOrBatteryCurrent(uint8_t * errorCode = nullptr);

    // SYS_CTL6

    virtual uint16_t getTrickleChargeCurrent(uint8_t * errorCode = nullptr);

    // SYS_CTL8

    uint16_t getChargeStopCurrent(uint8_t * errorCode = nullptr);
    uint16_t getCellRechargeThreshold(uint8_t * errorCode = nullptr);

    // SYS_CTL10

    virtual uint16_t getLowBatteryVoltage(uint8_t * errorCode = nullptr);

     // TypeC_CTL8

    TypeCMode getTypeCMode(uint8_t * errorCode = nullptr);

    // TypeC_CTL10 - TypeC_CTL14

    uint16_t getPDOCurrent5V(uint8_t * errorCode = nullptr);
    uint16_t getPDOCurrent9V(uint8_t * errorCode = nullptr);
    uint16_t getPDOCurrent12V(uint8_t * errorCode = nullptr);
    uint16_t getPDOCurrent15V(uint8_t * errorCode = nullptr);
    uint16_t getPDOCurrent20V(uint8_t * errorCode = nullptr);

    // TypeC_CTL23 - TypeC_CTL24

    uint16_t getPDOCurrentPPS1(uint8_t * errorCode = nullptr);
    uint16_t getPDOCurrentPPS2(uint8_t * errorCode = nullptr);

    // STATE_CTL0

    ChargeState getChargeState(uint8_t * errorCode = nullptr);

    uint8_t getChargeVoltage(uint8_t * errorCode = nullptr);

    // ADC

    uint16_t getVBATVoltage(uint8_t * errorCode = nullptr);
    uint16_t getVsysVoltage(uint8_t * errorCode = nullptr);
    uint16_t getBATCurrent(uint8_t * errorCode = nullptr);
    uint16_t getVsysCurrent(uint8_t * errorCode = nullptr);
    uint32_t getVsysPower(uint8_t * errorCode = nullptr);

    // GPIO

    uint16_t getNTCVoltage(uint8_t * errorCode = nullptr);

protected:
    uint8_t writeRegister(uint8_t regAddress, uint8_t value, uint8_t * errorCode = nullptr);
    uint8_t readRegister(uint8_t regAddress, uint8_t * errorCode = nullptr);
    void writeTypeCCurrentSetting(uint8_t reg, uint16_t current_mA, uint16_t step, uint16_t maxCurrent, uint8_t * errorCode = nullptr);
};
#endif //IP236X_H