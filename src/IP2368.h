#ifndef IP2368_H
#define IP2368_H

// #include <Arduino.h>
#include <Wire.h>

#include <stdint.h>

class IP2368
{
public:
    IP2368(uint8_t address = 0x75) : IP2368_address(address) {};
    void begin();

    uint8_t IP2368_address;

    // Enumeration for defining the charge state
    enum ChargeState
    {
        STANDBY = 0,
        TRICKLE_CHARGE = 1,
        CONSTANT_CURRENT = 2,
        CONSTANT_VOLTAGE = 3,
        CHARGE_WAIT = 4,
        CHARGE_FULL = 5,
        CHARGE_TIMEOUT = 6
    };

    // Enumeration for defining the type of battery
    enum BatteryType
    {
        LiFePO4 = 0, // Lithium iron phosphate batteries
        LiIon = 1    // Lithium ion batteries
    };

    // Enumeration for defining the current setting mode
    enum CurrentSettingMode
    {
        BatteryCurrent = 0, // Setting the battery current
        InputPower = 1      // Setting the input power
    };

    // Enumeration for defining the USB Type-C operating mode
    enum TypeCMode
    {
        UFP = 0, // Upstream Facing Port (device connected as a consumer)
        DFP = 1, // Downstream Facing Port (device connected as a power source)
        DRP = 3  // Dual Role Port (device can act as both a source and a consumer)
    };

    // Enumeration for selecting Vbus1 output power
    enum Vbus1OutputPower
    {
        W20 = 0,
        W25 = 1,
        W30 = 2,
        W45 = 3,
        W60 = 4,
        W100 = 5
    };

    // Enumeration for setting the charge timeout
    enum ChargeTimeout
    {
        Disabled = 0x00,
        H24 = 0x01,
        H36 = 0x02,
        H48 = 0x03
    };


    ///////// IS? ////////

    // SYS_CTL0

    bool isChargerEnabled();
    bool isVbusSinkCtrlEnabled();
    bool isVbusSinkSCPEnabled();
    bool isVbusSinkPDEnabled();
    bool isVbusSinkDPdMEnabled();
    bool isINTLowEnabled();
    bool isLoadOTPEnabled();

    // SYS_CTL1

    bool isBatteryTypeSettingEnabled();
    bool isCurrentOrPowerSettingModeEnabled();

    // SYS_CTL2

    bool isFullChargeVoltageSettingEnabled();

    // SYS_CTL3

    bool isPowerOrCurrentSettingEnabled();

    // SYS_CTL4

    bool isFullChargeCapacitySettingEnabled();

    // SYS_CTL9
    bool isStandbyModeEnabled();
    bool isBATlowSetEnabled();
    bool isBATLowEnabled();

    // SYS_CTL11

    bool isDcDcOutputEnabled();
    bool isVbusSrcDPdMEnabled();
    bool isVbusSrcPdEnabled();
    bool isVbusSrcSCPEnabled();

    // TypeC_CTL9

    bool is5VPdo3AEnabled();
    bool isPps2PdoIsetEnabled();
    bool isPps1PdoIsetEnabled();
    bool is20VPdoIsetEnabled();
    bool is15VPdoIsetEnabled();
    bool is12VPdoIsetEnabled();
    bool is9VPdoIsetEnabled();
    bool is5VPdoIsetEnabled();

    // TypeC_CTL17

    bool isSrcPdo9VEnabled();
    bool isSrcPdo12VEnabled();
    bool isSrcPdo15VEnabled();
    bool isSrcPdo20VEnabled();
    bool isSrcPps1PdoEnabled();
    bool isSrcPps2PdoEnabled();

    // STATE_CTL0

    bool isCharging();
    bool isChargeFull();
    bool isDischarging();

    // STATE_CTL1

    bool isFastCharge();

    // STATE_CTL2

    bool isVbusPresent();
    bool isVbusOvervoltage();

    // TypeC_STATE

    bool isTypeCSinkConnected();
    bool isTypeCSrcConnected();
    bool isTypeCSrcPdConnected();
    bool isTypeCSinkPdConnected();
    bool isVbusSinkQcActive();
    bool isVbusSrcQcActive();

    // MOS_STATE
    bool isVbusMosStateOpen();

    // STATE_CTL3

    bool isVsysOverCurrent();
    bool isVsysSdortCircuitDt();

    // ADC
    bool isOverHeat();

    ///////// SET ////////

    // SYS_CTL0

    void enableCharger(bool enable);
    void enableVbusSinkCtrl(bool enable);
    void enableVbusSinkSCP(bool enable);
    void enableVbusSinkPD(bool enable);
    void enableVbusSinkDPdM(bool enable);
    void enableINTLow(bool enable);
    void ResetMCU(bool enable);
    void enableLoadOTP(bool enable);

    // SYS_CTL1

    void enableBatteryTypeSetting(bool enable);
    void setBatteryType(BatteryType type);
    void enableCurrentOrPowerSettingMode(bool enable);
    void setCurrentOrPowerSettingMode(CurrentSettingMode mode);

    // SYS_CTL2

    void setFullChargeVoltage(uint16_t voltage);
    void enableFullChargeVoltageSetting(bool enable);

    // SYS_CTL3

    void enablePowerOrCurrentSetting(bool enable);
    void setMaxInputPowerOrBatteryCurrent(uint16_t value);

    // SYS_CTL4

    void enableFullChargeCapacitySetting(bool enable);
    void setFullChargeCapacity(uint16_t capacity);

    // SYS_CTL6

    void setCurrentBatteryCapacity(uint8_t capacity);

    // SYS_CTL7

    void setTrickleChargeCurrent(uint16_t current);
    void setTrickleChargeVoltage(uint16_t voltage_mv);
    void setChargeTimeout(ChargeTimeout timeout);

    // SYS_CTL8

    void setChargeStopCurrent(uint16_t current);
    void setCellRechargeThreshold(uint16_t voltageDrop_mV);

    // SYS_CTL9

    void enableStandbyMode(bool enable);
    void enableBATlowSet(bool enable);
    void enableBATLow(bool enable);

    // SYS_CTL10

    void setLowBatteryVoltage(uint16_t threshold_mV);

    // SYS_CTL11

    void setOutputFeatures(bool enableDcDcOutput, bool enableVbusSrcDPdM, bool enableVbusSrcPd, bool enableVbusSrcSCP);

    // SYS_CTL12

    void setMaxOutputPower(Vbus1OutputPower power);

    // TypeC_CTL8
    void setTypeCMode(TypeCMode mode);

    // TypeC_CTL9

    void enablePdoCurrentOutputSet(bool en5VPdoIset, bool en5VPdo3A, bool en9VPdoIset,
                                   bool en12VPdoIset, bool en15VPdoIset, bool en20VPdoIset,
                                   bool enPps1PdoIset, bool enPps2PdoIset);

    // TypeC_CTL10 - TypeC_CTL14

    void setPDOCurrent5V(uint16_t current_mA);
    void setPDOCurrent9V(uint16_t current_mA);
    void setPDOCurrent12V(uint16_t current_mA);
    void setPDOCurrent15V(uint16_t current_mA);
    void setPDOCurrent20V(uint16_t current_mA);

    // TypeC_CTL23 - TypeC_CTL24

    void setPDOCurrentPPS1(uint16_t current_mA);
    void setPDOCurrentPPS2(uint16_t current_mA);

    // TypeC_CTL17

    void enableSrcPdo(bool en9VPdo, bool en12VPdo, bool en15VPdo, bool en20VPdo, bool enPps1Pdo, bool enPps2Pdo);

    // SOC_CAP_DATA

    void setBatteryPercentage(uint8_t battery_level);

    ///////// GET ////////

    // SYS_CTL1

    BatteryType getBatteryType();
    CurrentSettingMode getPowerOrCurrentSettingMode();

    // SYS_CTL2

    uint16_t getFullChargeVoltage();

    // SYS_CTL3

    uint16_t getMaxInputPowerOrBatteryCurrent();

    // SYS_CTL4

    uint16_t getFullChargeCapacity();

    // SYS_CTL6

    uint8_t getCurrentBatteryCapacity();

    // SYS_CTL7

    uint16_t getTrickleChargeCurrent();
    uint16_t getTrickleChargeVoltage();
    ChargeTimeout getChargeTimeout();

    // SYS_CTL8

    uint16_t getChargeStopCurrent();
    uint16_t getCellRechargeThreshold();

    // SYS_CTL10

    uint16_t getLowBatteryVoltage();

    // SYS_CTL12

    Vbus1OutputPower getMaxOutputPower();

    // TypeC_CTL8

    TypeCMode getTypeCMode();

    // TypeC_CTL10 - TypeC_CTL14

    uint16_t getPDOCurrent5V();
    uint16_t getPDOCurrent9V();
    uint16_t getPDOCurrent12V();
    uint16_t getPDOCurrent15V();
    uint16_t getPDOCurrent20V();

    // TypeC_CTL23 - TypeC_CTL24

    uint16_t getPDOCurrentPPS1();
    uint16_t getPDOCurrentPPS2();

    // SOC_CAP_DATA

    uint8_t getBatteryPercentage();

    // STATE_CTL0

    ChargeState getChargeState();

    uint8_t getChargeVoltage();

    // ADC

    uint16_t getVBATVoltage();
    uint16_t getVsysVoltage();
    uint16_t getChargeInputCurrent();
    uint16_t getDischargeOutputCurrent();
    uint16_t getBATCurrent();
    uint16_t getVsysCurrent();
    uint32_t getVsysPower();

    // GPIO

    uint16_t getNTCVoltage();
    uint16_t getCurrentSettingVoltage();
    uint16_t getVoltageSettingVoltage();
    uint16_t getFCAPVoltage();
    uint16_t getBatteryCountVoltage();

private:
    uint8_t writeRegister(uint8_t regAddress, uint8_t value);
    uint8_t readRegister(uint8_t regAddress);
    inline uint8_t setBit(uint8_t value, uint8_t bit, bool enable);
    void writeTypeCCurrentSetting(uint8_t reg, uint16_t current_mA, uint16_t step, uint16_t maxCurrent);
};

#endif
