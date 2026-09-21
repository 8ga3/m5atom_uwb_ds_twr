#include "uwb_link.h"

#include <Arduino.h>

#include "hw_pins.h"

M5Stamp_UWB uwb;
M5Stamp_UWBDSRangeConfig rangeConfig;

bool initUwb(uint32_t spiFastHz)
{
    // Stamp-UWB connections to the QM33120 UWB transceiver.
    M5Stamp_UWBConfig config;
    config.pin_gp7    = M5STAMP_UWB_PIN_UNUSED;
    config.pin_wakeup = M5STAMP_UWB_PIN_UNUSED;
    config.pin_irq    = UWB_PIN_IRQ;
    config.pin_rst    = UWB_PIN_RST;
    config.pin_miso   = UWB_PIN_MISO;
    config.pin_mosi   = UWB_PIN_MOSI;
    config.pin_sck    = UWB_PIN_SCK;
    config.pin_cs     = UWB_PIN_CS;
    // Keep spi_slow_hz at the library default (2MHz): probing and the reset
    // sequence run before the chip's clock PLL is up, where only slow SPI is
    // guaranteed. Only the post-init rate is raised here.
    config.spi_fast_hz = spiFastHz;

    M5Stamp_UWBPHYConfig phy;
    // Channel 9 is the only UWB channel permitted in Japan; both sides must match.
    phy.channel = M5Stamp_UWBChannel::Channel9;

    // Both devices must use the same PAN and DS-TWR timing. initiatorAddress /
    // responderAddress は呼び出し側 (ANCHOR/TAG それぞれの setup()) が
    // initUwb() を呼ぶ前に設定済みという前提。
    rangeConfig.panId                          = 0xDECA;
    rangeConfig.responseRxAfterTxDelayUus      = 1500;
    rangeConfig.responseTxDelayUus             = 3000;
    rangeConfig.finalTxDelayUus                = 1800;
    rangeConfig.finalRxAfterResponseTxDelayUus = 500;
    rangeConfig.resultRxAfterFinalTxDelayUus   = 500;
    rangeConfig.rxTimeoutUus                   = 3000;
    rangeConfig.hostTimeoutMs                  = 100;
    rangeConfig.resultRepeatCount              = 3;
    rangeConfig.resultRepeatGapMs              = 3;

    if (!uwb.begin(config, phy)) {
        Serial.printf("UWB_BEGIN,result=FAIL,error=%s\n", uwb.lastErrorName());
        Serial.printf("UWB_RAW_ID,dev_id=0x%08lX\n", static_cast<unsigned long>(uwb.readRawDeviceId()));
        return false;
    }

    const uint32_t devId = uwb.deviceId();
    Serial.printf("UWB_ID,dev_id=0x%08lX,chip=%s\n", static_cast<unsigned long>(devId), uwb.chipName());
    if (devId != M5STAMP_UWB_QM33120_DEVICE_ID) {
        Serial.printf("UWB_ID,result=FAIL,expected=0xDECA0314\n");
        return false;
    }

    // deviceId() is the value cached while probing, when the bus still ran at
    // spi_slow_hz. The driver switched to spi_fast_hz inside begin(), so read
    // the register once more: a corrupt readback means this wiring cannot hold
    // the requested rate, and every later transfer would be silently unreliable.
    const uint32_t fastId = uwb.readRawDeviceId();
    if (fastId != M5STAMP_UWB_QM33120_DEVICE_ID) {
        Serial.printf("UWB_SPI,result=FAIL,fast_hz=%lu,dev_id=0x%08lX\n", static_cast<unsigned long>(spiFastHz),
                      static_cast<unsigned long>(fastId));
        return false;
    }
    Serial.printf("UWB_SPI,result=OK,fast_hz=%lu\n", static_cast<unsigned long>(spiFastHz));

    Serial.printf("UWB_CONFIG,result=OK,ch=%u,plen=%u,rate=6M8,tx_power=0x%08lX\n", static_cast<unsigned>(phy.channel),
                  static_cast<unsigned>(phy.preambleLength), static_cast<unsigned long>(phy.txPower));
    return true;
}
