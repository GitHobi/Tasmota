/*
  tasmota_template.h - template settings for Tasmota

  Copyright (C) 2020  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _TASMOTA_TEMPLATE_H_
#define _TASMOTA_TEMPLATE_H_

// User selectable GPIO functionality
// ATTENTION: Only add at the end of this list just before GPIO_SENSOR_END
//            Then add the same name(s) in a nice location in array kGpioNiceList
enum UserSelectablePins {
  GPIO_NONE,                           // Not used
  GPIO_KEY1, GPIO_KEY1_NP, GPIO_KEY1_INV, GPIO_KEY1_INV_NP, // 4 x Button
  GPIO_SWT1, GPIO_SWT1_NP,             // 8 x User connected external switches
  GPIO_REL1, GPIO_REL1_INV,            // 8 x Relays
  GPIO_LED1, GPIO_LED1_INV,            // 4 x Leds
  GPIO_CNTR1, GPIO_CNTR1_NP,           // 4 x Counter
  GPIO_PWM1, GPIO_PWM1_INV,            // 5 x PWM
  GPIO_BUZZER, GPIO_BUZZER_INV,        // Buzzer
  GPIO_LEDLNK, GPIO_LEDLNK_INV,        // Link led
  GPIO_I2C_SCL, GPIO_I2C_SDA,          // Software I2C
  GPIO_SPI_MISO, GPIO_SPI_MOSI, GPIO_SPI_CLK, GPIO_SPI_CS, GPIO_SPI_DC,        // Hardware SPI
  GPIO_SSPI_MISO, GPIO_SSPI_MOSI, GPIO_SSPI_SCLK, GPIO_SSPI_CS, GPIO_SSPI_DC,  // Software SPI
  GPIO_BACKLIGHT,                      // Display backlight control
  GPIO_OLED_RESET,                     // OLED Display Reset
  GPIO_IRSEND, GPIO_IRRECV,            // IR interface
  GPIO_RFSEND, GPIO_RFRECV,            // RF interface
  GPIO_DHT11, GPIO_DHT22, GPIO_SI7021, GPIO_DHT11_OUT,  // DHT11, DHT21, DHT22, AM2301, AM2302, AM2321
  GPIO_DSB, GPIO_DSB_OUT,              // DS18B20 or DS18S20
  GPIO_WS2812,                         // WS2812 Led string
  GPIO_MHZ_TXD, GPIO_MHZ_RXD,          // MH-Z19 Serial interface
  GPIO_PZEM0XX_TX, GPIO_PZEM004_RX, GPIO_PZEM016_RX, GPIO_PZEM017_RX, // PZEM Serial Modbus interface
  GPIO_SAIR_TX, GPIO_SAIR_RX,          // SenseAir Serial interface
  GPIO_PMS5003_TX, GPIO_PMS5003_RX,    // Plantower PMS5003 Serial interface
  GPIO_SDS0X1_TX, GPIO_SDS0X1_RX,      // Nova Fitness SDS011 Serial interface
  GPIO_SBR_TX, GPIO_SBR_RX,            // Serial Bridge Serial interface
  GPIO_SR04_TRIG, GPIO_SR04_ECHO,      // SR04 interface
  GPIO_SDM120_TX, GPIO_SDM120_RX,      // SDM120 Serial interface
  GPIO_SDM630_TX, GPIO_SDM630_RX,      // SDM630 Serial interface
  GPIO_TM16CLK, GPIO_TM16DIO, GPIO_TM16STB,  // TM1638 interface
  GPIO_MP3_DFR562,                     // RB-DFR-562, DFPlayer Mini MP3 Player
  GPIO_HX711_SCK, GPIO_HX711_DAT,      // HX711 Load Cell interface
  GPIO_TX2X_TXD_BLACK,                 // TX20/TX23 Transmission Pin
  GPIO_TUYA_TX, GPIO_TUYA_RX,          // Tuya Serial interface
  GPIO_MGC3130_XFER, GPIO_MGC3130_RESET,  // MGC3130 interface
  GPIO_RF_SENSOR,                      // Rf receiver with sensor decoding
  GPIO_AZ_TXD, GPIO_AZ_RXD,            // AZ-Instrument 7798 Serial interface
  GPIO_MAX31855CS, GPIO_MAX31855CLK, GPIO_MAX31855DO,  // MAX31855 Serial interface
  GPIO_NRG_SEL, GPIO_NRG_SEL_INV, GPIO_NRG_CF1, GPIO_HLW_CF, GPIO_HJL_CF,  // HLW8012/HJL-01/BL0937 energy monitoring
  GPIO_MCP39F5_TX, GPIO_MCP39F5_RX, GPIO_MCP39F5_RST,  // MCP39F501 Energy monitoring (Shelly2)
  GPIO_PN532_TXD, GPIO_PN532_RXD,      // PN532 NFC Serial interface
  GPIO_SM16716_CLK, GPIO_SM16716_DAT, GPIO_SM16716_SEL,  // SM16716 SELECT
  GPIO_DI, GPIO_DCKI,                  // my92x1 PWM controller
  GPIO_CSE7766_TX, GPIO_CSE7766_RX,    // CSE7766 Serial interface (S31 and Pow R2)
  GPIO_ARIRFRCV, GPIO_ARIRFSEL,        // Arilux RF Receive input
  GPIO_TXD, GPIO_RXD,                  // Serial interface
  GPIO_ROT1A, GPIO_ROT1B,              // Rotary switch
  GPIO_ADC_JOY,                        // Analog joystick
  GPIO_SSPI_MAX31865_CS1,              // MAX31865 Chip Select
  GPIO_HRE_CLOCK, GPIO_HRE_DATA,       // HR-E Water Meter
  GPIO_ADE7953_IRQ,                    // ADE7953 IRQ
  GPIO_SOLAXX1_TX, GPIO_SOLAXX1_RX,    // Solax Inverter Serial interface
  GPIO_ZIGBEE_TX, GPIO_ZIGBEE_RX,      // Zigbee Serial interface
  GPIO_RDM6300_RX,                     // RDM6300 RX
  GPIO_IBEACON_TX, GPIO_IBEACON_RX,    // HM17 IBEACON Serial interface
  GPIO_A4988_DIR, GPIO_A4988_STP, GPIO_A4988_ENA, GPIO_A4988_MS1,  // A4988 interface
  GPIO_OUTPUT_HI, GPIO_OUTPUT_LO,      // Fixed output state
  GPIO_DDS2382_TX, GPIO_DDS2382_RX,    // DDS2382 Serial interface
  GPIO_DDSU666_TX, GPIO_DDSU666_RX,    // DDSU666 Serial interface
  GPIO_SM2135_CLK, GPIO_SM2135_DAT,    // SM2135 PWM controller
  GPIO_DEEPSLEEP,                      // Kill switch for deepsleep
  GPIO_EXS_ENABLE,                     // EXS MCU Enable
  GPIO_TASMOTACLIENT_TXD, GPIO_TASMOTACLIENT_RXD,      // Client Serial interface
  GPIO_TASMOTACLIENT_RST, GPIO_TASMOTACLIENT_RST_INV,  // Client Reset
  GPIO_HPMA_RX, GPIO_HPMA_TX,          // Honeywell HPMA115S0 Serial interface
  GPIO_GPS_RX, GPIO_GPS_TX,            // GPS Serial interface
  GPIO_HM10_RX, GPIO_HM10_TX,          // HM10-BLE-Mijia-bridge Serial interface
  GPIO_LE01MR_RX, GPIO_LE01MR_TX,      // F&F LE-01MR energy meter
  GPIO_CC1101_GDO0, GPIO_CC1101_GDO2,  // CC1101 Serial interface
  GPIO_HRXL_RX,                        // Data from MaxBotix HRXL sonar range sensor
  GPIO_ELECTRIQ_MOODL_TX,              // ElectriQ iQ-wifiMOODL Serial TX
  GPIO_AS3935,                         // Franklin Lightning Sensor
  GPIO_ADC_INPUT,                      // Analog input
  GPIO_ADC_TEMP,                       // Analog Thermistor
  GPIO_ADC_LIGHT,                      // Analog Light sensor
  GPIO_ADC_BUTTON, GPIO_ADC_BUTTON_INV,  // Analog Button
  GPIO_ADC_RANGE,                      // Analog Range
  GPIO_ADC_CT_POWER,                   // ANalog Current
#ifdef ESP32
  GPIO_WEBCAM_PWDN, GPIO_WEBCAM_RESET, GPIO_WEBCAM_XCLK,  // Webcam
  GPIO_WEBCAM_SIOD, GPIO_WEBCAM_SIOC,  // Webcam I2C
  GPIO_WEBCAM_DATA,
  GPIO_WEBCAM_VSYNC, GPIO_WEBCAM_HREF, GPIO_WEBCAM_PCLK,
  GPIO_WEBCAM_PSCLK,
  GPIO_WEBCAM_HSD,
  GPIO_WEBCAM_PSRCS,
#endif
  GPIO_BOILER_OT_RX, GPIO_BOILER_OT_TX,  // OpenTherm Boiler TX pin
  GPIO_WINDMETER_SPEED,                // WindMeter speed counter pin
  GPIO_KEY1_TC,                        // Touch pin as button
  GPIO_BL0940_RX,                      // BL0940 serial interface
  GPIO_TCP_TX, GPIO_TCP_RX,            // TCP to serial bridge
#ifdef ESP32
  GPIO_ETH_PHY_POWER, GPIO_ETH_PHY_MDC, GPIO_ETH_PHY_MDIO,  // Ethernet
#endif
  GPIO_TELEINFO_RX,                    // Teleinfo telemetry data receive pin
  GPIO_TELEINFO_ENABLE,                // Teleinfo Enable Receive Pin
  GPIO_LMT01,                          // LMT01 input counting pin
  GPIO_IEM3000_TX, GPIO_IEM3000_RX,    // IEM3000 Serial interface
  GPIO_ZIGBEE_RST,                     // Zigbee reset
  GPIO_DYP_RX,
  GPIO_MIEL_HVAC_TX, GPIO_MIEL_HVAC_RX,  // Mitsubishi Electric HVAC
  GPIO_WE517_TX, GPIO_WE517_RX,        // ORNO WE517 Serial interface
  GPIO_AS608_TX, GPIO_AS608_RX,        // Serial interface AS608 / R503
  GPIO_SHELLY_DIMMER_BOOT0, GPIO_SHELLY_DIMMER_RST_INV,
  GPIO_RC522_RST,                      // RC522 reset
  GPIO_P9813_CLK, GPIO_P9813_DAT,      // P9813 Clock and Data
  GPIO_SENSOR_END };

enum ProgramSelectablePins {
  GPIO_FIX_START = 2046,
  GPIO_USER,           // User configurable needs to be 2047
  GPIO_MAX };

// Text in webpage Module Parameters and commands GPIOS and GPIO
const char kSensorNames[] PROGMEM =
  D_SENSOR_NONE "|"
  D_SENSOR_BUTTON "|" D_SENSOR_BUTTON "_n|" D_SENSOR_BUTTON "_i|" D_SENSOR_BUTTON "_in|"
  D_SENSOR_SWITCH "|" D_SENSOR_SWITCH "_n|"
  D_SENSOR_RELAY "|" D_SENSOR_RELAY "_i|"
  D_SENSOR_LED "|" D_SENSOR_LED "_i|"
  D_SENSOR_COUNTER "|" D_SENSOR_COUNTER "_n|"
  D_SENSOR_PWM "|" D_SENSOR_PWM "_i|"
  D_SENSOR_BUZZER "|" D_SENSOR_BUZZER "_i|"
  D_SENSOR_LED_LINK "|" D_SENSOR_LED_LINK "_i|"
  D_SENSOR_I2C_SCL "|" D_SENSOR_I2C_SDA "|"
  D_SENSOR_SPI_MISO "|" D_SENSOR_SPI_MOSI "|" D_SENSOR_SPI_CLK "|" D_SENSOR_SPI_CS "|" D_SENSOR_SPI_DC "|"
  D_SENSOR_SSPI_MISO "|" D_SENSOR_SSPI_MOSI "|" D_SENSOR_SSPI_SCLK "|" D_SENSOR_SSPI_CS "|" D_SENSOR_SSPI_DC "|"
  D_SENSOR_BACKLIGHT "|" D_SENSOR_OLED_RESET "|"
  D_SENSOR_IRSEND "|" D_SENSOR_IRRECV "|"
  D_SENSOR_RFSEND "|" D_SENSOR_RFRECV "|"
  D_SENSOR_DHT11 "|" D_SENSOR_AM2301 "|" D_SENSOR_SI7021 "|" D_SENSOR_DHT11 "_o|"
  D_SENSOR_DS18X20 "|" D_SENSOR_DS18X20 "_o|"
  D_SENSOR_WS2812 "|"
  D_SENSOR_MHZ_TX "|" D_SENSOR_MHZ_RX "|"
  D_SENSOR_PZEM0XX_TX "|" D_SENSOR_PZEM004_RX "|" D_SENSOR_PZEM016_RX "|" D_SENSOR_PZEM017_RX "|"
  D_SENSOR_SAIR_TX "|" D_SENSOR_SAIR_RX "|"
  D_SENSOR_PMS5003_TX "|" D_SENSOR_PMS5003_RX "|"
  D_SENSOR_SDS0X1_TX "|" D_SENSOR_SDS0X1_RX "|"
  D_SENSOR_SBR_TX "|" D_SENSOR_SBR_RX "|"
  D_SENSOR_SR04_TRIG "|" D_SENSOR_SR04_ECHO "|"
  D_SENSOR_SDM120_TX "|" D_SENSOR_SDM120_RX "|"
  D_SENSOR_SDM630_TX "|" D_SENSOR_SDM630_RX "|"
  D_SENSOR_TM1638_CLK "|" D_SENSOR_TM1638_DIO "|" D_SENSOR_TM1638_STB "|"
  D_SENSOR_DFR562 "|"
  D_SENSOR_HX711_SCK "|" D_SENSOR_HX711_DAT "|"
  D_SENSOR_TX2X_TX "|"
  D_SENSOR_TUYA_TX "|" D_SENSOR_TUYA_RX "|"
  D_SENSOR_MGC3130_XFER "|" D_SENSOR_MGC3130_RESET "|"
  D_SENSOR_RF_SENSOR "|"
  D_SENSOR_AZ_TX "|" D_SENSOR_AZ_RX "|"
  D_SENSOR_MAX31855_CS "|" D_SENSOR_MAX31855_CLK "|" D_SENSOR_MAX31855_DO "|"
  D_SENSOR_NRG_SEL "|" D_SENSOR_NRG_SEL "_i|" D_SENSOR_NRG_CF1 "|" D_SENSOR_HLW_CF "|" D_SENSOR_HJL_CF "|"
  D_SENSOR_MCP39F5_TX "|" D_SENSOR_MCP39F5_RX "|" D_SENSOR_MCP39F5_RST "|"
  D_SENSOR_PN532_TX "|" D_SENSOR_PN532_RX "|"
  D_SENSOR_SM16716_CLK "|" D_SENSOR_SM16716_DAT "|" D_SENSOR_SM16716_POWER "|"
  D_SENSOR_MY92X1_DI "|" D_SENSOR_MY92X1_DCKI "|"
  D_SENSOR_CSE7766_TX "|" D_SENSOR_CSE7766_RX "|"
  D_SENSOR_ARIRFRCV "|" D_SENSOR_ARIRFSEL "|"
  D_SENSOR_TXD "|" D_SENSOR_RXD "|"
  D_SENSOR_ROTARY "_a|" D_SENSOR_ROTARY "_b|"
  D_SENSOR_ADC_JOYSTICK "|"
  D_SENSOR_MAX31865_CS "|"
  D_SENSOR_HRE_CLOCK "|" D_SENSOR_HRE_DATA "|"
  D_SENSOR_ADE7953_IRQ "|"
  D_SENSOR_SOLAXX1_TX "|" D_SENSOR_SOLAXX1_RX "|"
  D_SENSOR_ZIGBEE_TXD "|" D_SENSOR_ZIGBEE_RXD "|"
  D_SENSOR_RDM6300_RX "|"
  D_SENSOR_IBEACON_TX "|" D_SENSOR_IBEACON_RX "|"
  D_SENSOR_A4988_DIR "|" D_SENSOR_A4988_STP "|" D_SENSOR_A4988_ENA "|" D_SENSOR_A4988_MS1 "|"
  D_SENSOR_OUTPUT_HI "|" D_SENSOR_OUTPUT_LO "|"
  D_SENSOR_DDS2382_TX "|" D_SENSOR_DDS2382_RX "|"
  D_SENSOR_DDSU666_TX "|" D_SENSOR_DDSU666_RX "|"
  D_SENSOR_SM2135_CLK "|" D_SENSOR_SM2135_DAT "|"
  D_SENSOR_DEEPSLEEP "|" D_SENSOR_EXS_ENABLE "|"
  D_SENSOR_CLIENT_TX "|" D_SENSOR_CLIENT_RX "|" D_SENSOR_CLIENT_RESET "|" D_SENSOR_CLIENT_RESET "_i|"
  D_SENSOR_HPMA_RX "|" D_SENSOR_HPMA_TX "|"
  D_SENSOR_GPS_RX "|" D_SENSOR_GPS_TX "|"
  D_SENSOR_HM10_RX "|" D_SENSOR_HM10_TX "|"
  D_SENSOR_LE01MR_RX "|" D_SENSOR_LE01MR_TX "|"
  D_SENSOR_CC1101_GDO0 "|" D_SENSOR_CC1101_GDO2 "|"
  D_SENSOR_HRXL_RX "|"
  D_SENSOR_ELECTRIQ_MOODL "|"
  D_SENSOR_AS3935 "|"
  D_SENSOR_ADC_INPUT "|"
  D_SENSOR_ADC_TEMP "|"
  D_SENSOR_ADC_LIGHT "|"
  D_SENSOR_ADC_BUTTON "|" D_SENSOR_ADC_BUTTON "_i|"
  D_SENSOR_ADC_RANGE "|"
  D_SENSOR_ADC_CT_POWER "|"
#ifdef ESP32
  D_GPIO_WEBCAM_PWDN "|" D_GPIO_WEBCAM_RESET "|" D_GPIO_WEBCAM_XCLK "|"
  D_GPIO_WEBCAM_SIOD "|" D_GPIO_WEBCAM_SIOC "|"
  D_GPIO_WEBCAM_DATA "|"
  D_GPIO_WEBCAM_VSYNC "|" D_GPIO_WEBCAM_HREF "|" D_GPIO_WEBCAM_PCLK "|"
  D_GPIO_WEBCAM_PSCLK "|"
  D_GPIO_WEBCAM_HSD "|"
  D_GPIO_WEBCAM_PSRCS "|"
#endif
  D_SENSOR_BOILER_OT_RX "|" D_SENSOR_BOILER_OT_TX "|"
  D_SENSOR_WINDMETER_SPEED "|" D_SENSOR_BUTTON "_tc|"
  D_SENSOR_BL0940_RX "|"
  D_SENSOR_TCP_TXD "|" D_SENSOR_TCP_RXD "|"
#ifdef ESP32
  D_SENSOR_ETH_PHY_POWER "|" D_SENSOR_ETH_PHY_MDC "|" D_SENSOR_ETH_PHY_MDIO "|"
#endif
  D_SENSOR_TELEINFO_RX "|" D_SENSOR_TELEINFO_ENABLE "|"
  D_SENSOR_LMT01_PULSE "|"
  D_SENSOR_IEM3000_TX "|" D_SENSOR_IEM3000_RX "|"
  D_SENSOR_ZIGBEE_RST "|"
  D_SENSOR_DYP_RX "|"
  D_SENSOR_MIEL_HVAC_TX "|" D_SENSOR_MIEL_HVAC_RX "|"
  D_SENSOR_WE517_TX "|" D_SENSOR_WE517_RX "|"
  D_SENSOR_AS608_TX "|" D_SENSOR_AS608_RX "|"
  D_SENSOR_SHELLY_DIMMER_BOOT0 "|" D_SENSOR_SHELLY_DIMMER_RST_INV "|"
  D_SENSOR_RC522_RST "|"
  D_SENSOR_P9813_CLK "|" D_SENSOR_P9813_DAT
  ;

const char kSensorNamesFixed[] PROGMEM =
  D_SENSOR_USER;

#define MAX_MAX31865S    6
#define MAX_A4988_MSS    3
#define MAX_WEBCAM_DATA  8
#define MAX_WEBCAM_HSD   3
#define MAX_SM2135_DAT   4

const uint16_t kGpioNiceList[] PROGMEM = {
  GPIO_NONE,                            // Not used
  AGPIO(GPIO_KEY1) + MAX_KEYS,          // Buttons
  AGPIO(GPIO_KEY1_NP) + MAX_KEYS,
  AGPIO(GPIO_KEY1_INV) + MAX_KEYS,
  AGPIO(GPIO_KEY1_INV_NP) + MAX_KEYS,
  AGPIO(GPIO_KEY1_TC) + MAX_KEYS,       // Touch button
  AGPIO(GPIO_SWT1) + MAX_SWITCHES,      // User connected external switches
  AGPIO(GPIO_SWT1_NP) + MAX_SWITCHES,
#ifdef ROTARY_V1
  AGPIO(GPIO_ROT1A) + MAX_ROTARIES,     // Rotary A Pin
  AGPIO(GPIO_ROT1B) + MAX_ROTARIES,     // Rotary B Pin
#endif
  AGPIO(GPIO_REL1) + MAX_RELAYS,        // Relays
  AGPIO(GPIO_REL1_INV) + MAX_RELAYS,
  AGPIO(GPIO_LED1) + MAX_LEDS,          // Leds
  AGPIO(GPIO_LED1_INV) + MAX_LEDS,
#ifdef USE_COUNTER
  AGPIO(GPIO_CNTR1) + MAX_COUNTERS,     // Counters
  AGPIO(GPIO_CNTR1_NP) + MAX_COUNTERS,
#endif
  AGPIO(GPIO_PWM1) + MAX_PWMS,          // RGB   Red   or C  Cold White
  AGPIO(GPIO_PWM1_INV) + MAX_PWMS,
#ifdef USE_BUZZER
  AGPIO(GPIO_BUZZER),                   // Buzzer
  AGPIO(GPIO_BUZZER_INV),               // Inverted buzzer
#endif
  AGPIO(GPIO_LEDLNK),                   // Link led
  AGPIO(GPIO_LEDLNK_INV),               // Inverted link led
  AGPIO(GPIO_OUTPUT_HI),                // Fixed output high
  AGPIO(GPIO_OUTPUT_LO),                // Fixed output low

/*-------------------------------------------------------------------------------------------*\
 * Protocol specifics
\*-------------------------------------------------------------------------------------------*/

#ifdef USE_I2C
  AGPIO(GPIO_I2C_SCL),                  // I2C SCL
  AGPIO(GPIO_I2C_SDA),                  // I2C SDA
#endif

#ifdef USE_SPI
  AGPIO(GPIO_SPI_MISO),       // SPI MISO
  AGPIO(GPIO_SPI_MOSI),       // SPI MOSI
  AGPIO(GPIO_SPI_CLK),        // SPI Clk
  AGPIO(GPIO_SPI_CS),         // SPI Chip Select
  AGPIO(GPIO_SPI_DC),         // SPI Data Direction
#endif
  AGPIO(GPIO_SSPI_MISO),      // Software SPI Master Input Client Output
  AGPIO(GPIO_SSPI_MOSI),      // Software SPI Master Output Client Input
  AGPIO(GPIO_SSPI_SCLK),      // Software SPI Serial Clock
  AGPIO(GPIO_SSPI_CS),        // Software SPI Chip Select
  AGPIO(GPIO_SSPI_DC),        // Software SPI Data or Command
#ifdef USE_DISPLAY
  AGPIO(GPIO_BACKLIGHT),      // Display backlight control
  AGPIO(GPIO_OLED_RESET),     // OLED Display Reset
#endif
#ifdef USE_MAX31865
  AGPIO(GPIO_SSPI_MAX31865_CS1) + MAX_MAX31865S,
#endif

  AGPIO(GPIO_TXD),            // Serial interface
  AGPIO(GPIO_RXD),            // Serial interface

/*-------------------------------------------------------------------------------------------*\
 * Single wire sensors
\*-------------------------------------------------------------------------------------------*/

#ifdef USE_DHT
  AGPIO(GPIO_DHT11),          // DHT11
  AGPIO(GPIO_DHT22),          // DHT21, DHT22, AM2301, AM2302, AM2321
  AGPIO(GPIO_SI7021),         // iTead SI7021
  AGPIO(GPIO_DHT11_OUT),      // Pseudo Single wire DHT11, DHT21, DHT22, AM2301, AM2302, AM2321
#endif
#ifdef USE_DS18x20
  AGPIO(GPIO_DSB),            // Single wire DS18B20 or DS18S20
  AGPIO(GPIO_DSB_OUT),        // Pseudo Single wire DS18B20 or DS18S20
#endif
#ifdef USE_LMT01
  AGPIO(GPIO_LMT01),          // LMT01, count pulses on GPIO
#endif

/*-------------------------------------------------------------------------------------------*\
 * Light
\*-------------------------------------------------------------------------------------------*/

#ifdef USE_LIGHT
#ifdef USE_WS2812
#if (USE_WS2812_HARDWARE == NEO_HW_P9813)
  AGPIO(GPIO_P9813_CLK),      // P9813 CLOCK
  AGPIO(GPIO_P9813_DAT),      // P9813 DATA
#else
  AGPIO(GPIO_WS2812),         // WS2812 Led string
#endif  // NEO_HW_P9813
#endif
#ifdef USE_ARILUX_RF
  AGPIO(GPIO_ARIRFRCV),       // AriLux RF Receive input
  AGPIO(GPIO_ARIRFSEL),       // Arilux RF Receive input selected
#endif
#ifdef USE_MY92X1
  AGPIO(GPIO_DI),             // my92x1 PWM input
  AGPIO(GPIO_DCKI),           // my92x1 CLK input
#endif  // USE_MY92X1
#ifdef USE_SM16716
  AGPIO(GPIO_SM16716_CLK),    // SM16716 CLOCK
  AGPIO(GPIO_SM16716_DAT),    // SM16716 DATA
  AGPIO(GPIO_SM16716_SEL),    // SM16716 SELECT
#endif  // USE_SM16716
#ifdef USE_SM2135
  AGPIO(GPIO_SM2135_CLK),                    // SM2135 CLOCK
  AGPIO(GPIO_SM2135_DAT) + MAX_SM2135_DAT,   // SM2135 DATA
#endif  // USE_SM2135
#ifdef USE_TUYA_MCU
  AGPIO(GPIO_TUYA_TX),        // Tuya Serial interface
  AGPIO(GPIO_TUYA_RX),        // Tuya Serial interface
#endif
#ifdef USE_EXS_DIMMER
  AGPIO(GPIO_EXS_ENABLE),     // EXS MCU Enable
#endif
#ifdef USE_ELECTRIQ_MOODL
  AGPIO(GPIO_ELECTRIQ_MOODL_TX),
#endif
#ifdef USE_SHELLY_DIMMER
  AGPIO(GPIO_SHELLY_DIMMER_BOOT0),
  AGPIO(GPIO_SHELLY_DIMMER_RST_INV),
#endif
#endif  // USE_LIGHT

/*-------------------------------------------------------------------------------------------*\
 * Transmission sensors
\*-------------------------------------------------------------------------------------------*/

#if defined(USE_IR_REMOTE) || defined(USE_IR_REMOTE_FULL)
  AGPIO(GPIO_IRSEND),         // IR remote
#if defined(USE_IR_RECEIVE) || defined(USE_IR_REMOTE_FULL)
  AGPIO(GPIO_IRRECV),         // IR receiver
#endif
#endif
#ifdef USE_RC_SWITCH
  AGPIO(GPIO_RFSEND),         // RF transmitter
  AGPIO(GPIO_RFRECV),         // RF receiver
#endif
#ifdef USE_RF_SENSOR
  AGPIO(GPIO_RF_SENSOR),      // Rf receiver with sensor decoding
#endif
#ifdef USE_SR04
  AGPIO(GPIO_SR04_TRIG),      // SR04 Tri/TXgger pin
  AGPIO(GPIO_SR04_ECHO),      // SR04 Ech/RXo pin
#endif
#ifdef USE_TM1638
  AGPIO(GPIO_TM16CLK),        // TM1638 Clock
  AGPIO(GPIO_TM16DIO),        // TM1638 Data I/O
  AGPIO(GPIO_TM16STB),        // TM1638 Strobe
#endif
#ifdef USE_HX711
  AGPIO(GPIO_HX711_SCK),      // HX711 Load Cell clock
  AGPIO(GPIO_HX711_DAT),      // HX711 Load Cell data
#endif

/*-------------------------------------------------------------------------------------------*\
 * Energy sensors
\*-------------------------------------------------------------------------------------------*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_HLW8012
  AGPIO(GPIO_NRG_SEL),        // HLW8012/HLJ-01 Sel output (1 = Voltage)
  AGPIO(GPIO_NRG_SEL_INV),    // HLW8012/HLJ-01 Sel output (0 = Voltage)
  AGPIO(GPIO_NRG_CF1),        // HLW8012/HLJ-01 CF1 voltage / current
  AGPIO(GPIO_HLW_CF),         // HLW8012 CF power
  AGPIO(GPIO_HJL_CF),         // HJL-01/BL0937 CF power
#endif
#if defined(USE_I2C) && defined(USE_ADE7953)
  AGPIO(GPIO_ADE7953_IRQ),    // ADE7953 IRQ
#endif
#ifdef USE_CSE7766
  AGPIO(GPIO_CSE7766_TX),     // CSE7766 Serial interface (S31 and Pow R2)
  AGPIO(GPIO_CSE7766_RX),     // CSE7766 Serial interface (S31 and Pow R2)
#endif
#ifdef USE_MCP39F501
  AGPIO(GPIO_MCP39F5_TX),     // MCP39F501 Serial interface (Shelly2)
  AGPIO(GPIO_MCP39F5_RX),     // MCP39F501 Serial interface (Shelly2)
  AGPIO(GPIO_MCP39F5_RST),    // MCP39F501 Reset (Shelly2)
#endif
#if defined(USE_PZEM004T) || defined(USE_PZEM_AC) || defined(USE_PZEM_DC)
  AGPIO(GPIO_PZEM0XX_TX),     // PZEM0XX Serial interface
#endif
#ifdef USE_PZEM004T
  AGPIO(GPIO_PZEM004_RX),     // PZEM004T Serial interface
#endif
#ifdef USE_PZEM_AC
  AGPIO(GPIO_PZEM016_RX),     // PZEM-014,016 Serial Modbus interface
#endif
#ifdef USE_PZEM_DC
  AGPIO(GPIO_PZEM017_RX),     // PZEM-003,017 Serial Modbus interface
#endif
#ifdef USE_SDM120
  AGPIO(GPIO_SDM120_TX),      // SDM120 Serial interface
  AGPIO(GPIO_SDM120_RX),      // SDM120 Serial interface
#endif
#ifdef USE_SDM630
  AGPIO(GPIO_SDM630_TX),      // SDM630 Serial interface
  AGPIO(GPIO_SDM630_RX),      // SDM630 Serial interface
#endif
#ifdef USE_DDS2382
  AGPIO(GPIO_DDS2382_TX),     // DDS2382 Serial interface
  AGPIO(GPIO_DDS2382_RX),     // DDS2382 Serial interface
#endif
#ifdef USE_DDSU666
  AGPIO(GPIO_DDSU666_TX),     // DDSU666 Serial interface
  AGPIO(GPIO_DDSU666_RX),     // DDSU666 Serial interface
#endif  // USE_DDSU666
#ifdef USE_SOLAX_X1
  AGPIO(GPIO_SOLAXX1_TX),     // Solax Inverter tx pin
  AGPIO(GPIO_SOLAXX1_RX),     // Solax Inverter rx pin
#endif // USE_SOLAX_X1
#ifdef USE_LE01MR
  AGPIO(GPIO_LE01MR_TX),     // F7F LE-01MR energy meter tx pin
  AGPIO(GPIO_LE01MR_RX),     // F7F LE-01MR energy meter rx pin
#endif // IFDEF:USE_LE01MR
#ifdef USE_BL0940
  AGPIO(GPIO_BL0940_RX),     // BL0940 Serial interface
#endif
#ifdef USE_IEM3000
  AGPIO(GPIO_IEM3000_TX),    // IEM3000 Serial interface
  AGPIO(GPIO_IEM3000_RX),    // IEM3000 Serial interface
#endif
#ifdef USE_WE517
  AGPIO(GPIO_WE517_TX),      // WE517 Serial interface
  AGPIO(GPIO_WE517_RX),      // WE517 Serial interface
#endif
#endif  // USE_ENERGY_SENSOR

/*-------------------------------------------------------------------------------------------*\
 * Serial sensors
\*-------------------------------------------------------------------------------------------*/

#ifdef USE_SERIAL_BRIDGE
  AGPIO(GPIO_SBR_TX),         // Serial Bridge Serial interface
  AGPIO(GPIO_SBR_RX),         // Serial Bridge Serial interface
#endif
#ifdef USE_TCP_BRIDGE
  AGPIO(GPIO_TCP_TX),         // TCP Serial bridge
  AGPIO(GPIO_TCP_RX),         // TCP Serial bridge
#endif
#ifdef USE_ZIGBEE
  AGPIO(GPIO_ZIGBEE_TX),      // Zigbee Serial interface
  AGPIO(GPIO_ZIGBEE_RX),      // Zigbee Serial interface
  AGPIO(GPIO_ZIGBEE_RST),     // Zigbee reset
#endif
#ifdef USE_MHZ19
  AGPIO(GPIO_MHZ_TXD),        // MH-Z19 Serial interface
  AGPIO(GPIO_MHZ_RXD),        // MH-Z19 Serial interface
#endif
#ifdef USE_SENSEAIR
  AGPIO(GPIO_SAIR_TX),        // SenseAir Serial interface
  AGPIO(GPIO_SAIR_RX),        // SenseAir Serial interface
#endif
#ifdef USE_NOVA_SDS
  AGPIO(GPIO_SDS0X1_TX),      // Nova Fitness SDS011 Serial interface
  AGPIO(GPIO_SDS0X1_RX),      // Nova Fitness SDS011 Serial interface
#endif
#ifdef USE_HPMA
  AGPIO(GPIO_HPMA_TX),        // Honeywell HPMA115S0 Serial interface
  AGPIO(GPIO_HPMA_RX),        // Honeywell HPMA115S0 Serial interface
#endif
#ifdef USE_PMS5003
  AGPIO(GPIO_PMS5003_TX),     // Plantower PMS5003 Serial interface
  AGPIO(GPIO_PMS5003_RX),     // Plantower PMS5003 Serial interface
#endif
#if defined(USE_TX20_WIND_SENSOR) || defined(USE_TX23_WIND_SENSOR)
  AGPIO(GPIO_TX2X_TXD_BLACK), // TX20/TX23 Transmission Pin
#endif
#ifdef USE_WINDMETER
  AGPIO(GPIO_WINDMETER_SPEED),
#endif
#ifdef USE_MP3_PLAYER
  AGPIO(GPIO_MP3_DFR562),     // RB-DFR-562, DFPlayer Mini MP3 Player Serial interface
#endif
#ifdef USE_AZ7798
  AGPIO(GPIO_AZ_TXD),         // AZ-Instrument 7798 CO2 datalogger Serial interface
  AGPIO(GPIO_AZ_RXD),         // AZ-Instrument 7798 CO2 datalogger Serial interface
#endif
#ifdef USE_PN532_HSU
  AGPIO(GPIO_PN532_TXD),      // PN532 HSU Tx
  AGPIO(GPIO_PN532_RXD),      // PN532 HSU Rx
#endif
#ifdef USE_TASMOTA_CLIENT
  AGPIO(GPIO_TASMOTACLIENT_TXD),     // Tasmota Client TX
  AGPIO(GPIO_TASMOTACLIENT_RXD),     // Tasmota Client RX
  AGPIO(GPIO_TASMOTACLIENT_RST),     // Tasmota Client Reset
  AGPIO(GPIO_TASMOTACLIENT_RST_INV), // Tasmota Client Reset Inverted
#endif
#ifdef USE_RDM6300
  AGPIO(GPIO_RDM6300_RX),
#endif
#ifdef USE_IBEACON
  AGPIO(GPIO_IBEACON_TX),
  AGPIO(GPIO_IBEACON_RX),
#endif
#ifdef USE_GPS
  AGPIO(GPIO_GPS_TX),         // GPS serial interface
  AGPIO(GPIO_GPS_RX),         // GPS serial interface
#endif
#ifdef USE_HM10
  AGPIO(GPIO_HM10_TX),         // GPS serial interface
  AGPIO(GPIO_HM10_RX),         // GPS serial interface
#endif
#ifdef USE_OPENTHERM
  AGPIO(GPIO_BOILER_OT_TX),
  AGPIO(GPIO_BOILER_OT_RX),
#endif
#ifdef USE_AS608
  AGPIO(GPIO_AS608_TX),
  AGPIO(GPIO_AS608_RX),
#endif

/*-------------------------------------------------------------------------------------------*\
 * Other sensors
\*-------------------------------------------------------------------------------------------*/

#ifdef USE_MGC3130
  AGPIO(GPIO_MGC3130_XFER),
  AGPIO(GPIO_MGC3130_RESET),
#endif
#ifdef USE_MAX31855
  AGPIO(GPIO_MAX31855CS),     // MAX31855 Serial interface
  AGPIO(GPIO_MAX31855CLK),    // MAX31855 Serial interface
  AGPIO(GPIO_MAX31855DO),     // MAX31855 Serial interface
#endif
#ifdef USE_HRE
  AGPIO(GPIO_HRE_CLOCK),
  AGPIO(GPIO_HRE_DATA),
#endif
#ifdef USE_A4988_STEPPER
  AGPIO(GPIO_A4988_DIR),     // A4988 direction pin
  AGPIO(GPIO_A4988_STP),     // A4988 step pin
  // folowing are not mandatory
  AGPIO(GPIO_A4988_ENA),     // A4988 enabled pin
  AGPIO(GPIO_A4988_MS1) + MAX_A4988_MSS,  // A4988 microstep pin1 to pin3
#endif
#ifdef USE_DEEPSLEEP
  AGPIO(GPIO_DEEPSLEEP),
#endif
#ifdef USE_KEELOQ
  AGPIO(GPIO_CC1101_GDO0),    // CC1101 pin for RX
  AGPIO(GPIO_CC1101_GDO2),    // CC1101 pin for RX
#endif
#ifdef USE_HRXL
  AGPIO(GPIO_HRXL_RX),
#endif
#ifdef USE_DYP
  AGPIO(GPIO_DYP_RX),
#endif
#ifdef USE_AS3935
  AGPIO(GPIO_AS3935),          // AS3935 IRQ Pin
#endif
#ifdef USE_TELEINFO
  AGPIO(GPIO_TELEINFO_RX),
  AGPIO(GPIO_TELEINFO_ENABLE),
#endif
#ifdef USE_MIEL_HVAC
  AGPIO(GPIO_MIEL_HVAC_TX),    // Mitsubishi Electric HVAC TX pin
  AGPIO(GPIO_MIEL_HVAC_RX),    // Mitsubishi Electric HVAC RX pin
#endif
#ifdef USE_RC522
  AGPIO(GPIO_RC522_RST),       // RC522 Rfid reset
#endif

#ifdef USE_SOMFY
  AGPIO(GPIO_CC1101_GDO0),   // CC1101 pin for TX
#endif

/*-------------------------------------------------------------------------------------------*\
 * ESP32 specifics
\*-------------------------------------------------------------------------------------------*/

#ifdef ESP32
#ifdef USE_WEBCAM
  AGPIO(GPIO_WEBCAM_PWDN),
  AGPIO(GPIO_WEBCAM_RESET),
  AGPIO(GPIO_WEBCAM_XCLK),
  AGPIO(GPIO_WEBCAM_SIOD),
  AGPIO(GPIO_WEBCAM_SIOC),
  AGPIO(GPIO_WEBCAM_DATA) + MAX_WEBCAM_DATA,
  AGPIO(GPIO_WEBCAM_VSYNC),
  AGPIO(GPIO_WEBCAM_HREF),
  AGPIO(GPIO_WEBCAM_PCLK),
  AGPIO(GPIO_WEBCAM_PSCLK),
  AGPIO(GPIO_WEBCAM_HSD) + MAX_WEBCAM_HSD,
  AGPIO(GPIO_WEBCAM_PSRCS),
#endif  // USE_WEBCAM
#ifdef USE_ETHERNET
  AGPIO(GPIO_ETH_PHY_POWER),
  AGPIO(GPIO_ETH_PHY_MDC),
  AGPIO(GPIO_ETH_PHY_MDIO),  // Ethernet
#endif  // USE_ETHERNET

/*-------------------------------------------------------------------------------------------*\
 * ESP32 multiple Analog / Digital converter inputs
\*-------------------------------------------------------------------------------------------*/

  AGPIO(GPIO_ADC_INPUT) + MAX_ADCS,       // Analog inputs
  AGPIO(GPIO_ADC_TEMP) + MAX_ADCS,        // Thermistor
  AGPIO(GPIO_ADC_LIGHT) + MAX_ADCS,       // Light sensor
  AGPIO(GPIO_ADC_BUTTON) + MAX_KEYS,      // Button
  AGPIO(GPIO_ADC_BUTTON_INV) + MAX_KEYS,
  AGPIO(GPIO_ADC_RANGE) + MAX_ADCS,       // Range
  AGPIO(GPIO_ADC_CT_POWER) + MAX_ADCS,    // Current
  AGPIO(GPIO_ADC_JOY) + MAX_ADCS,         // Joystick
#endif  // ESP32
};

/*-------------------------------------------------------------------------------------------*\
 * ESP8266 single Analog / Digital converter input
\*-------------------------------------------------------------------------------------------*/

#ifdef ESP8266
const uint16_t kAdcNiceList[] PROGMEM = {
  GPIO_NONE,                              // Not used
  AGPIO(GPIO_ADC_INPUT),                  // Analog inputs
  AGPIO(GPIO_ADC_TEMP),                   // Thermistor
  AGPIO(GPIO_ADC_LIGHT),                  // Light sensor
  AGPIO(GPIO_ADC_BUTTON) + MAX_KEYS,      // Button
  AGPIO(GPIO_ADC_BUTTON_INV) + MAX_KEYS,
  AGPIO(GPIO_ADC_RANGE),                  // Range
  AGPIO(GPIO_ADC_CT_POWER),               // Current
  AGPIO(GPIO_ADC_JOY),                    // Joystick
};
#endif  // ESP8266

// User selectable ADC functionality
enum UserSelectableAdc {
  ADC_NONE,           // Not used
  ADC_INPUT,          // Analog input
  ADC_TEMP,           // Thermistor
  ADC_LIGHT,          // Light sensor
  ADC_BUTTON,         // Button
  ADC_BUTTON_INV,
  ADC_RANGE,          // Range
  ADC_CT_POWER,       // Current
  ADC_JOY,            // Joystick
//  ADC_SWITCH,         // Switch
//  ADC_SWITCH_INV,
  ADC_END };

/*********************************************************************************************\
 * ATTENTION: No user changeable features beyond this point - do not add templates !!!
\*********************************************************************************************/

#define GPIO_ANY           32   // Any GPIO

#ifdef ESP8266

#define MAX_GPI8_PIN       17   // Number of supported GPIO (0..16)
#define FLASH_PINS         6    // Number of flash chip pins

#define MAX_GPIO_PIN       18   // Number of supported GPIO (0..16 + ADC0)
#define ADC0_PIN           17   // Pin number of ADC0
#define MIN_FLASH_PINS     4    // Number of flash chip pins unusable for configuration (GPIO6, 7, 8 and 11)
#define MAX_USER_PINS      14   // MAX_GPIO_PIN - MIN_FLASH_PINS
#define WEMOS_MODULE       17   // Wemos module

const char PINS_WEMOS[] PROGMEM = "D3TXD4RXD2D1flashcFLFLolD6D7D5D8D0A0";

typedef struct MYIO8 {
  uint8_t      io[MAX_GPI8_PIN];
} myio8;                        // 17 bytes

typedef struct MYCFGIO8285 {
  uint8_t      io[MAX_GPI8_PIN - MIN_FLASH_PINS];
} mycfgio8285;                  // 13 bytes

typedef struct MYTMPLT8285 {
  mycfgio8285  gp;
  uint8_t      flag;
} mytmplt8285;                  // 14 bytes

typedef struct MYCFGIO8266 {
  uint8_t      io[MAX_GPI8_PIN - FLASH_PINS];
} mycfgio8266;                  // 11 bytes

typedef struct MYTMPLT8266 {
  mycfgio8266  gp;
  uint8_t      flag;
} mytmplt8266;                  // 12 bytes

#endif  // ESP8266
#ifdef ESP32

#define MAX_GPIO_PIN       40   // Number of supported GPIO
#define MIN_FLASH_PINS     4    // Number of flash chip pins unusable for configuration (GPIO6, 7, 8 and 11)
#define MAX_USER_PINS      36   // MAX_GPIO_PIN - MIN_FLASH_PINS
#define WEMOS_MODULE       0    // Wemos module

//                                  0 1 2 3 4 5 6 7 8 9101112131415161718192021222324252627282930313233343536373839
const char PINS_WEMOS[] PROGMEM = "IOTXIORXIOIOflashcFLFLolIOIOIOIOIOIOIOIOIOIOIOIOIOIOIOIOIOIOIOIOAOAOIAIAIAIAIAIA";

#endif  // ESP32

//********************************************************************************************

typedef struct MYIO {
  uint16_t      io[MAX_GPIO_PIN];
} myio;                         // 18 * 2 = 36 bytes / 40 * 2 = 80 bytes

typedef struct MYCFGIO {
  uint16_t      io[MAX_USER_PINS];
} mycfgio;                      // 14 * 2 = 28 bytes / 36 * 2 = 72 bytes

#define GPIO_FLAG_USED       0  // Currently no flags used

typedef union {
  uint16_t data;
  struct {
    uint16_t spare00 : 1;
    uint16_t spare01 : 1;
    uint16_t spare02 : 1;
    uint16_t spare03 : 1;
    uint16_t spare04 : 1;
    uint16_t spare05 : 1;
    uint16_t spare06 : 1;
    uint16_t spare07 : 1;
    uint16_t spare08 : 1;
    uint16_t spare09 : 1;
    uint16_t spare10 : 1;
    uint16_t spare11 : 1;
    uint16_t spare12 : 1;
    uint16_t spare13 : 1;
    uint16_t spare14 : 1;
    uint16_t spare15 : 1;
  };
} gpio_flag;                    // 2 bytes

typedef struct MYTMPLT {
  mycfgio      gp;              // 28 / 72 bytes
  gpio_flag    flag;            // 2 bytes
} mytmplt;                      // 30 / 74 bytes

//********************************************************************************************

#ifdef ESP8266

// User selectable GPIO functionality
// ATTENTION: No additions are supported
enum LegacyUserSelectablePins {
  GPI8_NONE,           // Not used
  GPI8_DHT11,          // DHT11
  GPI8_DHT22,          // DHT21, DHT22, AM2301, AM2302, AM2321
  GPI8_SI7021,         // iTead SI7021
  GPI8_DSB,            // Single wire DS18B20 or DS18S20
  GPI8_I2C_SCL,        // I2C SCL
  GPI8_I2C_SDA,        // I2C SDA
  GPI8_WS2812,         // WS2812 Led string
  GPI8_IRSEND,         // IR remote
  GPI8_SWT1,           // User connected external switches
  GPI8_SWT2,
  GPI8_SWT3,
  GPI8_SWT4,
  GPI8_SWT5,
  GPI8_SWT6,
  GPI8_SWT7,
  GPI8_SWT8,
  GPI8_KEY1,           // Button usually connected to GPIO0
  GPI8_KEY2,
  GPI8_KEY3,
  GPI8_KEY4,
  GPI8_REL1,           // Relays
  GPI8_REL2,
  GPI8_REL3,
  GPI8_REL4,
  GPI8_REL5,
  GPI8_REL6,
  GPI8_REL7,
  GPI8_REL8,
  GPI8_REL1_INV,
  GPI8_REL2_INV,
  GPI8_REL3_INV,
  GPI8_REL4_INV,
  GPI8_REL5_INV,
  GPI8_REL6_INV,
  GPI8_REL7_INV,
  GPI8_REL8_INV,
  GPI8_PWM1,           // RGB   Red   or C  Cold White
  GPI8_PWM2,           // RGB   Green or CW Warm White
  GPI8_PWM3,           // RGB   Blue
  GPI8_PWM4,           // RGBW  (Cold) White
  GPI8_PWM5,           // RGBCW Warm White
  GPI8_CNTR1,
  GPI8_CNTR2,
  GPI8_CNTR3,
  GPI8_CNTR4,
  GPI8_PWM1_INV,       // RGB   Red   or C  Cold White
  GPI8_PWM2_INV,       // RGB   Green or CW Warm White
  GPI8_PWM3_INV,       // RGB   Blue
  GPI8_PWM4_INV,       // RGBW  (Cold) White
  GPI8_PWM5_INV,       // RGBCW Warm White
  GPI8_IRRECV,         // IR receiver
  GPI8_LED1,           // Leds
  GPI8_LED2,
  GPI8_LED3,
  GPI8_LED4,
  GPI8_LED1_INV,
  GPI8_LED2_INV,
  GPI8_LED3_INV,
  GPI8_LED4_INV,
  GPI8_MHZ_TXD,        // MH-Z19 Serial interface
  GPI8_MHZ_RXD,        // MH-Z19 Serial interface
  GPI8_PZEM0XX_TX,     // PZEM0XX Serial interface
  GPI8_PZEM004_RX,     // PZEM004T Serial interface
  GPI8_SAIR_TX,        // SenseAir Serial interface
  GPI8_SAIR_RX,        // SenseAir Serial interface
  GPI8_SPI_CS,         // SPI Chip Select
  GPI8_SPI_DC,         // SPI Data Direction
  GPI8_BACKLIGHT,      // Display backlight control
  GPI8_PMS5003_RX,     // Plantower PMS5003 Serial interface
  GPI8_SDS0X1_RX,      // Nova Fitness SDS011 Serial interface
  GPI8_SBR_TX,         // Serial Bridge Serial interface
  GPI8_SBR_RX,         // Serial Bridge Serial interface
  GPI8_SR04_TRIG,      // SR04 Trigger/TX pin
  GPI8_SR04_ECHO,      // SR04 Echo/RX pin
  GPI8_SDM120_TX,      // SDM120 Serial interface
  GPI8_SDM120_RX,      // SDM120 Serial interface
  GPI8_SDM630_TX,      // SDM630 Serial interface
  GPI8_SDM630_RX,      // SDM630 Serial interface
  GPI8_TM16CLK,        // TM1638 Clock
  GPI8_TM16DIO,        // TM1638 Data I/O
  GPI8_TM16STB,        // TM1638 Strobe
  GPI8_SWT1_NP,        // User connected external switches
  GPI8_SWT2_NP,
  GPI8_SWT3_NP,
  GPI8_SWT4_NP,
  GPI8_SWT5_NP,
  GPI8_SWT6_NP,
  GPI8_SWT7_NP,
  GPI8_SWT8_NP,
  GPI8_KEY1_NP,        // Button usually connected to GPIO0
  GPI8_KEY2_NP,
  GPI8_KEY3_NP,
  GPI8_KEY4_NP,
  GPI8_CNTR1_NP,
  GPI8_CNTR2_NP,
  GPI8_CNTR3_NP,
  GPI8_CNTR4_NP,
  GPI8_PZEM016_RX,     // PZEM-014,016 Serial Modbus interface
  GPI8_PZEM017_RX,     // PZEM-003,017 Serial Modbus interface
  GPI8_MP3_DFR562,     // RB-DFR-562, DFPlayer Mini MP3 Player
  GPI8_SDS0X1_TX,      // Nova Fitness SDS011 Serial interface
  GPI8_HX711_SCK,      // HX711 Load Cell clock
  GPI8_HX711_DAT,      // HX711 Load Cell data
  GPI8_TX2X_TXD_BLACK, // TX20/TX23 Transmission Pin
  GPI8_RFSEND,         // RF transmitter
  GPI8_RFRECV,         // RF receiver
  GPI8_TUYA_TX,        // Tuya Serial interface
  GPI8_TUYA_RX,        // Tuya Serial interface
  GPI8_MGC3130_XFER,   // MGC3130 Transfer
  GPI8_MGC3130_RESET,  // MGC3130 Reset
  GPI8_SSPI_MISO,      // Software SPI Master Input Client Output
  GPI8_SSPI_MOSI,      // Software SPI Master Output Client Input
  GPI8_SSPI_SCLK,      // Software SPI Serial Clock
  GPI8_SSPI_CS,        // Software SPI Chip Select
  GPI8_SSPI_DC,        // Software SPI Data or Command
  GPI8_RF_SENSOR,      // Rf receiver with sensor decoding
  GPI8_AZ_TXD,         // AZ-Instrument 7798 Serial interface
  GPI8_AZ_RXD,         // AZ-Instrument 7798 Serial interface
  GPI8_MAX31855CS,     // MAX31855 Serial interface
  GPI8_MAX31855CLK,    // MAX31855 Serial interface
  GPI8_MAX31855DO,     // MAX31855 Serial interface
  GPI8_KEY1_INV,       // Inverted buttons
  GPI8_KEY2_INV,
  GPI8_KEY3_INV,
  GPI8_KEY4_INV,
  GPI8_KEY1_INV_NP,    // Inverted buttons without pull-up
  GPI8_KEY2_INV_NP,
  GPI8_KEY3_INV_NP,
  GPI8_KEY4_INV_NP,
  GPI8_NRG_SEL,        // HLW8012/HLJ-01 Sel output (1 = Voltage)
  GPI8_NRG_SEL_INV,    // HLW8012/HLJ-01 Sel output (0 = Voltage)
  GPI8_NRG_CF1,        // HLW8012/HLJ-01 CF1 voltage / current
  GPI8_HLW_CF,         // HLW8012 CF power
  GPI8_HJL_CF,         // HJL-01/BL0937 CF power
  GPI8_MCP39F5_TX,     // MCP39F501 Serial interface (Shelly2)
  GPI8_MCP39F5_RX,     // MCP39F501 Serial interface (Shelly2)
  GPI8_MCP39F5_RST,    // MCP39F501 Reset (Shelly2)
  GPI8_PN532_TXD,      // PN532 NFC Serial Tx
  GPI8_PN532_RXD,      // PN532 NFC Serial Rx
  GPI8_SM16716_CLK,    // SM16716 CLOCK
  GPI8_SM16716_DAT,    // SM16716 DATA
  GPI8_SM16716_SEL,    // SM16716 SELECT
  GPI8_DI,             // my92x1 PWM input
  GPI8_DCKI,           // my92x1 CLK input
  GPI8_CSE7766_TX,     // CSE7766 Serial interface (S31 and Pow R2) - Not used anymore 20200121
  GPI8_CSE7766_RX,     // CSE7766 Serial interface (S31 and Pow R2)
  GPI8_ARIRFRCV,       // AriLux RF Receive input
  GPI8_TXD,            // Serial interface
  GPI8_RXD,            // Serial interface
  GPI8_ROT1A,          // Rotary switch1 A Pin
  GPI8_ROT1B,          // Rotary switch1 B Pin
  GPI8_ROT2A,          // Rotary switch2 A Pin
  GPI8_ROT2B,          // Rotary switch2 B Pin
  GPI8_HRE_CLOCK,      // Clock/Power line for HR-E Water Meter
  GPI8_HRE_DATA,       // Data line for HR-E Water Meter
  GPI8_ADE7953_IRQ,    // ADE7953 IRQ
  GPI8_LEDLNK,         // Link led
  GPI8_LEDLNK_INV,     // Inverted link led
  GPI8_ARIRFSEL,       // Arilux RF Receive input selected
  GPI8_BUZZER,         // Buzzer
  GPI8_BUZZER_INV,     // Inverted buzzer
  GPI8_OLED_RESET,     // OLED Display Reset
  GPI8_SOLAXX1_TX,     // Solax Inverter tx pin
  GPI8_SOLAXX1_RX,     // Solax Inverter rx pin
  GPI8_ZIGBEE_TX,      // Zigbee Serial interface
  GPI8_ZIGBEE_RX,      // Zigbee Serial interface
  GPI8_RDM6300_RX,     // RDM6300 RX
  GPI8_IBEACON_TX,     // HM17 IBEACON TX
  GPI8_IBEACON_RX,     // HM17 IBEACON RX
  GPI8_A4988_DIR,      // A4988 direction pin
  GPI8_A4988_STP,      // A4988 step pin
  GPI8_A4988_ENA,      // A4988 enabled pin
  GPI8_A4988_MS1,      // A4988 microstep pin1
  GPI8_A4988_MS2,      // A4988 microstep pin2
  GPI8_A4988_MS3,      // A4988 microstep pin3
  GPI8_DDS2382_TX,     // DDS2382 Serial interface
  GPI8_DDS2382_RX,     // DDS2382 Serial interface
  GPI8_DDSU666_TX,     // DDSU666 Serial interface
  GPI8_DDSU666_RX,     // DDSU666 Serial interface
  GPI8_SM2135_CLK,     // SM2135 Clk
  GPI8_SM2135_DAT,     // SM2135 Dat
  GPI8_DEEPSLEEP,      // Kill switch for deepsleep
  GPI8_EXS_ENABLE,     // EXS MCU Enable
  GPI8_TASMOTACLIENT_TXD,     // Client TX
  GPI8_TASMOTACLIENT_RXD,     // Client RX
  GPI8_TASMOTACLIENT_RST,     // Client Reset Pin
  GPI8_TASMOTACLIENT_RST_INV, // Client Reset Inverted
  GPI8_HPMA_RX,        // Honeywell HPMA115S0 Serial interface
  GPI8_HPMA_TX,        // Honeywell HPMA115S0 Serial interface
  GPI8_GPS_RX,         // GPS serial interface
  GPI8_GPS_TX,         // GPS serial interface
  GPI8_DSB_OUT,        // Pseudo Single wire DS18B20 or DS18S20
  GPI8_DHT11_OUT,      // Pseudo Single wire DHT11, DHT21, DHT22, AM2301, AM2302, AM2321
  GPI8_HM10_RX,        // HM10-BLE-Mijia-bridge serial interface
  GPI8_HM10_TX,        // HM10-BLE-Mijia-bridge serial interface
  GPI8_LE01MR_RX,      // F&F LE-01MR energy meter
  GPI8_LE01MR_TX,      // F&F LE-01MR energy meter
  GPI8_CC1101_GDO0,    // CC1101 pin for RX
  GPI8_CC1101_GDO2,    // CC1101 pin for RX
  GPI8_HRXL_RX,       // Data from MaxBotix HRXL sonar range sensor
  GPI8_ELECTRIQ_MOODL_TX, // ElectriQ iQ-wifiMOODL Serial TX
  GPI8_AS3935,         // Franklin Lightning Sensor
  GPI8_PMS5003_TX,     // Plantower PMS5003 Serial interface
  GPI8_BOILER_OT_RX,   // OpenTherm Boiler RX pin
  GPI8_BOILER_OT_TX,   // OpenTherm Boiler TX pin
  GPI8_WINDMETER_SPEED,  // WindMeter speed counter pin
  GPI8_BL0940_RX,      // BL0940 serial interface
  GPI8_TCP_TX,         // TCP Serial bridge
  GPI8_TCP_RX,         // TCP Serial bridge
  GPI8_TELEINFO_RX,    // TELEINFO serial interface
  GPI8_TELEINFO_ENABLE,// TELEINFO Enable PIN
  GPI8_LMT01,          // LMT01 input counting pin
  GPI8_IEM3000_TX,     // IEM3000 Serial interface
  GPI8_IEM3000_RX,     // IEM3000 Serial interface
  GPI8_ZIGBEE_RST,     // Zigbee reset
  GPI8_DYP_RX,
  GPI8_SENSOR_END };

// Programmer selectable GPIO functionality
enum LegacyProgramSelectablePins {
  GPI8_FIX_START = 251,
  GPI8_SPI_MISO,       // SPI MISO library fixed pin GPIO12
  GPI8_SPI_MOSI,       // SPI MOSI library fixed pin GPIO13
  GPI8_SPI_CLK,        // SPI Clk library fixed pin GPIO14
  GPI8_USER,           // User configurable needs to be 255
  GPI8_MAX };

// Indexed by LegacyUserSelectablePins to convert legacy (8-bit) GPIOs
const uint16_t kGpioConvert[] PROGMEM = {
  GPIO_NONE,
  AGPIO(GPIO_DHT11),          // DHT11
  AGPIO(GPIO_DHT22),          // DHT21, DHT22, AM2301, AM2302, AM2321
  AGPIO(GPIO_SI7021),         // iTead SI7021
  AGPIO(GPIO_DSB),            // Single wire DS18B20 or DS18S20
  AGPIO(GPIO_I2C_SCL),        // I2C SCL
  AGPIO(GPIO_I2C_SDA),        // I2C SDA
  AGPIO(GPIO_WS2812),         // WS2812 Led string
  AGPIO(GPIO_IRSEND),         // IR remote
  AGPIO(GPIO_SWT1),           // Switch
  AGPIO(GPIO_SWT1) +1,
  AGPIO(GPIO_SWT1) +2,
  AGPIO(GPIO_SWT1) +3,
  AGPIO(GPIO_SWT1) +4,
  AGPIO(GPIO_SWT1) +5,
  AGPIO(GPIO_SWT1) +6,
  AGPIO(GPIO_SWT1) +7,
  AGPIO(GPIO_KEY1),           // Button
  AGPIO(GPIO_KEY1) +1,
  AGPIO(GPIO_KEY1) +2,
  AGPIO(GPIO_KEY1) +3,
  AGPIO(GPIO_REL1),           // Relay
  AGPIO(GPIO_REL1) +1,
  AGPIO(GPIO_REL1) +2,
  AGPIO(GPIO_REL1) +3,
  AGPIO(GPIO_REL1) +4,
  AGPIO(GPIO_REL1) +5,
  AGPIO(GPIO_REL1) +6,
  AGPIO(GPIO_REL1) +7,
  AGPIO(GPIO_REL1_INV),       // Relay inverted
  AGPIO(GPIO_REL1_INV) +1,
  AGPIO(GPIO_REL1_INV) +2,
  AGPIO(GPIO_REL1_INV) +3,
  AGPIO(GPIO_REL1_INV) +4,
  AGPIO(GPIO_REL1_INV) +5,
  AGPIO(GPIO_REL1_INV) +6,
  AGPIO(GPIO_REL1_INV) +7,
  AGPIO(GPIO_PWM1),           // PWM
  AGPIO(GPIO_PWM1) +1,
  AGPIO(GPIO_PWM1) +2,
  AGPIO(GPIO_PWM1) +3,
  AGPIO(GPIO_PWM1) +4,
  AGPIO(GPIO_CNTR1),          // Counter
  AGPIO(GPIO_CNTR1) +1,
  AGPIO(GPIO_CNTR1) +2,
  AGPIO(GPIO_CNTR1) +3,
  AGPIO(GPIO_PWM1_INV),       // PWM inverted
  AGPIO(GPIO_PWM1_INV) +1,
  AGPIO(GPIO_PWM1_INV) +2,
  AGPIO(GPIO_PWM1_INV) +3,
  AGPIO(GPIO_PWM1_INV) +4,
  AGPIO(GPIO_IRRECV),         // IR receive
  AGPIO(GPIO_LED1),           // Led
  AGPIO(GPIO_LED1) +1,
  AGPIO(GPIO_LED1) +2,
  AGPIO(GPIO_LED1) +3,
  AGPIO(GPIO_LED1_INV),       // Led inverted
  AGPIO(GPIO_LED1_INV) +1,
  AGPIO(GPIO_LED1_INV) +2,
  AGPIO(GPIO_LED1_INV) +3,
  AGPIO(GPIO_MHZ_TXD),        // MH-Z19 Serial interface
  AGPIO(GPIO_MHZ_RXD),
  AGPIO(GPIO_PZEM0XX_TX),     // PZEM0XX Serial interface
  AGPIO(GPIO_PZEM004_RX),     // PZEM004T Serial interface
  AGPIO(GPIO_SAIR_TX),        // SenseAir Serial interface
  AGPIO(GPIO_SAIR_RX),        // SenseAir Serial interface
  AGPIO(GPIO_SPI_CS),         // SPI Chip Select
  AGPIO(GPIO_SPI_DC),         // SPI Data Direction
  AGPIO(GPIO_BACKLIGHT),      // Display backlight control
  AGPIO(GPIO_PMS5003_RX),     // Plantower PMS5003 Serial interface
  AGPIO(GPIO_SDS0X1_RX),      // Nova Fitness SDS011 Serial interface
  AGPIO(GPIO_SBR_TX),         // Serial Bridge Serial interface
  AGPIO(GPIO_SBR_RX),         // Serial Bridge Serial interface
  AGPIO(GPIO_SR04_TRIG),      // SR04 Tri/TXgger pin
  AGPIO(GPIO_SR04_ECHO),      // SR04 Ech/RXo pin
  AGPIO(GPIO_SDM120_TX),      // SDM120 Serial interface
  AGPIO(GPIO_SDM120_RX),      // SDM120 Serial interface
  AGPIO(GPIO_SDM630_TX),      // SDM630 Serial interface
  AGPIO(GPIO_SDM630_RX),      // SDM630 Serial interface
  AGPIO(GPIO_TM16CLK),        // TM1638 Clock
  AGPIO(GPIO_TM16DIO),        // TM1638 Data I/O
  AGPIO(GPIO_TM16STB),        // TM1638 Strobe
  AGPIO(GPIO_SWT1_NP),        // Switch no pullup
  AGPIO(GPIO_SWT1_NP) +1,
  AGPIO(GPIO_SWT1_NP) +2,
  AGPIO(GPIO_SWT1_NP) +3,
  AGPIO(GPIO_SWT1_NP) +4,
  AGPIO(GPIO_SWT1_NP) +5,
  AGPIO(GPIO_SWT1_NP) +6,
  AGPIO(GPIO_SWT1_NP) +7,
  AGPIO(GPIO_KEY1_NP),        // Button no pullup
  AGPIO(GPIO_KEY1_NP) +1,
  AGPIO(GPIO_KEY1_NP) +2,
  AGPIO(GPIO_KEY1_NP) +3,
  AGPIO(GPIO_CNTR1_NP),       // Counter no pullup
  AGPIO(GPIO_CNTR1_NP) +1,
  AGPIO(GPIO_CNTR1_NP) +2,
  AGPIO(GPIO_CNTR1_NP) +3,
  AGPIO(GPIO_PZEM016_RX),     // PZEM-014,016 Serial Modbus interface
  AGPIO(GPIO_PZEM017_RX),     // PZEM-003,017 Serial Modbus interface
  AGPIO(GPIO_MP3_DFR562),     // RB-DFR-562, DFPlayer Mini MP3 Player Serial interface
  AGPIO(GPIO_SDS0X1_TX),      // Nova Fitness SDS011 Serial interface
  AGPIO(GPIO_HX711_SCK),      // HX711 Load Cell clock
  AGPIO(GPIO_HX711_DAT),      // HX711 Load Cell data
  AGPIO(GPIO_TX2X_TXD_BLACK), // TX20/TX23 Transmission Pin
  AGPIO(GPIO_RFSEND),         // RF transmitter
  AGPIO(GPIO_RFRECV),         // RF receiver
  AGPIO(GPIO_TUYA_TX),        // Tuya Serial interface
  AGPIO(GPIO_TUYA_RX),        // Tuya Serial interface
  AGPIO(GPIO_MGC3130_XFER),
  AGPIO(GPIO_MGC3130_RESET),
  AGPIO(GPIO_SSPI_MISO),      // Software SPI Master Input Client Output
  AGPIO(GPIO_SSPI_MOSI),      // Software SPI Master Output Client Input
  AGPIO(GPIO_SSPI_SCLK),      // Software SPI Serial Clock
  AGPIO(GPIO_SSPI_CS),        // Software SPI Chip Select
  AGPIO(GPIO_SSPI_DC),        // Software SPI Data or Command
  AGPIO(GPIO_RF_SENSOR),      // Rf receiver with sensor decoding
  AGPIO(GPIO_AZ_TXD),         // AZ-Instrument 7798 CO2 datalogger Serial interface
  AGPIO(GPIO_AZ_RXD),         // AZ-Instrument 7798 CO2 datalogger Serial interface
  AGPIO(GPIO_MAX31855CS),     // MAX31855 Serial interface
  AGPIO(GPIO_MAX31855CLK),    // MAX31855 Serial interface
  AGPIO(GPIO_MAX31855DO),     // MAX31855 Serial interface
  AGPIO(GPIO_KEY1_INV),       // Button inverted
  AGPIO(GPIO_KEY1_INV) +1,
  AGPIO(GPIO_KEY1_INV) +2,
  AGPIO(GPIO_KEY1_INV) +3,
  AGPIO(GPIO_KEY1_INV_NP),    // Button inverted no pullup
  AGPIO(GPIO_KEY1_INV_NP) +1,
  AGPIO(GPIO_KEY1_INV_NP) +2,
  AGPIO(GPIO_KEY1_INV_NP) +3,
  AGPIO(GPIO_NRG_SEL),        // HLW8012/HLJ-01 Sel output (1 = Voltage)
  AGPIO(GPIO_NRG_SEL_INV),    // HLW8012/HLJ-01 Sel output (0 = Voltage)
  AGPIO(GPIO_NRG_CF1),        // HLW8012/HLJ-01 CF1 voltage / current
  AGPIO(GPIO_HLW_CF),         // HLW8012 CF power
  AGPIO(GPIO_HJL_CF),         // HJL-01/BL0937 CF power
  AGPIO(GPIO_MCP39F5_TX),     // MCP39F501 Serial interface (Shelly2)
  AGPIO(GPIO_MCP39F5_RX),     // MCP39F501 Serial interface (Shelly2)
  AGPIO(GPIO_MCP39F5_RST),    // MCP39F501 Reset (Shelly2)
  AGPIO(GPIO_PN532_TXD),      // PN532 HSU Tx
  AGPIO(GPIO_PN532_RXD),      // PN532 HSU Rx
  AGPIO(GPIO_SM16716_CLK),    // SM16716 CLOCK
  AGPIO(GPIO_SM16716_DAT),    // SM16716 DATA
  AGPIO(GPIO_SM16716_SEL),    // SM16716 SELECT
  AGPIO(GPIO_DI),             // my92x1 PWM input
  AGPIO(GPIO_DCKI),           // my92x1 CLK input
  AGPIO(GPIO_CSE7766_TX),     // CSE7766 Serial interface (S31 and Pow R2)
  AGPIO(GPIO_CSE7766_RX),     // CSE7766 Serial interface (S31 and Pow R2)
  AGPIO(GPIO_ARIRFRCV),       // AriLux RF Receive input
  AGPIO(GPIO_TXD),            // Serial interface
  AGPIO(GPIO_RXD),            // Serial interface
  AGPIO(GPIO_ROT1A),          // Rotary A Pin
  AGPIO(GPIO_ROT1B),          // Rotary B Pin
  AGPIO(GPIO_ROT1A) +1,       // Rotary A Pin
  AGPIO(GPIO_ROT1B) +1,       // Rotary B Pin
  AGPIO(GPIO_HRE_CLOCK),
  AGPIO(GPIO_HRE_DATA),
  AGPIO(GPIO_ADE7953_IRQ),    // ADE7953 IRQ
  AGPIO(GPIO_LEDLNK),         // Link led
  AGPIO(GPIO_LEDLNK_INV),     // Inverted link led
  AGPIO(GPIO_ARIRFSEL),       // Arilux RF Receive input selected
  AGPIO(GPIO_BUZZER),         // Buzzer
  AGPIO(GPIO_BUZZER_INV),     // Inverted buzzer
  AGPIO(GPIO_OLED_RESET),     // OLED Display Reset
  AGPIO(GPIO_SOLAXX1_TX),     // Solax Inverter tx pin
  AGPIO(GPIO_SOLAXX1_RX),     // Solax Inverter rx pin
  AGPIO(GPIO_ZIGBEE_TX),      // Zigbee Serial interface
  AGPIO(GPIO_ZIGBEE_RX),      // Zigbee Serial interface
  AGPIO(GPIO_RDM6300_RX),
  AGPIO(GPIO_IBEACON_TX),
  AGPIO(GPIO_IBEACON_RX),
  AGPIO(GPIO_A4988_DIR),      // A4988 direction pin
  AGPIO(GPIO_A4988_STP),      // A4988 step pin
  AGPIO(GPIO_A4988_ENA),      // A4988 enabled pin
  AGPIO(GPIO_A4988_MS1),      // A4988 microstep pin1
  AGPIO(GPIO_A4988_MS1) +1,   // A4988 microstep pin2
  AGPIO(GPIO_A4988_MS1) +2,   // A4988 microstep pin3
  AGPIO(GPIO_DDS2382_TX),     // DDS2382 Serial interface
  AGPIO(GPIO_DDS2382_RX),     // DDS2382 Serial interface
  AGPIO(GPIO_DDSU666_TX),     // DDSU666 Serial interface
  AGPIO(GPIO_DDSU666_RX),     // DDSU666 Serial interface
  AGPIO(GPIO_SM2135_CLK),     // SM2135 CLOCK
  AGPIO(GPIO_SM2135_DAT),     // SM2135 DATA
  AGPIO(GPIO_DEEPSLEEP),
  AGPIO(GPIO_EXS_ENABLE),     // EXS MCU Enable
  AGPIO(GPIO_TASMOTACLIENT_TXD),     // Tasmota Client TX
  AGPIO(GPIO_TASMOTACLIENT_RXD),     // Tasmota Client RX
  AGPIO(GPIO_TASMOTACLIENT_RST),     // Tasmota Client Reset
  AGPIO(GPIO_TASMOTACLIENT_RST_INV), // Tasmota Client Reset Inverted
  AGPIO(GPIO_HPMA_RX),        // Honeywell HPMA115S0 Serial interface
  AGPIO(GPIO_HPMA_TX),        // Honeywell HPMA115S0 Serial interface
  AGPIO(GPIO_GPS_RX),         // GPS serial interface
  AGPIO(GPIO_GPS_TX),         // GPS serial interface
  AGPIO(GPIO_DSB_OUT),        // Pseudo Single wire DS18B20 or DS18S20
  AGPIO(GPIO_DHT11_OUT),      // Pseudo Single wire DHT11, DHT21, DHT22, AM2301, AM2302, AM2321
  AGPIO(GPIO_HM10_RX),        // GPS serial interface
  AGPIO(GPIO_HM10_TX),        // GPS serial interface
  AGPIO(GPIO_LE01MR_RX),      // F7F LE-01MR energy meter rx pin
  AGPIO(GPIO_LE01MR_TX),      // F7F LE-01MR energy meter tx pin
  AGPIO(GPIO_CC1101_GDO0),    // CC1101 pin for RX
  AGPIO(GPIO_CC1101_GDO2),    // CC1101 pin for RX
  AGPIO(GPIO_HRXL_RX),
  AGPIO(GPIO_ELECTRIQ_MOODL_TX),
  AGPIO(GPIO_AS3935),         // AS3935 IRQ Pin
  AGPIO(GPIO_PMS5003_TX),     // Plantower PMS5003 Serial interface
  AGPIO(GPIO_BOILER_OT_RX),
  AGPIO(GPIO_BOILER_OT_TX),
  AGPIO(GPIO_WINDMETER_SPEED),
  AGPIO(GPIO_BL0940_RX),      // BL0940 Serial interface
  AGPIO(GPIO_TCP_TX),         // TCP Serial bridge
  AGPIO(GPIO_TCP_RX),         // TCP Serial bridge
  AGPIO(GPIO_TELEINFO_RX),
  AGPIO(GPIO_TELEINFO_ENABLE),
  AGPIO(GPIO_LMT01),          // LMT01, count pulses on GPIO
  AGPIO(GPIO_IEM3000_TX),     // IEM3000 Serial interface
  AGPIO(GPIO_IEM3000_RX),     // IEM3000 Serial interface
  AGPIO(GPIO_ZIGBEE_RST),     // Zigbee reset
  AGPIO(GPIO_DYP_RX),
};

/********************************************************************************************/
// Supported hardware modules

enum SupportedModules {
  SONOFF_BASIC, SONOFF_RF, SONOFF_SV, SONOFF_TH, SONOFF_DUAL, SONOFF_POW, SONOFF_4CH, SONOFF_S2X, SLAMPHER, SONOFF_TOUCH,
  SONOFF_LED, CH1, CH4, MOTOR, ELECTRODRAGON, EXS_RELAY, WION, WEMOS, SONOFF_DEV, H801,
  SONOFF_SC, SONOFF_BN, SONOFF_4CHPRO, HUAFAN_SS, SONOFF_BRIDGE, SONOFF_B1, AILIGHT, SONOFF_T11, SONOFF_T12, SONOFF_T13,
  SUPLA1, WITTY, YUNSHAN, MAGICHOME, LUANIHVIO, KMC_70011, ARILUX_LC01, ARILUX_LC11, SONOFF_DUAL_R2, ARILUX_LC06,
  SONOFF_S31, ZENGGE_ZF_WF017, SONOFF_POW_R2, SONOFF_IFAN02, BLITZWOLF_BWSHP, SHELLY1, SHELLY2, PHILIPS, NEO_COOLCAM, ESP_SWITCH,
  OBI, TECKIN, APLIC_WDP303075, TUYA_DIMMER, GOSUND, ARMTRONIX_DIMMERS, SK03_TUYA, PS_16_DZ, TECKIN_US, MANZOKU_EU_4,
  OBI2, YTF_IR_BRIDGE, DIGOO, KA10, ZX2820, MI_DESK_LAMP, SP10, WAGA, SYF05, SONOFF_L1,
  SONOFF_IFAN03, EXS_DIMMER, PWM_DIMMER, SONOFF_D1, SONOFF_ZB_BRIDGE,
  MAXMODULE };

#define USER_MODULE        255

const char kModuleNames[] PROGMEM =
  "Sonoff Basic|Sonoff RF|Sonoff SV|Sonoff TH|Sonoff Dual|Sonoff Pow|Sonoff 4CH|Sonoff S2X|Slampher|Sonoff Touch|"
  "Sonoff LED|1 Channel|4 Channel|Motor C/AC|ElectroDragon|EXS Relay(s)|WiOn|Generic|Sonoff Dev|H801|"
  "Sonoff SC|Sonoff BN-SZ|Sonoff 4CH Pro|Huafan SS|Sonoff Bridge|Sonoff B1|AiLight|Sonoff T1 1CH|Sonoff T1 2CH|Sonoff T1 3CH|"
  "Supla Espablo|Witty Cloud|Yunshan Relay|MagicHome|Luani HVIO|KMC 70011|Arilux LC01|Arilux LC11|Sonoff Dual R2|Arilux LC06|"
  "Sonoff S31|Zengge WF017|Sonoff Pow R2|Sonoff iFan02|BlitzWolf SHP|Shelly 1|Shelly 2|Xiaomi Philips|Neo Coolcam|ESP Switch|"
  "OBI Socket|Teckin|AplicWDP303075|Tuya MCU|Gosund SP1 v23|ARMTR Dimmer|SK03 Outdoor|PS-16-DZ|Teckin US|Manzoku strip|"
  "OBI Socket 2|YTF IR Bridge|Digoo DG-SP202|KA10|Luminea ZX2820|Mi Desk Lamp|SP10|WAGA CHCZ02MB|SYF05|Sonoff L1|"
  "Sonoff iFan03|EXS Dimmer|PWM Dimmer|Sonoff D1|Sonoff ZbBridge"
  ;

const uint8_t kModuleNiceList[] PROGMEM = {
  SONOFF_BASIC,        // Sonoff Relay Devices
  SONOFF_RF,
  SONOFF_TH,
  SONOFF_DUAL,
  SONOFF_DUAL_R2,
  SONOFF_POW,
  SONOFF_POW_R2,
  SONOFF_4CH,
  SONOFF_4CHPRO,
  SONOFF_S31,          // Sonoff Socket Relay Devices with Energy Monitoring
  SONOFF_S2X,          // Sonoff Socket Relay Devices
  SONOFF_TOUCH,        // Sonoff Switch Devices
  SONOFF_T11,
  SONOFF_T12,
  SONOFF_T13,
#ifdef USE_SONOFF_D1
  SONOFF_D1,           // Sonoff D1
#endif
  SONOFF_LED,          // Sonoff Light Devices
  SONOFF_BN,
#ifdef USE_SONOFF_L1
  SONOFF_L1,
#endif
  SONOFF_B1,           // Sonoff Light Bulbs
  SLAMPHER,
#ifdef USE_SONOFF_SC
  SONOFF_SC,           // Sonoff Environmemtal Sensor
#endif
#ifdef USE_SONOFF_IFAN
  SONOFF_IFAN02,       // Sonoff Fan
  SONOFF_IFAN03,
#endif
#ifdef USE_SONOFF_RF
  SONOFF_BRIDGE,       // Sonoff Bridge
#endif
#ifdef USE_ZIGBEE_EZSP
  SONOFF_ZB_BRIDGE,
#endif
  SONOFF_SV,           // Sonoff Development Devices
  SONOFF_DEV,
  CH1,                 // Relay Devices
  CH4,
  MOTOR,
  ELECTRODRAGON,
  EXS_RELAY,
  SUPLA1,
  LUANIHVIO,
  YUNSHAN,
  WION,
  SHELLY1,
  SHELLY2,
  BLITZWOLF_BWSHP,     // Socket Relay Devices with Energy Monitoring
  TECKIN,
  TECKIN_US,
  APLIC_WDP303075,
  GOSUND,
  ZX2820,
  SK03_TUYA,
  DIGOO,
  KA10,
  SP10,
  WAGA,
  NEO_COOLCAM,         // Socket Relay Devices
  OBI,
  OBI2,
  MANZOKU_EU_4,
  ESP_SWITCH,          // Switch Devices
#ifdef USE_TUYA_MCU
  TUYA_DIMMER,         // Dimmer Devices
#endif
#ifdef USE_ARMTRONIX_DIMMERS
  ARMTRONIX_DIMMERS,
#endif
#ifdef USE_PS_16_DZ
  PS_16_DZ,
#endif
#ifdef USE_EXS_DIMMER
  EXS_DIMMER,
#endif
#ifdef USE_PWM_DIMMER
  PWM_DIMMER,
#endif
  H801,                // Light Devices
  MAGICHOME,
  ARILUX_LC01,
  ARILUX_LC06,
  ARILUX_LC11,
  ZENGGE_ZF_WF017,
  HUAFAN_SS,
#ifdef ROTARY_V1
  MI_DESK_LAMP,
#endif
  KMC_70011,
  AILIGHT,             // Light Bulbs
  PHILIPS,
  SYF05,
  YTF_IR_BRIDGE,
  WITTY,               // Development Devices
  WEMOS
};

enum SupportedTemplates8285 {
  TMP_SONOFF_BASIC, TMP_SONOFF_SV, TMP_SONOFF_DUAL, TMP_SONOFF_POW, TMP_SONOFF_LED, TMP_ELECTRODRAGON,
  TMP_EXS_RELAY, TMP_WION, TMP_SONOFF_DEV, TMP_H801, TMP_SONOFF_SC, TMP_SONOFF_BN, TMP_HUAFAN_SS, TMP_SONOFF_BRIDGE,
  TMP_SONOFF_B1, TMP_AILIGHT, TMP_SONOFF_T11, TMP_SUPLA1, TMP_WITTY, TMP_YUNSHAN, TMP_MAGICHOME,
  TMP_LUANIHVIO, TMP_KMC_70011, TMP_ARILUX_LC01, TMP_ARILUX_LC11, TMP_ARILUX_LC06, TMP_ZENGGE_ZF_WF017,
  TMP_SONOFF_POW_R2, TMP_BLITZWOLF_BWSHP, TMP_SHELLY1, TMP_SHELLY2, TMP_PHILIPS, TMP_NEO_COOLCAM, TMP_ESP_SWITCH, TMP_OBI,
  TMP_TECKIN, TMP_APLIC_WDP303075, TMP_TUYA_DIMMER, TMP_GOSUND, TMP_ARMTRONIX_DIMMERS, TMP_SK03_TUYA, TMP_PS_16_DZ,
  TMP_TECKIN_US, TMP_MANZOKU_EU_4, TMP_OBI2, TMP_YTF_IR_BRIDGE, TMP_DIGOO, TMP_KA10, TMP_ZX2820, TMP_MI_DESK_LAMP, TMP_SP10,
  TMP_WAGA, TMP_SYF05, TMP_EXS_DIMMER, TMP_PWM_DIMMER, TMP_SONOFF_ZB_BRIDGE,
  TMP_MAXMODULE_8285 };

enum SupportedTemplates8266 {
  TMP_WEMOS = TMP_MAXMODULE_8285, TMP_SONOFF_4CH, TMP_SONOFF_T12, TMP_SONOFF_T13, TMP_SONOFF_DUAL_R2, TMP_SONOFF_IFAN03,
  TMP_MAXMODULE_8266 };

const uint8_t kModuleTemplateList[MAXMODULE] PROGMEM = {
  TMP_SONOFF_BASIC,
  TMP_SONOFF_BASIC,     // SONOFF_RF
  TMP_SONOFF_SV,
  TMP_SONOFF_BASIC,     // SONOFF_TH
  TMP_SONOFF_DUAL,
  TMP_SONOFF_POW,
  TMP_SONOFF_4CH,
  TMP_SONOFF_BASIC,     // SONOFF_S2X
  TMP_SONOFF_BASIC,     // SLAMPHER
  TMP_SONOFF_T11,       // SONOFF_TOUCH
  TMP_SONOFF_LED,
  TMP_SONOFF_BASIC,     // CH1
  TMP_SONOFF_DUAL,      // CH4
  TMP_SONOFF_BASIC,     // MOTOR
  TMP_ELECTRODRAGON,
  TMP_EXS_RELAY,
  TMP_WION,
  TMP_WEMOS,
  TMP_SONOFF_DEV,
  TMP_H801,
  TMP_SONOFF_SC,
  TMP_SONOFF_BN,
  TMP_SONOFF_4CH,       // SONOFF_4CHPRO
  TMP_HUAFAN_SS,
  TMP_SONOFF_BRIDGE,
  TMP_SONOFF_B1,
  TMP_AILIGHT,
  TMP_SONOFF_T11,
  TMP_SONOFF_T12,
  TMP_SONOFF_T13,
  TMP_SUPLA1,
  TMP_WITTY,
  TMP_YUNSHAN,
  TMP_MAGICHOME,
  TMP_LUANIHVIO,
  TMP_KMC_70011,
  TMP_ARILUX_LC01,
  TMP_ARILUX_LC11,
  TMP_SONOFF_DUAL_R2,
  TMP_ARILUX_LC06,
  TMP_SONOFF_POW_R2,    // SONOFF_S31
  TMP_ZENGGE_ZF_WF017,
  TMP_SONOFF_POW_R2,
  TMP_SONOFF_4CH,       // SONOFF_IFAN02
  TMP_BLITZWOLF_BWSHP,
  TMP_SHELLY1,
  TMP_SHELLY2,
  TMP_PHILIPS,
  TMP_NEO_COOLCAM,
  TMP_ESP_SWITCH,
  TMP_OBI,
  TMP_TECKIN,
  TMP_APLIC_WDP303075,
  TMP_TUYA_DIMMER,
  TMP_GOSUND,
  TMP_ARMTRONIX_DIMMERS,
  TMP_SK03_TUYA,
  TMP_PS_16_DZ,
  TMP_TECKIN_US,
  TMP_MANZOKU_EU_4,
  TMP_OBI2,
  TMP_YTF_IR_BRIDGE,
  TMP_DIGOO,
  TMP_KA10,
  TMP_ZX2820,
  TMP_MI_DESK_LAMP,
  TMP_SP10,
  TMP_WAGA,
  TMP_SYF05,
  TMP_SONOFF_DUAL,      // SONOFF_L1
  TMP_SONOFF_IFAN03,
  TMP_EXS_DIMMER,
  TMP_PWM_DIMMER,
  TMP_SONOFF_DUAL,      // SONOFF_D1
  TMP_SONOFF_ZB_BRIDGE,
  };

/*********************************************************************************************\
 * Templates with 12 usable pins (ESP8266)
\*********************************************************************************************/

const mytmplt8266 kModules8266[TMP_MAXMODULE_8285] PROGMEM = {
  {                     // SONOFF_BASIC - Sonoff Basic (ESP8266)
    GPI8_KEY1,          // GPIO00 Button
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Only available on newer Sonoff Basic R2 V1
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_USER,          // GPIO04 Optional sensor
    0,                  // GPIO05
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Red Led and Relay (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Green Led (0 = On, 1 = Off) - Link and Power status
    GPI8_USER,          // GPIO14 Optional sensor
    0,                  // GPIO15
    0,                  // GPIO16
    0                   // ADC0 Analog input
  },
  {                     // SONOFF_SV - Sonoff SV (ESP8266)
    GPI8_KEY1,          // GPIO00 Button
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    0,
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_USER,          // GPIO04 Optional sensor
    GPI8_USER,          // GPIO05 Optional sensor
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Red Led and Relay (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Green Led (0 = On, 1 = Off) - Link and Power status
    GPI8_USER,          // GPIO14 Optional sensor
    0, 0,
    GPI8_USER           // ADC0 Analog input
  },
  {                     // SONOFF_DUAL - Sonoff Dual (ESP8266)
    GPI8_USER,          // GPIO00 Pad
    GPI8_TXD,           // GPIO01 Relay control
    0,
    GPI8_RXD,           // GPIO03 Relay control
    GPI8_USER,          // GPIO04 Optional sensor
    0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    0,
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    GPI8_USER,          // GPIO14 Optional sensor
    0, 0, 0
  },
  {                     // SONOFF_POW - Sonoff Pow (ESP8266 - HLW8012)
    GPI8_KEY1,          // GPIO00 Button
    0, 0, 0, 0,
    GPI8_NRG_SEL,       // GPIO05 HLW8012 Sel output (1 = Voltage)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Red Led and Relay (0 = Off, 1 = On)
    GPI8_NRG_CF1,       // GPIO13 HLW8012 CF1 voltage / current
    GPI8_HLW_CF,        // GPIO14 HLW8012 CF power
    GPI8_LED1,          // GPIO15 Blue Led (0 = On, 1 = Off) - Link and Power status
    0, 0
  },
  {                     // SONOFF_LED - Sonoff LED (ESP8266)
    GPI8_KEY1,          // GPIO00 Button
    0, 0, 0,
    GPI8_USER,          // GPIO04 Optional sensor (PWM3 Green)
    GPI8_USER,          // GPIO05 Optional sensor (PWM2 Red)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM1,          // GPIO12 Cold light (PWM0 Cold)
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    GPI8_PWM2,          // GPIO14 Warm light (PWM1 Warm)
    GPI8_USER,          // GPIO15 Optional sensor (PWM4 Blue)
    0, 0
  },
  {                     // ELECTRODRAGON - ElectroDragon IoT Relay Board (ESP8266)
    GPI8_KEY2,          // GPIO00 Button 2
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_KEY1,          // GPIO02 Button 1
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_USER,          // GPIO04 Optional sensor
    GPI8_USER,          // GPIO05 Optional sensor
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL2,          // GPIO12 Red Led and Relay 2 (0 = Off, 1 = On)
    GPI8_REL1,          // GPIO13 Red Led and Relay 1 (0 = Off, 1 = On)
    GPI8_USER,          // GPIO14 Optional sensor
    GPI8_USER,          // GPIO15 Optional sensor
    GPI8_LED1,          // GPIO16 Green/Blue Led (1 = On, 0 = Off) - Link and Power status
    GPI8_USER           // ADC0   A0 Analog input
  },
  {                     // EXS_RELAY - ES-Store Latching relay(s) (ESP8266)
                        // https://ex-store.de/ESP8266-WiFi-Relay-V31
                        //   V3.1 Module Pin 1 VCC 3V3, Module Pin 6 GND
                        // https://ex-store.de/2-Kanal-WiFi-WLan-Relay-V5-Blackline-fuer-Unterputzmontage
    GPI8_USER,          // GPIO00 V3.1 Module Pin 8 - V5.0 Module Pin 4
    GPI8_USER,          // GPIO01 UART0_TXD V3.1 Module Pin 2 - V5.0 Module Pin 3
    GPI8_USER,          // GPIO02 V3.1 Module Pin 7
    GPI8_USER,          // GPIO03 UART0_RXD V3.1 Module Pin 3
    GPI8_USER,          // GPIO04 V3.1 Module Pin 10 - V5.0 Module Pin 2
    GPI8_USER,          // GPIO05 V3.1 Module Pin 9 - V5.0 Module Pin 1
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Relay1 ( 1 = Off)
    GPI8_REL2,          // GPIO13 Relay1 ( 1 = On)
    GPI8_USER,          // GPIO14 V3.1 Module Pin 5 - V5.0 GPI8_REL3_INV Relay2 ( 1 = Off)
    GPI8_LED1,          // GPIO15 V5.0 LED1 - Link and Power status
    GPI8_USER,          // GPIO16 V3.1 Module Pin 4 - V5.0 GPI8_REL4_INV Relay2 ( 1 = On)
    0
  },
  {                     // WION - Indoor Tap (ESP8266)
                        // https://www.amazon.com/gp/product/B00ZYLUBJU/ref=s9_acsd_al_bw_c_x_3_w
    GPI8_USER,          // GPIO00 Optional sensor (pm clock)
    0,
    GPI8_LED1,          // GPIO02 Green Led (1 = On, 0 = Off) - Link and Power status
    0, 0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12 Optional sensor (pm data)
    GPI8_KEY1,          // GPIO13 Button
    0,
    GPI8_REL1,          // GPIO15 Relay (0 = Off, 1 = On)
    0, 0
  },
  {                     // SONOFF_DEV - Sonoff Dev (ESP8266)
    GPI8_KEY1,          // GPIO00 E-FW Button
    GPI8_USER,          // GPIO01 TX Serial RXD and Optional sensor
    0,                         // GPIO02
    GPI8_USER,          // GPIO03 RX Serial TXD and Optional sensor
    GPI8_USER,          // GPIO04 Optional sensor
    GPI8_USER,          // GPIO05 Optional sensor
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12
    GPI8_USER,          // GPIO13 BLUE LED
    GPI8_USER,          // GPIO14 Optional sensor
    0,                  // GPIO15
    0,                  // GPIO16
    GPI8_USER           // ADC0 A0 Analog input
  },
  {                     // H801 - Lixada H801 Wifi (ESP8266)
    GPI8_USER,          // GPIO00 E-FW Button
    GPI8_LED1,          // GPIO01 Green LED - Link and Power status
    GPI8_USER,          // GPIO02 TX and Optional sensor - Pin next to TX on the PCB
    GPI8_USER,          // GPIO03 RX and Optional sensor - Pin next to GND on the PCB
    GPI8_PWM5,          // GPIO04 W2 - PWM5
    GPI8_LED2_INV,      // GPIO05 Red LED
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM3,          // GPIO12 Blue
    GPI8_PWM2,          // GPIO13 Green
    GPI8_PWM4,          // GPIO14 W1 - PWM4
    GPI8_PWM1,          // GPIO15 Red
    0, 0
  },
  {                     // SONOFF_SC - onoff SC (ESP8266)
    GPI8_KEY1,          // GPIO00 Button
    GPI8_TXD,           // GPIO01 RXD to ATMEGA328P
    GPI8_USER,          // GPIO02 Optional sensor
    GPI8_RXD,           // GPIO03 TXD to ATMEGA328P
    0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    0,
    GPI8_LED1_INV,      // GPIO13 Green Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // SONOFF_BN - Sonoff BN-SZ01 Ceiling led (ESP8285)
    0, 0, 0, 0, 0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM1,          // GPIO12 Light
    GPI8_LED1_INV,      // GPIO13 Red Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // HUAFAN_SS - Hua Fan Smart Socket (ESP8266) - like Sonoff Pow
    GPI8_LEDLNK_INV,    // GPIO00 Blue Led (0 = On, 1 = Off) - Link status
    0, 0,
    GPI8_LED1_INV,      // GPIO03 Red Led (0 = On, 1 = Off) - Power status
    GPI8_KEY1,          // GPIO04 Button
    GPI8_REL1_INV,      // GPIO05 Relay (0 = On, 1 = Off)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_CF1,       // GPIO12 HLW8012 CF1 voltage / current
    GPI8_NRG_SEL,       // GPIO13 HLW8012 Sel output (1 = Voltage)
    GPI8_HLW_CF,        // GPIO14 HLW8012 CF power
    0, 0, 0
  },
  {                     // SONOFF_BRIDGE - Sonoff RF Bridge 433 (ESP8285)
    GPI8_KEY1,          // GPIO00 Button
    GPI8_TXD,           // GPIO01 RF bridge control
    GPI8_USER,          // GPIO02 Optional sensor
    GPI8_RXD,           // GPIO03 RF bridge control
    GPI8_USER,          // GPIO04 Optional sensor
    GPI8_USER,          // GPIO05 Optional sensor
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12 Optional sensor
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    GPI8_USER,          // GPIO14 Optional sensor
    0, 0, 0
  },
  {                     // SONOFF_B1 - Sonoff B1 (ESP8285 - my9231)
    GPI8_KEY1,          // GPIO00 Pad
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor pad
    GPI8_USER,          // GPIO02 Optional sensor SDA pad
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor pad
    0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_DI,            // GPIO12 my9231 DI
    0,
    GPI8_DCKI,          // GPIO14 my9231 DCKI
    0, 0, 0
  },
  {                     // AILIGHT - Ai-Thinker RGBW led (ESP8266 - my9291)
    GPI8_KEY1,          // GPIO00 Pad
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor pad
    GPI8_USER,          // GPIO02 Optional sensor SDA pad
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor pad
    0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    0,
    GPI8_DI,            // GPIO13 my9291 DI
    0,
    GPI8_DCKI,          // GPIO15 my9291 DCKI
    0, 0
  },
  {                     // SONOFF_T11 - Sonoff T1 1CH (ESP8285)
    GPI8_KEY1,          // GPIO00 Button 1
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Optional Sensor (J3 Pin 5)
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Blue Led and Relay 1 (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // SUPLA1 - Supla Espablo (ESP8266)
                        // http://www.wykop.pl/ramka/3325399/diy-supla-do-puszki-instalacyjnej-podtynkowej-supla-org/
    0,                  // GPIO00 Flash jumper
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
#ifdef USE_DS18x20
    GPI8_DSB,           // GPIO02 DS18B20 sensor
#else
    GPI8_USER,          // GPIO02 Optional sensor
#endif
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_KEY1,          // GPIO04 Button 1
    GPI8_REL1,          // GPIO05 Relay 1 (0 = Off, 1 = On)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12 Optional sensor
    GPI8_REL2,          // GPIO13 Relay 2 (0 = Off, 1 = On)
    GPI8_USER,          // GPIO14 Optional sensor
    0,
    GPI8_LED1,          // GPIO16 Led (1 = On, 0 = Off) - Link and Power status
    GPI8_USER           // ADC0 A0 Analog input
  },
  {                     // WITTY - Witty Cloud Dev Board (ESP8266)
                        // https://www.aliexpress.com/item/ESP8266-serial-WIFI-Witty-cloud-Development-Board-ESP-12F-module-MINI-nodemcu/32643464555.html
    GPI8_USER,          // GPIO00 D3 flash push button on interface board
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_LED1_INV,      // GPIO02 D4 Blue Led (0 = On, 1 = Off) on ESP-12F - Link and Power status
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_KEY1,          // GPIO04 D2 push button on ESP-12F board
    GPI8_USER,          // GPIO05 D1 optional sensor
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM2,          // GPIO12 D6 RGB LED Green
    GPI8_PWM3,          // GPIO13 D7 RGB LED Blue
    GPI8_USER,          // GPIO14 D5 optional sensor
    GPI8_PWM1,          // GPIO15 D8 RGB LED Red
    GPI8_USER,          // GPIO16 D0 optional sensor
    GPI8_USER           // ADC0 A0 Light sensor / Requires USE_ADC_VCC in user_config.h to be disabled
  },
  {                     // YUNSHAN - Yunshan Wifi Relay (ESP8266)
                        // https://www.ebay.com/p/Esp8266-220v-10a-Network-Relay-WiFi-Module/1369583381
                        // Schematics and Info https://ucexperiment.wordpress.com/2016/12/18/yunshan-esp8266-250v-15a-acdc-network-wifi-relay-module/
    0,                  // GPIO00 Flash jumper - Module Pin 8
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor - Module Pin 2
    GPI8_LED1_INV,      // GPIO02 Blue Led (0 = On, 1 = Off) on ESP-12F - Module Pin 7 - Link and Power status
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor - Module Pin 3
    GPI8_REL1,          // GPIO04 Red Led and Relay (0 = Off, 1 = On) - Module Pin 10
    GPI8_KEY1,          // GPIO05 Blue Led and OptoCoupler input - Module Pin 9
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    0, 0, 0, 0, 0, 0
  },
  {                     // MAGICHOME - Magic Home (aka Flux-light) (ESP8266) and Arilux LC10 (ESP8285)
                        // https://www.aliexpress.com/item/Magic-Home-Mini-RGB-RGBW-Wifi-Controller-For-Led-Strip-Panel-light-Timing-Function-16million-colors/32686853650.html
    0,
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_LED1_INV,      // GPIO02 Blue onboard LED - Link and Power status
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_ARIRFRCV,      // GPIO04 IR or RF receiver (optional) (Arilux LC10)
    GPI8_PWM2,          // GPIO05 RGB LED Green
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM3,          // GPIO12 RGB LED Blue
    GPI8_USER,          // GPIO13 RGBW LED White (optional - set to PWM4 for Cold White or Warm White as used on Arilux LC10)
    GPI8_PWM1,          // GPIO14 RGB LED Red
    GPI8_ARIRFSEL,      // GPIO15 RF receiver control (Arilux LC10)
    0, 0
  },
  {                     // LUANIHVIO - ESP8266_HVIO
                        // https://luani.de/projekte/esp8266-hvio/
    0,                  // GPIO00 Flash jumper
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Optional sensor / I2C SDA pad
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_REL1,          // GPIO04 Relay 1 (0 = Off, 1 = On)
    GPI8_REL2,          // GPIO05 Relay 2 (0 = Off, 1 = On)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_SWT1,          // GPIO12 External input 1 (0 = On, 1 = Off)
    GPI8_SWT2,          // GPIO13 External input 2 (0 = On, 1 = Off)
    GPI8_USER,          // GPIO14 Optional sensor / I2C SCL pad
    GPI8_LED1,          // GPIO15 Led (1 = On, 0 = Off) - Link and Power status
    0,
    GPI8_USER           // ADC0 A0 Analog input
  },
  {                     // KMC_70011 - KMC 70011
                        // https://www.amazon.com/KMC-Timing-Monitoring-Network-125V-240V/dp/B06XRX2GTQ
    GPI8_KEY1,          // GPIO00 Button
    0, 0, 0,
    GPI8_HLW_CF,        // GPIO04 HLW8012 CF power
    GPI8_NRG_CF1,       // GPIO05 HLW8012 CF1 voltage / current
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL,       // GPIO12 HLW8012 SEL (1 = Voltage)
    GPI8_LED1_INV,      // GPIO13 Green Led - Link and Power status
    GPI8_REL1,          // GPIO14 Relay
    0, 0, 0
  },
  {                     // ARILUX_LC01 - Arilux AL-LC01 (ESP8285)
                        // https://www.banggood.com/nl/ARILUX-AL-LC01-Super-Mini-LED-WIFI-Smart-RGB-Controller-For-RGB-LED-Strip-Light-DC-9-12V-p-1058603.html
                        //  (PwmFrequency 1111Hz)
    GPI8_KEY1,          // GPIO00 Optional Button
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_ARIRFSEL,      // GPIO02 RF receiver control
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_ARIRFRCV,      // GPIO04 IR or RF receiver (optional)
    GPI8_PWM1,          // GPIO05 RGB LED Red
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM2,          // GPIO12 RGB LED Green
    GPI8_PWM3,          // GPIO13 RGB LED Blue
    GPI8_USER,          // GPIO14 RGBW LED White (optional - set to PWM4 for Cold White or Warm White)
    0, 0, 0
  },
  {                     // ARILUX_LC11 - Arilux AL-LC11 (ESP8266)
                        // https://www.banggood.com/nl/ARILUX-AL-LC11-Super-Mini-LED-WIFI-APP-Controller-RF-Remote-Control-For-RGBWW-LED-Strip-DC9-28V-p-1085112.html
                        //  (PwmFrequency 540Hz)
    GPI8_KEY1,          // GPIO00 Optional Button
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_ARIRFSEL,      // GPIO02 RF receiver control
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_PWM2,          // GPIO04 RGB LED Green
    GPI8_PWM1,          // GPIO05 RGB LED Red
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM5,          // GPIO12 RGBCW LED Warm
    GPI8_PWM4,          // GPIO13 RGBW LED Cold
    GPI8_PWM3,          // GPIO14 RGB LED Blue
    GPI8_ARIRFRCV,      // GPIO15 RF receiver input
    0, 0
  },
  {                     // ARILUX_LC06 - Arilux AL-LC06 (ESP8285)
                        // https://www.banggood.com/ARILUX-AL-LC06-LED-WIFI-Smartphone-Controller-Romote-5-Channels-DC12-24V-For-RGBWW-Strip-light-p-1061476.html
    GPI8_KEY1,          // GPIO00 Optional Button
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Empty pad
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_USER,          // GPIO04 W2 - PWM5
    0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM2,          // GPIO12 RGB LED Green
    GPI8_PWM3,          // GPIO13 RGB LED Blue
    GPI8_PWM1,          // GPIO14 RGB LED Red
    GPI8_USER,          // GPIO15 RGBW LED White
    0, 0
  },
  {                     // ZENGGE_ZF_WF017 - Zenggee ZJ-WF017-A (ESP12S))
                        // https://www.ebay.com/p/Smartphone-Android-IOS-WiFi-Music-Controller-for-RGB-5050-3528-LED-Strip-Light/534446632?_trksid=p2047675.l2644
    GPI8_KEY1,          // GPIO00 Optional Button
    0,
    GPI8_USER,          // GPIO02 Empty pad
    0,
    GPI8_USER,          // GPIO04 W2 - PWM5
    0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM2,          // GPIO12 RGB LED Green
    GPI8_PWM1,          // GPIO13 RGB LED Red
    GPI8_PWM3,          // GPIO14 RGB LED Blue
    0, 0, 0
  },
  {                     // SONOFF_POW_R2 - Sonoff Pow R2 (ESP8285 - CSE7766)
    GPI8_KEY1,          // GPIO00 Button
    GPI8_CSE7766_TX,    // GPIO01 Serial RXD 4800 baud 8E1 CSE7766 energy sensor
    0,
    GPI8_CSE7766_RX,    // GPIO03 Serial TXD
    0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Red Led and Relay (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // BLITZWOLF_BWSHP - BlitzWolf BW-SHP2 and BW-SHP6 (ESP8285 - BL0937 or HJL-01 Energy Monitoring)
                        // https://www.banggood.com/BlitzWolf-BW-SHP2-Smart-WIFI-Socket-EU-Plug-220V-16A-Work-with-Amazon-Alexa-Google-Assistant-p-1292899.html
                        // https://www.amazon.de/Steckdose-Homecube-intelligente-Verbrauchsanzeige-funktioniert/dp/B076Q2LKHG/ref=sr_1_fkmr0_1
                        // https://www.amazon.de/Intelligente-Stromverbrauch-Fernsteurung-Schaltbare-Energieklasse/dp/B076WZQS4S/ref=sr_1_1
                        // https://www.aliexpress.com/store/product/BlitzWolf-BW-SHP6-EU-Plug-Metering-Version-WIFI-Smart-Socket-220V-240V-10A-Work-with-Amazon/1965360_32945504669.html
    GPI8_LED1_INV,      // GPIO00 Red Led (1 = On, 0 = Off) - Power status
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_LEDLNK_INV,    // GPIO02 Blue Led (1 = On, 0 = Off) - Link status
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    0,
    GPI8_HJL_CF,        // GPIO05 BL0937 or HJL-01 CF power
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 or HJL-01 Sel output (0 = Voltage)
    GPI8_KEY1,          // GPIO13 Button
    GPI8_NRG_CF1,       // GPIO14 BL0937 or HJL-01 CF1 current / voltage
    GPI8_REL1,          // GPIO15 Relay (0 = Off, 1 = On)
    0, 0
  },
  {                     // SHELLY1 - Shelly1 Open Source (ESP8266 - 2MB) - https://shelly.cloud/shelly1-open-source/
    GPI8_USER,          // GPIO00 - Can be changed to GPI8_USER, only if Shelly is powered with 12V DC
    GPI8_USER,          // GPIO01 Serial RXD - Can be changed to GPI8_USER, only if Shelly is powered with 12V DC
    0,
    GPI8_USER,          // GPIO03 Serial TXD - Can be changed to GPI8_USER, only if Shelly is powered with 12V DC
    GPI8_REL1,          // GPIO04 Relay (0 = Off, 1 = On)
    GPI8_SWT1_NP,       // GPIO05 SW pin
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    0, 0, 0, 0, 0, 0
  },
  {                     // SHELLY2 - Shelly2 (ESP8266 - 2MB) - https://shelly.cloud/shelly2/
    0,
    GPI8_MCP39F5_TX,    // GPIO01 MCP39F501 Serial input
    0,
    GPI8_MCP39F5_RX,    // GPIO03 MCP39F501 Serial output
    GPI8_REL1,          // GPIO04
    GPI8_REL2,          // GPIO05
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_SWT1,          // GPIO12
    0,
    GPI8_SWT2,          // GPIO14
    GPI8_MCP39F5_RST,   // GPIO15 MCP39F501 Reset
    0,
    0
  },
  {                     // PHILIPS - Xiaomi Philips bulb (ESP8266)
    0, 0, 0, 0, 0, 0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_PWM2,          // GPIO12 cold/warm light
    0, 0,
    GPI8_PWM1,          // GPIO15 light intensity
    0, 0
  },
  {                     // NEO_COOLCAM - Neo Coolcam (ESP8266)
                        // https://www.banggood.com/NEO-COOLCAM-WiFi-Mini-Smart-Plug-APP-Remote-Control-Timing-Smart-Socket-EU-Plug-p-1288562.html?cur_warehouse=CN
    0, 0, 0, 0,
    GPI8_LED1_INV,      // GPIO04 Red Led (0 = On, 1 = Off) - Link and Power status
    0,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Red Led and Relay (0 = Off, 1 = On)
    GPI8_KEY1,          // GPIO13 Button
    0, 0, 0, 0
  },
  {                     // ESP_SWITCH - Michael Haustein 4 channel wall switch (ESP07 = ESP8266)
                        // Use rules for further actions like - rule on power1#state do publish cmnd/other_device/power %value% endon
    GPI8_KEY2,          // GPIO00 Button 2
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_REL3_INV,      // GPIO02 Yellow Led 3 (0 = On, 1 = Off)
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_KEY1,          // GPIO04 Button 1
    GPI8_REL2_INV,      // GPIO05 Red Led 2 (0 = On, 1 = Off)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL4_INV,      // GPIO12 Blue Led 4 (0 = On, 1 = Off)
    GPI8_KEY4,          // GPIO13 Button 4
    GPI8_KEY3,          // GPIO14 Button 3
    GPI8_LED1,          // GPIO15 Optional sensor
    GPI8_REL1_INV,      // GPIO16 Green Led 1 (0 = On, 1 = Off)
    0
  },
  {                     // OBI - OBI socket (ESP8266) - https://www.obi.de/hausfunksteuerung/wifi-stecker-schuko/p/2291706
    GPI8_USER,          // GPIO00
    GPI8_USER,          // GPIO01 Serial RXD
    0,
    GPI8_USER,          // GPIO03 Serial TXD
    GPI8_LED1,          // GPIO04 Blue LED - Link and Power status
    GPI8_REL1,          // GPIO05 (Relay OFF, but used as Relay Switch)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_LED3,          // GPIO12 (Relay ON, but set to LOW, so we can switch with GPIO05)
    GPI8_USER,          // GPIO13
    GPI8_KEY1,          // GPIO14 Button
    0,
    GPI8_USER,          // GPIO16
    GPI8_USER           // ADC0   A0 Analog input
  },
  {                     // TECKIN - https://www.amazon.de/gp/product/B07D5V139R
    0,
    GPI8_KEY1,          // GPIO01 Serial TXD and Button
    0,
    GPI8_LED1_INV,      // GPIO03 Serial RXD and Red Led (0 = On, 1 = Off) - Power status
    GPI8_HJL_CF,        // GPIO04 BL0937 or HJL-01 CF power
    GPI8_NRG_CF1,       // GPIO05 BL0937 or HJL-01 CF1 current / voltage
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 or HJL-01 Sel output (0 = Voltage)
    GPI8_LEDLNK_INV,    // GPIO13 Blue Led (0 = On, 1 = Off) - Link status
    GPI8_REL1,          // GPIO14 Relay (0 = Off, 1 = On)
    0, 0, 0
  },
  {                     // APLIC_WDP303075 - Aplic WDP 303075 (ESP8285 - HLW8012 Energy Monitoring)
                        // https://www.amazon.de/dp/B07CNWVNJ2
    0, 0, 0,
    GPI8_KEY1,          // GPIO03 Button
    GPI8_HLW_CF,        // GPIO04 HLW8012 CF power
    GPI8_NRG_CF1,       // GPIO05 HLW8012 CF1 current / voltage
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 HLW8012 CF Sel output (0 = Voltage)
    GPI8_LED1_INV,      // GPIO13 LED (0 = On, 1 = Off) - Link and Power status
    GPI8_REL1,          // GPIO14 Relay SRU 5VDC SDA (0 = Off, 1 = On )
    0, 0, 0
  },
  {                     // TUYA_DIMMER - Tuya MCU device (ESP8266 w/ separate MCU)
                        // https://www.amazon.com/gp/product/B07CTNSZZ8/ref=oh_aui_detailpage_o00_s00?ie=UTF8&psc=1
    GPI8_USER,          // Virtual Button (controlled by MCU)
    GPI8_USER,          // GPIO01 MCU serial control
    GPI8_USER,
    GPI8_USER,          // GPIO03 MCU serial control
    GPI8_USER,
    GPI8_USER,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,
    GPI8_USER,
    GPI8_USER,          // GPIO14 Green Led
    GPI8_USER,
    GPI8_USER,
    0
  },
  {                     // GOSUND - https://www.amazon.de/gp/product/B0777BWS1P
    0,
    GPI8_LEDLNK_INV,    // GPIO01 Serial RXD and LED1 (blue) inv - Link status
    0,
    GPI8_KEY1,          // GPIO03 Serial TXD and Button
    GPI8_HJL_CF,        // GPIO04 BL0937 or HJL-01 CF power
    GPI8_NRG_CF1,       // GPIO05 BL0937 or HJL-01 CF1 current / voltage
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 or HJL-01 Sel output (0 = Voltage)
    GPI8_LED1_INV,      // GPIO13 LED2 (red) inv - Power status
    GPI8_REL1,          // GPIO14 Relay (0 = Off, 1 = On)
    0, 0, 0
  },
  {                     // ARMTRONIX_DIMMERS - ARMTRONIX Dimmer, one or two channel (ESP8266 w/ separate MCU dimmer)
                        // https://www.tindie.com/products/Armtronix/wifi-ac-dimmer-two-triac-board/
                        // https://www.tindie.com/products/Armtronix/wifi-ac-dimmer-esp8266-one-triac-board-alexaecho/
    GPI8_USER,
    GPI8_TXD,           // GPIO01 MCU serial control
    GPI8_USER,
    GPI8_RXD,           // GPIO03 MCU serial control
    GPI8_USER,
    GPI8_USER,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,
    GPI8_USER,
    GPI8_USER,
    GPI8_USER,
    GPI8_USER,
    0
  },
  {                     // SK03_TUYA - Outdoor smart plug with power monitoring HLW8012 chip - https://www.amazon.com/gp/product/B07CG7MBPV
    GPI8_KEY1,          // GPIO00 Button
    0, 0, 0,
    GPI8_HLW_CF,        // GPIO04 HLW8012 CF power
    GPI8_NRG_CF1,       // GPIO05 HLW8012 CF1 current / voltage
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 HLW8012 CF Sel output (0 = Voltage)
    GPI8_LED1_INV,      // GPIO13 Red Led (0 = On, 1 = Off) - Power status
    GPI8_LEDLNK_INV,    // GPIO14 Blue Led (0 = On, 1 = Off) - Link status
    GPI8_REL1,          // GPIO15 Relay (0 = Off, 1 = On)
    0, 0
  },
  {                     // PS_16_DZ - PS-16-DZ Dimmer (ESP8266 w/ separate Nuvoton MCU dimmer)
                        // https://www.aliexpress.com/item/SM-Smart-WIFI-Wall-Dimmer-Light-Switch-US-Ewelink-APP-Remote-Control-Wi-Fi-Wirele-Work/32871151902.html
    GPI8_USER,
    GPI8_TXD,           // GPIO01 MCU serial control
    GPI8_USER,
    GPI8_RXD,           // GPIO03 MCU serial control
    GPI8_USER,
    GPI8_USER,
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,
    GPI8_LED1,          // GPIO13 WiFi LED - Link and Power status
    GPI8_USER,
    GPI8_USER,
    GPI8_USER,
    0
  },
  {                     // TECKIN_US - Teckin SP20 US with Energy Monitoring
                        // https://www.amazon.com/Outlet-Compatible-Monitoring-Function-Required/dp/B079Q5W22B
                        // https://www.amazon.com/Outlet-ZOOZEE-Monitoring-Function-Compatible/dp/B07J2LR5KN
    GPI8_LED1_INV,      // GPIO00 Red Led (1 = On, 0 = Off) - Power status
    0,
    GPI8_LEDLNK_INV,    // GPIO02 Blue Led (1 = On, 0 = Off) - Link status
    0,
    GPI8_REL1,          // GPIO04 Relay (0 = Off, 1 = On)
    GPI8_HJL_CF,        // GPIO05 BL0937 or HJL-01 CF power
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 or HJL-01 Sel output (0 = Voltage)
    GPI8_KEY1,          // GPIO13 Button
    GPI8_NRG_CF1,       // GPIO14 BL0937 or HJL-01 CF1 current / voltage
    0, 0, 0
  },
  {                     // MANZOKU_EU_4 - "MANZOKU" labeled power strip, EU version
                        // https://www.amazon.de/Steckdosenleiste-AOFO-Mehrfachsteckdose-Ãœberspannungsschutz-Sprachsteuerung/dp/B07GBSD11P/
                        // https://www.amazon.de/Steckdosenleiste-Geekbes-USB-Anschluss-Kompatibel-gesteuert/dp/B078W23BW9/
    0,                  // GPIO00
    0,                  // GPIO01 Serial RXD
    0,
    GPI8_KEY1,          // GPIO03 Serial TXD + Button
    GPI8_REL2,          // GPIO04 Relay 2
    GPI8_REL1,          // GPIO05 Relay 1
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL3,          // GPIO12 Relay 3
    GPI8_REL4,          // GPIO13 Relay 4
    GPI8_USER,          // GPIO14
    0,
    GPI8_USER,          // GPIO16
    0
  },
  {                     // OBI2 - OBI socket (ESP8266) - https://www.obi.de/hausfunksteuerung/wifi-stecker-schuko-2-stueck-weiss/p/4077673
    0,                  // GPIO00
    0,                  // GPIO01 Serial RXD
    0,
    0,                  // GPIO03 Serial TXD
    GPI8_REL1,          // GPIO04 Relay 1
    GPI8_KEY1,          // GPIO05 Button
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_LEDLNK_INV,    // GPIO12 Green LED - Link status
    GPI8_LED1,          // GPIO13 Red LED - Power status
    0, 0, 0, 0
  },
  {                     // YTF_IR_BRIDGE - https://www.aliexpress.com/item/Tuya-universal-Smart-IR-Hub-remote-control-Voice-Control-AC-TV-Work-With-Alexa-Google-Home/32951202513.html
    GPI8_USER,          // GPIO00
    GPI8_USER,          // GPIO01 Serial RXD
    GPI8_USER,          // GPIO02
    GPI8_USER,          // GPIO03 Serial TXD
    GPI8_LED1_INV,      // GPIO04 Blue Led - Link status
    GPI8_IRRECV,        // GPIO05 IR Receiver
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    0,                  // GPIO12
    GPI8_KEY1,          // GPIO13 Button
    GPI8_IRSEND,        // GPIO14 IR Transmitter
    0, 0, 0
  },
  {                     // DIGOO - Digoo DG-SP202
                        // https://www.banggood.com/DIGOO-DG-SP202-Dual-EU-Plug-Smart-WIFI-Socket-Individual-Controllable-Energy-Monitor-Remote-Control-Timing-Smart-Home-Outlet-let-p-1375323.html
    GPI8_KEY1,          // GPIO00 Button1
    0,                  // GPIO01 Serial RXD
    0,                  // GPIO02
    0,                  // GPIO03 Serial TXD
    GPI8_HJL_CF,        // GPIO04 BL0937 or HJL-01 CF power
    GPI8_NRG_CF1,       // GPIO05 BL0937 or HJL-01 CF1 current / voltage
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 or HJL-01 Sel output (0 = Voltage)
    GPI8_LED1,          // GPIO13 Blue Leds - Link Status
    GPI8_REL2,          // GPIO14 Relay2 (0 = Off, 1 = On) and Red Led
    GPI8_REL1,          // GPIO15 Relay1 (0 = Off, 1 = On) and Red Led
    GPI8_KEY2_NP,       // GPIO16 Button2, externally pulled up
    0
  },
  {                     // KA10 - SMANERGY KA10 (ESP8285 - BL0937 Energy Monitoring) - https://www.amazon.es/dp/B07MBTCH2Y
    0,                  // GPIO00
    GPI8_LEDLNK_INV,    // GPIO01 Blue LED - Link status
    0,                  // GPIO02
    GPI8_KEY1,          // GPIO03 Button
    GPI8_HJL_CF,        // GPIO04 BL0937 CF power
    GPI8_NRG_CF1,       // GPIO05 BL0937 CF1 voltage / current
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 Sel output (1 = Voltage)
    GPI8_LED1,          // GPIO13 Red LED - Power status
    GPI8_REL1,          // GPIO14 Relay 1
    0, 0, 0
  },
  {                     // ZX2820
    GPI8_KEY1,          // GPIO00 Button
    0, 0, 0,
    GPI8_HLW_CF,        // GPIO04 HLW8012 CF power
    GPI8_NRG_CF1,       // GPIO05 HLW8012 CF1 voltage / current
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 HLW8012 SEL (0 = Voltage)
    GPI8_LED1_INV,      // GPIO13 Green Led - Link and Power status
    GPI8_REL1,          // GPIO14 Relay
    0, 0, 0
  },
  {                     // MI_DESK_LAMP - Mi LED Desk Lamp - https://www.mi.com/global/smartlamp/
    0, 0,
    GPI8_KEY1,          // GPIO02 Button
    0,
    GPI8_PWM1,          // GPIO04 Cold White
    GPI8_PWM2,          // GPIO05 Warm White
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_ROT1A,         // GPIO12 Rotary switch A pin
    GPI8_ROT1B,         // GPIO13 Rotary switch B pin
    0, 0, 0, 0
  },
  {                     // SP10 - Tuya SP10 (BL0937 Energy Monitoring)
                        // https://www.aliexpress.com/item/Smart-Mini-WiFi-Plug-Outlet-Switch-Work-With-ForEcho-Alexa-Google-Home-Remote-EU-Smart-Socket/32963670423.html
    0,                  // GPIO00
    GPI8_PWM1,          // GPIO01 Nightlight
    0,                  // GPIO02
    GPI8_KEY1,          // GPIO03 Button
    GPI8_HJL_CF,        // GPIO04 BL0937 CF power
    GPI8_NRG_CF1,       // GPIO05 BL0937 CF1 voltage / current
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_NRG_SEL_INV,   // GPIO12 BL0937 Sel output (1 = Voltage)
    GPI8_LED1,          // GPIO13 Blue LED - Link status
    GPI8_REL1,          // GPIO14 Relay and red LED
    0, 0, 0
  },
  {                     // WAGA - WAGA life CHCZ02MB (HJL-01 Energy Monitoring)
                        // https://www.ebay.com/itm/332595697006
    GPI8_LED1_INV,      // GPIO00 Red LED
    0,                  // GPIO01 Serial RXD
    0,                  // GPIO02
    GPI8_NRG_SEL_INV,   // GPIO03 HJL-01 Sel output (1 = Voltage)
    0,                  // GPIO04
    GPI8_HJL_CF,        // GPIO05 HJL-01 CF power
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Relay
    GPI8_KEY1,          // GPIO13 Button
    GPI8_NRG_CF1,       // GPIO14 HJL-01 CF1 voltage / current
    GPI8_LEDLNK_INV,    // GPIO15 Blue LED - Link status
    0, 0
  },
  {                     // SYF05 - Sunyesmart SYF05 (a.k.a. Fcmila) = TYWE3S + SM16726
                        // Also works with Merkury 904 RGBW Bulbs with 13 set to GPI8_SM16716_SEL
                        // https://www.flipkart.com/fc-mila-bxav-xs-ad-smart-bulb/p/itmf85zgs45fzr7n
                        // https://docs.tuya.com/en/hardware/WiFi-module/wifi-e3s-module.html
                        // http://www.datasheet-pdf.com/PDF/SM16716-Datasheet-Sunmoon-932771
    GPI8_USER,          // GPIO00 N.C.
    0,                  // GPIO01 Serial RXD
    GPI8_USER,          // GPIO02 N.C.
    0,                  // GPIO03 Serial TXD
    GPI8_SM16716_CLK,   // GPIO04 SM16716 Clock
    GPI8_PWM1,          // GPIO05 White
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12 Alt. White on some devices
    GPI8_USER,          // GPIO13 SM16716 Enable on some devices
    GPI8_SM16716_DAT,   // GPIO14 SM16716 Data
    0,                  // GPIO15 wired to GND
    GPI8_USER,          // GPIO16 N.C.
    GPI8_USER           // ADC0 A0 Analog input
  },
  {                     // EXS_DIMMER - EX-Store WiFi Dimmer v4, two channel (ESP8266 w/ separate MCU dimmer)
                        // https://ex-store.de/2-Kanal-RS232-WiFi-WLan-Dimmer-Modul-V4-fuer-Unterputzmontage-230V-3A
                        // https://ex-store.de/2-Kanal-RS232-WiFi-WLan-Dimmer-Modul-V4-fuer-Unterputzmontage-230V-3A-ESP8266-V12-Stift-und-Buchsenleisten
    0,
    GPI8_TXD,           // GPIO01 MCU serial control
    GPI8_LEDLNK,        // GPIO02 LED Link
    GPI8_RXD,           // GPIO03 MCU serial control
    GPI8_USER,          // GPIO04
    GPI8_USER,          // GPIO05
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12
    GPI8_EXS_ENABLE,    // GPIO13 EXS MCU Enable
    GPI8_USER,          // GPIO14
    0,                  // GPIO15
    0, 0
  },
  {                     // PWM_DIMMER - Support for Martin Jerry/acenx/Tessan/NTONPOWER SD0x PWM
                        // dimmer switches. The brightness of the load for these dimmers is
                        // controlled by a PWM GPIO pin. There are typically power, up & down
                        // buttons and 4 LED's. Examples are:
                        // https://www.amazon.com/dp/B07FXYSVR1
                        // https://www.amazon.com/dp/B07V26Q3VD
                        // https://www.amazon.com/dp/B07K67D43J
                        // https://www.amazon.com/dp/B07TTGFWFM
    GPI8_KEY3,          // GPIO00 Up button
    GPI8_KEY2,          // GPIO01 Down button
    0,                  // GPIO02
    GPI8_LED4_INV,      // GPIO03 Level 5 LED
    GPI8_LEDLNK_INV,    // GPIO04 LED Link
    GPI8_LED3_INV,      // GPIO05 Level 4 LED
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_LED2_INV,      // GPIO12 Level 3 LED
    GPI8_PWM1,          // GPIO13 Dimmer PWM
    GPI8_LED1_INV,      // GPIO12 Level 2 LED
    GPI8_KEY1_INV,      // GPIO15 Power button
    GPI8_REL1_INV,      // GPIO16 Power relay/Level 1 LED
    0
  },
  {                     // SONOFF_ZB_BRIDGE - Sonoff Zigbee Bridge (ESP8266)
    GPI8_LED1_INV,      // GPIO00 Green Led (0 = On, 1 = Off) - Traffic between ESP and EFR
    GPI8_ZIGBEE_TX,     // GPIO01 Zigbee Serial control
    0,                  // GPIO02
    GPI8_ZIGBEE_RX,     // GPIO03 Zigbee Serial control
    GPI8_ZIGBEE_RST,    // GPIO04 Zigbee Reset
    0,                  // GPIO05 EFR32 Bootloader mode (drive Low for Gecko Bootloader, inactive or high for Zigbee EmberZNet)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
                        // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
                        // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_I2C_SDA,       // GPIO12 I2C SDA - connected to 512KB EEPROM
    GPI8_LEDLNK_INV,    // GPIO13 Blue Led (0 = On, 1 = Off) - Link status
    GPI8_I2C_SCL,       // GPIO14 I2C SCL - connected to 512KB EEPROM
    0,                  // GPIO15 connected to IO15 pad, also used for logging
    GPI8_KEY1,          // GPIO16 Button
    0
  }
};

/*********************************************************************************************\
 * Templates with 14 usable pins (ESP8285)
\*********************************************************************************************/

const mytmplt8285 kModules8285[TMP_MAXMODULE_8266 - TMP_WEMOS] PROGMEM = {
  {                     // WEMOS - Any ESP8266/ESP8285 device like WeMos and NodeMCU hardware (ESP8266)
    GPI8_USER,          // GPIO00 D3 Wemos Button Shield
    GPI8_USER,          // GPIO01 TX Serial RXD
    GPI8_USER,          // GPIO02 D4 Wemos DHT Shield
    GPI8_USER,          // GPIO03 RX Serial TXD and Optional sensor
    GPI8_USER,          // GPIO04 D2 Wemos I2C SDA
    GPI8_USER,          // GPIO05 D1 Wemos I2C SCL / Wemos Relay Shield (0 = Off, 1 = On) / Wemos WS2812B RGB led Shield
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
    GPI8_USER,          // GPIO09 (SD_DATA2 Flash QIO or ESP8285)
    GPI8_USER,          // GPIO10 (SD_DATA3 Flash QIO or ESP8285)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_USER,          // GPIO12 D6
    GPI8_USER,          // GPIO13 D7
    GPI8_USER,          // GPIO14 D5
    GPI8_USER,          // GPIO15 D8
    GPI8_USER,          // GPIO16 D0 Wemos Wake
    GPI8_USER           // ADC0 A0 Analog input
  },
  {                     // SONOFF_4CH - Sonoff 4CH (ESP8285)
    GPI8_KEY1,          // GPIO00 Button 1
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Optional sensor
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_REL3,          // GPIO04 Sonoff 4CH Red Led and Relay 3 (0 = Off, 1 = On)
    GPI8_REL2,          // GPIO05 Sonoff 4CH Red Led and Relay 2 (0 = Off, 1 = On)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
    GPI8_KEY2,          // GPIO09 Button 2
    GPI8_KEY3,          // GPIO10 Button 3
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Red Led and Relay 1 (0 = Off, 1 = On) - Link and Power status
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off)
    GPI8_KEY4,          // GPIO14 Button 4
    GPI8_REL4,          // GPIO15 Red Led and Relay 4 (0 = Off, 1 = On)
    0, 0
  },
  {                     // SONOFF_T12 - Sonoff T1 2CH (ESP8285)
    GPI8_KEY1,          // GPIO00 Button 1
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Optional Sensor (J3 Pin 5)
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    0,
    GPI8_REL2,          // GPIO05 Blue Led and Relay 2 (0 = Off, 1 = On)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
    GPI8_KEY2,          // GPIO09 Button 2
    0,                  // GPIO10
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Blue Led and Relay 1 (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // SONOFF_T13 - Sonoff T1 3CH (ESP8285)
    GPI8_KEY1,          // GPIO00 Button 1
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    GPI8_USER,          // GPIO02 Optional Sensor (J3 Pin 5)
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    GPI8_REL3,          // GPIO04 Blue Led and Relay 3 (0 = Off, 1 = On)
    GPI8_REL2,          // GPIO05 Blue Led and Relay 2 (0 = Off, 1 = On)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
    GPI8_KEY2,          // GPIO09 Button 2
    GPI8_KEY3,          // GPIO10 Button 3
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Blue Led and Relay 1 (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // SONOFF_DUAL_R2 - Sonoff Dual R2 (ESP8285)
    GPI8_USER,          // GPIO00 Button 0 on header (0 = On, 1 = Off)
    GPI8_USER,          // GPIO01 Serial RXD and Optional sensor
    0,
    GPI8_USER,          // GPIO03 Serial TXD and Optional sensor
    0,
    GPI8_REL2,          // GPIO05 Relay 2 (0 = Off, 1 = On)
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
    GPI8_USER,          // GPIO09 Button 1 on header (0 = On, 1 = Off)
    GPI8_KEY1,          // GPIO10 Button on casing
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL1,          // GPIO12 Relay 1 (0 = Off, 1 = On)
    GPI8_LED1_INV,      // GPIO13 Blue Led (0 = On, 1 = Off) - Link and Power status
    0, 0, 0, 0
  },
  {                     // SONOFF_IFAN03 - Sonoff iFan03 (ESP8285)
    GPI8_KEY1,          // GPIO00 WIFI_KEY0 Button 1
    GPI8_TXD,           // GPIO01 ESP_TXD Serial RXD connection to P0.5 of RF microcontroller
    0,                  // GPIO02 ESP_LOG
    GPI8_RXD,           // GPIO03 ESP_RXD Serial TXD connection to P0.4 of RF microcontroller
    0,                  // GPIO04 DEBUG_RX
    0,                  // GPIO05 DEBUG_TX
                        // GPIO06 (SD_CLK   Flash)
                        // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                        // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
    GPI8_REL1_INV,      // GPIO09 WIFI_O0 Relay 1 (0 = Off, 1 = On) controlling the light
    GPI8_BUZZER_INV,    // GPIO10 WIFI_O4 Buzzer (0 = Off, 1 = On)
                        // GPIO11 (SD_CMD   Flash)
    GPI8_REL3,          // GPIO12 WIFI_O2 Relay 3 (0 = Off, 1 = On) controlling the fan
    GPI8_LED1_INV,      // GPIO13 WIFI_CHK Blue Led on PCA (0 = On, 1 = Off) - Link and Power status
    GPI8_REL2,          // GPIO14 WIFI_O1 Relay 2 (0 = Off, 1 = On) controlling the fan
    GPI8_REL4,          // GPIO15 WIFI_O3 Relay 4 (0 = Off, 1 = On) controlling the fan
    0, 0
  }
};

#endif  // ESP8266
#ifdef ESP32

/********************************************************************************************/
// Supported hardware modules
enum SupportedModules { WEMOS, ESP32_CAM_AITHINKER, MAXMODULE };

#define USER_MODULE        255

const char kModuleNames[] PROGMEM = "ESP32-DevKit|ESP32 Cam AiThinker";

// Default module settings
const uint8_t kModuleNiceList[MAXMODULE] PROGMEM = { WEMOS, ESP32_CAM_AITHINKER };

const mytmplt kModules PROGMEM =
{                              // WEMOS - Espressif ESP32-DevKitC - Any ESP32 device like WeMos and NodeMCU hardware (ESP32)
  AGPIO(GPIO_USER),            // 0       (I)O                GPIO0, ADC2_CH1, TOUCH1, RTC_GPIO11, CLK_OUT1, EMAC_TX_CLK
  AGPIO(GPIO_USER),            // 1       IO     TXD0         GPIO1, U0TXD, CLK_OUT3, EMAC_RXD2
  AGPIO(GPIO_USER),            // 2       IO                  GPIO2, ADC2_CH2, TOUCH2, RTC_GPIO12, HSPIWP, HS2_DATA0, SD_DATA0
  AGPIO(GPIO_USER),            // 3       IO     RXD0         GPIO3, U0RXD, CLK_OUT2
  AGPIO(GPIO_USER),            // 4       IO                  GPIO4, ADC2_CH0, TOUCH0, RTC_GPIO10, HSPIHD, HS2_DATA1, SD_DATA1, EMAC_TX_ER
  AGPIO(GPIO_USER),            // 5       IO                  GPIO5, VSPICS0, HS1_DATA6, EMAC_RX_CLK
                               // 6       IO                  GPIO6, Flash CLK
                               // 7       IO                  GPIO7, Flash D0
                               // 8       IO                  GPIO8, Flash D1
  AGPIO(GPIO_USER),            // 9       IO                  GPIO9, Flash D2, U1RXD
  AGPIO(GPIO_USER),            // 10      IO                  GPIO10, Flash D3, U1TXD
                               // 11      IO                  GPIO11, Flash CMD
  AGPIO(GPIO_USER),            // 12      (I)O                GPIO12, ADC2_CH5, TOUCH5, RTC_GPIO15, MTDI, HSPIQ, HS2_DATA2, SD_DATA2, EMAC_TXD3       (If driven High, flash voltage (VDD_SDIO) is 1.8V not default 3.3V. Has internal pull-down, so unconnected = Low = 3.3V. May prevent flashing and/or booting if 3.3V flash is connected and pulled high. See ESP32 datasheet for more details.)
  AGPIO(GPIO_USER),            // 13      IO                  GPIO13, ADC2_CH4, TOUCH4, RTC_GPIO14, MTCK, HSPID, HS2_DATA3, SD_DATA3, EMAC_RX_ER
  AGPIO(GPIO_USER),            // 14      IO                  GPIO14, ADC2_CH6, TOUCH6, RTC_GPIO16, MTMS, HSPICLK, HS2_CLK, SD_CLK, EMAC_TXD2
  AGPIO(GPIO_USER),            // 15      (I)O                GPIO15, ADC2_CH3, TOUCH3, MTDO, HSPICS0, RTC_GPIO13, HS2_CMD, SD_CMD, EMAC_RXD3         (If driven Low, silences boot messages from normal boot. Has internal pull-up, so unconnected = High = normal output.)
  AGPIO(GPIO_USER),            // 16      IO                  GPIO16, HS1_DATA4, U2RXD, EMAC_CLK_OUT
  AGPIO(GPIO_USER),            // 17      IO                  GPIO17, HS1_DATA5, U2TXD, EMAC_CLK_OUT_180
  AGPIO(GPIO_USER),            // 18      IO                  GPIO18, VSPICLK, HS1_DATA7
  AGPIO(GPIO_USER),            // 19      IO                  GPIO19, VSPIQ, U0CTS, EMAC_TXD0
  0,                           // 20
  AGPIO(GPIO_USER),            // 21      IO                  GPIO21, VSPIHD, EMAC_TX_EN
  AGPIO(GPIO_USER),            // 22      IO      LED         GPIO22, VSPIWP, U0RTS, EMAC_TXD1
  AGPIO(GPIO_USER),            // 23      IO                  GPIO23, VSPID, HS1_STROBE
  0,                           // 24
  AGPIO(GPIO_USER),            // 25      IO                  GPIO25, DAC_1, ADC2_CH8, RTC_GPIO6, EMAC_RXD0
  AGPIO(GPIO_USER),            // 26      IO                  GPIO26, DAC_2, ADC2_CH9, RTC_GPIO7, EMAC_RXD1
  AGPIO(GPIO_USER),            // 27      IO                  GPIO27, ADC2_CH7, TOUCH7, RTC_GPIO17, EMAC_RX_DV
  0,                           // 28
  0,                           // 29
  0,                           // 30
  0,                           // 31
  AGPIO(GPIO_USER),            // 32      IO                  GPIO32, XTAL_32K_P (32.768 kHz crystal oscillator input), ADC1_CH4, TOUCH9, RTC_GPIO9
  AGPIO(GPIO_USER),            // 33      IO                  GPIO33, XTAL_32K_N (32.768 kHz crystal oscillator output), ADC1_CH5, TOUCH8, RTC_GPIO8
  AGPIO(GPIO_USER),            // 34      I   NO PULLUP       GPIO34, ADC1_CH6, RTC_GPIO4
  AGPIO(GPIO_USER),            // 35      I   NO PULLUP       GPIO35, ADC1_CH7, RTC_GPIO5
  AGPIO(GPIO_USER),            // 36      I   NO PULLUP       GPIO36, SENSOR_VP, ADC_H, ADC1_CH0, RTC_GPIO0
  0,                           // 37          NO PULLUP
  0,                           // 38          NO PULLUP
  AGPIO(GPIO_USER),            // 39      I   NO PULLUP       GPIO39, SENSOR_VN, ADC1_CH3, ADC_H, RTC_GPIO3
  0                            // Flag
};

/*********************************************************************************************\
 Known templates

{"NAME":"AITHINKER CAM","GPIO":[4992,1,1,1,1,5088,1,1,1,1,1,1,1,1,5089,5090,0,5091,5184,5152,0,5120,5024,5056,0,0,0,0,4928,1,5094,5095,5092,0,0,5093],"FLAG":0,"BASE":1}
{"NAME":"Olimex ESP32-PoE","GPIO":[1,1,1,1,1,1,0,0,5536,1,1,1,1,0,5600,0,0,0,0,5568,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,1],"FLAG":0,"BASE":1}
{"NAME":"wESP32","GPIO":[1,1,1,1,1,1,0,0,0,1,1,1,5568,5600,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,1],"FLAG":0,"BASE":1}
{"NAME":"Denky (Teleinfo)","GPIO":[1,1,1,1,5664,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1376,1,1,0,0,0,0,1,5632,1,1,1,0,0,1],"FLAG":0,"BASE":1}

\*********************************************************************************************/

#endif  // ESP32

#endif  // _TASMOTA_TEMPLATE_H_
