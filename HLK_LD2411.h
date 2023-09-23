#ifndef _HLK_LD2411_
#define _HLK_LD2411_

#define ld2411_begin_bytes_len 2
#define ld2411_end_bytes_len 2

#define ld2411_values_len 3

#define ld2411_all_datas_len 7

#define ld2411_send_begin_bytes_len 4
#define ld2411_send_end_bytes_len 4

//------------------------------------------------------

#define ld2411_send_conf_enable_bytes_len 6
#define ld2411_recv_conf_enable_bytes_len 10

//------------------------------------------------------

#define ld2411_send_conf_end_bytes_len 4
#define ld2411_recv_conf_end_bytes_len 6

//------------------------------------------------------

#define ld2411_send_read_params_bytes_len 4
#define ld2411_recv_read_params_bytes_len 56

//------------------------------------------------------

#define ld2411_send_set_params_bytes_len 34
#define ld2411_recv_set_params_bytes_len 6

//------------------------------------------------------

#define ld2411_send_read_firm_ver_bytes_len 4
#define ld2411_recv_read_firm_ver_bytes_len 14

//------------------------------------------------------

#define ld2411_send_open_close_blt_bytes_len 6
#define ld2411_recv_open_close_blt_bytes_len 6

//------------------------------------------------------

#define ld2411_send_get_mac_addr_bytes_len 6
#define ld2411_recv_get_mac_addr_bytes_len 12

//------------------------------------------------------

#define ld2411_send_reboot_bytes_len 4
#define ld2411_recv_reboot_bytes_len 6

//------------------------------------------------------

#define ld2411_send_reset_bytes_len 6
#define ld2411_recv_reset_bytes_len 6

//------------------------------------------------------

#define LD2411_BAUD_9600 0x01
#define LD2411_BAUD_19200 0x02
#define LD2411_BAUD_38400 0x03
#define LD2411_BAUD_57600 0x04
#define LD2411_BAUD_115200 0x05
#define LD2411_BAUD_230400 0x06
#define LD2411_BAUD_256000 0x07
#define LD2411_BAUD_460800 0x08

#define ld2411_send_set_baud_bytes_len 6
#define ld2411_recv_set_baud_bytes_len 6

//------------------------------------------------------

#include <Arduino.h>
#ifndef _BOARD_GENERIC_STM32F103C_H_
#include <SoftwareSerial.h>
#endif

class HLK_LD2411 {

  public:
#ifndef ESP32
#ifndef _BOARD_GENERIC_STM32F103C_H_
    HLK_LD2411(SoftwareSerial *ssUart);
#endif
#else
    HLK_LD2411(EspSoftwareSerial::UART *ssEspUart);
    HLK_LD2411(uint8_t rx, uint8_t tx, EspSoftwareSerial::UART *ssEspUart);
    HLK_LD2411(uint8_t rx, uint8_t tx, HardwareSerial *hsUart);
#endif
    HLK_LD2411(HardwareSerial *hsUart);
    HLK_LD2411(uint8_t rx, uint8_t tx);
    void begin(uint32_t baud = 256000);
    void read();
    uint16_t getCampaignTarget();
    uint16_t getMicromotionTarget();
	String getTargetType();
    bool enableConfiguration();
    bool endConfiguration();
    void readParameters();
    uint16_t getMotionRangeMin();
    uint16_t getMotionRangeMax();
    uint16_t getMicromotionRangeMin();
    uint16_t getMicromotionRangeMax();
    uint16_t getNoOneDuration();
    bool setParameters(uint16_t motion_range_min,
                       uint16_t motion_range_max,
                       uint16_t micromotion_range_min,
                       uint16_t micromotion_range_max,
                       uint16_t no_one_duration);
    String getFirmwareVersion();
    bool openBluetooth();
    bool closeBluetooth();
    String getMacAddress();
    bool reboot();
    bool factoryReset();
    bool setBaudrate(uint32_t baudrate);

  private:

    bool is_soft;

#ifndef ESP32
#ifndef _BOARD_GENERIC_STM32F103C_H_
    SoftwareSerial *SS;
#endif
#else
    EspSoftwareSerial::UART *SS;
#endif
    HardwareSerial *HS;

    bool with_pins;
    bool with_ser;

    uint16_t rx_pin;
    uint16_t tx_pin;

    byte begin_bytes[ld2411_begin_bytes_len] = {0xAA, 0xAA};
    byte end_bytes[ld2411_end_bytes_len] = {0x55, 0x55};

    uint16_t begin_bytes_sum = 0;
    uint16_t end_bytes_sum = 0;

    byte all_datas[ld2411_all_datas_len];

    struct ld2411_datas {
	  String target_type;
      uint16_t campaign_target;
      uint16_t micromotion_target;
    } ld2411_data;

    byte send_begin_bytes[ld2411_send_begin_bytes_len] = {0xFD, 0xFC, 0xFB, 0xFA};
    byte send_end_bytes[ld2411_send_end_bytes_len] = {0x04, 0x03, 0x02, 0x01};

    uint32_t send_begin_bytes_sum = 0;
    uint32_t send_end_bytes_sum = 0;

    bool conf_enabled = false;

    //----------------------------------------------------------------------------------

    byte send_configuration_enable_bytes[ld2411_send_conf_enable_bytes_len] = {0x04,
                                                                               0x00,
                                                                               0xFF,
                                                                               0x00,
                                                                               0x01,
                                                                               0x00
                                                                              };

    byte send_conf_enable_bytes[ld2411_send_begin_bytes_len +
                                ld2411_send_conf_enable_bytes_len +
                                ld2411_send_end_bytes_len];

    byte recv_configuration_enable_bytes[ld2411_recv_conf_enable_bytes_len] = {0x08,
                                                                               0x00,
                                                                               0xFF,
                                                                               0x01,
                                                                               0x00,
                                                                               0x00,
                                                                               0x01,
                                                                               0x00,
                                                                               0x40,
                                                                               0x00
                                                                              };

    byte recv_conf_enable_bytes[ld2411_send_begin_bytes_len +
                                ld2411_recv_conf_enable_bytes_len +
                                ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_configuration_end_bytes[ld2411_send_conf_end_bytes_len] = {0x02,
                                                                         0x00,
                                                                         0xFE,
                                                                         0x00
                                                                        };

    byte send_conf_end_bytes[ld2411_send_begin_bytes_len +
                             ld2411_send_conf_end_bytes_len +
                             ld2411_send_end_bytes_len];

    byte recv_configuration_end_bytes[ld2411_recv_conf_end_bytes_len] = {0x04,
                                                                         0x00,
                                                                         0xFE,
                                                                         0x01,
                                                                         0x00,
                                                                         0x00
                                                                        };

    byte recv_conf_end_bytes[ld2411_send_begin_bytes_len +
                             ld2411_recv_conf_end_bytes_len +
                             ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_read_parameters_bytes[ld2411_send_read_params_bytes_len] = {0x02,
                                                                          0x00,
                                                                          0x73,
                                                                          0x00
                                                                         };

    byte send_read_params_bytes[ld2411_send_begin_bytes_len +
                                ld2411_send_read_params_bytes_len +
                                ld2411_send_end_bytes_len];

    byte recv_read_params_bytes[ld2411_send_begin_bytes_len +
                                ld2411_recv_read_params_bytes_len +
                                ld2411_send_end_bytes_len];

    struct ld2411_read_params_data {
      uint16_t motion_range_min;
      uint16_t motion_range_max;
      uint16_t micromotion_range_min;
      uint16_t micromotion_range_max;
      uint16_t no_one_duration;
    } params_datas;

    //----------------------------------------------------------------------------------

    byte send_set_params_bytes[ld2411_send_begin_bytes_len +
                               ld2411_send_set_params_bytes_len +
                               ld2411_send_end_bytes_len];

    uint16_t send_set_params_length = 0x0020;
    uint16_t send_set_params_command = 0x0067;
    uint16_t send_set_params_reversed = 0x0000;
    uint16_t send_set_params_fixed = 1;

    byte recv_set_parameters_bytes[ld2411_recv_set_params_bytes_len] = {0x04,
                                                                        0x00,
                                                                        0x67,
                                                                        0x01,
                                                                        0x00,
                                                                        0x00
                                                                       };

    byte recv_set_params_bytes[ld2411_send_begin_bytes_len +
                               ld2411_recv_set_params_bytes_len +
                               ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_read_firmware_version_bytes[ld2411_send_read_firm_ver_bytes_len] = {0x02,
                                                                                  0x00,
                                                                                  0x00,
                                                                                  0x00
                                                                                 };

    byte send_read_firm_ver_bytes[ld2411_send_begin_bytes_len +
                                  ld2411_send_read_firm_ver_bytes_len +
                                  ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_open_bluetooth_bytes[ld2411_send_open_close_blt_bytes_len] = {0x04,
                                                                            0x00,
                                                                            0xA4,
                                                                            0x00,
                                                                            0x01,
                                                                            0x00
                                                                           };

    byte send_open_blt_bytes[ld2411_send_begin_bytes_len +
                             ld2411_send_open_close_blt_bytes_len +
                             ld2411_send_end_bytes_len];

    byte send_close_bluetooth_bytes[ld2411_send_open_close_blt_bytes_len] = {0x04,
                                                                             0x00,
                                                                             0xA4,
                                                                             0x00,
                                                                             0x00,
                                                                             0x00
                                                                            };

    byte send_close_blt_bytes[ld2411_send_begin_bytes_len +
                              ld2411_send_open_close_blt_bytes_len +
                              ld2411_send_end_bytes_len];

    byte recv_open_close_bluetooth_bytes[ld2411_recv_open_close_blt_bytes_len] = {0x04,
                                                                                  0x00,
                                                                                  0xA4,
                                                                                  0x01,
                                                                                  0x00,
                                                                                  0x00
                                                                                 };

    byte recv_open_close_blt_bytes[ld2411_send_begin_bytes_len +
                                   ld2411_recv_open_close_blt_bytes_len +
                                   ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_get_mac_address_bytes[ld2411_send_get_mac_addr_bytes_len] = {0x04,
                                                                           0x00,
                                                                           0xA5,
                                                                           0x00,
                                                                           0x01,
                                                                           0x00
                                                                          };

    byte send_get_mac_addr_bytes[ld2411_send_begin_bytes_len +
                                 ld2411_send_get_mac_addr_bytes_len +
                                 ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_reboot_module_bytes[ld2411_send_reboot_bytes_len] = {0x02,
                                                                   0x00,
                                                                   0xA3,
                                                                   0x00
                                                                  };

    byte send_reboot_bytes[ld2411_send_begin_bytes_len +
                           ld2411_send_reboot_bytes_len +
                           ld2411_send_end_bytes_len];

    byte recv_reboot_module_bytes[ld2411_recv_reboot_bytes_len] = {0x04,
                                                                   0x00,
                                                                   0xA3,
                                                                   0x01,
                                                                   0x00,
                                                                   0x00
                                                                  };

    byte recv_reboot_bytes[ld2411_send_begin_bytes_len +
                           ld2411_recv_reboot_bytes_len +
                           ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_factory_reset_bytes[ld2411_send_reset_bytes_len] = {0x02,
                                                                  0x00,
                                                                  0xA2,
                                                                  0x00
                                                                 };

    byte send_reset_bytes[ld2411_send_begin_bytes_len +
                          ld2411_send_reset_bytes_len +
                          ld2411_send_end_bytes_len];


    byte recv_factory_reset_bytes[ld2411_recv_reset_bytes_len] = {0x04,
                                                                  0x00,
                                                                  0xA2,
                                                                  0x01,
                                                                  0x00,
                                                                  0x00
                                                                 };

    byte recv_reset_bytes[ld2411_send_begin_bytes_len +
                          ld2411_recv_reset_bytes_len +
                          ld2411_send_end_bytes_len];

    //----------------------------------------------------------------------------------

    byte send_set_baud_bytes[ld2411_send_begin_bytes_len +
                             ld2411_send_set_baud_bytes_len +
                             ld2411_send_end_bytes_len];

    byte recv_set_baudrate_bytes[ld2411_recv_set_baud_bytes_len] = {0x04,
                                                                    0x00,
                                                                    0xA1,
                                                                    0x01,
                                                                    0x00,
                                                                    0x00
                                                                   };

    byte recv_set_baud_bytes[ld2411_send_begin_bytes_len +
                             ld2411_recv_set_baud_bytes_len +
                             ld2411_send_end_bytes_len];
};

#endif
