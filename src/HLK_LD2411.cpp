#include "HLK_LD2411.h"

#ifndef ESP32

#ifndef _BOARD_GENERIC_STM32F103C_H_

HLK_LD2411::HLK_LD2411(SoftwareSerial *ssUart) {
  is_soft = true;
  with_pins = false;
  with_ser = true;
  SS = ssUart;
}

#endif

#else

HLK_LD2411::HLK_LD2411(EspSoftwareSerial::UART *ssEspUart) {
  is_soft = true;
  with_pins = false;
  with_ser = true;
  SS = ssEspUart;
}

HLK_LD2411::HLK_LD2411(uint8_t rx, uint8_t tx, EspSoftwareSerial::UART *ssEspUart) {
  is_soft = true;
  with_pins = true;
  with_ser = true;
  rx_pin = rx;
  tx_pin = tx;
  SS = ssEspUart;
}

HLK_LD2411::HLK_LD2411(uint8_t rx, uint8_t tx, HardwareSerial *hsUart) {
  is_soft = false;
  with_pins = true;
  with_ser = true;
  rx_pin = rx;
  tx_pin = tx;
  HS = hsUart;
}

#endif

HLK_LD2411::HLK_LD2411(HardwareSerial *hsUart) {
  is_soft = false;
  with_pins = false;
  with_ser = true;
  HS = hsUart;
}

HLK_LD2411::HLK_LD2411(uint8_t rx, uint8_t tx) {
  is_soft = true;
  with_pins = true;
  with_ser = false;
  rx_pin = rx;
  tx_pin = tx;
}

void HLK_LD2411::begin(uint32_t baud) {

#ifndef ESP32

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {
    if (not with_pins)
      SS->begin(baud);
    else {
      SoftwareSerial ssUart(rx_pin, tx_pin);
      SS = &ssUart;
      SS->begin(baud);
    }
  }

#endif

  if (not is_soft)
    HS->begin(baud);

#else

  if (is_soft) {
    if (not with_pins)
      SS->begin(baud);
    else {
      if (with_ser)
        SS->begin(baud, EspSoftwareSerial::SWSERIAL_8N1, rx_pin, tx_pin);
      else {
        EspSoftwareSerial::UART ssUart(rx_pin, tx_pin);
        SS = &ssUart;
        SS->begin(baud);
      }
    }
  }

  else {
    if (not with_pins)
      HS->begin(baud);
    else
      HS->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  }

#endif

  for (int i = 0; i < ld2411_begin_bytes_len; i++) {
    begin_bytes_sum += begin_bytes[i] * (1 << 8 * i); // 1 << 8 * i
  }
  for (int i = 0; i < ld2411_end_bytes_len; i++) {
    end_bytes_sum += end_bytes[i] * (1 << 8 * i);
  }

  for (int i = 0; i < ld2411_send_begin_bytes_len; i++) {
    send_begin_bytes_sum += send_begin_bytes[i] * (1 << 8 * i);
  }
  for (int i = 0; i < ld2411_send_end_bytes_len; i++) {
    send_end_bytes_sum += send_end_bytes[i] * (1 << 8 * i);
  }

  //----------------------------------------------------------------------------------

  memcpy(send_conf_enable_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_conf_enable_bytes +
         ld2411_send_begin_bytes_len, send_configuration_enable_bytes, ld2411_send_conf_enable_bytes_len);
  memcpy(send_conf_enable_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_conf_enable_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  memcpy(recv_conf_enable_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_conf_enable_bytes +
         ld2411_send_begin_bytes_len, recv_configuration_enable_bytes, ld2411_recv_conf_enable_bytes_len);
  memcpy(recv_conf_enable_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_conf_enable_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_conf_end_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_conf_end_bytes +
         ld2411_send_begin_bytes_len, send_configuration_end_bytes, ld2411_send_conf_end_bytes_len);
  memcpy(send_conf_end_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_conf_end_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  memcpy(recv_conf_end_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_conf_end_bytes +
         ld2411_send_begin_bytes_len, recv_configuration_end_bytes, ld2411_recv_conf_end_bytes_len);
  memcpy(recv_conf_end_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_conf_end_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_read_params_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_read_params_bytes +
         ld2411_send_begin_bytes_len, send_read_parameters_bytes, ld2411_send_read_params_bytes_len);
  memcpy(send_read_params_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_read_params_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_set_params_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_set_params_bytes +
         ld2411_send_begin_bytes_len, &send_set_params_length, 2);
  memcpy(send_set_params_bytes +
         ld2411_send_begin_bytes_len + 2, &send_set_params_command, 2);
  memcpy(send_set_params_bytes +
         ld2411_send_begin_bytes_len + 4, &send_set_params_reversed, 2);
  for (int i = 0; i < 4; i++) {
    memcpy(send_set_params_bytes +
           ld2411_send_begin_bytes_len + 8 + i * 6, &send_set_params_reversed, 2);
    memcpy(send_set_params_bytes +
           ld2411_send_begin_bytes_len + 10 + i * 6, &send_set_params_fixed, 2);
    send_set_params_fixed++;
  }
  memcpy(send_set_params_bytes +
         ld2411_send_begin_bytes_len + 30, &send_set_params_reversed, 2);
  memcpy(send_set_params_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_set_params_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(recv_set_params_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_set_params_bytes +
         ld2411_send_begin_bytes_len, recv_set_parameters_bytes, ld2411_recv_set_params_bytes_len);
  memcpy(recv_set_params_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_set_params_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_read_firm_ver_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_read_firm_ver_bytes +
         ld2411_send_begin_bytes_len, send_read_firmware_version_bytes, ld2411_send_read_firm_ver_bytes_len);
  memcpy(send_read_firm_ver_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_read_firm_ver_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_open_blt_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_open_blt_bytes +
         ld2411_send_begin_bytes_len, send_open_bluetooth_bytes, ld2411_send_open_close_blt_bytes_len);
  memcpy(send_open_blt_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_open_close_blt_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_close_blt_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_close_blt_bytes +
         ld2411_send_begin_bytes_len, send_close_bluetooth_bytes, ld2411_send_open_close_blt_bytes_len);
  memcpy(send_close_blt_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_open_close_blt_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(recv_open_close_blt_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_open_close_blt_bytes +
         ld2411_send_begin_bytes_len, recv_open_close_bluetooth_bytes, ld2411_recv_open_close_blt_bytes_len);
  memcpy(recv_open_close_blt_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_open_close_blt_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_get_mac_addr_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_get_mac_addr_bytes +
         ld2411_send_begin_bytes_len, send_get_mac_address_bytes, ld2411_send_get_mac_addr_bytes_len);
  memcpy(send_get_mac_addr_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_get_mac_addr_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_reboot_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_reboot_bytes +
         ld2411_send_begin_bytes_len, send_reboot_module_bytes, ld2411_send_reboot_bytes_len);
  memcpy(send_reboot_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_reboot_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(recv_reboot_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_reboot_bytes +
         ld2411_send_begin_bytes_len, recv_reboot_module_bytes, ld2411_recv_reboot_bytes_len);
  memcpy(recv_reboot_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_reboot_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_reset_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(send_reset_bytes +
         ld2411_send_begin_bytes_len, send_factory_reset_bytes, ld2411_send_reset_bytes_len);
  memcpy(send_reset_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_reset_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(recv_reset_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_reset_bytes +
         ld2411_send_begin_bytes_len, recv_factory_reset_bytes, ld2411_recv_reset_bytes_len);
  memcpy(recv_reset_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_reset_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(send_set_baud_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  byte send_set_baud_part_bytes[4] = {0x04, 0x00, 0xA1, 0x00};
  memcpy(send_set_baud_bytes +
         ld2411_send_begin_bytes_len, send_set_baud_part_bytes, sizeof(send_set_baud_part_bytes));
  send_set_baud_bytes[ld2411_send_begin_bytes_len + 5] = 0;
  memcpy(send_set_baud_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_send_set_baud_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  memcpy(recv_set_baud_bytes, send_begin_bytes, ld2411_send_begin_bytes_len);
  memcpy(recv_set_baud_bytes +
         ld2411_send_begin_bytes_len, recv_set_baudrate_bytes, ld2411_recv_set_baud_bytes_len);
  memcpy(recv_set_baud_bytes +
         ld2411_send_begin_bytes_len +
         ld2411_recv_set_baud_bytes_len, send_end_bytes, ld2411_send_end_bytes_len);

  //----------------------------------------------------------------------------------

  if (conf_enabled) endConfiguration();
}

void HLK_LD2411::read() {

  if (conf_enabled) endConfiguration();

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    byte data = 0;

    while (data != begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    all_datas[0] = data;

    SS->readBytes(all_datas + 1, ld2411_all_datas_len - 1);

    uint16_t begin_bytes_sum_calc;
    uint16_t end_bytes_sum_calc;

    memcpy(&begin_bytes_sum_calc, all_datas, ld2411_begin_bytes_len);
    memcpy(&end_bytes_sum_calc, all_datas + ld2411_begin_bytes_len + ld2411_values_len, ld2411_end_bytes_len);

    if (begin_bytes_sum_calc == begin_bytes_sum and end_bytes_sum_calc == end_bytes_sum) {

      uint8_t data_type = all_datas[ld2411_begin_bytes_len];

      switch (data_type) {
        case 0:
          ld2411_data.campaign_target = 0;
          ld2411_data.micromotion_target = 0;
          break;
        case 1:
          memcpy(&ld2411_data.campaign_target, all_datas + ld2411_begin_bytes_len + 1, 2);
          break;
        case 2:
          memcpy(&ld2411_data.micromotion_target, all_datas + ld2411_begin_bytes_len + 1, 2);
          break;
      }

      //if (data_type == 1)
      //memcpy(&ld2411_data.campaign_target, all_datas + ld2411_begin_bytes_len + 1, 2);
      //ld2411_data.campaign_target = all_datas[ld2411_begin_bytes_len + 2] * 0x100 + all_datas[ld2411_begin_bytes_len + 1];

      //else if (data_type == 2)
      //memcpy(&ld2411_data.micromotion_target, all_datas + ld2411_begin_bytes_len + 1, 2);
      //ld2411_data.micromotion_target = all_datas[ld2411_begin_bytes_len + 2] * 0x100 + all_datas[ld2411_begin_bytes_len + 1];
    }
  }

#endif

  if (not is_soft) {

    byte data = 0;

    while (data != begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    all_datas[0] = data;

    HS->readBytes(all_datas + 1, ld2411_all_datas_len - 1);

    uint16_t begin_bytes_sum_calc;
    uint16_t end_bytes_sum_calc;

    memcpy(&begin_bytes_sum_calc, all_datas, ld2411_begin_bytes_len);
    memcpy(&end_bytes_sum_calc, all_datas + ld2411_begin_bytes_len + ld2411_values_len, ld2411_end_bytes_len);

    if (begin_bytes_sum_calc == begin_bytes_sum and end_bytes_sum_calc == end_bytes_sum) {

      uint8_t data_type = all_datas[ld2411_begin_bytes_len];

      if (data_type == 1)
        memcpy(&ld2411_data.campaign_target, all_datas + ld2411_begin_bytes_len + 1, 2);
      //ld2411_data.campaign_target = all_datas[ld2411_begin_bytes_len + 2] * 0x100 + all_datas[ld2411_begin_bytes_len + 1];

      else if (data_type == 2)
        memcpy(&ld2411_data.micromotion_target, all_datas + ld2411_begin_bytes_len + 1, 2);
      //ld2411_data.micromotion_target = all_datas[ld2411_begin_bytes_len + 2] * 0x100 + all_datas[ld2411_begin_bytes_len + 1];
    }
  }
}

uint16_t HLK_LD2411::getCampaignTarget() {
  return ld2411_data.campaign_target;
}

uint16_t HLK_LD2411::getMicromotionTarget() {
  return ld2411_data.micromotion_target;
}

bool HLK_LD2411::enableConfiguration() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    SS->write(send_conf_enable_bytes, sizeof(send_conf_enable_bytes));

    byte recv_conf_enable_bytes_calc[ld2411_send_begin_bytes_len +
                                     ld2411_recv_conf_enable_bytes_len +
                                     ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_conf_enable_bytes_calc[0] = data;

    SS->readBytes(recv_conf_enable_bytes_calc + 1, sizeof(recv_conf_enable_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_conf_enable_bytes); i++) {
      if (recv_conf_enable_bytes_calc[i] != recv_conf_enable_bytes[i]) {
        all_true = false;
        break;
      }
    }

    conf_enabled = true;

    return all_true;
  }

#endif

  if (not is_soft) {

    HS->write(send_conf_enable_bytes, sizeof(send_conf_enable_bytes));

    byte recv_conf_enable_bytes_calc[ld2411_send_begin_bytes_len +
                                     ld2411_recv_conf_enable_bytes_len +
                                     ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_conf_enable_bytes_calc[0] = data;

    HS->readBytes(recv_conf_enable_bytes_calc + 1, sizeof(recv_conf_enable_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_conf_enable_bytes); i++) {
      if (recv_conf_enable_bytes_calc[i] != recv_conf_enable_bytes[i]) {
        all_true = false;
        break;
      }
    }

    conf_enabled = true;

    return all_true;
  }
}

bool HLK_LD2411::endConfiguration() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    SS->write(send_conf_end_bytes, sizeof(send_conf_end_bytes));

    byte recv_conf_end_bytes_calc[ld2411_send_begin_bytes_len +
                                  ld2411_recv_conf_end_bytes_len +
                                  ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_conf_end_bytes_calc[0] = data;

    SS->readBytes(recv_conf_end_bytes_calc + 1, sizeof(recv_conf_end_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_conf_end_bytes); i++) {
      if (recv_conf_end_bytes_calc[i] != recv_conf_end_bytes[i]) {
        all_true = false;
        break;
      }
    }

    conf_enabled = false;

    return all_true;
  }

#endif

  if (not is_soft) {

    HS->write(send_conf_end_bytes, sizeof(send_conf_end_bytes));

    byte recv_conf_end_bytes_calc[ld2411_send_begin_bytes_len +
                                  ld2411_recv_conf_end_bytes_len +
                                  ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_conf_end_bytes_calc[0] = data;

    HS->readBytes(recv_conf_end_bytes_calc + 1, sizeof(recv_conf_end_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_conf_end_bytes); i++) {
      //Serial.print("0x");
      //Serial.println(recv_conf_end_bytes_calc[i], HEX);
      if (recv_conf_end_bytes_calc[i] != recv_conf_end_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-----------------------------------------------------------");

    conf_enabled = false;

    return all_true;
  }
}

void HLK_LD2411::readParameters() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    SS->write(send_read_params_bytes, sizeof(send_read_params_bytes));

    byte recv_read_params_bytes_calc[ld2411_send_begin_bytes_len +
                                     ld2411_recv_read_params_bytes_len +
                                     ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_read_params_bytes_calc[0] = data;

    SS->readBytes(recv_read_params_bytes_calc + 1, sizeof(recv_read_params_bytes_calc) - 1);

    uint32_t send_begin_bytes_sum_calc;
    uint32_t send_end_bytes_sum_calc;

    memcpy(&send_begin_bytes_sum_calc, recv_read_params_bytes_calc, ld2411_send_begin_bytes_len);
    memcpy(&send_end_bytes_sum_calc, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + ld2411_recv_read_params_bytes_len, ld2411_send_end_bytes_len);

    if (send_begin_bytes_sum_calc == send_begin_bytes_sum and send_end_bytes_sum_calc == send_end_bytes_sum) {

      memcpy(&params_datas.motion_range_max, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 6, 2);
      memcpy(&params_datas.motion_range_min, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 8, 2);

      memcpy(&params_datas.micromotion_range_max, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 10, 2);
      memcpy(&params_datas.micromotion_range_min, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 12, 2);

      memcpy(&params_datas.no_one_duration, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 14, 2);
    }

    if (conf_enabled) endConfiguration();
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    HS->write(send_read_params_bytes, sizeof(send_read_params_bytes));

    byte recv_read_params_bytes_calc[ld2411_send_begin_bytes_len +
                                     ld2411_recv_read_params_bytes_len +
                                     ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_read_params_bytes_calc[0] = data;

    HS->readBytes(recv_read_params_bytes_calc + 1, sizeof(recv_read_params_bytes_calc) - 1);

    uint32_t send_begin_bytes_sum_calc;
    uint32_t send_end_bytes_sum_calc;

    memcpy(&send_begin_bytes_sum_calc, recv_read_params_bytes_calc, ld2411_send_begin_bytes_len);
    memcpy(&send_end_bytes_sum_calc, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + ld2411_recv_read_params_bytes_len, ld2411_send_end_bytes_len);

    if (send_begin_bytes_sum_calc == send_begin_bytes_sum and send_end_bytes_sum_calc == send_end_bytes_sum) {

      memcpy(&params_datas.motion_range_max, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 6, 2);
      memcpy(&params_datas.motion_range_min, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 8, 2);

      memcpy(&params_datas.micromotion_range_max, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 10, 2);
      memcpy(&params_datas.micromotion_range_min, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 12, 2);

      memcpy(&params_datas.no_one_duration, recv_read_params_bytes_calc + ld2411_send_begin_bytes_len + 14, 2);
    }

    if (conf_enabled) endConfiguration();
  }
}

uint16_t HLK_LD2411::getMotionRangeMin() {
  return params_datas.motion_range_min;
}

uint16_t HLK_LD2411::getMotionRangeMax() {
  return params_datas.motion_range_max;
}

uint16_t HLK_LD2411::getMicromotionRangeMin() {
  return params_datas.micromotion_range_min;
}

uint16_t HLK_LD2411::getMicromotionRangeMax() {
  return params_datas.micromotion_range_max;
}

uint16_t HLK_LD2411::getNoOneDuration() {
  return params_datas.no_one_duration;
}

bool HLK_LD2411::setParameters(uint16_t motion_range_min,
                               uint16_t motion_range_max,
                               uint16_t micromotion_range_min,
                               uint16_t micromotion_range_max,
                               uint16_t no_one_duration) {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 6, &motion_range_max, 2);
    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 12, &motion_range_min, 2);

    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 18, &micromotion_range_max, 2);
    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 24, &micromotion_range_min, 2);

    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 30, &no_one_duration, 2);

    SS->write(send_set_params_bytes, sizeof(send_set_params_bytes));

    byte recv_set_params_bytes_calc[ld2411_send_begin_bytes_len +
                                    ld2411_recv_set_params_bytes_len +
                                    ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_set_params_bytes_calc[0] = data;

    SS->readBytes(recv_set_params_bytes_calc + 1, sizeof(recv_set_params_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_set_params_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_set_params_bytes_calc[i], HEX);
      if (recv_set_params_bytes_calc[i] != recv_set_params_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 6, &motion_range_max, 2);
    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 12, &motion_range_min, 2);

    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 18, &micromotion_range_max, 2);
    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 24, &micromotion_range_min, 2);

    memcpy(send_set_params_bytes + ld2411_send_begin_bytes_len + 30, &no_one_duration, 2);

    HS->write(send_set_params_bytes, sizeof(send_set_params_bytes));

    byte recv_set_params_bytes_calc[ld2411_send_begin_bytes_len +
                                    ld2411_recv_set_params_bytes_len +
                                    ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_set_params_bytes_calc[0] = data;

    HS->readBytes(recv_set_params_bytes_calc + 1, sizeof(recv_set_params_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_set_params_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_set_params_bytes_calc[i], HEX);
      if (recv_set_params_bytes_calc[i] != recv_set_params_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }
}

String HLK_LD2411::getFirmwareVersion() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    for (int i = 0; i < sizeof(send_read_firm_ver_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_read_firm_ver_bytes[i], HEX);
    }

    Serial.println("************************************************");

    HS->write(send_read_firm_ver_bytes, sizeof(send_read_firm_ver_bytes));

    delay(1000);

    byte recv_read_firm_ver_bytes_calc[ld2411_send_begin_bytes_len +
                                       ld2411_recv_read_firm_ver_bytes_len +
                                       ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_read_firm_ver_bytes_calc[0] = data;

    HS->readBytes(recv_read_firm_ver_bytes_calc + 1, sizeof(recv_read_firm_ver_bytes_calc) - 1);

    for (int i = 0; i < sizeof(recv_read_firm_ver_bytes_calc); i++) {
      Serial.print("0x");
      Serial.println(recv_read_firm_ver_bytes_calc[i], HEX);
    }

    Serial.println("----------------------------------------------------");

    uint32_t send_begin_bytes_sum_calc;
    uint32_t send_end_bytes_sum_calc;

    memcpy(&send_begin_bytes_sum_calc, recv_read_firm_ver_bytes_calc, ld2411_send_begin_bytes_len);
    memcpy(&send_end_bytes_sum_calc, recv_read_firm_ver_bytes_calc + ld2411_send_begin_bytes_len + ld2411_recv_read_firm_ver_bytes_len, ld2411_send_end_bytes_len);

    if (send_begin_bytes_sum_calc == send_begin_bytes_sum and send_end_bytes_sum_calc == send_end_bytes_sum) {

      String version = "V";

      uint8_t ver0 = recv_read_firm_ver_bytes_calc[ld2411_send_begin_bytes_len + 7];
      version += String(ver0);

      version += ".";

      uint8_t ver1 = recv_read_firm_ver_bytes_calc[ld2411_send_begin_bytes_len + 8];
      version += ver1 < 16 ? "0" : "" + String(ver1);

      version += ".";

      uint8_t ver2 = recv_read_firm_ver_bytes_calc[ld2411_send_begin_bytes_len + 12];
      version += ver2 < 16 ? "0" : "" + String(ver1);

      if (conf_enabled) endConfiguration();

      return version;
    }

    else {
      if (conf_enabled) endConfiguration();
      return "V0";
    }
  }
}

bool HLK_LD2411::openBluetooth() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    SS->write(send_open_blt_bytes, sizeof(send_open_blt_bytes));

    byte recv_open_close_blt_bytes_calc[ld2411_send_begin_bytes_len +
                                        ld2411_recv_open_close_blt_bytes_len +
                                        ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_open_close_blt_bytes_calc[0] = data;

    SS->readBytes(recv_open_close_blt_bytes_calc + 1, sizeof(recv_open_close_blt_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_open_close_blt_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_open_close_blt_bytes_calc[i], HEX);
      if (recv_open_close_blt_bytes_calc[i] != recv_open_close_blt_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    HS->write(send_open_blt_bytes, sizeof(send_open_blt_bytes));

    byte recv_open_close_blt_bytes_calc[ld2411_send_begin_bytes_len +
                                        ld2411_recv_open_close_blt_bytes_len +
                                        ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_open_close_blt_bytes_calc[0] = data;

    HS->readBytes(recv_open_close_blt_bytes_calc + 1, sizeof(recv_open_close_blt_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_open_close_blt_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_open_close_blt_bytes_calc[i], HEX);
      if (recv_open_close_blt_bytes_calc[i] != recv_open_close_blt_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }
}

bool HLK_LD2411::closeBluetooth() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    SS->write(send_close_blt_bytes, sizeof(send_close_blt_bytes));

    byte recv_open_close_blt_bytes_calc[ld2411_send_begin_bytes_len +
                                        ld2411_recv_open_close_blt_bytes_len +
                                        ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_open_close_blt_bytes_calc[0] = data;

    SS->readBytes(recv_open_close_blt_bytes_calc + 1, sizeof(recv_open_close_blt_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_open_close_blt_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_set_params_bytes_calc[i], HEX);
      if (recv_open_close_blt_bytes_calc[i] != recv_open_close_blt_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    HS->write(send_close_blt_bytes, sizeof(send_close_blt_bytes));

    byte recv_open_close_blt_bytes_calc[ld2411_send_begin_bytes_len +
                                        ld2411_recv_open_close_blt_bytes_len +
                                        ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_open_close_blt_bytes_calc[0] = data;

    HS->readBytes(recv_open_close_blt_bytes_calc + 1, sizeof(recv_open_close_blt_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_open_close_blt_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_set_params_bytes_calc[i], HEX);
      if (recv_open_close_blt_bytes_calc[i] != recv_open_close_blt_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }
}

String HLK_LD2411::getMacAddress() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    SS->write(send_get_mac_addr_bytes, sizeof(send_get_mac_addr_bytes));

    byte recv_get_mac_addr_bytes_calc[ld2411_send_begin_bytes_len +
                                      ld2411_recv_get_mac_addr_bytes_len +
                                      ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_get_mac_addr_bytes_calc[0] = data;

    SS->readBytes(recv_get_mac_addr_bytes_calc + 1, sizeof(recv_get_mac_addr_bytes_calc) - 1);

    uint32_t send_begin_bytes_sum_calc;
    uint32_t send_end_bytes_sum_calc;

    memcpy(&send_begin_bytes_sum_calc, recv_get_mac_addr_bytes_calc, ld2411_send_begin_bytes_len);
    memcpy(&send_end_bytes_sum_calc, recv_get_mac_addr_bytes_calc + ld2411_send_begin_bytes_len + ld2411_recv_get_mac_addr_bytes_len, ld2411_send_end_bytes_len);

    if (send_begin_bytes_sum_calc == send_begin_bytes_sum and send_end_bytes_sum_calc == send_end_bytes_sum) {

      String mac = "";

      for (int i = ld2411_send_begin_bytes_len + 6; i < ld2411_send_begin_bytes_len + 11; i++) {
        char mac_chr[3];
        sprintf(mac_chr, "%X", recv_get_mac_addr_bytes_calc[i]);
        uint8_t mac_int;
        sscanf(mac_chr, "%X", &mac_int);
        mac += (mac_int < 0x10 ? "0" : "") + String(mac_int);
        mac += ":";
      }

      char mac_chr[3];
      sprintf(mac_chr, "%X", recv_get_mac_addr_bytes_calc[ld2411_send_begin_bytes_len + 11]);
      uint8_t mac_int;
      sscanf(mac_chr, "%X", &mac_int);
      mac += (mac_int < 0x10 ? "0" : "") + String(mac_int);

      if (conf_enabled) endConfiguration();

      return mac;
    }

    else {
      if (conf_enabled) endConfiguration();
      return "00:00:00:00:00:00";
    }
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    HS->write(send_get_mac_addr_bytes, sizeof(send_get_mac_addr_bytes));

    /*for (int i=0; i<sizeof(send_get_mac_addr_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_get_mac_addr_bytes[i], HEX);
      }

      Serial.println("------------------------------------------");*/

    byte recv_get_mac_addr_bytes_calc[ld2411_send_begin_bytes_len +
                                      ld2411_recv_get_mac_addr_bytes_len +
                                      ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_get_mac_addr_bytes_calc[0] = data;

    HS->readBytes(recv_get_mac_addr_bytes_calc + 1, sizeof(recv_get_mac_addr_bytes_calc) - 1);

    /*for (int i=0; i<sizeof(recv_get_mac_addr_bytes_calc); i++) {
      Serial.print("0x");
      Serial.println(recv_get_mac_addr_bytes_calc[i], HEX);
      }

      Serial.println("------------------------------------------");*/

    uint32_t send_begin_bytes_sum_calc;
    uint32_t send_end_bytes_sum_calc;

    memcpy(&send_begin_bytes_sum_calc, recv_get_mac_addr_bytes_calc, ld2411_send_begin_bytes_len);
    memcpy(&send_end_bytes_sum_calc, recv_get_mac_addr_bytes_calc + ld2411_send_begin_bytes_len + ld2411_recv_get_mac_addr_bytes_len, ld2411_send_end_bytes_len);

    if (send_begin_bytes_sum_calc == send_begin_bytes_sum and send_end_bytes_sum_calc == send_end_bytes_sum) {

      String mac = "";

      for (int i = ld2411_send_begin_bytes_len + 6; i < ld2411_send_begin_bytes_len + 11; i++) {
        char mac_chr[3];
        sprintf(mac_chr, "%X", recv_get_mac_addr_bytes_calc[i]);
        uint8_t mac_int;
        sscanf(mac_chr, "%X", &mac_int);
        mac += (mac_int < 0x10 ? "0" : "") + String(mac_int);
        mac += ":";
      }

      char mac_chr[3];
      sprintf(mac_chr, "%X", recv_get_mac_addr_bytes_calc[ld2411_send_begin_bytes_len + 11]);
      uint8_t mac_int;
      sscanf(mac_chr, "%X", &mac_int);
      mac += (mac_int < 0x10 ? "0" : "") + String(mac_int);

      if (conf_enabled) endConfiguration();

      return mac;
    }

    else {
      if (conf_enabled) endConfiguration();
      return "00:00:00:00:00:00";
    }
  }
}

bool HLK_LD2411::reboot() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    SS->write(send_reboot_bytes, sizeof(send_reboot_bytes));

    /*for (int i = 0; i < sizeof(send_reboot_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_reboot_bytes[i], HEX);
      }

      Serial.println("==============================================");*/

    byte recv_reboot_bytes_calc[ld2411_send_begin_bytes_len +
                                ld2411_recv_reboot_bytes_len +
                                ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_reboot_bytes_calc[0] = data;

    SS->readBytes(recv_reboot_bytes_calc + 1, sizeof(recv_reboot_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_reboot_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_reboot_bytes_calc[i], HEX);
      if (recv_reboot_bytes_calc[i] != recv_reboot_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    HS->write(send_reboot_bytes, sizeof(send_reboot_bytes));

    /*for (int i = 0; i < sizeof(send_reboot_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_reboot_bytes[i], HEX);
      }

      Serial.println("==============================================");*/

    byte recv_reboot_bytes_calc[ld2411_send_begin_bytes_len +
                                ld2411_recv_reboot_bytes_len +
                                ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_reboot_bytes_calc[0] = data;

    HS->readBytes(recv_reboot_bytes_calc + 1, sizeof(recv_reboot_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_reboot_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_reboot_bytes_calc[i], HEX);
      if (recv_reboot_bytes_calc[i] != recv_reboot_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }
}

bool HLK_LD2411::factoryReset() {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    SS->write(send_reset_bytes, sizeof(send_reset_bytes));

    /*for (int i = 0; i < sizeof(send_reboot_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_reboot_bytes[i], HEX);
      }

      Serial.println("==============================================");*/

    byte recv_reset_bytes_calc[ld2411_send_begin_bytes_len +
                               ld2411_recv_reset_bytes_len +
                               ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_reset_bytes_calc[0] = data;

    SS->readBytes(recv_reset_bytes_calc + 1, sizeof(recv_reset_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_reset_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_reboot_bytes_calc[i], HEX);
      if (recv_reset_bytes_calc[i] != recv_reset_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    HS->write(send_reset_bytes, sizeof(send_reset_bytes));

    /*for (int i = 0; i < sizeof(send_reboot_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_reboot_bytes[i], HEX);
      }

      Serial.println("==============================================");*/

    byte recv_reset_bytes_calc[ld2411_send_begin_bytes_len +
                               ld2411_recv_reset_bytes_len +
                               ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_reset_bytes_calc[0] = data;

    HS->readBytes(recv_reset_bytes_calc + 1, sizeof(recv_reset_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_reset_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_reboot_bytes_calc[i], HEX);
      if (recv_reset_bytes_calc[i] != recv_reset_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }
}

bool HLK_LD2411::setBaudrate(uint32_t baudrate) {

#ifndef _BOARD_GENERIC_STM32F103C_H_

  if (is_soft) {

    if (not conf_enabled) enableConfiguration();

    //send_set_baud_bytes[ld2411_send_begin_bytes_len + 5] = 0;

    switch (baudrate) {
      case 9600:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_9600;
        break;
      case 19200:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_19200;
        break;
      case 38400:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_38400;
        break;
      case 57600:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_57600;
        break;
      case 115200:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_115200;
        break;
      case 230400:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_230400;
        break;
      case 256000:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_256000;
        break;
      case 460800:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_460800;
        break;
      default:
        return false;
    }

    SS->write(send_set_baud_bytes, sizeof(send_set_baud_bytes));

    /*for (int i = 0; i < sizeof(send_set_baud_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_set_baud_bytes[i], HEX);
      }

      Serial.println("==============================================");*/

    byte recv_set_baud_bytes_calc[ld2411_send_begin_bytes_len +
                                  ld2411_recv_set_baud_bytes_len +
                                  ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (SS->available() > 0)
        data = SS->read();

    recv_set_baud_bytes_calc[0] = data;

    SS->readBytes(recv_set_baud_bytes_calc + 1, sizeof(recv_set_baud_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_set_baud_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_set_baud_bytes_calc[i], HEX);
      if (recv_set_baud_bytes_calc[i] != recv_set_baud_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }

#endif

  if (not is_soft) {

    if (not conf_enabled) enableConfiguration();

    //send_set_baud_bytes[ld2411_send_begin_bytes_len + 5] = 0;

    switch (baudrate) {
      case 9600:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_9600;
        break;
      case 19200:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_19200;
        break;
      case 38400:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_38400;
        break;
      case 57600:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_57600;
        break;
      case 115200:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_115200;
        break;
      case 230400:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_230400;
        break;
      case 256000:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_256000;
        break;
      case 460800:
        send_set_baud_bytes[ld2411_send_begin_bytes_len + 4] = LD2411_BAUD_460800;
        break;
      default:
        return false;
    }

    HS->write(send_set_baud_bytes, sizeof(send_set_baud_bytes));

    /*for (int i = 0; i < sizeof(send_set_baud_bytes); i++) {
      Serial.print("0x");
      Serial.println(send_set_baud_bytes[i], HEX);
      }

      Serial.println("==============================================");*/

    byte recv_set_baud_bytes_calc[ld2411_send_begin_bytes_len +
                                  ld2411_recv_set_baud_bytes_len +
                                  ld2411_send_end_bytes_len];

    byte data;

    while (data != send_begin_bytes[0])
      if (HS->available() > 0)
        data = HS->read();

    recv_set_baud_bytes_calc[0] = data;

    HS->readBytes(recv_set_baud_bytes_calc + 1, sizeof(recv_set_baud_bytes_calc) - 1);

    bool all_true = true;

    for (int i = 0; i < sizeof(recv_set_baud_bytes_calc); i++) {
      //Serial.print("0x");
      //Serial.println(recv_set_baud_bytes_calc[i], HEX);
      if (recv_set_baud_bytes_calc[i] != recv_set_baud_bytes[i]) {
        all_true = false;
        break;
      }
    }

    //Serial.println("-------------------------------------------------");

    if (conf_enabled) endConfiguration();

    return all_true;
  }
}
