#ifndef SI4684_COMMANDS_H
#define SI4684_COMMANDS_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// I2C Configuration
#define SI4684_I2C_ADDR         0x11
#define SI4684_I2C_TIMEOUT_MS   1000

// Common Commands (All Modes)
#define CMD_RD_REPLY            0x00
#define CMD_POWER_UP            0x01
#define CMD_HOST_LOAD           0x04
#define CMD_FLASH_LOAD          0x05
#define CMD_LOAD_INIT           0x06
#define CMD_BOOT                0x07
#define CMD_GET_PART_INFO       0x08
#define CMD_GET_SYS_STATE       0x09
#define CMD_GET_POWER_UP_ARGS   0x0A
#define CMD_READ_OFFSET         0x10
#define CMD_GET_FUNC_INFO       0x12
#define CMD_SET_PROPERTY        0x13
#define CMD_GET_PROPERTY        0x14
#define CMD_GET_AGC_STATUS      0x17

// FM/FMHD Commands (0x30-0x3F)
#define CMD_FM_TUNE_FREQ        0x30
#define CMD_FM_SEEK_START       0x31
#define CMD_FM_RSQ_STATUS       0x32
#define CMD_FM_ACF_STATUS       0x33
#define CMD_FM_RDS_STATUS       0x34
#define CMD_FM_RDS_BLOCKCOUNT   0x35

// AM/AMHD Commands (0x40-0x4F)
#define CMD_AM_TUNE_FREQ        0x40
#define CMD_AM_SEEK_START       0x41
#define CMD_AM_RSQ_STATUS       0x42
#define CMD_AM_ACF_STATUS       0x43

// Digital Service Commands (0x80-0x9F)
#define CMD_GET_DIGITAL_SERVICE_LIST    0x80
#define CMD_START_DIGITAL_SERVICE       0x81
#define CMD_STOP_DIGITAL_SERVICE        0x82
#define CMD_GET_DIGITAL_SERVICE_DATA    0x84

// HD Radio Commands (0x92-0x9C)
#define CMD_HD_DIGRAD_STATUS    0x92
#define CMD_HD_GET_EVENT_STATUS 0x93
#define CMD_HD_GET_STATION_INFO 0x94
#define CMD_HD_GET_PSD_DECODE   0x95
#define CMD_HD_GET_ALERT_MSG    0x96
#define CMD_HD_PLAY_ALERT_TONE  0x97
#define CMD_HD_TEST_GET_BER_INFO 0x98
#define CMD_HD_SET_ENABLED_PORTS 0x99
#define CMD_HD_GET_ENABLED_PORTS 0x9A
#define CMD_HD_ACF_STATUS       0x9C

// DAB Commands (0xB0-0xC2)
#define CMD_DAB_TUNE_FREQ       0xB0
#define CMD_DAB_DIGRAD_STATUS   0xB2
#define CMD_DAB_GET_EVENT_STATUS 0xB3
#define CMD_DAB_GET_ENSEMBLE_INFO 0xB4
#define CMD_DAB_GET_ANNOUNCEMENT_SUPPORT_INFO 0xB5
#define CMD_DAB_GET_ANNOUNCEMENT_INFO 0xB6
#define CMD_DAB_GET_SERVICE_LINKING_INFO 0xB7
#define CMD_DAB_SET_FREQ_LIST   0xB8
#define CMD_DAB_GET_FREQ_LIST   0xB9
#define CMD_DAB_GET_COMPONENT_INFO 0xBB
#define CMD_DAB_GET_TIME        0xBC
#define CMD_DAB_GET_AUDIO_INFO  0xBD
#define CMD_DAB_GET_SUBCHAN_INFO 0xBE
#define CMD_DAB_GET_FREQ_INFO   0xBF
#define CMD_DAB_GET_SERVICE_INFO 0xC0
#define CMD_DAB_GET_OE_SERVICES_INFO 0xC1
#define CMD_DAB_ACF_STATUS      0xC2

// Test Commands (0xE5-0xE8)
#define CMD_TEST_GET_RSSI       0xE5
#define CMD_DAB_TEST_GET_BER_INFO 0xE8

// Status Bits
#define STATUS_CTS              0x80
#define STATUS_ERR_CMD          0x40
#define STATUS_DACQINT          0x20
#define STATUS_DSRVINT          0x10
#define STATUS_RSQINT           0x08
#define STATUS_RDSINT           0x04
#define STATUS_ACFINT           0x02
#define STATUS_STCINT           0x01
#define STATUS_DEVNTINT         0x10
#define STATUS_DACFINT          0x01

// Power-up modes
#define CLKMODE_POWERED_DOWN    0x00
#define CLKMODE_CRYSTAL         0x01
#define CLKMODE_SINGLE_ENDED    0x02
#define CLKMODE_DIFFERENTIAL    0x03

// Image types
#define IMAGE_BOOTLOADER        0x00
#define IMAGE_FMHD              0x01
#define IMAGE_DAB               0x02
#define IMAGE_TDMB              0x03
#define IMAGE_FMHD_DEMOD        0x04
#define IMAGE_AMHD              0x05
#define IMAGE_AMHD_DEMOD        0x06
#define IMAGE_DAB_DEMOD         0x07

// Structure for SI4684 handle
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
} si4684_handle_t;

// Function Prototypes - Common Commands
esp_err_t si4684_init(si4684_handle_t *handle, i2c_port_t i2c_port, uint8_t addr);
esp_err_t si4684_write_command(si4684_handle_t *handle, const uint8_t *cmd, size_t cmd_len);
esp_err_t si4684_read_reply(si4684_handle_t *handle, uint8_t *reply, size_t reply_len);
esp_err_t si4684_wait_for_cts(si4684_handle_t *handle, uint32_t timeout_ms);
esp_err_t si4684_power_up(si4684_handle_t *handle, uint8_t clk_mode, uint32_t xtal_freq, 
                          uint8_t tr_size, uint8_t ibias, uint8_t ctun, uint8_t ibias_run);
esp_err_t si4684_load_init(si4684_handle_t *handle);
esp_err_t si4684_host_load(si4684_handle_t *handle, const uint8_t *data, size_t len);
esp_err_t si4684_flash_load(si4684_handle_t *handle, uint32_t flash_start_addr);
esp_err_t si4684_boot(si4684_handle_t *handle);
esp_err_t si4684_get_part_info(si4684_handle_t *handle, uint8_t *chip_rev, 
                               uint8_t *rom_id, uint16_t *part_num);
esp_err_t si4684_get_sys_state(si4684_handle_t *handle, uint8_t *image);
esp_err_t si4684_get_power_up_args(si4684_handle_t *handle, uint8_t *clk_mode, 
                                   uint32_t *xtal_freq, uint8_t *tr_size, 
                                   uint8_t *ibias, uint8_t *ctun, uint8_t *ibias_run);
esp_err_t si4684_read_offset(si4684_handle_t *handle, uint16_t offset, 
                             uint8_t *data, size_t len);
esp_err_t si4684_get_func_info(si4684_handle_t *handle, uint8_t *rev_ext, 
                               uint8_t *rev_branch, uint8_t *rev_int, uint32_t *svn_id);
esp_err_t si4684_set_property(si4684_handle_t *handle, uint16_t prop_id, uint16_t value);
esp_err_t si4684_get_property(si4684_handle_t *handle, uint16_t prop_id, uint16_t *value);
esp_err_t si4684_get_agc_status(si4684_handle_t *handle, uint8_t *rfagc_dis, 
                                uint8_t *ifagc_dis, uint8_t *lnagainidx);

// FM/FMHD Commands
esp_err_t si4684_fm_tune_freq(si4684_handle_t *handle, uint16_t freq, uint16_t ant_cap, 
                              uint8_t tune_mode, uint8_t injection);
esp_err_t si4684_fm_seek_start(si4684_handle_t *handle, uint8_t seek_up, uint8_t wrap);
esp_err_t si4684_fm_rsq_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *rssi, uint8_t *snr, uint16_t *freq, 
                               uint8_t *valid, uint8_t *afc_rail, uint8_t *valid_freq);
esp_err_t si4684_fm_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *blend_int, uint8_t *hicut_int, 
                               uint8_t *softmute_int, uint8_t *pilot);
esp_err_t si4684_fm_rds_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *recv, uint8_t *sync, uint16_t *pi, 
                               uint8_t *block_a_h, uint8_t *block_a_l);
esp_err_t si4684_fm_rds_blockcount(si4684_handle_t *handle, uint8_t clear, 
                                   uint16_t *expected, uint16_t *received, 
                                   uint16_t *uncorrectable);

// AM/AMHD Commands
esp_err_t si4684_am_tune_freq(si4684_handle_t *handle, uint16_t freq, uint16_t ant_cap, 
                              uint8_t tune_mode);
esp_err_t si4684_am_seek_start(si4684_handle_t *handle, uint8_t seek_up, uint8_t wrap);
esp_err_t si4684_am_rsq_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *rssi, uint8_t *snr, uint16_t *freq, 
                               uint8_t *valid);
esp_err_t si4684_am_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *softmute_int, uint8_t *hicut_int);

// Digital Service Commands
esp_err_t si4684_get_digital_service_list(si4684_handle_t *handle, uint8_t serv_idx_list, 
                                          uint16_t *list_size, uint8_t *service_list, 
                                          size_t max_list_len);
esp_err_t si4684_start_digital_service(si4684_handle_t *handle, uint32_t service_id, 
                                       uint32_t comp_id, uint8_t audio_data);
esp_err_t si4684_stop_digital_service(si4684_handle_t *handle, uint32_t service_id, 
                                      uint32_t comp_id);
esp_err_t si4684_get_digital_service_data(si4684_handle_t *handle, uint8_t int_ack, 
                                          uint8_t serv_idx_list, uint16_t *data_cnt_avail, 
                                          uint16_t *data_src, uint8_t *data, size_t max_data_len);

// HD Radio Commands
esp_err_t si4684_hd_digrad_status(si4684_handle_t *handle, uint8_t int_ack, uint8_t dig_acq_int_ack, 
                                  uint8_t *acq, uint8_t *daai, uint8_t *audio_prog);
esp_err_t si4684_hd_get_event_status(si4684_handle_t *handle, uint8_t int_ack, 
                                     uint16_t *event_avail, uint16_t *event_flags);
esp_err_t si4684_hd_get_station_info(si4684_handle_t *handle, char *short_name, 
                                     char *long_name, uint8_t *loc_cc, size_t name_len);
esp_err_t si4684_hd_get_psd_decode(si4684_handle_t *handle, uint8_t id_filter, 
                                   uint8_t prog_id, char *title, char *artist, 
                                   char *album, size_t max_str_len);
esp_err_t si4684_hd_get_alert_msg(si4684_handle_t *handle, uint8_t int_ack, 
                                  char *msg, size_t max_msg_len);
esp_err_t si4684_hd_play_alert_tone(si4684_handle_t *handle, uint8_t play);
esp_err_t si4684_hd_test_get_ber_info(si4684_handle_t *handle, uint8_t clear, 
                                      uint32_t *ber_count);
esp_err_t si4684_hd_set_enabled_ports(si4684_handle_t *handle, uint32_t port_mask);
esp_err_t si4684_hd_get_enabled_ports(si4684_handle_t *handle, uint32_t *port_mask);
esp_err_t si4684_hd_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *blend_int, uint8_t *audio_acq_int);

// DAB Commands
esp_err_t si4684_dab_tune_freq(si4684_handle_t *handle, uint8_t freq_index, 
                               uint16_t ant_cap, uint8_t injection);
esp_err_t si4684_dab_digrad_status(si4684_handle_t *handle, uint8_t int_ack, uint8_t dig_acq_int_ack, 
                                   uint8_t *acq, uint8_t *rssi, uint8_t *snr, 
                                   uint8_t *fic_quality, uint16_t *tune_freq);
esp_err_t si4684_dab_get_event_status(si4684_handle_t *handle, uint8_t int_ack, 
                                      uint16_t *event_avail, uint16_t *event_flags);
esp_err_t si4684_dab_get_ensemble_info(si4684_handle_t *handle, char *label, 
                                       uint16_t *ensemble_id, uint8_t *ensemble_ecc, 
                                       size_t max_label_len);
esp_err_t si4684_dab_get_announcement_support_info(si4684_handle_t *handle, uint32_t service_id, 
                                                   uint32_t comp_id, uint16_t *asw_flags);
esp_err_t si4684_dab_get_announcement_info(si4684_handle_t *handle, uint8_t buf_empty, 
                                           uint32_t *service_id, uint32_t *comp_id, 
                                           uint16_t *asw_flags);
esp_err_t si4684_dab_get_service_linking_info(si4684_handle_t *handle, uint32_t service_id, 
                                              uint8_t *link_info, size_t max_len);
esp_err_t si4684_dab_set_freq_list(si4684_handle_t *handle, uint8_t num_freqs, 
                                   const uint32_t *freq_list);
esp_err_t si4684_dab_get_freq_list(si4684_handle_t *handle, uint8_t *num_freqs, 
                                   uint32_t *freq_list, size_t max_freqs);
esp_err_t si4684_dab_get_component_info(si4684_handle_t *handle, uint32_t service_id, 
                                        uint8_t comp_idx, uint32_t *comp_id, 
                                        uint8_t *cmp_info);
esp_err_t si4684_dab_get_time(si4684_handle_t *handle, uint16_t *year, uint8_t *month, 
                              uint8_t *day, uint8_t *hour, uint8_t *minute, 
                              uint8_t *second);
esp_err_t si4684_dab_get_audio_info(si4684_handle_t *handle, uint8_t *bit_rate, 
                                    uint8_t *sample_rate, uint8_t *mode);
esp_err_t si4684_dab_get_subchan_info(si4684_handle_t *handle, uint32_t service_id, 
                                      uint32_t comp_id, uint8_t *subchan_id, 
                                      uint16_t *subchan_size);
esp_err_t si4684_dab_get_freq_info(si4684_handle_t *handle, uint8_t freq_index, 
                                   uint32_t *freq, uint8_t *control);
esp_err_t si4684_dab_get_service_info(si4684_handle_t *handle, uint32_t service_id, 
                                      char *label, uint8_t *pty, uint8_t *pd_flag, 
                                      size_t max_label_len);
esp_err_t si4684_dab_get_oe_services_info(si4684_handle_t *handle, uint32_t service_id, 
                                          uint8_t *oe_info, size_t max_len);
esp_err_t si4684_dab_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                                uint8_t *softmute_int);

// Test Commands
esp_err_t si4684_test_get_rssi(si4684_handle_t *handle, int16_t *rssi);
esp_err_t si4684_dab_test_get_ber_info(si4684_handle_t *handle, uint8_t clear, 
                                       uint32_t *ber_count, uint32_t *fib_error_count);

#endif // SI4684_COMMANDS_H
