#include "si4684_commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SI4684";

// ============================================================================
// COMMON FUNCTIONS
// ============================================================================

esp_err_t si4684_init(si4684_handle_t *handle, i2c_port_t i2c_port, uint8_t addr) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->i2c_port = i2c_port;
    handle->i2c_addr = addr;
    return ESP_OK;
}

esp_err_t si4684_write_command(si4684_handle_t *handle, const uint8_t *cmd, size_t cmd_len) {
    if (handle == NULL || cmd == NULL || cmd_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_write_to_device(handle->i2c_port, handle->i2c_addr, 
                                      cmd, cmd_len, pdMS_TO_TICKS(SI4684_I2C_TIMEOUT_MS));
}

esp_err_t si4684_read_reply(si4684_handle_t *handle, uint8_t *reply, size_t reply_len) {
    if (handle == NULL || reply == NULL || reply_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t cmd = CMD_RD_REPLY;
    esp_err_t ret = i2c_master_write_to_device(handle->i2c_port, handle->i2c_addr, 
                                                &cmd, 1, pdMS_TO_TICKS(SI4684_I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_read_from_device(handle->i2c_port, handle->i2c_addr, 
                                       reply, reply_len, pdMS_TO_TICKS(SI4684_I2C_TIMEOUT_MS));
}

esp_err_t si4684_wait_for_cts(si4684_handle_t *handle, uint32_t timeout_ms) {
    uint8_t status[4];
    uint32_t start_time = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms)) {
        esp_err_t ret = si4684_read_reply(handle, status, 4);
        if (ret != ESP_OK) {
            return ret;
        }
        
        if (status[0] & STATUS_CTS) {
            if (status[0] & STATUS_ERR_CMD) {
                ESP_LOGE(TAG, "Command error");
                return ESP_FAIL;
            }
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG, "Timeout waiting for CTS");
    return ESP_ERR_TIMEOUT;
}

// ============================================================================
// BOOTLOADER & INITIALIZATION COMMANDS
// ============================================================================

esp_err_t si4684_power_up(si4684_handle_t *handle, uint8_t clk_mode, uint32_t xtal_freq, 
                          uint8_t tr_size, uint8_t ibias, uint8_t ctun, uint8_t ibias_run) {
    uint8_t cmd[16] = {0};
    
    cmd[0] = CMD_POWER_UP;
    cmd[1] = 0x00;
    cmd[2] = (clk_mode << 4) | (tr_size & 0x0F);
    cmd[3] = ibias & 0x7F;
    cmd[4] = (xtal_freq >> 0) & 0xFF;
    cmd[5] = (xtal_freq >> 8) & 0xFF;
    cmd[6] = (xtal_freq >> 16) & 0xFF;
    cmd[7] = (xtal_freq >> 24) & 0xFF;
    cmd[8] = ctun & 0x3F;
    cmd[9] = 0x10;
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    cmd[12] = 0x00;
    cmd[13] = ibias_run & 0x7F;
    cmd[14] = 0x00;
    cmd[15] = 0x00;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 16);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_load_init(si4684_handle_t *handle) {
    uint8_t cmd[2] = {CMD_LOAD_INIT, 0x00};
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(3));
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_host_load(si4684_handle_t *handle, const uint8_t *data, size_t len) {
    if (len > 4096) {
        ESP_LOGE(TAG, "Host load data too large (max 4096 bytes)");
        return ESP_ERR_INVALID_SIZE;
    }
    
    uint8_t *cmd = malloc(len + 4);
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    cmd[0] = CMD_HOST_LOAD;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    memcpy(&cmd[4], data, len);
    
    esp_err_t ret = si4684_write_command(handle, cmd, len + 4);
    free(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_flash_load(si4684_handle_t *handle, uint32_t flash_start_addr) {
    uint8_t cmd[12] = {0};
    
    cmd[0] = CMD_FLASH_LOAD;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = (flash_start_addr >> 0) & 0xFF;
    cmd[5] = (flash_start_addr >> 8) & 0xFF;
    cmd[6] = (flash_start_addr >> 16) & 0xFF;
    cmd[7] = (flash_start_addr >> 24) & 0xFF;
    cmd[8] = 0x00;
    cmd[9] = 0x00;
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    return si4684_wait_for_cts(handle, 5000);
}

esp_err_t si4684_boot(si4684_handle_t *handle) {
    uint8_t cmd[2] = {CMD_BOOT, 0x00};
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
    return si4684_wait_for_cts(handle, 1000);
}

// ============================================================================
// INFORMATION QUERY COMMANDS
// ============================================================================

esp_err_t si4684_get_part_info(si4684_handle_t *handle, uint8_t *chip_rev, 
                                uint8_t *rom_id, uint16_t *part_num) {
    uint8_t cmd[2] = {CMD_GET_PART_INFO, 0x00};
    uint8_t reply[23];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 23);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (chip_rev) *chip_rev = reply[4];
    if (rom_id) *rom_id = reply[5];
    if (part_num) *part_num = (reply[9] << 8) | reply[8];
    
    return ESP_OK;
}

esp_err_t si4684_get_sys_state(si4684_handle_t *handle, uint8_t *image) {
    uint8_t cmd[2] = {CMD_GET_SYS_STATE, 0x00};
    uint8_t reply[6];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (image) *image = reply[4];
    
    return ESP_OK;
}

esp_err_t si4684_get_power_up_args(si4684_handle_t *handle, uint8_t *clk_mode, 
                                   uint32_t *xtal_freq, uint8_t *tr_size, 
                                   uint8_t *ibias, uint8_t *ctun, uint8_t *ibias_run) {
    uint8_t cmd[2] = {CMD_GET_POWER_UP_ARGS, 0x00};
    uint8_t reply[18];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 18);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (clk_mode) *clk_mode = (reply[6] >> 4) & 0x03;
    if (tr_size) *tr_size = reply[6] & 0x0F;
    if (ibias) *ibias = reply[7] & 0x7F;
    if (xtal_freq) *xtal_freq = reply[8] | (reply[9] << 8) | (reply[10] << 16) | (reply[11] << 24);
    if (ctun) *ctun = reply[12] & 0x3F;
    if (ibias_run) *ibias_run = reply[17] & 0x7F;
    
    return ESP_OK;
}

esp_err_t si4684_read_offset(si4684_handle_t *handle, uint16_t offset, 
                             uint8_t *data, size_t len) {
    uint8_t cmd[4];
    
    cmd[0] = CMD_READ_OFFSET;
    cmd[1] = 0x00;
    cmd[2] = offset & 0xFF;
    cmd[3] = (offset >> 8) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_read_reply(handle, data, len);
}

esp_err_t si4684_get_func_info(si4684_handle_t *handle, uint8_t *rev_ext, 
                               uint8_t *rev_branch, uint8_t *rev_int, uint32_t *svn_id) {
    uint8_t cmd[2] = {CMD_GET_FUNC_INFO, 0x00};
    uint8_t reply[12];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (rev_ext) *rev_ext = reply[4];
    if (rev_branch) *rev_branch = reply[5];
    if (rev_int) *rev_int = reply[6];
    if (svn_id) *svn_id = reply[8] | (reply[9] << 8) | (reply[10] << 16) | (reply[11] << 24);
    
    return ESP_OK;
}

// ============================================================================
// PROPERTY MANAGEMENT COMMANDS
// ============================================================================

esp_err_t si4684_set_property(si4684_handle_t *handle, uint16_t prop_id, uint16_t value) {
    uint8_t cmd[6];
    
    cmd[0] = CMD_SET_PROPERTY;
    cmd[1] = 0x00;
    cmd[2] = (prop_id >> 8) & 0xFF;
    cmd[3] = prop_id & 0xFF;
    cmd[4] = (value >> 8) & 0xFF;
    cmd[5] = value & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_get_property(si4684_handle_t *handle, uint16_t prop_id, uint16_t *value) {
    uint8_t cmd[4];
    uint8_t reply[6];
    
    cmd[0] = CMD_GET_PROPERTY;
    cmd[1] = 0x01;
    cmd[2] = (prop_id >> 8) & 0xFF;
    cmd[3] = prop_id & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (value) {
        *value = (reply[5] << 8) | reply[4];
    }
    
    return ESP_OK;
}

esp_err_t si4684_get_agc_status(si4684_handle_t *handle, uint8_t *rfagc_dis, 
                                uint8_t *ifagc_dis, uint8_t *lnagainidx) {
    uint8_t cmd[2] = {CMD_GET_AGC_STATUS, 0x00};
    uint8_t reply[7];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (rfagc_dis) *rfagc_dis = (reply[4] >> 2) & 0x01;
    if (ifagc_dis) *ifagc_dis = (reply[4] >> 1) & 0x01;
    if (lnagainidx) *lnagainidx = reply[5];
    
    return ESP_OK;
}

// ============================================================================
// FM/FMHD COMMANDS
// ============================================================================

esp_err_t si4684_fm_tune_freq(si4684_handle_t *handle, uint16_t freq, uint16_t ant_cap, 
                              uint8_t tune_mode, uint8_t injection) {
    uint8_t cmd[7];
    
    cmd[0] = CMD_FM_TUNE_FREQ;
    cmd[1] = ((tune_mode & 0x03) << 4) | (injection & 0x03);
    cmd[2] = freq & 0xFF;
    cmd[3] = (freq >> 8) & 0xFF;
    cmd[4] = ant_cap & 0xFF;
    cmd[5] = (ant_cap >> 8) & 0xFF;
    cmd[6] = 0x00;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_fm_seek_start(si4684_handle_t *handle, uint8_t seek_up, uint8_t wrap) {
    uint8_t cmd[2];
    
    cmd[0] = CMD_FM_SEEK_START;
    cmd[1] = ((seek_up & 0x01) << 1) | (wrap & 0x01);
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 5000);
}

esp_err_t si4684_fm_rsq_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *rssi, uint8_t *snr, uint16_t *freq, 
                               uint8_t *valid, uint8_t *afc_rail, uint8_t *valid_freq) {
    uint8_t cmd[2];
    uint8_t reply[23];
    
    cmd[0] = CMD_FM_RSQ_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 23);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (valid) *valid = reply[5] & 0x01;
    if (afc_rail) *afc_rail = (reply[5] >> 1) & 0x01;
    if (valid_freq) *valid_freq = (reply[5] >> 2) & 0x01;
    if (freq) *freq = reply[6] | (reply[7] << 8);
    if (rssi) *rssi = reply[9];
    if (snr) *snr = reply[10];
    
    return ESP_OK;
}

esp_err_t si4684_fm_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *blend_int, uint8_t *hicut_int, 
                               uint8_t *softmute_int, uint8_t *pilot) {
    uint8_t cmd[2];
    uint8_t reply[9];
    
    cmd[0] = CMD_FM_ACF_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 9);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (blend_int) *blend_int = reply[4] & 0x01;
    if (hicut_int) *hicut_int = (reply[4] >> 1) & 0x01;
    if (softmute_int) *softmute_int = (reply[4] >> 2) & 0x01;
    if (pilot) *pilot = (reply[5] >> 7) & 0x01;
    
    return ESP_OK;
}

esp_err_t si4684_fm_rds_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *recv, uint8_t *sync, uint16_t *pi, 
                               uint8_t *block_a_h, uint8_t *block_a_l) {
    uint8_t cmd[2];
    uint8_t reply[21];
    
    cmd[0] = CMD_FM_RDS_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 21);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (recv) *recv = reply[4] & 0x01;
    if (sync) *sync = (reply[4] >> 1) & 0x01;
    if (pi) *pi = reply[12] | (reply[13] << 8);
    if (block_a_h) *block_a_h = reply[16];
    if (block_a_l) *block_a_l = reply[17];
    
    return ESP_OK;
}

esp_err_t si4684_fm_rds_blockcount(si4684_handle_t *handle, uint8_t clear, 
                                   uint16_t *expected, uint16_t *received, 
                                   uint16_t *uncorrectable) {
    uint8_t cmd[2];
    uint8_t reply[11];
    
    cmd[0] = CMD_FM_RDS_BLOCKCOUNT;
    cmd[1] = clear & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 11);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (expected) *expected = reply[4] | (reply[5] << 8);
    if (received) *received = reply[6] | (reply[7] << 8);
    if (uncorrectable) *uncorrectable = reply[8] | (reply[9] << 8);
    
    return ESP_OK;
}

// ============================================================================
// AM/AMHD COMMANDS
// ============================================================================

esp_err_t si4684_am_tune_freq(si4684_handle_t *handle, uint16_t freq, uint16_t ant_cap, 
                              uint8_t tune_mode) {
    uint8_t cmd[6];
    
    cmd[0] = CMD_AM_TUNE_FREQ;
    cmd[1] = (tune_mode & 0x03) << 4;
    cmd[2] = freq & 0xFF;
    cmd[3] = (freq >> 8) & 0xFF;
    cmd[4] = ant_cap & 0xFF;
    cmd[5] = (ant_cap >> 8) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_am_seek_start(si4684_handle_t *handle, uint8_t seek_up, uint8_t wrap) {
    uint8_t cmd[2];
    
    cmd[0] = CMD_AM_SEEK_START;
    cmd[1] = ((seek_up & 0x01) << 1) | (wrap & 0x01);
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 5000);
}

esp_err_t si4684_am_rsq_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *rssi, uint8_t *snr, uint16_t *freq, 
                               uint8_t *valid) {
    uint8_t cmd[2];
    uint8_t reply[15];
    
    cmd[0] = CMD_AM_RSQ_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 15);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (valid) *valid = reply[5] & 0x01;
    if (freq) *freq = reply[6] | (reply[7] << 8);
    if (rssi) *rssi = reply[9];
    if (snr) *snr = reply[10];
    
    return ESP_OK;
}

esp_err_t si4684_am_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *softmute_int, uint8_t *hicut_int) {
    uint8_t cmd[2];
    uint8_t reply[7];
    
    cmd[0] = CMD_AM_ACF_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (softmute_int) *softmute_int = reply[4] & 0x01;
    if (hicut_int) *hicut_int = (reply[4] >> 1) & 0x01;
    
    return ESP_OK;
}

// ============================================================================
// DIGITAL SERVICE COMMANDS (Common for HD Radio and DAB)
// ============================================================================

esp_err_t si4684_get_digital_service_list(si4684_handle_t *handle, uint8_t serv_idx_list, 
                                          uint16_t *list_size, uint8_t *service_list, 
                                          size_t max_list_len) {
    uint8_t cmd[2];
    uint8_t reply[8];
    
    cmd[0] = CMD_GET_DIGITAL_SERVICE_LIST;
    cmd[1] = serv_idx_list & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // First read to get list size
    ret = si4684_read_reply(handle, reply, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (list_size) *list_size = reply[4] | (reply[5] << 8);
    
    // Read the actual service list if buffer provided
    if (service_list && max_list_len > 0) {
        size_t to_read = (*list_size < max_list_len) ? *list_size : max_list_len;
        ret = si4684_read_offset(handle, 8, service_list, to_read);
    }
    
    return ret;
}

esp_err_t si4684_start_digital_service(si4684_handle_t *handle, uint32_t service_id, 
                                       uint32_t comp_id, uint8_t audio_data) {
    uint8_t cmd[12];
    
    cmd[0] = CMD_START_DIGITAL_SERVICE;
    cmd[1] = (audio_data & 0x01) << 7;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = (service_id >> 0) & 0xFF;
    cmd[5] = (service_id >> 8) & 0xFF;
    cmd[6] = (service_id >> 16) & 0xFF;
    cmd[7] = (service_id >> 24) & 0xFF;
    cmd[8] = (comp_id >> 0) & 0xFF;
    cmd[9] = (comp_id >> 8) & 0xFF;
    cmd[10] = (comp_id >> 16) & 0xFF;
    cmd[11] = (comp_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 2000);
}

esp_err_t si4684_stop_digital_service(si4684_handle_t *handle, uint32_t service_id, 
                                      uint32_t comp_id) {
    uint8_t cmd[12];
    
    cmd[0] = CMD_STOP_DIGITAL_SERVICE;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = (service_id >> 0) & 0xFF;
    cmd[5] = (service_id >> 8) & 0xFF;
    cmd[6] = (service_id >> 16) & 0xFF;
    cmd[7] = (service_id >> 24) & 0xFF;
    cmd[8] = (comp_id >> 0) & 0xFF;
    cmd[9] = (comp_id >> 8) & 0xFF;
    cmd[10] = (comp_id >> 16) & 0xFF;
    cmd[11] = (comp_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_get_digital_service_data(si4684_handle_t *handle, uint8_t int_ack, 
                                          uint8_t serv_idx_list, uint16_t *data_cnt_avail, 
                                          uint16_t *data_src, uint8_t *data, size_t max_data_len) {
    uint8_t cmd[2];
    uint8_t reply[12];
    
    cmd[0] = CMD_GET_DIGITAL_SERVICE_DATA;
    cmd[1] = ((int_ack & 0x01) << 1) | (serv_idx_list & 0x01);
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (data_cnt_avail) *data_cnt_avail = reply[10] | (reply[11] << 8);
    if (data_src) *data_src = reply[8] | (reply[9] << 8);
    
    // Read actual data if buffer provided
    if (data && max_data_len > 0 && *data_cnt_avail > 0) {
        size_t to_read = (*data_cnt_avail < max_data_len) ? *data_cnt_avail : max_data_len;
        ret = si4684_read_offset(handle, 12, data, to_read);
    }
    
    return ret;
}

// ============================================================================
// HD RADIO SPECIFIC COMMANDS
// ============================================================================

esp_err_t si4684_hd_digrad_status(si4684_handle_t *handle, uint8_t int_ack, uint8_t dig_acq_int_ack, 
                                  uint8_t *acq, uint8_t *daai, uint8_t *audio_prog) {
    uint8_t cmd[2];
    uint8_t reply[22];
    
    cmd[0] = CMD_HD_DIGRAD_STATUS;
    cmd[1] = ((int_ack & 0x01) << 1) | (dig_acq_int_ack & 0x01);
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 22);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (acq) *acq = reply[5] & 0x01;
    if (daai) *daai = (reply[5] >> 7) & 0x01;
    if (audio_prog) *audio_prog = reply[19] & 0x0F;
    
    return ESP_OK;
}

esp_err_t si4684_hd_get_psd_decode(si4684_handle_t *handle, uint8_t id_filter, 
                                   uint8_t prog_id, char *title, char *artist, 
                                   char *album, size_t max_str_len) {
    uint8_t cmd[4];
    uint8_t reply[256];
    
    cmd[0] = CMD_HD_GET_PSD_DECODE;
    cmd[1] = ((id_filter & 0x01) << 2);
    cmd[2] = prog_id & 0x0F;
    cmd[3] = 0x00;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 256);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse PSD data (simplified - actual parsing depends on format)
    // This would need to be expanded based on the actual PSD structure
    
    return ESP_OK;
}

// ============================================================================
// DAB SPECIFIC COMMANDS
// ============================================================================

esp_err_t si4684_dab_tune_freq(si4684_handle_t *handle, uint8_t freq_index, 
                               uint16_t ant_cap, uint8_t injection) {
    uint8_t cmd[6];
    
    cmd[0] = CMD_DAB_TUNE_FREQ;
    cmd[1] = injection & 0x03;
    cmd[2] = freq_index;
    cmd[3] = 0x00;
    cmd[4] = ant_cap & 0xFF;
    cmd[5] = (ant_cap >> 8) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_dab_digrad_status(si4684_handle_t *handle, uint8_t int_ack, uint8_t dig_acq_int_ack, 
                                   uint8_t *acq, uint8_t *rssi, uint8_t *snr, 
                                   uint8_t *fic_quality, uint16_t *tune_freq) {
    uint8_t cmd[2];
    uint8_t reply[22];
    
    cmd[0] = CMD_DAB_DIGRAD_STATUS;
    cmd[1] = ((int_ack & 0x01) << 1) | (dig_acq_int_ack & 0x01);
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 22);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (acq) *acq = reply[5] & 0x01;
    if (rssi) *rssi = reply[9];
    if (snr) *snr = reply[10];
    if (fic_quality) *fic_quality = reply[11];
    if (tune_freq) *tune_freq = reply[6] | (reply[7] << 8);
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_ensemble_info(si4684_handle_t *handle, char *label, 
                                       uint16_t *ensemble_id, uint8_t *ensemble_ecc, 
                                       size_t max_label_len) {
    uint8_t cmd[2] = {CMD_DAB_GET_ENSEMBLE_INFO, 0x00};
    uint8_t reply[24];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 24);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (ensemble_id) *ensemble_id = reply[4] | (reply[5] << 8);
    if (ensemble_ecc) *ensemble_ecc = reply[6];
    
    if (label && max_label_len > 0) {
        size_t label_len = (max_label_len < 16) ? max_label_len : 16;
        memcpy(label, &reply[8], label_len);
        label[label_len - 1] = '\0';
    }
    
    return ESP_OK;
}

esp_err_t si4684_dab_set_freq_list(si4684_handle_t *handle, uint8_t num_freqs, 
                                   const uint32_t *freq_list) {
    if (num_freqs > 48) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t cmd_len = 4 + (num_freqs * 4);
    uint8_t *cmd = malloc(cmd_len);
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    cmd[0] = CMD_DAB_SET_FREQ_LIST;
    cmd[1] = 0x00;
    cmd[2] = num_freqs;
    cmd[3] = 0x00;
    
    for (int i = 0; i < num_freqs; i++) {
        uint32_t freq = freq_list[i];
        cmd[4 + (i * 4) + 0] = (freq >> 0) & 0xFF;
        cmd[4 + (i * 4) + 1] = (freq >> 8) & 0xFF;
        cmd[4 + (i * 4) + 2] = (freq >> 16) & 0xFF;
        cmd[4 + (i * 4) + 3] = (freq >> 24) & 0xFF;
    }
    
    esp_err_t ret = si4684_write_command(handle, cmd, cmd_len);
    free(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_dab_get_time(si4684_handle_t *handle, uint16_t *year, uint8_t *month, 
                              uint8_t *day, uint8_t *hour, uint8_t *minute, 
                              uint8_t *second) {
    uint8_t cmd[2] = {CMD_DAB_GET_TIME, 0x00};
    uint8_t reply[11];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 11);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (year) *year = reply[4] | (reply[5] << 8);
    if (month) *month = reply[6];
    if (day) *day = reply[7];
    if (hour) *hour = reply[8];
    if (minute) *minute = reply[9];
    if (second) *second = reply[10];
    
    return ESP_OK;
}

// ============================================================================
// TEST COMMANDS
// ============================================================================

esp_err_t si4684_test_get_rssi(si4684_handle_t *handle, int16_t *rssi) {
    uint8_t cmd[2] = {CMD_TEST_GET_RSSI, 0x00};
    uint8_t reply[6];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (rssi) {
        *rssi = (int16_t)(reply[4] | (reply[5] << 8));
    }
    
    return ESP_OK;
}

// ============================================================================
// HD RADIO COMMANDS
// ============================================================================

esp_err_t si4684_hd_get_event_status(si4684_handle_t *handle, uint8_t int_ack, 
                                     uint16_t *event_avail, uint16_t *event_flags) {
    uint8_t cmd[2];
    uint8_t reply[8];
    
    cmd[0] = CMD_HD_GET_EVENT_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (event_avail) *event_avail = reply[4] | (reply[5] << 8);
    if (event_flags) *event_flags = reply[6] | (reply[7] << 8);
    
    return ESP_OK;
}

esp_err_t si4684_hd_get_station_info(si4684_handle_t *handle, char *short_name, 
                                     char *long_name, uint8_t *loc_cc, size_t name_len) {
    uint8_t cmd[2] = {CMD_HD_GET_STATION_INFO, 0x00};
    uint8_t reply[48];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 48);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (short_name && name_len >= 8) {
        memcpy(short_name, &reply[4], 7);
        short_name[7] = '\0';
    }
    
    if (long_name && name_len >= 32) {
        memcpy(long_name, &reply[12], 31);
        long_name[31] = '\0';
    }
    
    if (loc_cc) *loc_cc = reply[44];
    
    return ESP_OK;
}

esp_err_t si4684_hd_get_alert_msg(si4684_handle_t *handle, uint8_t int_ack, 
                                  char *msg, size_t max_msg_len) {
    uint8_t cmd[2];
    uint8_t reply[256];
    
    cmd[0] = CMD_HD_GET_ALERT_MSG;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 256);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (msg && max_msg_len > 0) {
        uint16_t msg_len = reply[4] | (reply[5] << 8);
        size_t copy_len = (msg_len < max_msg_len - 1) ? msg_len : max_msg_len - 1;
        memcpy(msg, &reply[6], copy_len);
        msg[copy_len] = '\0';
    }
    
    return ESP_OK;
}

esp_err_t si4684_hd_play_alert_tone(si4684_handle_t *handle, uint8_t play) {
    uint8_t cmd[2];
    
    cmd[0] = CMD_HD_PLAY_ALERT_TONE;
    cmd[1] = play & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_hd_test_get_ber_info(si4684_handle_t *handle, uint8_t clear, 
                                      uint32_t *ber_count) {
    uint8_t cmd[2];
    uint8_t reply[8];
    
    cmd[0] = CMD_HD_TEST_GET_BER_INFO;
    cmd[1] = clear & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (ber_count) {
        *ber_count = reply[4] | (reply[5] << 8) | (reply[6] << 16) | (reply[7] << 24);
    }
    
    return ESP_OK;
}

esp_err_t si4684_hd_set_enabled_ports(si4684_handle_t *handle, uint32_t port_mask) {
    uint8_t cmd[6];
    
    cmd[0] = CMD_HD_SET_ENABLED_PORTS;
    cmd[1] = 0x00;
    cmd[2] = (port_mask >> 0) & 0xFF;
    cmd[3] = (port_mask >> 8) & 0xFF;
    cmd[4] = (port_mask >> 16) & 0xFF;
    cmd[5] = (port_mask >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return si4684_wait_for_cts(handle, 1000);
}

esp_err_t si4684_hd_get_enabled_ports(si4684_handle_t *handle, uint32_t *port_mask) {
    uint8_t cmd[2] = {CMD_HD_GET_ENABLED_PORTS, 0x00};
    uint8_t reply[8];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (port_mask) {
        *port_mask = reply[4] | (reply[5] << 8) | (reply[6] << 16) | (reply[7] << 24);
    }
    
    return ESP_OK;
}

esp_err_t si4684_hd_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                               uint8_t *blend_int, uint8_t *audio_acq_int) {
    uint8_t cmd[2];
    uint8_t reply[9];
    
    cmd[0] = CMD_HD_ACF_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 9);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (blend_int) *blend_int = reply[4] & 0x01;
    if (audio_acq_int) *audio_acq_int = (reply[4] >> 1) & 0x01;
    
    return ESP_OK;
}

// ============================================================================
// DAB COMMANDS
// ============================================================================

esp_err_t si4684_dab_get_event_status(si4684_handle_t *handle, uint8_t int_ack, 
                                      uint16_t *event_avail, uint16_t *event_flags) {
    uint8_t cmd[2];
    uint8_t reply[8];
    
    cmd[0] = CMD_DAB_GET_EVENT_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (event_avail) *event_avail = reply[4] | (reply[5] << 8);
    if (event_flags) *event_flags = reply[6] | (reply[7] << 8);
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_announcement_support_info(si4684_handle_t *handle, uint32_t service_id, 
                                                   uint32_t comp_id, uint16_t *asw_flags) {
    uint8_t cmd[12];
    uint8_t reply[6];
    
    cmd[0] = CMD_DAB_GET_ANNOUNCEMENT_SUPPORT_INFO;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = (service_id >> 0) & 0xFF;
    cmd[5] = (service_id >> 8) & 0xFF;
    cmd[6] = (service_id >> 16) & 0xFF;
    cmd[7] = (service_id >> 24) & 0xFF;
    cmd[8] = (comp_id >> 0) & 0xFF;
    cmd[9] = (comp_id >> 8) & 0xFF;
    cmd[10] = (comp_id >> 16) & 0xFF;
    cmd[11] = (comp_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (asw_flags) *asw_flags = reply[4] | (reply[5] << 8);
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_announcement_info(si4684_handle_t *handle, uint8_t buf_empty, 
                                           uint32_t *service_id, uint32_t *comp_id, 
                                           uint16_t *asw_flags) {
    uint8_t cmd[2];
    uint8_t reply[14];
    
    cmd[0] = CMD_DAB_GET_ANNOUNCEMENT_INFO;
    cmd[1] = buf_empty & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (service_id) *service_id = reply[4] | (reply[5] << 8) | (reply[6] << 16) | (reply[7] << 24);
    if (comp_id) *comp_id = reply[8] | (reply[9] << 8) | (reply[10] << 16) | (reply[11] << 24);
    if (asw_flags) *asw_flags = reply[12] | (reply[13] << 8);
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_service_linking_info(si4684_handle_t *handle, uint32_t service_id, 
                                              uint8_t *link_info, size_t max_len) {
    uint8_t cmd[6];
    uint8_t reply[256];
    
    cmd[0] = CMD_DAB_GET_SERVICE_LINKING_INFO;
    cmd[1] = 0x00;
    cmd[2] = (service_id >> 0) & 0xFF;
    cmd[3] = (service_id >> 8) & 0xFF;
    cmd[4] = (service_id >> 16) & 0xFF;
    cmd[5] = (service_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 256);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (link_info && max_len > 0) {
        uint16_t data_size = reply[4] | (reply[5] << 8);
        size_t copy_len = (data_size < max_len) ? data_size : max_len;
        memcpy(link_info, &reply[6], copy_len);
    }
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_freq_list(si4684_handle_t *handle, uint8_t *num_freqs, 
                                   uint32_t *freq_list, size_t max_freqs) {
    uint8_t cmd[2] = {CMD_DAB_GET_FREQ_LIST, 0x00};
    uint8_t reply[200];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 200);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t count = reply[4];
    if (num_freqs) *num_freqs = count;
    
    if (freq_list && max_freqs > 0) {
        size_t to_copy = (count < max_freqs) ? count : max_freqs;
        for (size_t i = 0; i < to_copy; i++) {
            freq_list[i] = reply[6 + (i * 4)] | 
                          (reply[7 + (i * 4)] << 8) | 
                          (reply[8 + (i * 4)] << 16) | 
                          (reply[9 + (i * 4)] << 24);
        }
    }
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_component_info(si4684_handle_t *handle, uint32_t service_id, 
                                        uint8_t comp_idx, uint32_t *comp_id, 
                                        uint8_t *cmp_info) {
    uint8_t cmd[7];
    uint8_t reply[16];
    
    cmd[0] = CMD_DAB_GET_COMPONENT_INFO;
    cmd[1] = 0x00;
    cmd[2] = comp_idx;
    cmd[3] = 0x00;
    cmd[4] = (service_id >> 0) & 0xFF;
    cmd[5] = (service_id >> 8) & 0xFF;
    cmd[6] = (service_id >> 16) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 16);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (comp_id) {
        *comp_id = reply[4] | (reply[5] << 8) | (reply[6] << 16) | (reply[7] << 24);
    }
    
    if (cmp_info) *cmp_info = reply[8];
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_audio_info(si4684_handle_t *handle, uint8_t *bit_rate, 
                                    uint8_t *sample_rate, uint8_t *mode) {
    uint8_t cmd[2] = {CMD_DAB_GET_AUDIO_INFO, 0x00};
    uint8_t reply[7];
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (bit_rate) *bit_rate = reply[4];
    if (sample_rate) *sample_rate = reply[5];
    if (mode) *mode = reply[6];
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_subchan_info(si4684_handle_t *handle, uint32_t service_id, 
                                      uint32_t comp_id, uint8_t *subchan_id, 
                                      uint16_t *subchan_size) {
    uint8_t cmd[12];
    uint8_t reply[7];
    
    cmd[0] = CMD_DAB_GET_SUBCHAN_INFO;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = (service_id >> 0) & 0xFF;
    cmd[5] = (service_id >> 8) & 0xFF;
    cmd[6] = (service_id >> 16) & 0xFF;
    cmd[7] = (service_id >> 24) & 0xFF;
    cmd[8] = (comp_id >> 0) & 0xFF;
    cmd[9] = (comp_id >> 8) & 0xFF;
    cmd[10] = (comp_id >> 16) & 0xFF;
    cmd[11] = (comp_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (subchan_id) *subchan_id = reply[4];
    if (subchan_size) *subchan_size = reply[5] | (reply[6] << 8);
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_freq_info(si4684_handle_t *handle, uint8_t freq_index, 
                                   uint32_t *freq, uint8_t *control) {
    uint8_t cmd[4];
    uint8_t reply[9];
    
    cmd[0] = CMD_DAB_GET_FREQ_INFO;
    cmd[1] = 0x00;
    cmd[2] = freq_index;
    cmd[3] = 0x00;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 4);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 9);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (freq) {
        *freq = reply[4] | (reply[5] << 8) | (reply[6] << 16) | (reply[7] << 24);
    }
    
    if (control) *control = reply[8];
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_service_info(si4684_handle_t *handle, uint32_t service_id, 
                                      char *label, uint8_t *pty, uint8_t *pd_flag, 
                                      size_t max_label_len) {
    uint8_t cmd[6];
    uint8_t reply[24];
    
    cmd[0] = CMD_DAB_GET_SERVICE_INFO;
    cmd[1] = 0x00;
    cmd[2] = (service_id >> 0) & 0xFF;
    cmd[3] = (service_id >> 8) & 0xFF;
    cmd[4] = (service_id >> 16) & 0xFF;
    cmd[5] = (service_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 24);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (pty) *pty = reply[4];
    if (pd_flag) *pd_flag = reply[5] & 0x01;
    
    if (label && max_label_len > 0) {
        size_t label_len = (max_label_len < 17) ? max_label_len : 17;
        memcpy(label, &reply[6], label_len - 1);
        label[label_len - 1] = '\0';
    }
    
    return ESP_OK;
}

esp_err_t si4684_dab_get_oe_services_info(si4684_handle_t *handle, uint32_t service_id, 
                                          uint8_t *oe_info, size_t max_len) {
    uint8_t cmd[6];
    uint8_t reply[256];
    
    cmd[0] = CMD_DAB_GET_OE_SERVICES_INFO;
    cmd[1] = 0x00;
    cmd[2] = (service_id >> 0) & 0xFF;
    cmd[3] = (service_id >> 8) & 0xFF;
    cmd[4] = (service_id >> 16) & 0xFF;
    cmd[5] = (service_id >> 24) & 0xFF;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 256);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (oe_info && max_len > 0) {
        uint8_t num_services = reply[4];
        size_t data_len = num_services * 6; // Each OE service entry is 6 bytes
        size_t copy_len = (data_len < max_len) ? data_len : max_len;
        memcpy(oe_info, &reply[5], copy_len);
    }
    
    return ESP_OK;
}

esp_err_t si4684_dab_acf_status(si4684_handle_t *handle, uint8_t int_ack, 
                                uint8_t *softmute_int) {
    uint8_t cmd[2];
    uint8_t reply[7];
    
    cmd[0] = CMD_DAB_ACF_STATUS;
    cmd[1] = int_ack & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (softmute_int) *softmute_int = reply[4] & 0x01;
    
    return ESP_OK;
}

esp_err_t si4684_dab_test_get_ber_info(si4684_handle_t *handle, uint8_t clear, 
                                       uint32_t *ber_count, uint32_t *fib_error_count) {
    uint8_t cmd[2];
    uint8_t reply[12];
    
    cmd[0] = CMD_DAB_TEST_GET_BER_INFO;
    cmd[1] = clear & 0x01;
    
    esp_err_t ret = si4684_write_command(handle, cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_wait_for_cts(handle, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = si4684_read_reply(handle, reply, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (ber_count) {
        *ber_count = reply[4] | (reply[5] << 8) | (reply[6] << 16) | (reply[7] << 24);
    }
    
    if (fib_error_count) {
        *fib_error_count = reply[8] | (reply[9] << 8) | (reply[10] << 16) | (reply[11] << 24);
    }
    
    return ESP_OK;
}
