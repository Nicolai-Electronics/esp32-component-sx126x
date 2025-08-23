#include "sx126x.h"
#include <iso646.h>
#include <stdint.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

// Commands
#define SX126X_CMD_SET_SLEEP                  0x84
#define SX126X_CMD_SET_STANDBY                0x80
#define SX126X_CMD_SET_FS                     0xC1
#define SX126X_CMD_SET_TX                     0x83
#define SX126X_CMD_SET_RX                     0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE     0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE          0x94
#define SX126X_CMD_SET_CAD                    0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE     0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE   0xD2
#define SX126X_CMD_SET_REGULATOR_MODE         0x96
#define SX126X_CMD_CALIBRATE                  0x89
#define SX126X_CMD_CALIBRATE_IMAGE            0x98
#define SX126X_CMD_SET_PA_CONFIG              0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE    0x93
#define SX126X_CMD_WRITE_REGISTER             0x0D
#define SX126X_CMD_READ_REGISTER              0x1D
#define SX126X_CMD_WRITE_BUFFER               0x0E
#define SX126X_CMD_READ_BUFFER                0x1E
#define SX126X_CMD_SET_DIO_IRQ_PARAMS         0x08
#define SX126X_CMD_GET_IRQ_STATUS             0x12
#define SX126X_CMD_CLEAR_IRQ_STATUS           0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL 0x9D
#define SX126X_CMD_SET_DIO3_AS_TXCO_CTRL      0x97
#define SX126X_CMD_SET_RF_FREQUENCY           0x86
#define SX126X_CMD_SET_PACKET_TYPE            0x8A
#define SX126X_CMD_GET_PACKET_TYPE            0x11
#define SX126X_CMD_SET_TX_PARAMS              0x8E
#define SX126X_CMD_SET_MODULATION_PARAMS      0x8B
#define SX126X_CMD_SET_PACKET_PARAMS          0x8C
#define SX126X_CMD_SET_CAD_PARAMS             0x88
#define SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT  0xA0
#define SX126X_CMD_GET_STATUS                 0xC0
#define SX126X_CMD_GET_RX_BUFFER_STATUS       0x13
#define SX126X_CMD_GET_PACKET_STATUS          0x14
#define SX126X_CMD_GET_RSSI_INST              0x15
#define SX126X_CMD_GET_STATS                  0x10
#define SX126X_CMD_RESET_STATS                0x00
#define SX126X_CMD_GET_DEVICE_ERRORS          0x17
#define SX126X_CMD_CLEAR_DEVICE_ERRORS        0x07

// Registers
#define SX126X_REG_RX_GAIN_RETENTION(X)             (0x029F + (X))  // 0-2
#define SX126X_REG_VERSION_STRING                   0x0320
#define SX126X_REG_HOPPING_ENABLE                   0x0385
#define SX126X_REG_LR_FHSS_PACKET_LENGTH            0x0386
#define SX126X_REG_LR_FHSS_NUM_HOPPING_BLOCKS       0x0387
#define SX126X_REG_LR_FHSS_NUM_SYMBOLS_FREQX_MSB(X) (0x0388 + (X) * 6)
#define SX126X_REG_LR_FHSS_NUM_SYMBOLS_FREQX_LSB(X) (0x0389 + (X) * 6)
#define SX126X_REG_LR_FHSS_FREQX_0(X)               (0x038A + (X) * 6)
#define SX126X_REG_LR_FHSS_FREQX_1(X)               (0x038B + (X) * 6)
#define SX126X_REG_LR_FHSS_FREQX_2(X)               (0x038C + (X) * 6)
#define SX126X_REG_LR_FHSS_FREQX_3(X)               (0x038D + (X) * 6)
#define SX126X_REG_SPECTRAL_SCAN_RESULT             0x0401
#define SX126X_REG_DIOX_OUT_ENABLE                  0x0580
#define SX126X_REG_DIOX_DRIVE_STRENGTH              0x0582
#define SX126X_REG_DIOX_IN_ENABLE                   0x0583
#define SX126X_REG_DIOX_PULL_UP_CTRL                0x0584
#define SX126X_REG_DIOX_PULL_DOWN_CTRL              0x0585
#define SX126X_REG_TX_BITBANG_ENABLE_0              0x0587
#define SX126X_REG_PATCH_UPDATE_ENABLE              0x0610
#define SX126X_REG_TX_BITBANG_ENABLE_1              0x0680
#define SX126X_REG_WHITENING_INITIAL_MSB            0x06B8
#define SX126X_REG_WHITENING_INITIAL_LSB            0x06B9
#define SX126X_REG_RX_TX_PLD_LEN                    0x06BB
#define SX126X_REG_CRC_INITIAL_MSB                  0x06BC
#define SX126X_REG_CRC_INITIAL_LSB                  0x06BD
#define SX126X_REG_CRC_POLYNOMIAL_MSB               0x06BE
#define SX126X_REG_CRC_POLYNOMIAL_LSB               0x06BF
#define SX126X_REG_SYNC_WORD(X)                     (0x06C0 + (X))  // 0-6
#define SX126X_REG_NODE_ADDRESS                     0x06CD
#define SX126X_REG_BROADCAST_ADDRESS                0x06CE
#define SX126X_REG_PAYLOAD_LENGTH                   0x0702
#define SX126X_REG_PACKET_PARAMS                    0x0704
#define SX126X_REG_LORA_SYNC_TIMEOUT                0x0706
#define SX126X_REG_IQ_CONFIG                        0x0736
#define SX126X_REG_LORA_SYNC_WORD_MSB               0x0740
#define SX126X_REG_LORA_SYNC_WORD_LSB               0x0741
#define SX126X_REG_FREQ_ERROR                       0x076B
#define SX126X_REG_SPECTRAL_SCAN_STATUS             0x07CD
#define SX126X_REG_RX_ADDR_PTR                      0x0803
#define SX126X_REG_RANDOM_NUMBER_0                  0x0819
#define SX126X_REG_RANDOM_NUMBER_1                  0x081A
#define SX126X_REG_RANDOM_NUMBER_2                  0x081B
#define SX126X_REG_RANDOM_NUMBER_3                  0x081C
#define SX126X_REG_SENSITIVITY_CONFIG               0x0889
#define SX126X_REG_RF_FREQUENCY(X)                  (0x088B + (X))  // 0-3
#define SX126X_REG_RSSI_AVG_WINDOW                  0x089B
#define SX126X_REG_RX_GAIN                          0x08AC
#define SX126X_REG_TX_CLAMP_CONFIG                  0x08D8
#define SX126X_REG_ANA_LNA                          0x08E2
#define SX126X_REG_LNA_CAP_TUNE_N                   0x08E3
#define SX126X_REG_LNA_CAP_TUNE_P                   0x08E4
#define SX126X_REG_ANA_MIXER                        0x08E5
#define SX126X_REG_OCP_CONFIGURATION                0x08E7
#define SX126X_REG_RTC_CTRL                         0x0902
#define SX126X_REG_XTA_TRIM                         0x0911
#define SX126X_REG_XTB_TRIM                         0x0912
#define SX126X_REG_DIO3_OUT_VOLTAGE_CTRL            0x0920
#define SX126X_REG_EVENT_MASK                       0x0944
#define SX126X_REG_PATCH_MEMORY_BASE                0x8000

static const char* TAG = "sx126x";

esp_err_t sx126x_set_op_mode_sleep(sx126x_handle_t* handle, bool warm_start, bool rtc_timeout_disable) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t sleep_config  = 0;
    sleep_config         |= (warm_start ? 0x01 : 0x00) << 2;                            // Warm start
    sleep_config         |= ((!warm_start && rtc_timeout_disable) ? 0x01 : 0x00) << 0;  // RTC timeout disable

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_SLEEP,
        .length    = 8,
        .tx_buffer = &sleep_config,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_op_mode_standby(sx126x_handle_t* handle, bool use_xosc) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t standby_config  = 0;
    standby_config         |= (use_xosc ? 0x01 : 0x00) << 0;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_STANDBY,
        .length    = 8,
        .tx_buffer = &standby_config,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_op_mode_fs(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_FS,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_op_mode_tx(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_TX,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_op_mode_rx(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_RX,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_stop_timer_on_preamble(sx126x_handle_t* handle, bool stop) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t parameter = stop ? 0x01 : 0x00;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_STOP_TIMER_ON_PREAMBLE,
        .length    = 8,
        .tx_buffer = &parameter,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_rx_duty_cycle(sx126x_handle_t* handle, uint32_t rx_period, uint32_t sleep_period) {
    if (handle == NULL || rx_period > 0x00FFFFFF || sleep_period > 0x00FFFFFF) return ESP_ERR_INVALID_ARG;
    uint8_t parameters[6];
    parameters[0] = (rx_period >> 16) & 0xFF;
    parameters[1] = (rx_period >> 8) & 0xFF;
    parameters[2] = rx_period & 0xFF;
    parameters[3] = (sleep_period >> 16) & 0xFF;
    parameters[4] = (sleep_period >> 8) & 0xFF;
    parameters[5] = sleep_period & 0xFF;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_RX_DUTY_CYCLE,
        .length    = sizeof(parameters) * 8,
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_cad(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_CAD,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_tx_continuous_wave(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_TX_CONTINUOUS_WAVE,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_tx_infinite_preamble(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_TX_INFINITE_PREAMBLE,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_regulator_mode(sx126x_handle_t* handle, bool use_dc_dc) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t parameter = use_dc_dc ? 0x01 : 0x00;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_REGULATOR_MODE,
        .length    = 8,
        .tx_buffer = &parameter,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_calibrate(sx126x_handle_t* handle, bool rc64k, bool rc13m, bool pll, bool adc_pulse, bool adc_bulk_n,
                           bool adc_bulk_p, bool image) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t parameter = (rc64k & 1) | ((rc13m & 1) << 1) | ((pll & 1) << 2) | ((adc_pulse & 1) << 3) |
                        ((adc_bulk_n & 1) << 4) | ((adc_bulk_p & 1) << 5) | ((image & 1) << 6);

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_CALIBRATE,
        .length    = 8,
        .tx_buffer = &parameter,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_calibrate_image(sx126x_handle_t* handle, uint8_t frequency1, uint8_t frequency2) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t parameters[2] = {frequency1, frequency2};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_CALIBRATE_IMAGE,
        .length    = 16,
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_pa_config(sx126x_handle_t* handle, uint8_t pa_duty_cycle, uint8_t hp_max) {
    if (handle == NULL || hp_max > 0x07) return ESP_ERR_INVALID_ARG;
    uint8_t parameters[4] = {pa_duty_cycle, hp_max, 0x00, 0x01};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_PA_CONFIG,
        .length    = 32,
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_rx_tx_fallback_mode(sx126x_handle_t* handle, sx126x_fallback_mode_t fallback_mode) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t parameter = (uint8_t)fallback_mode;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_RX_TX_FALLBACK_MODE,
        .length    = 8,
        .tx_buffer = &parameter,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_write_register(sx126x_handle_t* handle, uint16_t address, const uint8_t* values, size_t length) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_ext_t t = {
        .base =
            {
                .flags     = SPI_TRANS_VARIABLE_ADDR,
                .cmd       = SX126X_CMD_WRITE_REGISTER,
                .addr      = address,
                .length    = length * 8,
                .tx_buffer = values,
                .rx_buffer = NULL,
            },
        .address_bits = 16,
    };
    return spi_device_transmit(handle->device, (spi_transaction_t*)&t);
}

esp_err_t sx126x_read_register(sx126x_handle_t* handle, uint16_t address, uint8_t* out_values, size_t length) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_ext_t t = {
        .base =
            {
                .flags     = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY,
                .cmd       = SX126X_CMD_READ_REGISTER,
                .addr      = address,
                .length    = length * 8,
                .tx_buffer = NULL,
                .rx_buffer = out_values,
            },
        .address_bits = 16,
        .dummy_bits   = 8,
    };
    return spi_device_transmit(handle->device, (spi_transaction_t*)&t);
}

esp_err_t sx126x_write_buffer(sx126x_handle_t* handle, uint8_t offset, const uint8_t* values, size_t length) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_ext_t t = {
        .base =
            {
                .flags     = SPI_TRANS_VARIABLE_ADDR,
                .cmd       = SX126X_CMD_WRITE_BUFFER,
                .addr      = offset,
                .length    = length * 8,
                .tx_buffer = values,
                .rx_buffer = NULL,
            },
        .address_bits = 8,
    };
    return spi_device_transmit(handle->device, (spi_transaction_t*)&t);
}

esp_err_t sx126x_read_buffer(sx126x_handle_t* handle, uint8_t offset, uint8_t* out_values, size_t length) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    spi_transaction_ext_t t = {
        .base =
            {
                .flags     = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY,
                .cmd       = SX126X_CMD_READ_BUFFER,
                .addr      = offset,
                .length    = length * 8,
                .tx_buffer = NULL,
                .rx_buffer = out_values,
            },
        .address_bits = 8,
        .dummy_bits   = 8,
    };
    return spi_device_transmit(handle->device, (spi_transaction_t*)&t);
}

esp_err_t sx126x_set_dio_irq_params(sx126x_handle_t* handle, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask,
                                    uint16_t dio3_mask) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameters[8];
    parameters[0] = (irq_mask >> 8) & 0xFF;
    parameters[1] = irq_mask & 0xFF;
    parameters[2] = (dio1_mask >> 8) & 0xFF;
    parameters[3] = dio1_mask & 0xFF;
    parameters[4] = (dio2_mask >> 8) & 0xFF;
    parameters[5] = dio2_mask & 0xFF;
    parameters[6] = (dio3_mask >> 8) & 0xFF;
    parameters[7] = dio3_mask & 0xFF;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_DIO_IRQ_PARAMS,
        .length    = 8 * 8,
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_get_irq_status(sx126x_handle_t* handle, uint16_t* out_irq_status, uint8_t* out_status) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[3];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_IRQ_STATUS,
        .length    = 8 * sizeof(result),
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_irq_status) {
        *out_irq_status = (result[1] << 8) | result[2];
    }

    if (out_status) {
        *out_status = result[0];
    }

    return ESP_OK;
}

esp_err_t sx126x_clear_irq_status(sx126x_handle_t* handle, uint16_t irq_mask) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameters[2];
    parameters[0] = (irq_mask >> 8) & 0xFF;
    parameters[1] = irq_mask & 0xFF;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_CLEAR_IRQ_STATUS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_dio2_as_rf_switch_ctrl(sx126x_handle_t* handle, bool enable) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameter = enable ? 0x01 : 0x00;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL,
        .length    = 8,
        .tx_buffer = &parameter,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_dio3_as_txco_ctrl(sx126x_handle_t* handle, float voltage, float delay_us) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t voltage_value = 0;
    if (voltage >= 1.6) voltage_value = 0x01;
    if (voltage >= 1.7) voltage_value = 0x02;
    if (voltage >= 1.8) voltage_value = 0x03;
    if (voltage >= 2.2) voltage_value = 0x04;
    if (voltage >= 2.7) voltage_value = 0x05;
    if (voltage >= 3.0) voltage_value = 0x06;
    if (voltage >= 3.3) voltage_value = 0x07;

    uint32_t delay_value = (uint32_t)(delay_us / 15.62f);

    if (delay_value > 0x00FFFFFF) {
        return ESP_ERR_INVALID_ARG;  // Delay exceeds maximum value
    }

    uint8_t parameters[4];
    parameters[0] = voltage_value;
    parameters[1] = (delay_value >> 16) & 0xFF;
    parameters[2] = (delay_value >> 8) & 0xFF;
    parameters[3] = delay_value & 0xFF;

    printf("Setting DIO3 as TXCO control: voltage=%.2fV, delay=%.2fus, %" PRIu8 ", %" PRIu32 "\n", voltage, delay_us,
           voltage_value, delay_value);

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_DIO3_AS_TXCO_CTRL,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_rf_frequency(sx126x_handle_t* handle, float frequency) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    const float xtal_frequency  = 32.0f;
    uint32_t    frequency_value = (uint32_t)((frequency * (1 << 25))) / xtal_frequency;
    uint8_t     frequency_bytes[4];
    frequency_bytes[0] = (frequency_value >> 24) & 0xFF;
    frequency_bytes[1] = (frequency_value >> 16) & 0xFF;
    frequency_bytes[2] = (frequency_value >> 8) & 0xFF;
    frequency_bytes[3] = frequency_value & 0xFF;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_RF_FREQUENCY,
        .length    = 8 * sizeof(frequency_bytes),
        .tx_buffer = frequency_bytes,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_packet_type(sx126x_handle_t* handle, sx126x_packet_type_t packet_type) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t packet_type_value = (uint8_t)packet_type;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_PACKET_TYPE,
        .length    = 8,
        .tx_buffer = &packet_type_value,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_get_packet_type(sx126x_handle_t* handle, sx126x_packet_type_t* out_packet_type) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t data[2];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_PACKET_TYPE,
        .length    = 8 * sizeof(data),
        .tx_buffer = NULL,
        .rx_buffer = data,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);

    if (out_packet_type && res == ESP_OK) {
        *out_packet_type = data[1];
    }
    return res;
}

esp_err_t sx126x_set_tx_params(sx126x_handle_t* handle, int8_t power, bool pa_is_high_power, uint32_t ramp_time_us) {
    // You can set the PA mode using the SetPaConfig command, make sure to set the pa_is_high_power flag correctly
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    if (pa_is_high_power && (power < -17 || power > 14)) {
        return ESP_ERR_INVALID_ARG;  // Power out of range for high power mode
    } else if (power < -9 || power > 22) {
        return ESP_ERR_INVALID_ARG;  // Power out of range for low power mode
    }

    uint8_t ramp_time_value = 0;
    if (ramp_time_us >= 20) ramp_time_value = 0x01;    // 20 us
    if (ramp_time_us >= 40) ramp_time_value = 0x02;    // 40 us
    if (ramp_time_us >= 80) ramp_time_value = 0x03;    // 80 us
    if (ramp_time_us >= 200) ramp_time_value = 0x04;   // 200 us
    if (ramp_time_us >= 800) ramp_time_value = 0x05;   // 800 us
    if (ramp_time_us >= 1700) ramp_time_value = 0x06;  // 1700 us
    if (ramp_time_us >= 3400) ramp_time_value = 0x07;  // 3400 us

    uint8_t parameters[2] = {(uint8_t)(power), ramp_time_value};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_TX_PARAMS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_modulation_params_lora(sx126x_handle_t* handle, sx126x_lora_spreading_factor_t spreading_factor,
                                            sx126x_lora_bandwidth_t bandwidth, sx126x_lora_coding_rate_t coding_rate,
                                            bool low_data_rate_optimization) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameters[4] = {(uint8_t)spreading_factor, (uint8_t)bandwidth, (uint8_t)coding_rate,
                             low_data_rate_optimization ? 0x01 : 0x00};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_MODULATION_PARAMS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_modulation_params_gfsk(sx126x_handle_t* handle, uint32_t bit_rate,
                                            sx1262_gfsk_pulse_shape_t pulse_shape, sx126x_gfsk_bandwidth_t bandwidth,
                                            uint32_t frequency_deviation) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    const float xtal_frequency            = 32.0f;
    uint32_t    bit_rate_value            = (32 * xtal_frequency) / bit_rate;
    uint32_t    frequency_deviation_value = (uint32_t)((frequency_deviation * (1 << 25))) / xtal_frequency;

    uint8_t parameters[8] = {(bit_rate_value >> 16) & 0xFF,
                             (bit_rate_value >> 8) & 0xFF,
                             bit_rate_value & 0xFF,
                             (uint8_t)pulse_shape,
                             (uint8_t)bandwidth,
                             (frequency_deviation_value >> 16) & 0xFF,
                             (frequency_deviation_value >> 8) & 0xFF,
                             frequency_deviation_value & 0xFF};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_MODULATION_PARAMS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_packet_params_lora(sx126x_handle_t* handle, uint16_t preamble_length, bool fixed_packet_length,
                                        uint8_t payload_length, bool crc_enabled, bool inverted_iq) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameters[6] = {
        (preamble_length >> 8) & 0xFF,      // Preamble length MSB
        preamble_length & 0xFF,             // Preamble length LSB
        fixed_packet_length ? 0x01 : 0x00,  // Header type (0: variable, 1: fixed)
        payload_length,                     // Payload length
        crc_enabled ? 0x01 : 0x00,          // CRC enabled flag
        inverted_iq ? 0x01 : 0x00,          // Inverted IQ flag
    };

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_PACKET_PARAMS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_packet_params_gfsk(sx126x_handle_t* handle, uint16_t preamble_length,
                                        uint8_t preamble_detector_length, uint8_t sync_word_length,
                                        uint8_t address_compensation, bool packet_length_variable,
                                        uint8_t payload_length, uint8_t crc_type, bool whitening) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameters[9] = {
        (preamble_length >> 8) & 0xFF,         // Preamble length MSB
        preamble_length & 0xFF,                // Preamble length LSB
        preamble_detector_length,              // Preamble detector length
        sync_word_length,                      // Sync word length
        address_compensation,                  // Address compensation
        packet_length_variable ? 0x01 : 0x00,  // Packet length variable flag
        payload_length,                        // Payload length
        crc_type,                              // CRC type
        whitening ? 0x01 : 0x00                // Whitening flag
    };

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_PACKET_PARAMS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_cad_params(sx126x_handle_t* handle, sx126x_cad_symbol_num_t symbol_num, uint8_t det_peak,
                                uint8_t det_min, bool exit_mode, uint32_t timeout) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t parameters[7] = {
        (uint8_t)symbol_num,      //
        det_peak,                 //
        det_min,                  //
        exit_mode ? 0x01 : 0x00,  //
        (timeout >> 16) & 0xFF,   //
        (timeout >> 8) & 0xFF,    //
        timeout & 0xFF,           //
    };

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_CAD_PARAMS,
        .length    = 8 * sizeof(parameters),
        .tx_buffer = parameters,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_set_lora_symb_num_timeout(sx126x_handle_t* handle, uint8_t symb_num) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT,
        .length    = 8,
        .tx_buffer = &symb_num,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_get_status(sx126x_handle_t* handle, uint8_t* out_command_status, uint8_t* out_chip_mode) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t status = 0;

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_STATUS,
        .length    = 8,
        .tx_buffer = NULL,
        .rx_buffer = &status,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_command_status) {
        *out_command_status = (status >> 1) & 0x07;
    }

    if (out_chip_mode) {
        *out_chip_mode = (status >> 4) & 0x07;
    }

    return res;
}

esp_err_t sx126x_get_rx_buffer_status(sx126x_handle_t* handle, uint8_t* out_payload_length,
                                      uint8_t* out_start_pointer) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[3];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_RX_BUFFER_STATUS,
        .length    = 8 * sizeof(result),
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_payload_length) {
        *out_payload_length = result[1];
    }

    if (out_start_pointer) {
        *out_start_pointer = result[2];
    }

    return res;
}

esp_err_t sx126x_get_packet_status_lora(sx126x_handle_t* handle, uint8_t* out_rx_status, uint8_t* out_rssi_sync,
                                        uint8_t* out_rssi_avg) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[4] = {0};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_PACKET_STATUS,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_rx_status) {
        *out_rx_status = result[1];
    }

    if (out_rssi_sync) {
        *out_rssi_sync = result[2];
    }

    if (out_rssi_avg) {
        *out_rssi_avg = result[3];
    }

    return res;
}

esp_err_t sx126x_get_packet_status_gfsk(sx126x_handle_t* handle, uint8_t* out_rx_status, uint8_t* out_rssi_sync,
                                        uint8_t* out_rssi_avg) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[4] = {0};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_PACKET_STATUS,
        .length    = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_rx_status) {
        *out_rx_status = result[1];
    }

    if (out_rssi_sync) {
        *out_rssi_sync = result[2];
    }

    if (out_rssi_avg) {
        *out_rssi_avg = result[3];
    }

    return res;
}

esp_err_t sx126x_get_rssi_inst(sx126x_handle_t* handle, float* out_signal_power) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[2];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_RSSI_INST,
        .length    = 8 * sizeof(result),
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);

    if (out_signal_power && res == ESP_OK) {
        *out_signal_power = -result[1] / 2.0f;
    }

    return res;
}

esp_err_t sx126x_get_stats_lora(sx126x_handle_t* handle, uint16_t* out_nb_pkt_received, uint16_t* out_nb_pkt_crc_error,
                                uint16_t* out_nb_pkt_header_error) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[7];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_STATS,
        .length    = 8 * sizeof(result),
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_nb_pkt_received) {
        *out_nb_pkt_received = (result[1] << 8) | result[2];
    }

    if (out_nb_pkt_crc_error) {
        *out_nb_pkt_crc_error = (result[3] << 8) | result[4];
    }

    if (out_nb_pkt_header_error) {
        *out_nb_pkt_header_error = (result[5] << 8) | result[6];
    }

    return res;
}

esp_err_t sx126x_get_stats_gfsk(sx126x_handle_t* handle, uint16_t* out_nb_pkt_received, uint16_t* out_nb_pkt_crc_error,
                                uint16_t* out_nb_pkt_length_error) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[7];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_STATS,
        .length    = 8 * sizeof(result),
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_nb_pkt_received) {
        *out_nb_pkt_received = (result[1] << 8) | result[2];
    }

    if (out_nb_pkt_crc_error) {
        *out_nb_pkt_crc_error = (result[3] << 8) | result[4];
    }

    if (out_nb_pkt_length_error) {
        *out_nb_pkt_length_error = (result[5] << 8) | result[6];
    }

    return res;
}

esp_err_t sx126x_reset_stats(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    const uint8_t dummy[6] = {0};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_RESET_STATS,
        .length    = 8 * sizeof(dummy),
        .tx_buffer = dummy,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(handle->device, &t);
}

esp_err_t sx126x_get_device_errors(sx126x_handle_t* handle, uint16_t* out_errors) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[3];

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_GET_DEVICE_ERRORS,
        .length    = 8 * sizeof(result),
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) return res;

    if (out_errors) {
        *out_errors = (result[1] << 8) | result[2];
    }

    return res;
}

esp_err_t sx126x_clear_device_errors(sx126x_handle_t* handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t result[2] = {0};

    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = SX126X_CMD_CLEAR_DEVICE_ERRORS,
        .length    = 8 * 2,
        .tx_buffer = NULL,
        .rx_buffer = result,
    };
    esp_err_t res = spi_device_transmit(handle->device, &t);
    if (res != ESP_OK) {
        return res;
    }

    // Can use status, is in result[0] and result[1]
    printf("Device errors cleared, status: %02X %02X\n", result[0], result[1]);

    return ESP_OK;
}

esp_err_t sx1262_reset(sx126x_handle_t* handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t res = gpio_set_level(handle->reset, 0);
    if (res != ESP_OK) {
        return res;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    res = gpio_set_level(handle->reset, 1);
    if (res != ESP_OK) {
        return res;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

esp_err_t sx126x_init(sx126x_handle_t* handle, spi_host_device_t spi_host_id, gpio_num_t nss, gpio_num_t reset,
                      gpio_num_t dio1, gpio_num_t busy) {
    esp_err_t res;

    ESP_LOGI(TAG, "Initializing LoRa SPI device...");

    spi_device_interface_config_t devcfg = {
        .command_bits   = 8,        // SX1262 uses 8-bit commands
        .clock_speed_hz = 1000000,  // SX1262 support maximum 16MHz SPI clock
        .mode           = 0,
        .spics_io_num   = nss,
        .queue_size     = 8,
    };
    res = spi_bus_add_device(spi_host_id, &devcfg, &handle->device);
    if (res != ESP_OK) {
        return res;
    }

    gpio_config_t gpio_reset_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(reset),
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&gpio_reset_conf);

    handle->reset = reset;

    gpio_config_t gpio_dio1_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(dio1),
        .pull_down_en = 0,
        .pull_up_en   = 1,
    };
    gpio_config(&gpio_dio1_conf);

    handle->dio1 = dio1;

    gpio_config_t gpio_busy_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(busy),
        .pull_down_en = 0,
        .pull_up_en   = 1,
    };
    gpio_config(&gpio_busy_conf);

    handle->busy = busy;

    res = sx1262_reset(handle);

    return res;
}

bool sx126x_is_busy(sx126x_handle_t* handle) {
    if (handle == NULL) return false;
    return gpio_get_level(handle->busy) == 1;
}
