#pragma once
#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"

typedef struct {
    spi_device_handle_t device;
    gpio_num_t          reset;
    gpio_num_t          dio1;
    gpio_num_t          busy;
} sx126x_handle_t;

typedef enum {
    SX126X_PACKET_TYPE_GFSK    = 0x00,
    SX126X_PACKET_TYPE_LORA    = 0x01,
    SX126X_PACKET_TYPE_LR_FHSS = 0x03,
} sx126x_packet_type_t;

typedef enum {
    SX126X_FALLBACK_MODE_FS         = 0x40,
    SX126X_FALLBACK_MODE_STDBY_XOSC = 0x30,
    SX126X_FALLBACK_MODE_STDBY_RC   = 0x20,
} sx126x_fallback_mode_t;

typedef enum {
    SX126X_CAD_ON_1_SYMB  = 0x00,
    SX126X_CAD_ON_2_SYMB  = 0x01,
    SX126X_CAD_ON_4_SYMB  = 0x02,
    SX126X_CAD_ON_8_SYMB  = 0x03,
    SX126X_CAD_ON_16_SYMB = 0x04,
} sx126x_cad_symbol_num_t;

typedef enum {
    SK126X_GFSK_PULSE_SHAPE_NO_FILTER       = 0x00,
    SX126X_GFSK_PULSE_SHAPE_GAUSSIAN_BT_0_3 = 0x08,
    SX126X_GFSK_PULSE_SHAPE_GAUSSIAN_BT_0_5 = 0x09,
    SX126X_GFSK_PULSE_SHAPE_GAUSSIAN_BT_0_7 = 0x0A,
    SX126X_GFSK_PULSE_SHAPE_GAUSSIAN_BT_1_0 = 0x0B,
} sx1262_gfsk_pulse_shape_t;

typedef enum {
    SX126X_GFSK_BANDWIDTH_4800   = 0x1F,
    SX126X_GFSK_BANDWIDTH_5800   = 0x17,
    SX126X_GFSK_BANDWIDTH_7300   = 0x0F,
    SX126X_GFSK_BANDWIDTH_9700   = 0x1E,
    SX126X_GFSK_BANDWIDTH_11700  = 0x16,
    SX126X_GFSK_BANDWIDTH_14600  = 0x0E,
    SX126X_GFSK_BANDWIDTH_19500  = 0x1D,
    SX126X_GFSK_BANDWIDTH_23400  = 0x15,
    SX126X_GFSK_BANDWIDTH_29300  = 0x0D,
    SX126X_GFSK_BANDWIDTH_39000  = 0x1C,
    SX126X_GFSK_BANDWIDTH_46900  = 0x14,
    SX126X_GFSK_BANDWIDTH_58600  = 0x0C,
    SX126X_GFSK_BANDWIDTH_78200  = 0x1B,
    SX126X_GFSK_BANDWIDTH_93800  = 0x13,
    SX126X_GFSK_BANDWIDTH_117300 = 0x0B,
    SX126X_GFSK_BANDWIDTH_156200 = 0x1A,
    SX126X_GFSK_BANDWIDTH_187200 = 0x12,
    SX126X_GFSK_BANDWIDTH_234300 = 0x0A,
    SX126X_GFSK_BANDWIDTH_312000 = 0x19,
    SX126X_GFSK_BANDWIDTH_373600 = 0x11,
    SX126X_GFSK_BANDWIDTH_467000 = 0x09,
} sx126x_gfsk_bandwidth_t;

typedef enum {
    SX126X_LORA_SPREADING_FACTOR_5  = 0x05,
    SX126X_LORA_SPREADING_FACTOR_6  = 0x06,
    SX126X_LORA_SPREADING_FACTOR_7  = 0x07,
    SX126X_LORA_SPREADING_FACTOR_8  = 0x08,
    SX126X_LORA_SPREADING_FACTOR_9  = 0x09,
    SX126X_LORA_SPREADING_FACTOR_10 = 0x0A,
    SX126X_LORA_SPREADING_FACTOR_11 = 0x0B,
    SX126X_LORA_SPREADING_FACTOR_12 = 0x0C,
} sx126x_lora_spreading_factor_t;

typedef enum {
    SX126X_LORA_BANDWIDTH_7   = 0x00,
    SX126X_LORA_BANDWIDTH_10  = 0x08,
    SX126X_LORA_BANDWIDTH_15  = 0x01,
    SX126X_LORA_BANDWIDTH_20  = 0x09,
    SX126X_LORA_BANDWIDTH_31  = 0x02,
    SX126X_LORA_BANDWIDTH_41  = 0x0A,
    SX126X_LORA_BANDWIDTH_62  = 0x03,
    SX126X_LORA_BANDWIDTH_125 = 0x04,
    SX126X_LORA_BANDWIDTH_250 = 0x05,
    SX126X_LORA_BANDWIDTH_500 = 0x06,
} sx126x_lora_bandwidth_t;

typedef enum {
    SX126X_LORA_CODING_RATE_4_5    = 0x01,
    SX126X_LORA_CODING_RATE_4_6    = 0x02,
    SX126X_LORA_CODING_RATE_4_7    = 0x03,
    SX126X_LORA_CODING_RATE_4_8    = 0x04,
    SX126X_LORA_CODING_RATE_4_5_LI = 0x05,
    SX126X_LORA_CODING_RATE_4_6_LI = 0x06,
    SX126X_LORA_CODING_RATE_4_8_LI = 0x07,
} sx126x_lora_coding_rate_t;

typedef enum {
    SX126X_ERROR_RC64K_CALIB_ERR = 0x0001,
    SX126X_ERROR_RC13M_CALIB_ERR = 0x0002,
    SX126X_ERROR_PLL_CALIB_ERR   = 0x0004,
    SX126X_ERROR_ADC_CALIB_ERR   = 0x0008,
    SX126X_ERROR_IMG_CALIB_ERR   = 0x0010,
    SX126X_XOSC_START_ERR        = 0x0020,
    SX126X_PLL_LOCK_ERR          = 0x0040,
    SX126X_PA_RAMP_ERR           = 0x0100,
} sx126x_error_t;

esp_err_t sx126x_set_op_mode_sleep(sx126x_handle_t* handle, bool warm_start, bool rtc_timeout_disable);
esp_err_t sx126x_set_op_mode_standby(sx126x_handle_t* handle, bool use_xosc);
esp_err_t sx126x_set_op_mode_fs(sx126x_handle_t* handle);
esp_err_t sx126x_set_op_mode_tx(sx126x_handle_t* handle);
esp_err_t sx126x_set_op_mode_rx(sx126x_handle_t* handle);
esp_err_t sx126x_stop_timer_on_preamble(sx126x_handle_t* handle, bool stop);
esp_err_t sx126x_set_rx_duty_cycle(sx126x_handle_t* handle, uint32_t rx_period, uint32_t sleep_period);
esp_err_t sx126x_set_cad(sx126x_handle_t* handle);
esp_err_t sx126x_tx_continuous_wave(sx126x_handle_t* handle);
esp_err_t sx126x_set_tx_infinite_preamble(sx126x_handle_t* handle);
esp_err_t sx126x_set_regulator_mode(sx126x_handle_t* handle, bool use_dc_dc);
esp_err_t sx126x_calibrate(sx126x_handle_t* handle, bool rc64k, bool rc13m, bool pll, bool adc_pulse, bool adc_bulk_n,
                           bool adc_bulk_p, bool image);
esp_err_t sx126x_calibrate_image(sx126x_handle_t* handle, uint8_t frequency1, uint8_t frequency2);
esp_err_t sx126x_set_pa_config(sx126x_handle_t* handle, uint8_t pa_duty_cycle, uint8_t hp_max);
esp_err_t sx126x_set_rx_tx_fallback_mode(sx126x_handle_t* handle, sx126x_fallback_mode_t fallback_mode);
esp_err_t sx126x_write_register(sx126x_handle_t* handle, uint16_t address, const uint8_t* values, size_t length);
esp_err_t sx126x_read_register(sx126x_handle_t* handle, uint16_t address, uint8_t* out_values, size_t length);
esp_err_t sx126x_write_buffer(sx126x_handle_t* handle, uint8_t offset, const uint8_t* values, size_t length);
esp_err_t sx126x_read_buffer(sx126x_handle_t* handle, uint8_t offset, uint8_t* out_values, size_t length);
esp_err_t sx126x_set_dio_irq_params(sx126x_handle_t* handle, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask,
                                    uint16_t dio3_mask);
esp_err_t sx126x_get_irq_status(sx126x_handle_t* handle, uint16_t* out_irq_status, uint8_t* out_status);
esp_err_t sx126x_clear_irq_status(sx126x_handle_t* handle, uint16_t irq_mask);
esp_err_t sx126x_set_dio2_as_rf_switch_ctrl(sx126x_handle_t* handle, bool enable);
esp_err_t sx126x_set_dio3_as_txco_ctrl(sx126x_handle_t* handle, float voltage, float delay_us);
esp_err_t sx126x_set_rf_frequency(sx126x_handle_t* handle, float frequency);
esp_err_t sx126x_set_packet_type(sx126x_handle_t* handle, sx126x_packet_type_t packet_type);
esp_err_t sx126x_get_packet_type(sx126x_handle_t* handle, sx126x_packet_type_t* out_packet_type);
esp_err_t sx126x_set_tx_params(sx126x_handle_t* handle, int8_t power, bool pa_is_high_power, uint32_t ramp_time_us);
esp_err_t sx126x_set_modulation_params_lora(sx126x_handle_t* handle, sx126x_lora_spreading_factor_t spreading_factor,
                                            sx126x_lora_bandwidth_t bandwidth, sx126x_lora_coding_rate_t coding_rate,
                                            bool low_data_rate_optimization);
esp_err_t sx126x_set_modulation_params_gfsk(sx126x_handle_t* handle, uint32_t bit_rate,
                                            sx1262_gfsk_pulse_shape_t pulse_shape, sx126x_gfsk_bandwidth_t bandwidth,
                                            uint32_t frequency_deviation);
esp_err_t sx126x_set_packet_params_lora(sx126x_handle_t* handle, uint16_t preamble_length, bool fixed_packet_length,
                                        uint8_t payload_length, bool crc_enabled, bool inverted_iq);
esp_err_t sx126x_set_packet_params_gfsk(sx126x_handle_t* handle, uint16_t preamble_length,
                                        uint8_t preamble_detector_length, uint8_t sync_word_length,
                                        uint8_t address_compensation, bool packet_length_variable,
                                        uint8_t payload_length, uint8_t crc_type, bool whitening);
esp_err_t sx126x_set_cad_params(sx126x_handle_t* handle, sx126x_cad_symbol_num_t symbol_num, uint8_t det_peak,
                                uint8_t det_min, bool exit_mode, uint32_t timeout);
esp_err_t sx126x_set_lora_symb_num_timeout(sx126x_handle_t* handle, uint8_t symb_num);
esp_err_t sx126x_get_status(sx126x_handle_t* handle, uint8_t* out_command_status, uint8_t* out_chip_mode);
esp_err_t sx126x_get_rx_buffer_status(sx126x_handle_t* handle, uint8_t* out_payload_length, uint8_t* out_start_pointer);
esp_err_t sx126x_get_packet_status_lora(sx126x_handle_t* handle, uint8_t* out_rx_status, uint8_t* out_rssi_sync,
                                        uint8_t* out_rssi_avg);
esp_err_t sx126x_get_packet_status_gfsk(sx126x_handle_t* handle, uint8_t* out_rx_status, uint8_t* out_rssi_sync,
                                        uint8_t* out_rssi_avg);
esp_err_t sx126x_get_rssi_inst(sx126x_handle_t* handle, float* out_signal_power);
esp_err_t sx126x_get_stats_lora(sx126x_handle_t* handle, uint16_t* out_nb_pkt_received, uint16_t* out_nb_pkt_crc_error,
                                uint16_t* out_nb_pkt_header_error);
esp_err_t sx126x_get_stats_gfsk(sx126x_handle_t* handle, uint16_t* out_nb_pkt_received, uint16_t* out_nb_pkt_crc_error,
                                uint16_t* out_nb_pkt_length_error);
esp_err_t sx126x_reset_stats(sx126x_handle_t* handle);
esp_err_t sx126x_get_device_errors(sx126x_handle_t* handle, uint16_t* out_errors);
esp_err_t sx126x_clear_device_errors(sx126x_handle_t* handle);
esp_err_t sx1262_reset(sx126x_handle_t* handle);
esp_err_t sx126x_init(sx126x_handle_t* handle, spi_host_device_t spi_host_id, gpio_num_t nss, gpio_num_t reset,
                      gpio_num_t dio1, gpio_num_t busy);
bool      sx126x_is_busy(sx126x_handle_t* handle);
