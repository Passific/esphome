#pragma once

#include "epaper_spi.h"
#include "epaper_it8951_defs.h"

namespace esphome::epaper_spi {

struct IT8951DevInfo {
  uint16_t panel_width{0};
  uint16_t panel_height{0};
  uint16_t img_buf_addr_l{0};
  uint16_t img_buf_addr_h{0};
  uint16_t fw_version[8]{};
  uint16_t lut_version[8]{};
};

class EPaperIT8951 : public EPaperBase {
 public:
  EPaperIT8951(const char *name, uint16_t width, uint16_t height, const uint8_t *init_sequence,
                size_t init_sequence_length)
      : EPaperBase(name, width, height, init_sequence, init_sequence_length,
                   display::DisplayType::DISPLAY_TYPE_GRAYSCALE) {
    // IT8951 uses 4 bits per pixel (2 pixels per byte)
    this->row_width_ = static_cast<uint16_t>((static_cast<uint32_t>(width) + 1) / 2);
    this->buffer_length_ = static_cast<size_t>(this->row_width_) * static_cast<size_t>(height);
  }

  // Component overrides
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  /// Named update modes: "GC16" (default/full), "DU" (fast), "GL16", "GLR16", "GLD16", "DU4", "A2", "INIT".
  void update_mode(const std::string &mode) override;
  void set_vcom(uint16_t vcom) { this->vcom_ = vcom; }
  void set_force_1bpp(bool force_1bpp) { this->force_1bpp_ = force_1bpp; }

  // Drawing overrides
  void fill(Color color) override;
  void clear() override { this->fill(COLOR_OFF); }
  void draw_pixel_at(int x, int y, Color color) override;

 protected:
  // EPaperBase required overrides
  bool reset() override;
  bool transfer_data() override;
  void refresh_screen(bool partial) override;
  void power_on() override;
  void power_off() override;
  void deep_sleep() override;

 private:
  // IT8951 SPI protocol methods
  void write_two_byte16_(uint16_t type, uint16_t cmd);
  uint16_t read_word_();
  void read_words_(uint16_t *buf, uint32_t word_count);
  void write_command_(uint16_t cmd);
  void write_word_(uint16_t cmd);
  void write_reg_(uint16_t addr, uint16_t data);
  void set_target_memory_addr_(uint16_t tar_addr_l, uint16_t tar_addr_h);
  void write_args_(uint16_t cmd, const uint16_t *args, uint16_t length);
  void write_words_separate_(uint16_t cmd, const uint16_t *args, uint16_t length);

  // Display area management
  void set_area_(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool force_area = false);
  void update_area_(uint16_t x, uint16_t y, uint16_t w, uint16_t h, UpdateModeE mode);
  void update_area_1bpp_(uint16_t x, uint16_t y, uint16_t w, uint16_t h, UpdateModeE mode, uint8_t bg_gray,
                         uint8_t fg_gray);

  // Busy/idle management (IT8951 busy pin polarity: HIGH = ready, LOW = busy)
  void wait_busy_(uint32_t timeout = 5000);
  bool is_display_busy_();
  void wait_for_display_ready_(uint32_t timeout = 30000);
  void sleep_ms_(uint32_t ms);

  // VCOM management
  uint16_t get_vcom_();
  void write_vcom_(uint16_t selector, uint16_t vcom);
  bool try_write_vcom_selector_(uint16_t selector);

  // Transfer helpers
  bool prepare_transfer_(UpdateModeE &mode);
  bool transfer_row_data_();
  bool transfer_row_data_1bpp_();
  void start_update_(UpdateModeE hw_mode);
  void process_state_();
  void set_state_(EPaperState state, uint16_t delay = 0);
  bool framebuffer_is_binary_();
  uint8_t get_pixel_nibble_(uint16_t x, uint16_t y);
  bool is_seeed_model_() const;
  void write_enable_pins_(bool value);
  void hardware_reset_();
  void power_cycle_();
  bool has_valid_dev_info_() const;
  void get_dev_info_();
  bool probe_controller_(const char *label, bool send_sys_run, uint16_t selector);

  // Color conversion
  uint8_t color_to_nibble_(const Color &color) const;

  // IT8951 device info
  IT8951DevInfo dev_info_{};
  uint16_t us_img_buf_addr_l_{0x36E0};
  uint16_t us_img_buf_addr_h_{0x0012};
  uint16_t m_endian_type_{0};
  uint16_t m_pix_bpp_{0};
  uint16_t vcom_{IT8951_DEFAULT_VCOM};
  uint16_t vcom_readback_{0};
  uint16_t vcom_write_selector_{0};
  const char *probe_path_{nullptr};
  bool use_1bpp_{false};
  bool force_1bpp_{false};
  uint32_t configured_data_rate_{4'000'000};

  // State tracking
  bool initialized_{false};
  uint32_t partial_update_{0};
  UpdateModeE pending_mode_{UPDATE_MODE_NONE};
  UpdateModeE queued_update_mode_{UPDATE_MODE_NONE};
  uint16_t pending_x_{0}, pending_y_{0}, pending_w_{0}, pending_h_{0};
  uint16_t transfer_row_{0};
  uint32_t update_started_at_{0};
  bool update_timing_active_{false};
  bool update_pending_{false};
  bool update_buffer_prepared_{false};
  bool waiting_for_controller_ready_{false};
};

}  // namespace esphome::epaper_spi

