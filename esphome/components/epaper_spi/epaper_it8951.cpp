#include <algorithm>
#include <array>
#include <cstring>
#include <string>

#include "epaper_it8951.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::epaper_spi {

static const char *const TAG = "epaper_spi.it8951";
static constexpr uint32_t IT8951_SPI_PROBE_FREQUENCY = 1'000'000;
static constexpr uint32_t IT8951_SLEEP_CHUNK_MS = 50;
static constexpr uint32_t IT8951_HRDY_TIMEOUT_MS = 3'000;
static constexpr size_t IT8951_MAX_4BPP_ROW_BYTES = 936;
static constexpr size_t IT8951_MAX_1BPP_ROW_BYTES = 234;

struct ProbeAttempt {
  const char *label;
  bool send_sys_run;
  uint16_t selector;
};

static constexpr ProbeAttempt SEEED_PROBE_ATTEMPTS[] = {
    {"cold read", false, 0},
    {"wake then read", true, 0},
    {"wake + VCOM 0x0001", true, IT8951_I80_CMD_VCOM_WRITE},
    {"wake + VCOM 0x0002", true, IT8951_I80_CMD_VCOM_WRITE_ALT},
};

static UpdateModeE parse_update_mode(const std::string &mode) {
  if (mode == "DU" || mode == "fast")
    return UPDATE_MODE_DU;
  if (mode == "GC16" || mode == "full")
    return UPDATE_MODE_GC16;
  if (mode == "GL16")
    return UPDATE_MODE_GL16;
  if (mode == "GLR16")
    return UPDATE_MODE_GLR16;
  if (mode == "GLD16")
    return UPDATE_MODE_GLD16;
  if (mode == "DU4")
    return UPDATE_MODE_DU4;
  if (mode == "A2")
    return UPDATE_MODE_A2;
  if (mode == "INIT")
    return UPDATE_MODE_INIT;
  return UPDATE_MODE_NONE;
}

static uint16_t encode_uint16(uint8_t a, uint8_t b) { return static_cast<uint16_t>(a) << 8 | b; }

bool EPaperIT8951::is_seeed_model_() const {
  return strcmp(this->name_, "seeed-reterminal-e1003") == 0 || strcmp(this->name_, "SEEED-RETERMINAL-E1003") == 0 ||
         strcmp(this->name_, "seeed-ee03") == 0 || strcmp(this->name_, "SEEED-EE03") == 0;
}

void EPaperIT8951::sleep_ms_(uint32_t ms) {
  while (ms != 0) {
    const uint32_t chunk_ms = std::min<uint32_t>(ms, IT8951_SLEEP_CHUNK_MS);
    delay(chunk_ms);
    App.feed_wdt();
    ms -= chunk_ms;
  }
}

void EPaperIT8951::write_enable_pins_(bool value) {
  for (auto *pin : this->enable_pins_) {
    if (pin != nullptr)
      pin->digital_write(value);
  }
}

void EPaperIT8951::hardware_reset_() {
  if (this->cs_ != nullptr) {
    this->cs_->digital_write(true);
  }
  this->write_enable_pins_(true);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(true);
    this->sleep_ms_(50);
    this->reset_pin_->digital_write(false);
    this->sleep_ms_(10);
    this->reset_pin_->digital_write(true);
    this->sleep_ms_(10);
  }
}

void EPaperIT8951::power_cycle_() {
  if (this->cs_ != nullptr) {
    this->cs_->digital_write(true);
  }
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(true);
  }
  this->write_enable_pins_(false);
  this->sleep_ms_(100);
  this->write_enable_pins_(true);
  this->sleep_ms_(500);
  this->hardware_reset_();
  this->sleep_ms_(1500);
}

void EPaperIT8951::write_two_byte16_(uint16_t type, uint16_t cmd) {
  this->wait_busy_();
  this->enable();
  this->write_byte16(type);
  this->wait_busy_();
  this->write_byte16(cmd);
  this->disable();
}

uint16_t EPaperIT8951::read_word_() {
  this->wait_busy_();
  this->enable();
  this->write_byte16(IT8951_PACKET_TYPE_READ);
  this->wait_busy_();
  uint8_t dummy[2];
  this->read_array(dummy, sizeof(dummy));
  this->wait_busy_();
  uint8_t recv[2];
  this->read_array(recv, sizeof(recv));
  this->disable();
  return encode_uint16(recv[0], recv[1]);
}

void EPaperIT8951::read_words_(uint16_t *buf, uint32_t word_count) {
  this->wait_busy_();
  this->enable();
  this->write_byte16(IT8951_PACKET_TYPE_READ);
  this->wait_busy_();
  uint8_t dummy[2];
  this->read_array(dummy, sizeof(dummy));
  this->wait_busy_();

  for (uint32_t i = 0; i < word_count; i++) {
    uint8_t recv[2];
    this->read_array(recv, sizeof(recv));
    buf[i] = encode_uint16(recv[0], recv[1]);
  }

  this->disable();
}

void EPaperIT8951::write_command_(uint16_t cmd) { this->write_two_byte16_(IT8951_PACKET_TYPE_CMD, cmd); }

void EPaperIT8951::write_word_(uint16_t data) { this->write_two_byte16_(IT8951_PACKET_TYPE_WRITE, data); }

void EPaperIT8951::write_reg_(uint16_t addr, uint16_t data) {
  this->write_command_(IT8951_TCON_REG_WR);
  this->wait_busy_();
  this->enable();
  this->write_byte16(IT8951_PACKET_TYPE_WRITE);
  this->wait_busy_();
  this->write_byte16(addr);
  this->wait_busy_();
  this->write_byte16(data);
  this->disable();
}

void EPaperIT8951::set_target_memory_addr_(uint16_t tar_addr_l, uint16_t tar_addr_h) {
  this->write_reg_(IT8951_LISAR + 2, tar_addr_h);
  this->write_reg_(IT8951_LISAR, tar_addr_l);
}

void EPaperIT8951::write_args_(uint16_t cmd, const uint16_t *args, uint16_t length) {
  this->write_command_(cmd);
  this->wait_busy_();
  this->enable();
  this->write_byte16(IT8951_PACKET_TYPE_WRITE);
  this->wait_busy_();
  for (uint16_t i = 0; i < length; i++) {
    this->write_byte16(args[i]);
  }
  this->disable();
}

void EPaperIT8951::write_words_separate_(uint16_t cmd, const uint16_t *args, uint16_t length) {
  this->write_command_(cmd);
  for (uint16_t i = 0; i < length; i++) {
    this->write_word_(args[i]);
  }
}

void EPaperIT8951::set_area_(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool force_area) {
  if (!force_area && x == 0 && y == 0 && w == this->get_width_internal() && h == this->get_height_internal()) {
    const uint16_t args[1] = {static_cast<uint16_t>((this->m_endian_type_ << 8) | (this->m_pix_bpp_ << 4))};
    this->write_args_(IT8951_TCON_LD_IMG, args, 1);
    return;
  }

  const uint16_t args[5] = {
      static_cast<uint16_t>((this->m_endian_type_ << 8) | (this->m_pix_bpp_ << 4)), x, y, w, h,
  };
  if (this->is_seeed_model_()) {
    this->write_words_separate_(IT8951_TCON_LD_IMG_AREA, args, 5);
  } else {
    this->write_args_(IT8951_TCON_LD_IMG_AREA, args, 5);
  }
}

void EPaperIT8951::wait_busy_(uint32_t timeout) {
  if (this->busy_pin_ == nullptr)
    return;

  const uint32_t start_time = millis();
  while (!this->busy_pin_->digital_read()) {
    if (millis() - start_time > timeout) {
      ESP_LOGE(TAG, "Busy pin timeout (%ums)", timeout);
      break;
    }
    App.feed_wdt();
    delay(1);
  }
}

bool EPaperIT8951::is_display_busy_() {
  this->write_command_(IT8951_TCON_REG_RD);
  this->write_word_(IT8951_LUTAFSR);
  return this->read_word_() != 0;
}

void EPaperIT8951::wait_for_display_ready_(uint32_t timeout) {
  const uint32_t start_time = millis();
  while (this->is_display_busy_()) {
    if (millis() - start_time > timeout) {
      ESP_LOGW(TAG, "Display-ready timeout while waiting for LUTAFSR to clear");
      break;
    }
    App.feed_wdt();
    delay(10);
  }
}

void EPaperIT8951::update_area_(uint16_t x, uint16_t y, uint16_t w, uint16_t h, UpdateModeE mode) {
  if (this->is_seeed_model_()) {
    if (!this->use_1bpp_) {
      this->write_command_(IT8951_TCON_REG_RD);
      this->write_word_(IT8951_UP1SR + 2);
      const uint16_t up1sr_high = this->read_word_();
      this->write_reg_(IT8951_UP1SR + 2, static_cast<uint16_t>(up1sr_high & ~(1U << 2)));
    }
    const uint16_t args[5] = {x, y, w, h, static_cast<uint16_t>(mode)};
    this->write_words_separate_(IT8951_I80_CMD_DPY_AREA, args, 5);
    return;
  }

  const uint16_t args[7] = {x, y, w, h, static_cast<uint16_t>(mode), this->us_img_buf_addr_l_, this->us_img_buf_addr_h_};
  this->write_args_(IT8951_I80_CMD_DPY_BUF_AREA, args, 7);
}

void EPaperIT8951::update_area_1bpp_(uint16_t x, uint16_t y, uint16_t w, uint16_t h, UpdateModeE mode,
                                      uint8_t bg_gray, uint8_t fg_gray) {
  this->write_command_(IT8951_TCON_REG_RD);
  this->write_word_(IT8951_UP1SR + 2);
  const uint16_t up1sr_high = this->read_word_();
  this->write_reg_(IT8951_UP1SR + 2, static_cast<uint16_t>(up1sr_high | (1U << 2)));
  this->write_reg_(IT8951_BGVR, static_cast<uint16_t>((uint16_t(bg_gray) << 8) | fg_gray));
  this->update_area_(x, y, w, h, mode);
}

bool EPaperIT8951::reset() {
  this->hardware_reset_();
  return true;
}

uint16_t EPaperIT8951::get_vcom_() {
  this->write_command_(IT8951_I80_CMD_VCOM);
  this->write_word_(IT8951_I80_CMD_VCOM_READ);
  return this->read_word_();
}

void EPaperIT8951::write_vcom_(uint16_t selector, uint16_t vcom) {
  this->write_command_(IT8951_I80_CMD_VCOM);
  this->write_word_(selector);
  this->write_word_(vcom);
}

bool EPaperIT8951::try_write_vcom_selector_(uint16_t selector) {
  this->write_vcom_(selector, this->vcom_);
  this->vcom_readback_ = this->get_vcom_();
  if (this->vcom_readback_ == this->vcom_) {
    this->vcom_write_selector_ = selector;
    return true;
  }
  return false;
}

bool EPaperIT8951::has_valid_dev_info_() const {
  return this->dev_info_.panel_width > 0 && this->dev_info_.panel_width < 10000 && this->dev_info_.panel_height > 0 &&
         this->dev_info_.panel_height < 10000;
}

void EPaperIT8951::get_dev_info_() {
  memset(&this->dev_info_, 0, sizeof(this->dev_info_));
  this->write_command_(IT8951_I80_CMD_GET_DEV_INFO);
  this->read_words_(reinterpret_cast<uint16_t *>(&this->dev_info_), sizeof(this->dev_info_) / sizeof(uint16_t));
}

bool EPaperIT8951::probe_controller_(const char *label, bool send_sys_run, uint16_t selector) {
  this->probe_path_ = label;
  this->vcom_readback_ = 0;
  this->vcom_write_selector_ = 0;
  memset(&this->dev_info_, 0, sizeof(this->dev_info_));

  if (send_sys_run) {
    this->write_command_(IT8951_TCON_SYS_RUN);
    this->sleep_ms_(10);
  }

  if (selector != 0) {
    this->write_vcom_(selector, this->vcom_);
    this->vcom_readback_ = this->get_vcom_();
    if (this->vcom_readback_ == this->vcom_) {
      this->vcom_write_selector_ = selector;
    }
  }

  this->get_dev_info_();
  return this->has_valid_dev_info_();
}

void EPaperIT8951::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IT8951...");

  this->setup_pins_();

  this->configured_data_rate_ = this->data_rate_;

  if (this->is_seeed_model_()) {
    this->set_data_rate(IT8951_SPI_PROBE_FREQUENCY);
    this->spi_setup();

    bool found_device = false;
    for (const auto &attempt : SEEED_PROBE_ATTEMPTS) {
      ESP_LOGD(TAG, "Probe attempt: %s", attempt.label);
      this->power_cycle_();
      this->wait_busy_(IT8951_HRDY_TIMEOUT_MS);
      if (this->probe_controller_(attempt.label, attempt.send_sys_run, attempt.selector)) {
        found_device = true;
        break;
      }
    }

    if (!found_device) {
      this->mark_failed(LOG_STR("IT8951 never returned valid device info"));
      return;
    }

    if (this->vcom_write_selector_ == 0) {
      if (!this->try_write_vcom_selector_(IT8951_I80_CMD_VCOM_WRITE_ALT)) {
        this->try_write_vcom_selector_(IT8951_I80_CMD_VCOM_WRITE);
      }
    }

    this->width_ = this->dev_info_.panel_width;
    this->height_ = this->dev_info_.panel_height;
    this->row_width_ = static_cast<uint16_t>((static_cast<uint32_t>(this->width_) + 1) / 2);
    this->buffer_length_ = static_cast<size_t>(this->row_width_) * static_cast<size_t>(this->height_);
    this->us_img_buf_addr_l_ = this->dev_info_.img_buf_addr_l;
    this->us_img_buf_addr_h_ = this->dev_info_.img_buf_addr_h;

    this->write_reg_(IT8951_I80CPCR, 0x0001);
    const uint16_t temp_args[2] = {0x0001, 14};
    this->write_args_(IT8951_I80_CMD_TEMP, temp_args, 2);

    this->spi_teardown();
    this->set_data_rate(this->configured_data_rate_);
    this->spi_setup();
  } else {
    this->spi_setup();
    if (this->reset_pin_ != nullptr) {
      this->hardware_reset_();
    }

    this->write_command_(IT8951_TCON_SYS_RUN);
    this->write_reg_(IT8951_I80CPCR, 0x0001);

    this->get_dev_info_();
    if (this->has_valid_dev_info_()) {
      this->width_ = this->dev_info_.panel_width;
      this->height_ = this->dev_info_.panel_height;
      this->row_width_ = static_cast<uint16_t>((static_cast<uint32_t>(this->width_) + 1) / 2);
      this->buffer_length_ = static_cast<size_t>(this->row_width_) * static_cast<size_t>(this->height_);
      this->us_img_buf_addr_l_ = this->dev_info_.img_buf_addr_l;
      this->us_img_buf_addr_h_ = this->dev_info_.img_buf_addr_h;
    }

    if (!this->try_write_vcom_selector_(IT8951_I80_CMD_VCOM_WRITE)) {
      this->try_write_vcom_selector_(IT8951_I80_CMD_VCOM_WRITE_ALT);
    }
    this->vcom_readback_ = this->get_vcom_();
  }

  if (!this->init_buffer_(this->buffer_length_)) {
    this->mark_failed(LOG_STR("Failed to allocate display buffer"));
    return;
  }

  this->initialized_ = true;
  ESP_LOGCONFIG(TAG, "IT8951 setup complete.");
}

void EPaperIT8951::loop() {
  const auto now = millis();
  if (static_cast<int32_t>(now - this->delay_until_) < 0)
    return;
  if (this->waiting_for_idle_) {
    if (this->busy_pin_ == nullptr || this->busy_pin_->digital_read()) {
      this->waiting_for_idle_ = false;
    } else {
      return;
    }
  }
  this->process_state_();
}

void EPaperIT8951::set_state_(EPaperState state, uint16_t delay) {
  this->state_ = state;
  this->delay_until_ = millis() + delay;
  this->waiting_for_idle_ = (state > EPaperState::SHOULD_WAIT);
  if (state == EPaperState::IDLE) {
    if (this->update_pending_) {
      this->update_pending_ = false;
      this->pending_mode_ = this->queued_update_mode_;
      this->update_started_at_ = millis();
      this->update_timing_active_ = true;
      this->update_buffer_prepared_ = false;
      this->waiting_for_controller_ready_ = false;
      this->state_ = EPaperState::UPDATE;
      return;
    }
    this->disable_loop();
  }
}

bool EPaperIT8951::framebuffer_is_binary_() {
  for (size_t i = 0; i < this->buffer_length_; i++) {
    const uint8_t byte = this->buffer_[i];
    const uint8_t hi = byte >> 4;
    const uint8_t lo = byte & 0x0F;
    if ((hi != 0x00 && hi != 0x0F) || (lo != 0x00 && lo != 0x0F)) {
      return false;
    }
  }
  return true;
}

uint8_t EPaperIT8951::get_pixel_nibble_(uint16_t x, uint16_t y) {
  const uint32_t index = static_cast<uint32_t>(y) * this->row_width_ + (x >> 1);
  const uint8_t byte = this->buffer_[index];
  if ((x & 1U) == 0) {
    return byte >> 4;
  }
  return byte & 0x0F;
}

bool EPaperIT8951::prepare_transfer_(UpdateModeE &mode) {
  this->partial_update_++;
  if (this->full_update_every_ > 0 && this->partial_update_ >= this->full_update_every_) {
    this->partial_update_ = 0;
    mode = UPDATE_MODE_GC16;
  }

  if (this->is_seeed_model_()) {
    if (this->is_display_busy_()) {
      this->waiting_for_controller_ready_ = true;
      return false;
    }
    this->waiting_for_controller_ready_ = false;
    this->pending_x_ = 0;
    this->pending_y_ = 0;
    this->pending_w_ = this->get_width_internal();
    this->pending_h_ = this->get_height_internal();
    this->transfer_row_ = 0;
    const bool force_binary = this->force_1bpp_ || this->is_seeed_model_();
    this->use_1bpp_ = force_binary || this->framebuffer_is_binary_();

    this->x_low_ = this->width_;
    this->x_high_ = 0;
    this->y_low_ = this->height_;
    this->y_high_ = 0;

    ESP_LOGD(TAG, "Transfer: %dx%d @ 0,0 mode=%d (%s path)", this->pending_w_, this->pending_h_, static_cast<int>(mode),
             this->use_1bpp_ ? "1bpp" : "4bpp");
    return true;
  }

  if (this->x_high_ <= this->x_low_ || this->y_high_ <= this->y_low_) {
    return false;
  }

  this->x_low_ &= 0xFFFC;
  uint16_t temp_max = this->x_high_ > 0 ? static_cast<uint16_t>(this->x_high_ - 1) : 0;
  temp_max = static_cast<uint16_t>(temp_max | 0x0003);
  if (temp_max >= this->get_width_internal()) {
    temp_max = this->get_width_internal() - 1;
  }
  this->x_high_ = static_cast<uint16_t>(temp_max + 1);

  const uint16_t x = static_cast<uint16_t>(this->x_low_);
  const uint16_t y = static_cast<uint16_t>(this->y_low_);
  const uint16_t width = static_cast<uint16_t>(this->x_high_ - this->x_low_);
  const uint16_t height = static_cast<uint16_t>(this->y_high_ - this->y_low_);

  if (x >= this->get_width_internal() || y >= this->get_height_internal() || (x + width) > this->get_width_internal() ||
      (y + height) > this->get_height_internal()) {
    ESP_LOGE(TAG, "Transfer area (%u,%u %ux%u) out of bounds", x, y, width, height);
    this->x_low_ = this->width_;
    this->x_high_ = 0;
    this->y_low_ = this->height_;
    this->y_high_ = 0;
    return false;
  }

  this->pending_x_ = x;
  this->pending_y_ = y;
  this->pending_w_ = width;
  this->pending_h_ = height;
  this->transfer_row_ = 0;
  this->use_1bpp_ = false;

  this->x_low_ = this->width_;
  this->x_high_ = 0;
  this->y_low_ = this->height_;
  this->y_high_ = 0;

  ESP_LOGD(TAG, "Transfer: %dx%d @ %d,%d mode=%d", width, height, x, y, static_cast<int>(mode));
  return true;
}

bool EPaperIT8951::transfer_row_data_1bpp_() {
  const uint16_t area_w = this->pending_w_;
  const uint16_t area_h = this->pending_h_;
  const uint16_t words_per_row = static_cast<uint16_t>((area_w + 15) / 16);
  const uint16_t row_size_bytes = static_cast<uint16_t>(words_per_row * 2);

  if (row_size_bytes > IT8951_MAX_1BPP_ROW_BYTES) {
    ESP_LOGE(TAG, "1bpp row buffer too small for %u-byte transfer", row_size_bytes);
    return true;
  }

  const uint32_t start_time = millis();
  std::array<uint16_t, IT8951_MAX_1BPP_ROW_BYTES / 2> row_words{};
  std::array<uint8_t, IT8951_MAX_1BPP_ROW_BYTES> row_buffer{};

  if (this->transfer_row_ == 0) {
    this->m_endian_type_ = IT8951_LDIMG_L_ENDIAN;
    this->m_pix_bpp_ = IT8951_8BPP;
    this->set_target_memory_addr_(this->us_img_buf_addr_l_, this->us_img_buf_addr_h_);
    this->set_area_(0, 0, static_cast<uint16_t>(area_w / 8), area_h, true);
  }

  this->wait_busy_();
  this->enable();
  this->write_byte16(IT8951_PACKET_TYPE_WRITE);
  this->wait_busy_();

  while (this->transfer_row_ < area_h) {
    row_words.fill(0);

    const uint16_t row_y = static_cast<uint16_t>(this->pending_y_ + this->transfer_row_);
    for (uint16_t x = 0; x < area_w; x++) {
      const uint8_t nibble = this->get_pixel_nibble_(static_cast<uint16_t>(this->pending_x_ + x), row_y);
      if (nibble <= 0x07) {
        row_words[x / 16] |= static_cast<uint16_t>(0x8000U >> (x & 0x0F));
      }
    }

    for (uint16_t i = 0; i < words_per_row; i++) {
      const uint16_t word = row_words[words_per_row - 1 - i];
      const uint16_t dst = static_cast<uint16_t>(i * 2);
      row_buffer[dst] = static_cast<uint8_t>(word >> 8);
      row_buffer[dst + 1] = static_cast<uint8_t>(word & 0xFF);
    }

    this->write_array(row_buffer.data(), row_size_bytes);
    this->transfer_row_++;
    if ((this->transfer_row_ & 0x07U) == 0) {
      App.feed_wdt();
    }

    if (millis() - start_time >= MAX_TRANSFER_TIME) {
      break;
    }
  }

  this->disable();

  if (this->transfer_row_ >= area_h) {
    this->write_command_(IT8951_TCON_LD_IMG_END);
  }

  return this->transfer_row_ >= area_h;
}

bool EPaperIT8951::transfer_row_data_() {
  if (this->use_1bpp_) {
    return this->transfer_row_data_1bpp_();
  }

  const uint16_t area_x = this->pending_x_;
  const uint16_t area_y = this->pending_y_;
  const uint16_t area_w = this->pending_w_;
  const uint16_t area_h = this->pending_h_;
  const bool seeed = this->is_seeed_model_();
  const uint32_t start_time = millis();

  this->m_endian_type_ = seeed ? IT8951_LDIMG_L_ENDIAN : IT8951_LDIMG_B_ENDIAN;
  this->m_pix_bpp_ = IT8951_4BPP;

  if (this->transfer_row_ == 0) {
    this->set_target_memory_addr_(this->us_img_buf_addr_l_, this->us_img_buf_addr_h_);
    if (seeed) {
      this->write_command_(IT8951_TCON_REG_RD);
      this->write_word_(IT8951_UP1SR + 2);
      const uint16_t up1sr_high = this->read_word_();
      this->write_reg_(IT8951_UP1SR + 2, static_cast<uint16_t>(up1sr_high & ~(1U << 2)));
    }
    this->set_area_(area_x, area_y, area_w, area_h, seeed);
  }

  this->wait_busy_();
  this->enable();
  this->write_byte16(IT8951_PACKET_TYPE_WRITE);
  this->wait_busy_();

  std::array<uint8_t, IT8951_MAX_4BPP_ROW_BYTES> row_buffer{};
  const bool full_width = (area_x == 0 && area_w == this->get_width_internal());
  const uint16_t bytes_per_row = full_width ? this->row_width_ : static_cast<uint16_t>(area_w >> 1);

  if (bytes_per_row > IT8951_MAX_4BPP_ROW_BYTES) {
    ESP_LOGE(TAG, "4bpp row buffer too small for %u-byte transfer", bytes_per_row);
    this->disable();
    return true;
  }

  while (this->transfer_row_ < area_h) {
    const uint32_t row_y = area_y + this->transfer_row_;
    const uint32_t offset = row_y * this->row_width_ + (full_width ? 0 : (area_x >> 1));

    if (seeed) {
      const uint16_t words_per_row = static_cast<uint16_t>(bytes_per_row / 2);
      for (uint16_t i = 0; i < words_per_row; i++) {
        const uint32_t src = offset + static_cast<uint32_t>(words_per_row - 1 - i) * 2U;
        const uint16_t dst = static_cast<uint16_t>(i * 2);
        row_buffer[dst] = this->buffer_[src + 1];
        row_buffer[dst + 1] = this->buffer_[src];
      }
    } else {
      for (uint16_t i = 0; i < bytes_per_row; i++) {
        row_buffer[i] = this->buffer_[offset + i];
      }
    }

    this->write_array(row_buffer.data(), bytes_per_row);
    this->transfer_row_++;
    if ((this->transfer_row_ & 0x07U) == 0) {
      App.feed_wdt();
    }

    if (millis() - start_time >= MAX_TRANSFER_TIME) {
      break;
    }
  }

  this->disable();

  if (this->transfer_row_ >= area_h) {
    this->write_command_(IT8951_TCON_LD_IMG_END);
  }

  return this->transfer_row_ >= area_h;
}

bool EPaperIT8951::transfer_data() { return this->transfer_row_data_(); }

void EPaperIT8951::refresh_screen(bool partial) {
  (void) partial;
  if (this->queued_update_mode_ == UPDATE_MODE_NONE) {
    return;
  }

  if (!this->is_seeed_model_() && this->is_display_busy_()) {
    this->waiting_for_idle_ = true;
    return;
  }

  if (this->use_1bpp_) {
    this->update_area_1bpp_(this->pending_x_, this->pending_y_, this->pending_w_, this->pending_h_, this->queued_update_mode_,
                            0xFF, 0x00);
  } else {
    this->update_area_(this->pending_x_, this->pending_y_, this->pending_w_, this->pending_h_, this->queued_update_mode_);
  }

  if (this->update_timing_active_) {
    ESP_LOGD(TAG, "Update took %ums (mode=%d area=%ux%u@%u,%u)", millis() - this->update_started_at_,
             static_cast<int>(this->queued_update_mode_), this->pending_w_, this->pending_h_, this->pending_x_,
             this->pending_y_);
    this->update_timing_active_ = false;
  }
}

void EPaperIT8951::power_on() { this->write_command_(IT8951_TCON_SYS_RUN); }

void EPaperIT8951::power_off() {}

void EPaperIT8951::deep_sleep() { this->write_command_(IT8951_TCON_SLEEP); }

void EPaperIT8951::process_state_() {
  switch (this->state_) {
    case EPaperState::IDLE:
      this->disable_loop();
      break;

    case EPaperState::UPDATE:
      if (!this->update_buffer_prepared_) {
        this->do_update_();
        this->update_buffer_prepared_ = true;
      }
      if (!this->prepare_transfer_(this->pending_mode_)) {
        if (this->waiting_for_controller_ready_) {
          return;
        }
        this->update_buffer_prepared_ = false;
        this->update_timing_active_ = false;
        this->set_state_(EPaperState::IDLE);
        break;
      }
      this->update_buffer_prepared_ = false;
      this->set_state_(this->sleep_when_done_ ? EPaperState::POWER_ON : EPaperState::TRANSFER_DATA);
      break;

    case EPaperState::POWER_ON:
      this->power_on();
      this->set_state_(EPaperState::TRANSFER_DATA);
      break;

    case EPaperState::TRANSFER_DATA:
      if (this->transfer_row_data_()) {
        this->set_state_(EPaperState::REFRESH_SCREEN);
      }
      break;

    case EPaperState::REFRESH_SCREEN:
      if (this->queued_update_mode_ == UPDATE_MODE_NONE) {
        this->set_state_(EPaperState::IDLE);
        break;
      }
      if (!this->is_seeed_model_() && this->is_display_busy_()) {
        return;
      }
      if (this->use_1bpp_) {
        this->update_area_1bpp_(this->pending_x_, this->pending_y_, this->pending_w_, this->pending_h_,
                                this->queued_update_mode_, 0xFF, 0x00);
      } else {
        this->update_area_(this->pending_x_, this->pending_y_, this->pending_w_, this->pending_h_,
                           this->queued_update_mode_);
      }
      if (this->update_timing_active_) {
        ESP_LOGD(TAG, "Update took %ums (mode=%d area=%ux%u@%u,%u)", millis() - this->update_started_at_,
                 static_cast<int>(this->queued_update_mode_), this->pending_w_, this->pending_h_, this->pending_x_,
                 this->pending_y_);
        this->update_timing_active_ = false;
      }
      this->set_state_(this->sleep_when_done_ ? EPaperState::POWER_OFF : EPaperState::IDLE);
      break;

    case EPaperState::POWER_OFF:
      this->set_state_(EPaperState::DEEP_SLEEP);
      break;

    case EPaperState::DEEP_SLEEP:
      this->deep_sleep();
      this->set_state_(EPaperState::IDLE);
      break;

    default:
      ESP_LOGE(TAG, "Unhandled state %d", static_cast<int>(this->state_));
      this->set_state_(EPaperState::IDLE);
      break;
  }
}

void EPaperIT8951::start_update_(UpdateModeE hw_mode) {
  if (this->state_ == EPaperState::IDLE) {
    this->update_started_at_ = millis();
    this->update_timing_active_ = true;
    this->pending_mode_ = hw_mode;
    this->queued_update_mode_ = hw_mode;
    this->update_buffer_prepared_ = false;
    this->waiting_for_controller_ready_ = false;
    this->enable_loop();
    this->set_state_(EPaperState::UPDATE);
  } else {
    this->update_pending_ = true;
    this->queued_update_mode_ = hw_mode;
  }
}

void EPaperIT8951::update_mode(const std::string &mode) {
  if (!this->is_ready() || !this->initialized_) {
    return;
  }

  const UpdateModeE hw_mode = parse_update_mode(mode);
  if (hw_mode == UPDATE_MODE_NONE) {
    ESP_LOGW(TAG, "Unknown update mode '%s'", mode.c_str());
    return;
  }

  this->start_update_(hw_mode);
}

void EPaperIT8951::update() {
  if (!this->is_ready() || !this->initialized_) {
    return;
  }

  if (!this->get_update_mode().empty()) {
    this->update_mode(this->get_update_mode());
    return;
  }

  this->start_update_(UPDATE_MODE_GC16);
}

uint8_t EPaperIT8951::color_to_nibble_(const Color &color) const {
  if (color.raw_32 == 0) {
    return 0x00;
  }
  if (color.raw_32 == 0xFFFFFFFF) {
    return 0x0F;
  }

  if (color.g == 0 && color.b == 0 && color.w == 0 && color.r <= 0x0F) {
    return color.r;
  }

  uint16_t gray = static_cast<uint16_t>(color.r) + color.g + color.b;
  gray /= 3;
  if (color.w > gray) {
    gray = color.w;
  }
  uint8_t nibble = static_cast<uint8_t>((gray + 8) >> 4);
  if (nibble > 0x0F) {
    nibble = 0x0F;
  }
  return nibble;
}

void EPaperIT8951::fill(Color color) {
  if (this->get_clipping().is_set()) {
    Display::fill(color);
    return;
  }

  uint8_t packed_color;
  const bool force_binary = this->force_1bpp_ || this->is_seeed_model_();
  if (force_binary) {
    packed_color = color.is_on() ? 0x00 : 0x0F;
    if (this->reversed_) {
      packed_color = 0x0F - packed_color;
    }
  } else {
    packed_color = this->color_to_nibble_(color);
    if (!this->reversed_) {
      packed_color = 0x0F - packed_color;
    }
  }
  const uint8_t fill_byte = static_cast<uint8_t>((packed_color << 4) | packed_color);

  this->buffer_.fill(fill_byte);
  this->x_high_ = this->get_width_internal();
  this->y_high_ = this->get_height_internal();
  this->x_low_ = 0;
  this->y_low_ = 0;
}

void HOT EPaperIT8951::draw_pixel_at(int x, int y, Color color) {
  if (!this->rotate_coordinates_(x, y)) {
    return;
  }

  uint8_t internal_color;
  const bool force_binary = this->force_1bpp_ || this->is_seeed_model_();
  if (force_binary) {
    internal_color = color.is_on() ? 0x00 : 0x0F;
    if (this->reversed_) {
      internal_color = 0x0F - internal_color;
    }
  } else {
    internal_color = this->color_to_nibble_(color) & 0x0F;
    if (!this->reversed_) {
      internal_color = 0x0F - internal_color;
    }
  }

  const uint32_t index = static_cast<uint32_t>(y) * this->row_width_ + (static_cast<uint32_t>(x) >> 1);

  uint8_t buf = this->buffer_[index];
  if ((x & 0x1) != 0) {
    buf = static_cast<uint8_t>((buf & 0xF0) | internal_color);
  } else {
    buf = static_cast<uint8_t>((buf & 0x0F) | (internal_color << 4));
  }
  this->buffer_[index] = buf;
}

void EPaperIT8951::dump_config() {
  LOG_DISPLAY("", "IT8951 E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: %s", this->name_);
  ESP_LOGCONFIG(TAG, "  Dimensions: %dx%d", this->get_width_internal(), this->get_height_internal());
  ESP_LOGCONFIG(TAG, "  Buffer: %u bytes in %u segment(s)", static_cast<unsigned>(this->buffer_length_),
                static_cast<unsigned>(this->buffer_.get_buffer_count()));
  ESP_LOGCONFIG(TAG, "  Image buffer addr: 0x%04X%04X", this->us_img_buf_addr_h_, this->us_img_buf_addr_l_);
  ESP_LOGCONFIG(TAG, "  Sleep when done: %s", YESNO(this->sleep_when_done_));
  ESP_LOGCONFIG(TAG, "  Full update every: %u", this->full_update_every_);
  ESP_LOGCONFIG(TAG, "  Reversed colors: %s", YESNO(this->reversed_));
  ESP_LOGCONFIG(TAG, "  Force 1bpp: %s", YESNO(this->force_1bpp_ || this->is_seeed_model_()));
  ESP_LOGCONFIG(TAG, "  Reset duration: %ums", this->reset_duration_);
  ESP_LOGCONFIG(TAG, "  VCOM: %u", this->vcom_);
  ESP_LOGCONFIG(TAG, "  VCOM selector: 0x%04X", this->vcom_write_selector_);
  ESP_LOGCONFIG(TAG, "  Probe path: %s", this->probe_path_ != nullptr ? this->probe_path_ : "n/a");
  ESP_LOGCONFIG(TAG, "  DevInfo Panel: %ux%u", this->dev_info_.panel_width, this->dev_info_.panel_height);
  ESP_LOGCONFIG(TAG, "  DevInfo ImgBuf: 0x%04X%04X", this->dev_info_.img_buf_addr_h, this->dev_info_.img_buf_addr_l);
  ESP_LOGCONFIG(TAG, "  VCOM read-back: %u (0x%04X)", this->vcom_readback_, this->vcom_readback_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  for (auto *enable_pin : this->enable_pins_) {
    LOG_PIN("  Enable Pin: ", enable_pin);
  }
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace esphome::epaper_spi

