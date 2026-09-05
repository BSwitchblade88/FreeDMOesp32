// =============================================================================
// dmo_label_emulator.cpp
//
// ESP32-S2 mini build: one I2C slave bus, one always-active emulated tag,
// switchable at runtime by index. No real-reader master bus, no printer
// power-state monitoring.
// =============================================================================

#include "dmo_label_emulator.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dmo_label_emulator {

static const char *const TAG = "dmo_label_emulator";

static constexpr i2c_port_t SLAVE_PORT = I2C_NUM_0;  // the only I2C peripheral we use
static constexpr int SLAVE_RX_BUF_LEN = 512;
static constexpr int SLAVE_TX_BUF_LEN = 512;

// -----------------------------------------------------------------------------
void DmoLabelEmulator::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DMO label emulator...");

  state_mutex_ = xSemaphoreCreateMutex();

  // Start on SKU index 0. If you want this to remember your last choice
  // across reboots too, that's handled on the YAML side by the template
  // select's `restore_value: true` - it will call set_sku() again shortly
  // after boot once it restores its own state, overriding this default.
  set_sku(0);

  // Configure the mainboard-facing I2C bus as a SLAVE. We are pretending to
  // be a genuine CLRC688 reader chip from the mainboard's point of view.
  i2c_config_t slave_conf = {};
  slave_conf.mode = I2C_MODE_SLAVE;
  slave_conf.sda_io_num = (gpio_num_t) slave_sda_;
  slave_conf.scl_io_num = (gpio_num_t) slave_scl_;
  slave_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  slave_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  slave_conf.slave.addr_10bit_en = 0;
  slave_conf.slave.slave_addr = slave_address_;
  if (i2c_param_config(SLAVE_PORT, &slave_conf) != ESP_OK ||
      i2c_driver_install(SLAVE_PORT, I2C_MODE_SLAVE, SLAVE_RX_BUF_LEN, SLAVE_TX_BUF_LEN, 0) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install I2C slave driver");
    this->mark_failed();
    return;
  }

  pinMode(irq_pin_, OUTPUT);
  digitalWrite(irq_pin_, LOW);

  // Launch the task that services the mainboard's I2C reads/writes.
  // The ESP32-S2 is single-core, so unlike a dual-core chip we can't
  // isolate this task on its own CPU - it shares core 0 with Wi-Fi and the
  // rest of ESPHome. The elevated priority (configMAX_PRIORITIES - 2) is
  // what keeps it responsive: FreeRTOS will preempt lower-priority tasks
  // to run this one, it just doesn't get a dedicated core to itself.
  xTaskCreatePinnedToCore(&DmoLabelEmulator::slave_task_trampoline, "dmo_slave", 4096, this,
                          configMAX_PRIORITIES - 2, &slave_task_handle_, 0);

  ESP_LOGCONFIG(TAG, "DMO label emulator ready");
}

void DmoLabelEmulator::dump_config() {
  ESP_LOGCONFIG(TAG, "DMO Label Emulator:");
  ESP_LOGCONFIG(TAG, "  Slave I2C addr: 0x%02X", slave_address_);
}

// -----------------------------------------------------------------------------
// SKU switching - called directly from the YAML template select's lambda.
// -----------------------------------------------------------------------------
void DmoLabelEmulator::set_sku(uint8_t index) {
  if (index >= SKU_TABLE_SIZE) {
    ESP_LOGW(TAG, "Ignoring out-of-range SKU index %u", index);
    return;
  }

  // Guard against the I2C slave task reading blocks_ mid-copy. state_mutex_
  // is null on the very first call (from setup(), before it's created) -
  // guard against that too.
  if (state_mutex_ != nullptr) xSemaphoreTake(state_mutex_, portMAX_DELAY);
  memcpy(blocks_, SKU_TABLE[index], SLIX2_BLOCKS * sizeof(uint32_t));
  reset_counter();
  if (state_mutex_ != nullptr) xSemaphoreGive(state_mutex_);

  ESP_LOGI(TAG, "Switched emulated label to SKU index %u", index);
}

// Ported from EMU_SLIX2_CounterReset(). Blocks 0x0F/0x10 encode "total
// labels" and a safety margin in their high 16 bits.
void DmoLabelEmulator::reset_counter() {
  uint16_t amount_of_labels = blocks_[0x0F] >> 16;
  uint16_t counter_margin = blocks_[0x10] >> 16;
  counter_ = 0xFFFF - amount_of_labels - counter_margin;
  tag_present_ = false;
}

// -----------------------------------------------------------------------------
// SLIX2 tag emulation - ported near-verbatim from EMU_SLIX2_Communication()
// -----------------------------------------------------------------------------
void DmoLabelEmulator::handle_slix2_command(const uint8_t *in, uint8_t in_len, uint8_t *out, uint8_t *out_len) {
  switch (in[1]) {  // emulated command byte
    case 0x01: {    // inventory
      reset_counter();
      out[0] = 0;
      memcpy(out + 1, TAG_INVENTORY, SLIX2_INVENTORY_LEN);
      *out_len = 1 + SLIX2_INVENTORY_LEN;
      tag_present_ = true;
      break;
    }
    case 0x21: {  // write single block (only the label counter is writable)
      if (in[10] == 0x4F) {
        if (in[11] == 1 && in[12] == 0 && in[13] == 0 && in[14] == 0) {
          counter_++;  // printer only ever increments by exactly 1
        } else {
          out[0] = 0x01; out[1] = 0x0F; *out_len = 2;  // error
          break;
        }
      }
      out[0] = 0; *out_len = 1;
      break;
    }
    case 0x23: {  // read multiple blocks
      uint8_t blk = in[10];
      if (blk >= SLIX2_BLOCKS) {
        out[0] = 0x01; out[1] = 0x0F; *out_len = 2;
        break;
      }
      uint8_t cnt = in[11] + 1;
      if (blk + cnt > SLIX2_BLOCKS) cnt = SLIX2_BLOCKS - blk;
      out[0] = 0;
      memcpy(out + 1, &blocks_[blk], cnt * 4);
      *out_len = 1 + cnt * 4;

      // Splice in the live counter value if block 79 is part of this read.
      if (blk == 79 || blk + cnt >= 79) {
        out[1 + (79 - blk) * 4 + 0] = (uint8_t) counter_;
        out[1 + (79 - blk) * 4 + 1] = (uint8_t) (counter_ >> 8);
        out[1 + (79 - blk) * 4 + 2] = 0;
        out[1 + (79 - blk) * 4 + 3] = 1;
      }
      break;
    }
    case 0x26: {  // "reset to ready" - mainboard checks the tag is still present
      if (tag_present_) { out[0] = 0; *out_len = 1; }
      else { out[0] = 0x01; out[1] = 0x0F; *out_len = 2; }
      break;
    }
    case 0x2B: out[0] = 0; memcpy(out + 1, TAG_SYSINFO, SLIX2_SYSINFO_LEN); *out_len = 1 + SLIX2_SYSINFO_LEN; break;
    case 0xAB: out[0] = 0; memcpy(out + 1, TAG_NXPSYSINFO, SLIX2_NXPSYSINFO_LEN); *out_len = 1 + SLIX2_NXPSYSINFO_LEN; break;
    case 0xB2: out[0] = 0; out[1] = (uint8_t) millis(); out[2] = (uint8_t) (millis() >> 8); *out_len = 3; break;
    case 0xB3: out[0] = 0; *out_len = 1; break;  // "set password" - just claim success
    case 0xBD: out[0] = 0; memcpy(out + 1, TAG_SIGNATURE, SLIX2_SIGNATURE_LEN); *out_len = 1 + SLIX2_SIGNATURE_LEN; break;
    default:   out[0] = 0; *out_len = 1; break;  // always claim success for anything unhandled
  }
}

// -----------------------------------------------------------------------------
// CLRC688 register-level emulation - ported from EMU_CLRC688_Communication()
// -----------------------------------------------------------------------------
void DmoLabelEmulator::handle_clrc688_command(const uint8_t *in, uint8_t in_len, uint8_t *out, uint8_t *out_len) {
  static uint8_t fifo_buffer[256];
  static uint8_t fifo_length = 0;

  *out_len = 0;
  if (in_len == 0) return;

  switch (in[0]) {
    case 0x00: {  // command register
      if (in[1] == 0x07) {  // "transceive to/from tag"
        handle_slix2_command(fifo_buffer, fifo_length, fifo_buffer, &fifo_length);
        digitalWrite(irq_pin_, HIGH);  // signal IRQ
      }
      if (in[1] & 0x80) reset_counter();  // entering standby resets the counter
      break;
    }
    case 0x04: {  // FIFO length register
      if (in_len == 1) { out[0] = fifo_length; *out_len = 1; }
      break;
    }
    case 0x05: {  // FIFO data register
      if (in_len > 1) {
        memcpy(fifo_buffer, in + 1, in_len - 1);  // mainboard writing into our FIFO
      } else {
        if (fifo_length) memcpy(out, fifo_buffer, fifo_length);
        *out_len = fifo_length;
        fifo_length = 0;
      }
      break;
    }
    case 0x06:  // IRQ0
    case 0x07: {  // IRQ1
      if (in_len == 1) { out[0] = 0x7F; *out_len = 1; }  // "all interrupts pending"
      else digitalWrite(irq_pin_, LOW);                   // mainboard clearing IRQs
      break;
    }
    case 0x0A: { out[0] = 0x00; *out_len = 1; break; }  // error register: always "no error"
    default: break;  // ignore writes to any other register
  }
}

// -----------------------------------------------------------------------------
// I2C slave servicing task
// -----------------------------------------------------------------------------
void DmoLabelEmulator::slave_task_trampoline(void *param) {
  static_cast<DmoLabelEmulator *>(param)->slave_task();
}

void DmoLabelEmulator::slave_task() {
  uint8_t rx_buf[300];

  for (;;) {
    // Blocks until the mainboard writes something.
    int rx_len = i2c_slave_read_buffer(SLAVE_PORT, rx_buf, sizeof(rx_buf), portMAX_DELAY);
    if (rx_len <= 0) continue;

    uint8_t tx_buf[300];
    uint8_t tx_len = 0;

    xSemaphoreTake(state_mutex_, portMAX_DELAY);
    handle_clrc688_command(rx_buf, (uint8_t) rx_len, tx_buf, &tx_len);
    xSemaphoreGive(state_mutex_);

    if (tx_len > 0) {
      // NOTE: staging the reply here, right after processing the write, is
      // the part worth bench-validating against your real printer mainboard
      // with a logic analyzer - see the project notes for why.
      i2c_slave_write_buffer(SLAVE_PORT, tx_buf, tx_len, pdMS_TO_TICKS(50));
    }
  }
}

}  // namespace dmo_label_emulator
}  // namespace esphome
