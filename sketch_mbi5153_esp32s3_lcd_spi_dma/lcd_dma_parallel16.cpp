/*********************************************************************************************
  Simple example of using the ESP32-S3's LCD peripheral for general-purpose
  (non-LCD) parallel data output with DMA. Connect 8 LEDs (or logic analyzer),
  cycles through a pattern among them at about 1 Hz.
  This code is ONLY for the ESP32-S3, NOT the S2, C3 or original ESP32.
  None of this is authoritative canon, just a lot of trial error w/datasheet
  and register poking. Probably more robust ways of doing this still TBD.


 FULL CREDIT goes to AdaFruit and  https://github.com/PaintYourDragon
 
 https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/
 
 https://github.com/adafruit/Adafruit_Protomatter/blob/master/src/arch/esp32-s3.h

 PLEASE SUPPORT THEM!

 ********************************************************************************************/
#if __has_include(<hal/lcd_ll.h>)
// Stop compile errors: /src/platforms/esp32s3/gdma_lcd_parallel16.hpp:64:10: fatal error: hal/lcd_ll.h: No such file or directory

static const char *const TAG = "lcd_dma_parallel16";

#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#endif

#include "lcd_dma_parallel16.hpp"
#include "esp_attr.h"
#include <soc/lldesc.h>  // for lldesc_get_required_num and LLDESC_MAX_NUM_PER_DESC


// End-of-DMA-transfer callback
IRAM_ATTR bool gdma_on_trans_eof_callback(gdma_channel_handle_t dma_chan,
                                          gdma_event_data_t *event_data, void *user_data) {

  // This DMA callback seems to trigger a moment before the last data has
  // issued (buffering between DMA & LCD peripheral?), so pause a moment
  // before stopping LCD data out. The ideal delay may depend on the LCD
  // clock rate...this one was determined empirically by monitoring on a
  // logic analyzer. YMMV.
  esp_rom_delay_us(100);
  // The LCD peripheral stops transmitting at the end of the DMA xfer, but
  // clear the lcd_start flag anyway -- we poll it in loop() to decide when
  // the transfer has finished, and the same flag is set later to trigger
  // the next transfer.

  //LCD_CAM.lcd_user.lcd_start = 0;

  return true;
}

lcd_cam_dev_t *getDev() {
  return &LCD_CAM;
}


// ------------------------------------------------------------------------------
void Bus_Parallel16::config(const config_t &cfg) {
  _cfg = cfg;
}


//https://github.com/adafruit/Adafruit_Protomatter/blob/master/src/arch/esp32-s3.h
esp_err_t Bus_Parallel16::setup_lcd_dma_periph(void) {
  esp_err_t ret;

  _dev = getDev();

  // LCD_CAM peripheral isn't enabled by default -- MUST begin with this:
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Reset LCD bus
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(100);  // Wait some time for whatever reason


  // Configure LCD clock. Since this program generates human-perceptible
  // output and not data for LED matrices or NeoPixels, use almost the
  // slowest LCD clock rate possible. The S3-mini module used on Feather
  // ESP32-S3 has a 40 MHz crystal. A 2-stage clock division of 1:16000
  // is applied (250*64), yielding 2,500 Hz. Still much too fast for
  // human eyes, so later we set up the data to repeat each output byte
  // many times over.
  //LCD_CAM.lcd_clock.clk_en = 0;             // Enable peripheral clock

  // LCD_CAM_LCD_CLK_SEL Select LCD module source clock. 0: clock source is disabled. 1: XTAL_CLK. 2: PLL_D2_CLK. 3: PLL_F160M_CLK. (R/W)
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;  // Use 160Mhz Clock Source

  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;   // PCLK low in 1st half cycle
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;  // PCLK low idle

  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;  // Should never be zero

  //LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0; // PCLK = CLK / (CLKCNT_N+1)
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1;  // PCLK = CLK / 1 (... so 160Mhz still)

  // divider of 18 = 8.888mhz
  // divider of 16 = 10mhz
  //auto  _div_num = 20; // DO NOT CHANGE! // 8Mhz
  /*
      auto  _div_num = 16; // DO NOT CHANGE! // 10Mhz

      LCD_CAM.lcd_clock.lcd_clkm_div_num = _div_num;      
      LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;     // fractal clock divider numerator
      LCD_CAM.lcd_clock.lcd_clkm_div_a = 1;     // denominator
*/

  /* 
      The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data. 
      */

  // Set the DCLK frequency. Refer to spi_dma_tx_loop.c for the GCLK.
  // Don't change these unless you know what you are doing, and you probably don't!
  //LCD_CAM.lcd_clock.lcd_clkm_div_num = 22;  // 7mhz  // Anything > 8Mhz seems to introduce noise when using jumper
  //LCD_CAM.lcd_clock.lcd_clkm_div_num = 80;  // 2mhz
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 18;  // 7mhz  // Anything > 8Mhz seems to introduce noise when using jumper

  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;  // fractal clock divider numerator
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 1;  // denominator

  ESP_LOGI("S3", "Clock divider is %d", (int)LCD_CAM.lcd_clock.lcd_clkm_div_num);
  ESP_LOGD(TAG, "Resulting output clock frequency: %d Hz", (int)(160000000L / LCD_CAM.lcd_clock.lcd_clkm_div_num));

  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;     // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;  // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;   // Do NOT auto-frame
  LCD_CAM.lcd_misc.lcd_bk_en = 1;           // https://esp32.com/viewtopic.php?t=24459&start=60#p91835


  //  LCD_CAM_DOUTn_MODE (n = 0 ­ 15) The output data bit n is delayed by module clock LCD_CLK.
  //  0: output without delay. 1: delayed by the rising edge of LCD_CLK. 2: delayed by the falling edge
  //  of LCD_CLK. (R/W)
  LCD_CAM.lcd_data_dout_mode.val = 0;  // No data delays

  LCD_CAM.lcd_user.lcd_always_out_en = 1;  // Enable 'always out' mode
  LCD_CAM.lcd_user.lcd_8bits_order = 0;    // Do not swap bytes
  LCD_CAM.lcd_user.lcd_bit_order = 0;      // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_2byte_en = 1;       // 16-bit data mode

  // "Dummy phases" are initial LCD peripheral clock cycles before data
  // begins transmitting when requested. After much testing, determined
  // that at least one dummy phase MUST be enabled for DMA to trigger
  // reliably.
  LCD_CAM.lcd_user.lcd_dummy = 1;           // Dummy phase(s) @ LCD start
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 1;  // 1+1 dummy phase
  LCD_CAM.lcd_user.lcd_cmd = 0;             // No command at LCD start


  // Route 8 LCD data signals to GPIO pins
  int8_t *pins = _cfg.pin_data;

  for (int i = 0; i < 16; i++) {
    if (pins[i] >= 0) {  // -1 value will CRASH the ESP32!
      esp_rom_gpio_connect_out_signal(pins[i], LCD_DATA_OUT0_IDX + i, false, false);
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pins[i]], PIN_FUNC_GPIO);
      gpio_set_drive_capability((gpio_num_t)pins[i], (gpio_drive_cap_t)3);
    }
  }

  // Clock
  esp_rom_gpio_connect_out_signal(_cfg.pin_wr, LCD_PCLK_IDX, _cfg.invert_pclk, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[_cfg.pin_wr], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)_cfg.pin_wr, (gpio_drive_cap_t)3);

  // Remaining descriptor elements are initialized before each DMA transfer.
  // Allocate DMA channel and connect it to the LCD peripheral
  static gdma_channel_alloc_config_t dma_chan_config = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = {
      .reserve_sibling = 0 }
  };
  gdma_new_channel(&dma_chan_config, &dma_chan);
  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  static gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false
  };
  gdma_apply_strategy(dma_chan, &strategy_config);

  gdma_transfer_ability_t ability = {
    .sram_trans_align = 32,
    .psram_trans_align = 64,
  };
  gdma_set_transfer_ability(dma_chan, &ability);


// Disable DMA transfer callback as it's pointless, there's about 3 caches between the DMA perph
// and the LCD module, so when the DMA engine triggers this callback, it doesn't actually mean
// the LCD periph has sent out all the data yet.
/*
  // Enable DMA transfer callback
  static gdma_tx_event_callbacks_t tx_cbs = {
    // .on_trans_eof is literally the only gdma tx event type available
    .on_trans_eof = gdma_on_trans_eof_callback
  };
  gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);
*/

  // This uses a busy loop to wait for each DMA transfer to complete...
  // but the whole point of DMA is that one's code can do other work in
  // the interim. The CPU is totally free while the transfer runs!
  while (LCD_CAM.lcd_user.lcd_start)
    ;  // Wait for DMA completion callback

  // After much experimentation, each of these steps is required to get
  // a clean start on the next LCD transfer:
  ret = gdma_reset(dma_chan);  // Reset DMA to known state.
  esp_rom_delay_us(10);

  LCD_CAM.lcd_user.lcd_dout = 1;         // Enable data out
  LCD_CAM.lcd_user.lcd_update = 1;       // Update registers
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;  // Reset LCD TX FIFO

  //return true; // no return val = illegal instruction
  assert(ret == ESP_OK);

  return ESP_OK;
}

esp_err_t Bus_Parallel16::release(void) {
  if (_i80_bus) {
    esp_lcd_del_i80_bus(_i80_bus);
  }
  if (_dmadesc_a) {
    heap_caps_free(_dmadesc_a);
    _dmadesc_a = nullptr;
    _dmadesc_count = 0;
  }

  return ESP_OK;
}


// Need this to work for double buffers etc.
bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {
  // Only allocate more ram if we don't have enough for the requested number of dma descriptors
  if (len > _dmadesc_count_sram_alloc) {
    if (_dmadesc_a) { heap_caps_free(_dmadesc_a); }  // free all dma descrptios previously

    ESP_LOGD(TAG, "Had to allocate more memory for DMA descriptors. Had space for %d, but need %d.", (int)_dmadesc_count_sram_alloc, (int)len);
    _dmadesc_count_sram_alloc = len;

    ESP_LOGD(TAG, "Allocating %d bytes memory for DMA descriptors.", (int)sizeof(HUB75_DMA_DESCRIPTOR_T) * len);
    _dmadesc_a = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

    if (_dmadesc_a == nullptr) {
      ESP_LOGE(TAG, "ERROR: Couldn't malloc _dmadesc_a. Not enough memory.");
      return false;
    }
  }

  // Length
  _dmadesc_count = len;

  return true;
}

esp_err_t Bus_Parallel16::dma_transfer_start() {
  esp_err_t ret = gdma_start(dma_chan, (intptr_t)&_dmadesc_a[0]);  // Start DMA w/updated descriptor(s)
  esp_rom_delay_us(10);                                            // Must 'bake' a moment before...
  LCD_CAM.lcd_user.lcd_start = 1;                                  // Trigger LCD DMA transfer

  while (LCD_CAM.lcd_user.lcd_start)
    ;

  return ret;

}  // end



esp_err_t Bus_Parallel16::send_stuff_once(void *data, size_t size_in_bytes, bool is_greyscale_data) {

  ESP_LOGI(TAG, "Sending DMA payload of length %d bytes.", size_in_bytes);

  // Small optimisation
  // If a previous call to this function was to send greyscale data, then we assume all the
  // DMA desciptors have already been setup and are pointing to the right memory address of
  // the greyscale image buffer data. No need to do all these chekcs again and rebuild the
  // DMA chain again.
  /*
    if ((_prev_payload_was_greyscale == is_greyscale_data) && (_dmadesc_a != nullptr))
    {    
        // Just send it and leave.
        return dma_transfer_start();
    }
    */

  int len = size_in_bytes;

  int dma_lldesc_required = lldesc_get_required_num(size_in_bytes);
  ESP_LOGI(TAG, "Number of DMA descriptors required for LCD payload is: %d.", dma_lldesc_required);

  // Allocate descriptor block of memory if it hasn't already been allocated
  allocate_dma_desc_memory(dma_lldesc_required);

  // ripped from soc/lldesc.c
  int n = 0;
  while (len) {
    int dmachunklen = len;
    if (dmachunklen > LLDESC_MAX_NUM_PER_DESC) {
      dmachunklen = LLDESC_MAX_NUM_PER_DESC;
    }

    _dmadesc_a[n].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    _dmadesc_a[n].dw0.suc_eof = 0;
    _dmadesc_a[n].dw0.size = _dmadesc_a[n].dw0.length = dmachunklen;
    _dmadesc_a[n].buffer = (uint8_t *)data;
    _dmadesc_a[n].next = (dma_descriptor_t *)&_dmadesc_a[n + 1];

    len -= dmachunklen;
    data += dmachunklen;

    ESP_LOGD(TAG, "Configured _dmadesc_a at pos %d, memory location %08x, pointing to data of size %d. Next dma desc is %d.", n, (uintptr_t)&_dmadesc_a[n], (int)_dmadesc_a[n].dw0.size, n + 1);
    n++;
  }

  ESP_LOGD(TAG, "Configured _dmadesc_a at pos %d, to have EOF flag = 1.", n - 1);

  // All done, no looping here this time!
  _dmadesc_a[n - 1].dw0.suc_eof = 1;  //Mark last DMA desc as end of stream.
  _dmadesc_a[n - 1].next = NULL;      // no next item
  //_dmadesc_a[n-1].next        = (dma_descriptor_t *) &_dmadesc_a[0]; // loop forever


  _prev_payload_was_greyscale = is_greyscale_data;

  // Send it!
  return dma_transfer_start();
}


/*
  void Bus_Parallel16::dma_transfer_restart()
  {
    LCD_CAM.lcd_user.lcd_start = 1;        // Trigger LCD DMA transfer
    
  } // end 

    void Bus_Parallel16::dma_transfer_pause()
  {
    LCD_CAM.lcd_user.lcd_start = 0;        // Trigger LCD DMA transfer
    
  } // end 

  void Bus_Parallel16::dma_transfer_stop()
  {

 //      LCD_CAM.lcd_user.lcd_reset = 1;        // Trigger LCD DMA transfer
  //      LCD_CAM.lcd_user.lcd_update = 1;        // Trigger LCD DMA transfer

        gdma_stop(dma_chan);   
        
  } // end 
  */


#endif
