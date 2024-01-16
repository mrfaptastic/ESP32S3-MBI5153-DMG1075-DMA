#include "ewaste_MBI5153.hpp"
#include "app_constants.hpp"
#include "Arduino.h"
#include <iostream>

static const char *TAG = "mbi5153";

/* clocking the MBI5153 driver data line*/
void mbi_clock(uint8_t clock) {
  while (clock--) {
    gpio_set_level((gpio_num_t)MBI_DCLK, 1);
//    gpio_set_level((gpio_num_t)MBI_GCLK, 1); // gclk is overridden by DMA, so this won't do anything anyway. 
//    esp_rom_delay_us(clock_delay);
    gpio_set_level((gpio_num_t)MBI_DCLK, 0);
//    gpio_set_level((gpio_num_t)MBI_GCLK, 0);
  }
}

/*pre-activation command - sent before sending configuration register data*/
void mbi_pre_active() {
  ESP_LOGD(TAG, "Send MBI Pre-Active.");

  gpio_set_level((gpio_num_t)MBI_LAT, 1);
  mbi_clock(14);
  gpio_set_level((gpio_num_t)MBI_LAT, 0);

  /* LE/LAT should be low for any rising edge of DCLK */
  mbi_clock(2);
}

/*vertical sync - updates frame data on outputs, used in conjunction with vertical scan*/
// 9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signa
void mbi_v_sync() {
  ESP_LOGD(TAG, "Send MBI Vert Sync.");
  gpio_set_level((gpio_num_t)MBI_LAT, 1);
  mbi_clock(2);
  gpio_set_level((gpio_num_t)MBI_LAT, 0);
  mbi_clock(2);
}

/*soft reset*/
void mbi_soft_reset() {
  /* “Software reset” command makes MBI5153 go back to the initial state except configuration register value. After this 
command is received, the output channels will be turned off and will display again with last gray-scale value after 
new “Vsync” command is received.
*/
  ESP_LOGD(TAG, "Send MBI Soft Reset.");

  gpio_set_level((gpio_num_t)MBI_LAT, 1);
  mbi_clock(10);
  gpio_set_level((gpio_num_t)MBI_LAT, 0);
}


/*настройка регистра конфигурации*/
void mbi_configuration(uint8_t ghost_elimination, uint8_t line_num, uint8_t gray_scale, uint8_t gclk_multiplier, uint8_t current) {
  uint16_t config_reg1_val = 0;
  
  config_reg1_val = 0b1001001110101011;

  //Documentation says set bits E and F of Config1 Reg to 0
  // config_reg1_val = (config_reg1_val | (ghost_elimination << 14) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | (current));

  ESP_LOGD(TAG, "Reg1 value. %u", config_reg1_val);

  for (int i = 0; i < PANEL_MBI_LENGTH; i++)
    mbi_send_config(config_reg1_val, (i == (PANEL_MBI_LENGTH - 1)));  // on last panel, latch
}

/*настройка регистра конфигурации*/
void mbi_configuration2() 
{

  // F E D C B A 9 8 7 6 5 4 3 2 1 0

  uint16_t config_reg2_val = 0b0001000000010000; // default value apparently

  // config_reg2_val = 0b1001000000011110; // removes ghosting except for red

  ESP_LOGD(TAG, "Reg2 value. %u", config_reg2_val);
  
  for (int i = 0; i < PANEL_MBI_LENGTH; i++)
    mbi_send_config(config_reg2_val, (i == (PANEL_MBI_LENGTH - 1)), true);  // on last panel, latch
}


/*отправка регистра конфигурации*/
/* Need to set config for all MBI input on the panel at once as they all share the same latch pin */
void mbi_send_config(uint16_t config_reg, bool latch, bool reg2) {
  //  ESP_LOGD(TAG, "Send MBI config");

  int latch_trigger_point = (reg2) ? 9:5; 

  bool config_bit[16] = { 0 };
  int data_mask = 0x1;
  for (size_t i = 0; i < 16; i++)  // parses bytes into a boolean array bit by bit
  {
    config_bit[i] = config_reg & (data_mask << i);
  }

  for (size_t i = 16; i > 0; i--)  // sends data to the controller leg from the array
  {
    /* 
         * For the love of god don't change a fucking thing here! 
         * Refactoring it, even changing gpio_set_level order, results in the panel to display garbage!
         * Potential electrical crosstalk!?
         */
    if ((i < latch_trigger_point) && (latch == true))  // for reg1, data latch on the last 4 bits when latch is true (on last send)
    {
      gpio_set_level((gpio_num_t)MBI_LAT, 1);
      gpio_set_level(MBI_G1, config_bit[i - 1]);
      gpio_set_level(MBI_G2, config_bit[i - 1]);
      gpio_set_level(MBI_G3, config_bit[i - 1]);
      gpio_set_level(MBI_G4, config_bit[i - 1]);

      gpio_set_level(MBI_R1, config_bit[i - 1]);
      gpio_set_level(MBI_R2, config_bit[i - 1]);
      gpio_set_level(MBI_R3, config_bit[i - 1]);
      gpio_set_level(MBI_R4, config_bit[i - 1]);

      gpio_set_level(MBI_B1, config_bit[i - 1]);
      gpio_set_level(MBI_B2, config_bit[i - 1]);
      gpio_set_level(MBI_B3, config_bit[i - 1]);
      gpio_set_level(MBI_B4, config_bit[i - 1]);

      mbi_clock(1);
      //     ESP_LOGD(TAG, "Sent config bit with latch HIGH");
    } else {
      gpio_set_level(MBI_G1, config_bit[i - 1]);
      gpio_set_level(MBI_G2, config_bit[i - 1]);
      gpio_set_level(MBI_G3, config_bit[i - 1]);
      gpio_set_level(MBI_G4, config_bit[i - 1]);

      gpio_set_level(MBI_R1, config_bit[i - 1]);
      gpio_set_level(MBI_R2, config_bit[i - 1]);
      gpio_set_level(MBI_R3, config_bit[i - 1]);
      gpio_set_level(MBI_R4, config_bit[i - 1]);

      gpio_set_level(MBI_B1, config_bit[i - 1]);
      gpio_set_level(MBI_B2, config_bit[i - 1]);
      gpio_set_level(MBI_B3, config_bit[i - 1]);
      gpio_set_level(MBI_B4, config_bit[i - 1]);

      mbi_clock(1);
    }
  }
  gpio_set_level((gpio_num_t)MBI_LAT, 0);

  gpio_set_level(MBI_G1, 0);
  gpio_set_level(MBI_G2, 0);
  gpio_set_level(MBI_G3, 0);
  gpio_set_level(MBI_G4, 0);


  gpio_set_level(MBI_R1, 0);
  gpio_set_level(MBI_R2, 0);
  gpio_set_level(MBI_R3, 0);
  gpio_set_level(MBI_R4, 0);

  gpio_set_level(MBI_B1, 0);
  gpio_set_level(MBI_B2, 0);
  gpio_set_level(MBI_B3, 0);
  gpio_set_level(MBI_B4, 0);

  // ESP_LOGD(TAG, "Completed MBI config sending. Latch is now low.");
}

/*transfer 2 bytes of data to the MBI5153 driver*/
void mbi_send_data_test(uint16_t data, bool latch) {
  // ESP_LOGD(TAG, "Transfer 2 bytes of data to the MBI5153 driver..");

  bool data_bit[16] = { 0 };
  int data_mask = 0x1;
  for (size_t i = 0; i < 16; i++)  //parses bytes into an array bit by bit
  {
    data_bit[i] = data & (data_mask << i);
  }
  for (size_t i = 16; i > 0; i--)  //sends data to the controller leg from the array
  {
    if ((i == 1) && (latch == true))  //data latch on the last bit, when sending the last byte set latch=1
    {
      gpio_set_level((gpio_num_t)MBI_LAT, 1);
      gpio_set_level((gpio_num_t)MBI_G1, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_R2, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_B3, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_G4, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_R4, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_B4, data_bit[i - 1]);  // only one pin so far
      mbi_clock(1);
      gpio_set_level((gpio_num_t)MBI_LAT, 0);
      latch = 0;
    } else {
      gpio_set_level((gpio_num_t)MBI_G1, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_R2, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_B3, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_G4, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_R4, data_bit[i - 1]);  // only one pin so far
      gpio_set_level((gpio_num_t)MBI_B4, data_bit[i - 1]);  // only one pin so far
      mbi_clock(1);
    }
  }
}


/*sends data for 1 frame*/
void mbi_set_frame_lvgl_rgb(const uint8_t *rgb_888_data) {
  uint16_t b1_data, g1_data, r1_data, b2_data, g2_data, r2_data, b3_data, g3_data, r3_data, b4_data, g4_data, r4_data;

  // Convert one fucked up format to another.
  for (int row = 0; row < PANEL_SCAN_LINES; row++) {
    // Stuff that's clocked in FIRST shows on the LEFT of the panel - which makes things easier.
    // TO DO, need to compensate for the LEFT 2 pixels being dropped as we clock in 80 px wide of data, but there's pyhsically only 78 pixels on the panel
    int x_pixel_counter = 0;
    for (int chan = 0; chan < 16; chan++) {
      for (int ic = 0; ic < PANEL_MBI_LENGTH; ic++)  // number of chained ICs
      {
        // Get pixel colour data from LVGL_TRUE_COLOR c array
        // Get the right data from the LVGL 80x80 c array.
        // TODO: WE LOSE THE LEFT MOST 2 PIXELS DUE TO THE PANEL ONLY BEING 78 pixels.
        int rgb_data_start_pos = (row * 80);  // 80 pixels per row, only 78 visible pixels per row
        rgb_data_start_pos += (ic * 16);      // 16 pixels per 5153 IC
        rgb_data_start_pos += chan;           // offset by current
        rgb_data_start_pos *= 4;              // to get 32bit offset for CF_TRUE_COLOR c array

        // std::cout << "row " << row << " ic " << ic << " chan " << chan << " equals offset (start byte) " << rgb_data_start_pos <<  ". x pixel counter " << x_pixel_counter << "\n";

        b1_data = rgb_888_data[(6400 * 0) + rgb_data_start_pos];
        g1_data = rgb_888_data[(6400 * 0) + rgb_data_start_pos + 1];
        r1_data = rgb_888_data[(6400 * 0) + rgb_data_start_pos + 2];

        b1_data = lumConvTab[b1_data];
        g1_data = lumConvTab[g1_data];
        r1_data = lumConvTab[r1_data];

        r2_data = rgb_888_data[(6400 * 1) + rgb_data_start_pos];
        g2_data = rgb_888_data[(6400 * 1) + rgb_data_start_pos + 1];
        b2_data = rgb_888_data[(6400 * 1) + rgb_data_start_pos + 2];

        b2_data = lumConvTab[b2_data];
        g2_data = lumConvTab[g2_data];
        r2_data = lumConvTab[r2_data];

        b3_data = rgb_888_data[(6400 * 2) + rgb_data_start_pos];
        g3_data = rgb_888_data[(6400 * 2) + rgb_data_start_pos + 1];
        r3_data = rgb_888_data[(6400 * 2) + rgb_data_start_pos + 2];

        b3_data = lumConvTab[b3_data];
        g3_data = lumConvTab[g3_data];
        r3_data = lumConvTab[r3_data];

        b4_data = rgb_888_data[(6400 * 3) + rgb_data_start_pos];
        g4_data = rgb_888_data[(6400 * 3) + rgb_data_start_pos + 1];
        r4_data = rgb_888_data[(6400 * 3) + rgb_data_start_pos + 2];

        b4_data = lumConvTab[b4_data];
        g4_data = lumConvTab[g4_data];
        r4_data = lumConvTab[r4_data];  


        // CF_TRUE_COLOR byte order is B,G,R,0xff
        //printf("blue: %#08x  green: %#08x  red: %#08x\n\n", rgb_888_data[rgb_data_start_pos], rgb_888_data[rgb_data_start_pos+1], rgb_888_data[rgb_data_start_pos+2]);
        // printf("%#04x, %#04x, %#04x\n\n", b1_data, g1_data, r1_data);
        // ICS => 0, 1, 2, 3, 4,


        //
        // Step 2, shift to panel
        //

        // data latch on the last bit, when sending the last byte set latch=1
        int latch = 0;
        if (ic == 4) { latch = 1; }  // latch on last channel / ic

        int bit_offset = 16;
        while (bit_offset > 0)  // shift out MSB first per the documentation.
        {
          bit_offset--;  // start from 15

          if (latch == 1 && bit_offset == 0) {
            gpio_set_level((gpio_num_t)MBI_LAT, 1);
          }


          //printf("v2: bit %d has value %d\n\n", bit_offset, ((data[pos] >> bit_offset) & 0x1));

          // Need to convert the 8bit RGB values to 16 bit, shift the 8 LSB to the MSB byte or they'll just
          // get shifted out after 8 cycles.
          // Also improves brightness..


          // Shift out the First 1/4 of RGB panel - 20 rows

          // Offset for next block of scan lines
          // 80px * 20 * (4 bytes per the LVGL True Color format) = 1600
          // 80*20*4 = 6400

          // Shift out the First 1/4 of RGB panel - 20 rows
          gpio_set_level(MBI_B1, ((b1_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_G1, ((g1_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_R1, ((r1_data >> bit_offset) & 0x1));

          // Second Quarter
          gpio_set_level(MBI_R2, ((r2_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_G2, ((g2_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_B2, ((b2_data >> bit_offset) & 0x1));

          // Offset for next block of scan lines
          gpio_set_level(MBI_R3, ((r3_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_G3, ((g3_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_B3, ((b3_data >> bit_offset) & 0x1));

          gpio_set_level(MBI_R4, ((r4_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_G4, ((g4_data >> bit_offset) & 0x1));
          gpio_set_level(MBI_B4, ((b4_data >> bit_offset) & 0x1));

          mbi_clock(1);

          if (latch == 1 && bit_offset == 0) {
            gpio_set_level((gpio_num_t)MBI_LAT, 0);
          }
        }

      }  // MBI LENGTH
    }    // MBI Channel
  }      // Row Scan

  // give it more time to latch into system
  mbi_clock(50);

  //flag_change_frame = 1;
}
