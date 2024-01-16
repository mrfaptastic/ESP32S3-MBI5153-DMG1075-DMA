#pragma once
#include "app_constants.hpp"
#include "gdma_lcd_parallel16.hpp"

Bus_Parallel16 dma_bus;

uint8_t *dma_gpio_data;
size_t   dma_gpio_size;


/*
  // MBI5152 Application Note V1.00- EN

  Section 2: The Setting of Gray Scale 
  The setting of gray scale data describes as below. 
  1. The sequence of input data starts from scan line 1 scan line 2….  scan line M-1 scan line M 
  (M≦16) 
  2. The data sequence of cascaded IC is ICnICn-1…. IC2IC1. 
  3. The data sequence of each channel is ch15ch14…. ch0. 
  4. The data length of each channel is 16-bits, and the default PWM mode is 16-bits. The sequence of gray 
  sacle is bit15bit14bit13…bit0 as figure 4 shows. The 14-bits gray scale can be set through Bit[7]=1 
  in configuration register 1, and the sequence of gray scale data is bit13bit12… bit0 0 0, the 
  last 2-bits (LSB) are set to “0”. 
  5. The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data. 
  6. LE executes the data latch to send gray scale data into SRAM. Each 16xN bits data needs a “data latch 
  command”, where N means the number of cascaded driver. 
  7. After the last data latch, it needs at least 50 GCLKs to read the gray scale data into internal display buffer 
  before the Vsync command comes. 
  8. Display is updated immediately when MBI5152 receives the Vsync signal. 
  9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signal. 
  10. The period of dead time (ie. The 1025th GCLK) must be larger than 100ns.

  The gray scale data needs the GCLK to save the data into SRAM. The frequency of GCLK must be higher 
  than 20% of DCLK to get the correct data. 

  // MBI5153
  After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display 
  buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives 
  the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.

*/
