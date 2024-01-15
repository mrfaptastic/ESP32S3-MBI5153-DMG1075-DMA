#pragma once
#include "hal/gpio_types.h"

// Updated ESP32-S3 Pin GPIO
/* Experimenting with the ESP32-S3 Dev Module has uncovered that ONLY THESE pins seems to work ok
 * use other pins at your risk. Don't use 0, 19, 20, 26-38 etc.
 * https://api.riot-os.org/group__cpu__esp32__esp32s3.html
 */

#define GPIO_MAPPING_DEFAULT     1
#define GPIO_MAPPING_PCB_DEVS3   2
#define GPIO_MAPPING_PCB_DHRUV   3 

// Selected mode
#define GPIO_MAPPING GPIO_MAPPING_PCB_DHRUV


#if (GPIO_MAPPING == GPIO_MAPPING_DEFAULT)

  #define ADDR_A_PIN              GPIO_NUM_5
  #define ADDR_B_PIN              GPIO_NUM_4
  #define ADDR_C_PIN              GPIO_NUM_42
  #define ADDR_D_PIN              GPIO_NUM_7
  #define ADDR_E_PIN              GPIO_NUM_41

  #define MBI_GCLK                GPIO_NUM_1  // OE PIN IS GCLK apparently
  #define MBI_LAT                 GPIO_NUM_6  //  data/command
  #define MBI_DCLK                GPIO_NUM_2  // data clocking line?
  #define MBI_SRCLK               GPIO_NUM_48   // I assume SR stands for Scan Row??  // When this is HIGH on these boards, output is disabled?

  // First 1/4 of panel -> 20 rows
  #define MBI_G1                  GPIO_NUM_40 
  #define MBI_B1                  GPIO_NUM_16 
  #define MBI_R1                  GPIO_NUM_15  

  // Second 1/4 of panel -> 20 rows
  #define MBI_G2                  GPIO_NUM_39    
  #define MBI_B2                  GPIO_NUM_17  
  #define MBI_R2                  GPIO_NUM_18  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3                  GPIO_NUM_8  
  #define MBI_B3                  GPIO_NUM_45  
  #define MBI_R3                  GPIO_NUM_46  

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4                  GPIO_NUM_3    
  #define MBI_B4                  GPIO_NUM_21  
  #define MBI_R4                  GPIO_NUM_47  

#elif (GPIO_MAPPING == GPIO_MAPPING_PCB_DEVS3)  
// S3-Dev module header / quick and dirty pcb.

#elif (GPIO_MAPPING == GPIO_MAPPING_PCB_DHRUV)  
  #define ADDR_A_PIN              GPIO_NUM_21
  #define ADDR_B_PIN              GPIO_NUM_46
  #define ADDR_C_PIN              GPIO_NUM_45
  #define ADDR_D_PIN              GPIO_NUM_8
  #define ADDR_E_PIN              GPIO_NUM_39

  #define MBI_GCLK                GPIO_NUM_47  // OE PIN IS GCLK apparently
  #define MBI_LAT                 GPIO_NUM_3  //  data/command
  #define MBI_DCLK                GPIO_NUM_48  // data clocking line?
  #define MBI_SRCLK               GPIO_NUM_4   // I assume SR stands for Scan Row??  // When this is HIGH on these boards, output is disabled?

  // First 1/4 of panel -> 20 rows
  #define MBI_G1                  GPIO_NUM_40 
  #define MBI_B1                  GPIO_NUM_17 
  #define MBI_R1                  GPIO_NUM_18  

  // Second 1/4 of panel -> 20 rows
  #define MBI_G2                  GPIO_NUM_41    
  #define MBI_B2                  GPIO_NUM_15  
  #define MBI_R2                  GPIO_NUM_16  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3                  GPIO_NUM_7  
  #define MBI_B3                  GPIO_NUM_2  
  #define MBI_R3                  GPIO_NUM_42  

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4                  GPIO_NUM_6    
  #define MBI_B4                  GPIO_NUM_5  
  #define MBI_R4                  GPIO_NUM_1  

#endif


// Spare Unencumbered GPIOS
// https://api.riot-os.org/group__cpu__esp32__esp32s3.html
#define MBI_SPARE               GPIO_NUM_9  
#define MBI_SPARE2              GPIO_NUM_10
#define MBI_SPARE3              GPIO_NUM_11
#define MBI_SPARE4              GPIO_NUM_12
#define MBI_SPARE5              GPIO_NUM_13
#define MBI_SPARE6              GPIO_NUM_14


// -------------------------------------------------------
// Internals specific to code and hardware. Do not change.
// -------------------------------------------------------

// DMA engine (if used)
#define ESP32_I2S_DMA_STORAGE_TYPE uint8_t // DMA output of one uint16_t at a time.

// For DMA GLCK and Address Line Data
#define BIT_GCLK (1 << 0)
#define BIT_A (1 << 1)
#define BIT_B (1 << 2)
#define BIT_C (1 << 3)
#define BIT_D (1 << 4)
#define BIT_E (1 << 5)

// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines
#define PANEL_MBI_LENGTH        5  // number of ic's changed for each subpixel
