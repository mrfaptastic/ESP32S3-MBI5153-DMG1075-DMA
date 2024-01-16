#include <Arduino.h>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp_log.h>
#include "esp_task_wdt.h"
#include "main.hpp"
#include "ewaste_MBI5153.hpp"
#include "esp32s3_peripheral.h"
#include "test_pattern.h"
#include "dma_data.h"
#include "Fastnoise.h"

//#include "wificonnect.h"
//#include "e131.h"


#define GCLK_ADDR_MODE_DMA 1

// Selected mode
#define GCLK_ADDR_MODE GCLK_ADDR_MODE_DMA


static const char *TAG  = "app_main";
volatile int refresh    = 0;
volatile int image      = 0;

uint8_t wsRawData[80 * 80 * 4];

//E131 e131;

FastNoiseLite noise;

int simplexColorR = 0;
int simplexColorG = 209;
int simplexColorB = 255;

int simplexBrightness = -33;
float simplexContrast = 72;
float simplexScale = 5;
float simplexSpeed = 20;

int row = 0;

unsigned long  last_display  = 0;

unsigned long dma_delay = 5;
unsigned long dma_delay_last = 0;

bool alloc_dma_data_buffer() 
{ 
    dma_gpio_size        = sizeof(dma_data_nodt);
    ESP_LOGD("I2S-DMA", "Size of DMA data we need to use to drive is %u.", dma_gpio_size);  
    
    // Defined in main.hpp
    dma_gpio_data = (ESP32_I2S_DMA_STORAGE_TYPE *)heap_caps_malloc(dma_gpio_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    if (dma_gpio_data == nullptr) 
    {
        ESP_LOGE("I2S-DMA", "Could not malloc space..");
        return false;
        // TODO: should we release all previous rowBitStructs here???
    }  

    ESP_LOGD("I2S-DMA", "Copying bootstrap DMA data across.");                
    memcpy(dma_gpio_data, dma_data_nodt, dma_gpio_size);    // Using the same data for now.

    return true;
} // end dma alloc

bool configure_dma_gclk()
{  
    if ( !alloc_dma_data_buffer() )
    {
        ESP_LOGE("I2S-DMA", "Failed to allocate DMA memory.");          
        return false;
    }

    // Calculate DMA descriptors required
    int dma_descriptors_required = (dma_gpio_size/DMA_MAX) + (dma_gpio_size % DMA_MAX != 0);
    int last_dma_packet_size     = (dma_gpio_size % DMA_MAX == 0) ? DMA_MAX:(dma_gpio_size % DMA_MAX);
        
    dma_bus.allocate_dma_desc_memory(dma_descriptors_required);
    ESP_LOGI("I2S-DMA", "%d DMA descriptors required for cover buffer data.", dma_descriptors_required);    

    // Link up DMA descriptors chain to DMA data.
    int dma_buffer_offset = 0;
    for (int dma_desc = 0; dma_desc < dma_descriptors_required-1; dma_desc++)
    {
        dma_bus.create_dma_desc_link(&dma_gpio_data[dma_buffer_offset], DMA_MAX, false);
        dma_buffer_offset += DMA_MAX;
    }
    dma_bus.create_dma_desc_link(&dma_gpio_data[dma_buffer_offset], last_dma_packet_size, false);
    

    // Setup DMA and Output to GPIO
    auto bus_cfg = dma_bus.config(); 

    bus_cfg.pin_wr      = MBI_SPARE; // must be allocated a clock pin. even if not used
    bus_cfg.invert_pclk = false;

    bus_cfg.pin_d0 = MBI_GCLK; // 
    bus_cfg.pin_d1 = ADDR_A_PIN;
    bus_cfg.pin_d2 = ADDR_B_PIN;
    bus_cfg.pin_d3 = ADDR_C_PIN;
    bus_cfg.pin_d4 = ADDR_D_PIN;
    bus_cfg.pin_d5 = ADDR_E_PIN;
    bus_cfg.pin_d6 = -1;
    bus_cfg.pin_d7 = -1;

    dma_bus.config(bus_cfg);

    dma_bus.init();

    return true;
}



// Start the App
void setup(void)
{
  delay(4000);
  //  Serial.begin(115200);

  // Serial.setDebugOutput(true);

//    connectWiFi();

//    e131.begin();

    //Noise settings
    noise.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2S);
    noise.SetRotationType3D(FastNoiseLite::RotationType3D_ImproveXYPlanes);
    noise.SetFrequency(.0004);
    //   noise.SetFractalType(FastNoiseLite::FractalType_FBm);
    //   noise.SetFractalLacunarity(2.7);
    //   noise.SetFractalOctaves(6);
    //   noise.SetFractalGain(0.1);
    //   noise.SetFractalLacunarity(2.7);
    //   noise.SetFractalWeightedStrength(.1);
    noise.SetDomainWarpType(FastNoiseLite::DomainWarpType_OpenSimplex2);
    noise.SetDomainWarpAmp(240);

    ESP_LOGD(TAG, "Configure GPIOs");     
       
    setup_gpio_dir();
    setup_gpio_output();    
    
    // esp_task_wdt_deinit();
      
    delay(2);
 
    #if (GCLK_ADDR_MODE == GCLK_ADDR_MODE_DMA)

        ESP_LOGD(TAG, "Setup MBI5153 with DMA"); 

        configure_dma_gclk();  
        dma_bus.dma_transfer_start();  // start gclk + addr toggle
      
        // Confiure register1
        mbi_soft_reset();    
        mbi_pre_active(); // must      
        mbi_configuration(ghost_elimination_ON,(PANEL_SCAN_LINES-1),gray_scale_14,gclk_multiplier_OFF,current_max);  

        // Configure register 2
        mbi_pre_active(); // must          
        mbi_configuration2();    
        
    #endif

}

void loop()
{
    if (refresh)
    {
       for (int i = 0; i < 80; i++) {
         for (int j = 0; j < 80; j++) {
           int col = int((1 + noise.GetNoise(j * simplexScale * 10, i * simplexScale * 10, float(millis() * simplexSpeed / 50))) * 127);
           col += simplexBrightness;
           col = constrain(col, 0, 255);
           float contrastFactor = (259 * (simplexContrast + 255)) / (255 * (259 - simplexContrast));
           col = contrastFactor * (col - 128) + 128;
           col = constrain(col, 0, 255);
           
           int index = (i * 80 + j) * 4;
           wsRawData[index] = uint8_t(col * simplexColorB / 255.0f); // Blue
           wsRawData[index + 1] = uint8_t(col * simplexColorG / 255.0f); // Green
           wsRawData[index + 2] = uint8_t(col * simplexColorR / 255.0f); // Red
           wsRawData[index + 3] = 0xFF; // blank
         }
       }
       mbi_set_frame_lvgl_rgb(wsRawData);
    
      
      // if (image > 2) image = 0;
      
      // switch (image)
      // {
      //     case 0: mbi_set_frame_lvgl_rgb(test_pattern_1); // house
      //             break;
      //     case 1: mbi_set_frame_lvgl_rgb(test_pattern_2); // lines
      //             break;
      //     case 2: mbi_set_frame_lvgl_rgb(test_pattern_3); // text
      //             break;
      // }
      //   image++;

        dma_bus.dma_transfer_stop();          

        /* Need to put a delay in for the moment as when we call dma_transfer_stop() the DMA transfer doesn't actually stop immediately
         * and continues in an async manner in the background. This causes a stuffup / corruption of pixel data around the time of the immediate vsync 
         *
         * Not sure what SRCLK does either, but toggling it high and then low around time of greyscale data transfer stops visible noise showing.
         */
        gpio_set_level(MBI_SRCLK,  1);
        dma_delay_last = millis();
        while(millis() - dma_delay_last < dma_delay) {};
        mbi_v_sync(); 
        dma_bus.dma_transfer_start();
        gpio_set_level(MBI_SRCLK,  0);  

        refresh = 0;
    }
    


    if ((millis() - last_display) > 0)  {
    
        last_display = millis();
  
        refresh = 1;

        ESP_LOGI("Main", "Requeseted refresh with image %d", image);

    }    
}
