#include "esp32s3_peripheral.h"
#include "app_constants.hpp"

static const char *TAG = "esp32s3_peripheral";

uint16_t    active_row      = 0;
int         completed_trans = 0;

void setup_gpio_dir()
{
    ESP_LOGD(TAG, "Setup GPIOs");       

    // GPIO Setup
    esp_rom_gpio_pad_select_gpio(ADDR_A_PIN);
    esp_rom_gpio_pad_select_gpio(ADDR_B_PIN);
    esp_rom_gpio_pad_select_gpio(ADDR_C_PIN);
    esp_rom_gpio_pad_select_gpio(ADDR_D_PIN);        
    esp_rom_gpio_pad_select_gpio(ADDR_E_PIN);

    esp_rom_gpio_pad_select_gpio(MBI_GCLK);
    esp_rom_gpio_pad_select_gpio(MBI_LAT);
    esp_rom_gpio_pad_select_gpio(MBI_DCLK);
    
    esp_rom_gpio_pad_select_gpio(MBI_G1);
    esp_rom_gpio_pad_select_gpio(MBI_R1);    
    esp_rom_gpio_pad_select_gpio(MBI_B1); 
           
    esp_rom_gpio_pad_select_gpio(MBI_G2);
    esp_rom_gpio_pad_select_gpio(MBI_R2);    
    esp_rom_gpio_pad_select_gpio(MBI_B2);   

    esp_rom_gpio_pad_select_gpio(MBI_G3);
    esp_rom_gpio_pad_select_gpio(MBI_R3);    
    esp_rom_gpio_pad_select_gpio(MBI_B3);
            
    esp_rom_gpio_pad_select_gpio(MBI_G4);
    esp_rom_gpio_pad_select_gpio(MBI_R4);    
    esp_rom_gpio_pad_select_gpio(MBI_B4);       
        
    esp_rom_gpio_pad_select_gpio(MBI_SRCLK); // scan row clock in?
    esp_rom_gpio_pad_select_gpio(MBI_SPARE); // scan row clock in?    


    gpio_set_direction(ADDR_A_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADDR_B_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADDR_C_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADDR_D_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADDR_E_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(MBI_GCLK, GPIO_MODE_OUTPUT);
     gpio_set_direction(MBI_LAT, GPIO_MODE_OUTPUT);
    gpio_set_direction(MBI_DCLK, GPIO_MODE_OUTPUT);
    
    gpio_set_direction(MBI_G1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MBI_B1, GPIO_MODE_OUTPUT);    
    gpio_set_direction(MBI_R1, GPIO_MODE_OUTPUT);    
    gpio_set_direction(MBI_G2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MBI_B2, GPIO_MODE_OUTPUT);    
    gpio_set_direction(MBI_R2, GPIO_MODE_OUTPUT);    

    gpio_set_direction(MBI_G3, GPIO_MODE_OUTPUT);
    gpio_set_direction(MBI_B3, GPIO_MODE_OUTPUT);    
    gpio_set_direction(MBI_R3, GPIO_MODE_OUTPUT);   
     
    gpio_set_direction(MBI_G4, GPIO_MODE_OUTPUT);
    gpio_set_direction(MBI_B4, GPIO_MODE_OUTPUT);    
    gpio_set_direction(MBI_R4, GPIO_MODE_OUTPUT);       

    gpio_set_direction(MBI_SRCLK, GPIO_MODE_OUTPUT);    
    gpio_set_direction(MBI_SPARE, GPIO_MODE_OUTPUT);            
}     
    


void setup_gpio_output()
{
    // Set some default values
    gpio_set_level(ADDR_A_PIN,  0);
    gpio_set_level(ADDR_B_PIN,  0);
    gpio_set_level(ADDR_C_PIN,  0);
    gpio_set_level(ADDR_D_PIN,  0);        
    gpio_set_level(ADDR_E_PIN,  0);       

    gpio_set_level(MBI_GCLK,    0);
    gpio_set_level(MBI_LAT,     0);
    gpio_set_level(MBI_DCLK,    0);
    
    gpio_set_level(MBI_G1,      0);
    gpio_set_level(MBI_B1,      0);
    gpio_set_level(MBI_R1,      0);        

    gpio_set_level(MBI_G2,      0);
    gpio_set_level(MBI_B2,      0);
    gpio_set_level(MBI_R2,      0);      

    gpio_set_level(MBI_G3,      0);
    gpio_set_level(MBI_B3,      0);
    gpio_set_level(MBI_R3,      0);        

    gpio_set_level(MBI_G4,      0);
    gpio_set_level(MBI_B4,      0);
    gpio_set_level(MBI_R4,      0);              

    // Set some default values
    gpio_set_level(MBI_SRCLK,   0);  
    gpio_set_level(MBI_SPARE,   0);      
}
