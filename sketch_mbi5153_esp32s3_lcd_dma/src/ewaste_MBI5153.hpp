#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
//#include "driver/rmt_tx.h"

#define ghost_elimination_ON    1   //enable
#define ghost_elimination_OFF   0   //disable
#define gray_scale_13           1   //шкала серого 13 бит
#define gray_scale_14           0   //шкала серого 14 бит

/* GCLK multiplier 
MBI5153 provides a GCLK multiplier function by setting the configuration register1 bit [6]. The default value is set 
to ’0’ for GCLK multiplier disable = 513 required for each row.

GCLK multiplier enabled (configuration register1 bit [6] = 1) 
= 257 gclocks for each row!
*/

#define gclk_multiplier_ON      1   // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF     0   // GCLK Multipler On - You MUST use exactly 513 clocks for each rowscan!
#define current_min             0   //12.5%
#define current_def             32  //106%
#define current_max             64  //200%

#define clock_delay             0
#define pwm_hi                  65535  
#define pwm_lo                  0 


// Config for each 
/*
struct panel_cfg
{
    uint16_t mbi_blue[4];  // b1 to b4  //0..3
    uint16_t mbi_green[4]; // g1 to r4  //0..3
    uint16_t mbi_red[4];  // r1 to r4   //0..3
    int scan_lines = 4;
} panel_mbi_configs;
*/
void mbi_clock (uint8_t clock);
void mbi_configuration(uint8_t ghost_elimination, uint8_t line_num, uint8_t gray_scale, uint8_t gclk_multiplier,uint8_t current);
void mbi_configuration2();
void mbi_send_config(uint16_t config, bool latch, bool reg2 = false);
void mbi_pre_active ();
void mbi_v_sync ();
void mbi_soft_reset();

void mbi_set_frame_test();
void mbi_set_frame_lvgl_rgb(const uint8_t *rgb_data); //80 x 80 pixels


/***************************************************************************************/
// C/p'ed from https://ledshield.wordpress.com/2012/11/13/led-brightness-to-your-eye-gamma-correction-no/
//  Example calculator: https://gist.github.com/mathiasvr/19ce1d7b6caeab230934080ae1f1380e
//  need to make sure this would end up in RAM for fastest access
#ifndef NO_CIE1931
/*
static const uint8_t DRAM_ATTR lumConvTab[]={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 30, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 38, 38, 39, 40, 41, 41, 42, 43, 44, 45, 45, 46, 47, 48, 49, 50, 51, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 73, 74, 75, 76, 77, 78, 80, 81, 82, 83, 84, 86, 87, 88, 90, 91, 92, 93, 95, 96, 98, 99, 100, 102, 103, 105, 106, 107, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123, 124, 126, 128, 129, 131, 133, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150, 152, 154, 156, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187, 189, 192, 194, 196, 198, 200, 203, 205, 207, 209, 212, 214, 216, 218, 221, 223, 226, 228, 230, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255, 255};
*/
// This is 16-bit version of the table,
// the constants taken from the example in the article above, each entries subtracted from 65535:
static const uint16_t DRAM_ATTR lumConvTab[] = {
    0, 27, 56, 84, 113, 141, 170, 198, 227, 255, 284, 312, 340, 369, 397, 426,
    454, 483, 511, 540, 568, 597, 626, 657, 688, 720, 754, 788, 824, 860, 898, 936,
    976, 1017, 1059, 1102, 1146, 1191, 1238, 1286, 1335, 1385, 1436, 1489, 1543, 1598, 1655, 1713,
    1772, 1833, 1895, 1958, 2023, 2089, 2156, 2225, 2296, 2368, 2441, 2516, 2592, 2670, 2750, 2831,
    2914, 2998, 3084, 3171, 3260, 3351, 3443, 3537, 3633, 3731, 3830, 3931, 4034, 4138, 4245, 4353,
    4463, 4574, 4688, 4803, 4921, 5040, 5161, 5284, 5409, 5536, 5665, 5796, 5929, 6064, 6201, 6340,
    6482, 6625, 6770, 6917, 7067, 7219, 7372, 7528, 7687, 7847, 8010, 8174, 8341, 8511, 8682, 8856,
    9032, 9211, 9392, 9575, 9761, 9949, 10139, 10332, 10527, 10725, 10925, 11127, 11332, 11540, 11750, 11963,
    12178, 12395, 12616, 12839, 13064, 13292, 13523, 13757, 13993, 14231, 14473, 14717, 14964, 15214, 15466, 15722,
    15980, 16240, 16504, 16771, 17040, 17312, 17587, 17865, 18146, 18430, 18717, 19006, 19299, 19595, 19894, 20195,
    20500, 20808, 21119, 21433, 21750, 22070, 22393, 22720, 23049, 23382, 23718, 24057, 24400, 24745, 25094, 25446,
    25802, 26160, 26522, 26888, 27256, 27628, 28004, 28382, 28765, 29150, 29539, 29932, 30328, 30727, 31130, 31536,
    31946, 32360, 32777, 33197, 33622, 34049, 34481, 34916, 35354, 35797, 36243, 36692, 37146, 37603, 38064, 38528,
    38996, 39469, 39945, 40424, 40908, 41395, 41886, 42382, 42881, 43383, 43890, 44401, 44916, 45434, 45957, 46484,
    47014, 47549, 48088, 48630, 49177, 49728, 50283, 50842, 51406, 51973, 52545, 53120, 53700, 54284, 54873, 55465,
    56062, 56663, 57269, 57878, 58492, 59111, 59733, 60360, 60992, 61627, 62268, 62912, 63561, 64215, 64873, 65535};
#endif
