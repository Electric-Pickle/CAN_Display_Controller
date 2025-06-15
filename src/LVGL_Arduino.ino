/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include "../include/pin_config.h"
#include "../include/CAN.h"

#include "Display_ST7789.h"
#include "RTC_PCF85063.h"
#include "Gyro_QMI8658.h"
#include "LVGL_Driver.h"
#include "ui.h"
#include "PWR_Key.h"
// #include "LVGL_Example.h"
#include "BAT_Driver.h"

void DriverTask(void *parameter) {
  while(1){
    PWR_Loop();
    BAT_Get_Volts();
    PCF85063_Loop();
    QMI8658_Loop(); 
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void Driver_Loop() {
  xTaskCreatePinnedToCore(
    DriverTask,           
    "DriverTask",         
    4096,                 
    NULL,                 
    3,                    
    NULL,                 
    0                     
  );  
}
void setup()
{
  PWR_Init(); // POWER LATCH/UNLATCH
  BAT_Init(); // BAT VOLTAGE READOUT
  I2C_Init(); // TOUCH ?!?
  PCF85063_Init(); // RTC
  QMI8658_Init(); // IMU

  CANbus::init();
  pinMode(sw1pin, INPUT_PULLDOWN);
  pinMode(sw2pin, INPUT_PULLDOWN);
  // pinMode(bootBTN, INPUT); // be carefull with this not to interfere with booting

  Backlight_Init(); // ...
  LCD_Init();
  Lvgl_Init();

  ui_init();
  // Lvgl_Example1();
  Driver_Loop();
}

void loop()
{
  Lvgl_Loop();
  vTaskDelay(pdMS_TO_TICKS(5));
}
