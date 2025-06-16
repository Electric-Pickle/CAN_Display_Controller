/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <stdio.h>
#include <string.h>

#include "../include/pin_config.h"
#include "../include/CAN.h"
#include "../lib/ui/ui.h"

#include "../include/Display_ST7789.h"
#include "../include/RTC_PCF85063.h"
#include "../include/Gyro_QMI8658.h"
#include "../include/LVGL_Driver.h"
#include "../include/PWR_Key.h"
#include "../include/BAT_Driver.h"

void DriverTask(void *parameter)
{
  while (1)
  {
    PWR_Loop();
    BAT_Get_Volts();
    PCF85063_Loop();
    QMI8658_Loop();
    CANbus::hv_vs_frame();
    if (!digitalRead(sw2pin))
    {
      CANbus::enable_pack(static_cast<CANbus::CMD_STS_T>(!digitalRead(sw1pin)));
    }

    CANbus::share_data_t data = CANbus::get_message_data();
    if (CANbus::get_rx_OK())
    {
      Serial.print("COMMS");
      lv_label_set_text(ui_COMMS, "COMMS OK");
      char print_str[15];
      itoa(roundf(data.volts), print_str, 10);
      lv_label_set_text(ui_VOLT, print_str);
      itoa(roundf(data.current), print_str, 10);
      lv_label_set_text(ui_POWER, print_str);
    }
    else
    {
      lv_label_set_text(ui_COMMS, "NO COMMS");
      lv_label_set_text(ui_VOLT, "0v");
      lv_label_set_text(ui_POWER, "0w");
      lv_label_set_text(ui_TEMP, "0c");
      lv_label_set_text(ui_TEXT1, "TEXT1");
      lv_label_set_text(ui_TEXT2, "TEXT2");
      lv_label_set_text(ui_SOC, "0%");
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void Driver_Loop()
{
  xTaskCreatePinnedToCore(
      DriverTask,
      "DriverTask",
      4096,
      NULL,
      3,
      NULL,
      0);
}
void setup()
{
  vTaskDelay(pdMS_TO_TICKS(100)); // make sure we boot good n proper before we do any configuring and using the boot button as an input
  PWR_Init();                     // POWER LATCH/UNLATCH
  BAT_Init();                     // BAT VOLTAGE READOUT
  I2C_Init();                     // TOUCH ?!?
  PCF85063_Init();                // RTC
  QMI8658_Init();                 // IMU

  CANbus::init();
  pinMode(sw1pin, INPUT_PULLUP);
  pinMode(sw2pin, INPUT_PULLUP);
  pinMode(bootBTN, INPUT); // be carefull with this not to interfere with booting

  Backlight_Init(); // ...
  LCD_Init();
  Lvgl_Init();

  ui_init();
  Driver_Loop();

  Serial.begin(115200); // virtual COM via programmer
}

void loop()
{
  Lvgl_Loop();
  vTaskDelay(pdMS_TO_TICKS(5));
}
