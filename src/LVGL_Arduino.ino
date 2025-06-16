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
      static CANbus::share_data_t data_save;

      Serial.print("COMMS");
      lv_label_set_text(ui_COMMS, "COMMS OK");
      char print_str[30];
      char print_str2[15];
      char print_str3[15];
      // line 1, Actual Performance
      if ((data.volts != data_save.volts) || (data.current != data_save.current)) // only rewite the line if there are changes to the data
      {
        itoa(roundf(data.volts), print_str, 10u);
        itoa(roundf(data.current), print_str2, 10u);
        data.power = data.volts * data.current;
        itoa(roundf((data.power / 1000.0f)), print_str3, 10u);
        strcat(print_str, "v ");
        strcat(print_str, print_str2);
        strcat(print_str, "a ");
        strcat(print_str, print_str3);
        strcat(print_str, "kw");
        lv_label_set_text(ui_VOLT, print_str);
      }
      // line 2, Allowed Power limits
      if ((data.power_crg != data_save.power_crg) || (data.power_dsg != data_save.power_dsg)) // only rewite the line if there are changes to the data
      {
        itoa(roundf(data.power_dsg), print_str, 10u);
        itoa(roundf(data.power_crg), print_str2, 10u);
        strcat(print_str, "kw dsg ");
        strcat(print_str, print_str2);
        strcat(print_str, "kw crg");
        lv_label_set_text(ui_POWER, print_str);
      }
      // line 3, TEMP average
      if ((data.t_lowest != data_save.t_lowest) || (data.t_highest != data_save.t_highest)) // only rewite the line if there are changes to the data
      {
        itoa(roundf((data.t_lowest + data.t_highest) / 2.0f), print_str, 10u);
        strcat(print_str, "c ");
        lv_label_set_text(ui_TEMP, print_str);
      }
      // line 4, soc
      if (data.t_lowest != data_save.t_lowest)
      {
        itoa(roundf(data.soc), print_str, 10u);
        strcat(print_str, "% ");
        lv_label_set_text(ui_SOC, print_str);
      }
      // line 5, op sts
      if (data.op_sts != data_save.op_sts)
      {
        switch (data.op_sts)
        {
        case CANbus::OP_STATUS_T::STATUS_OK:
          lv_label_set_text(ui_TEXT1, "OP OK");
          break;
        case CANbus::OP_STATUS_T::STATUS_FAULTED:
          lv_label_set_text(ui_TEXT1, "OP FAULTED");
          break;
        default:
          lv_label_set_text(ui_TEXT1, "OP NA");
          break;
        }
      }
      // line 6, BUS sts
      if (data.bus_sts != data_save.bus_sts)
      {
        switch (data.bus_sts)
        {
        case CANbus::BUS_STATUS_T::BUS_OFF:
          lv_label_set_text(ui_TEXT2, "BUS OFF");
          break;
        case CANbus::BUS_STATUS_T::BUS_ON:
          lv_label_set_text(ui_TEXT2, "BUS ON");
          break;
        case CANbus::BUS_STATUS_T::PRECHARGE:
          lv_label_set_text(ui_TEXT2, "BUS PRECAHRGE");
          break;
        case CANbus::BUS_STATUS_T::STOP_REQ:
          lv_label_set_text(ui_TEXT2, "BUS STOP");
          break;
        default:
          lv_label_set_text(ui_TEXT2, "BUS NA");
          break;
        }
      }

      data_save = data; // update previous data to match current data for next comparison
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
