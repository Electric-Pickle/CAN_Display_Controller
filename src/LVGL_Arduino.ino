/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <stdio.h>
#include <string.h>

#include "../include/pin_config.h"
#include "../include/CAN.h"
#include "../lib/ui/ui.h"

#include "../include/Display_ST7789.h"
#include "../include/LVGL_Driver.h"
// #include "../include/RTC_PCF85063.h"
// #include "../include/Gyro_QMI8658.h"
#include "../include/PWR_Key.h"
#include "../include/BAT_Driver.h"

void CANbusTask(void *parameter)
{
  while (1)
  {
    CANbus::hv_vs_frame(); // send this frame for funsies, only really needed once
    if (!digitalRead(sw2pin))
    {
      CANbus::enable_pack(static_cast<CANbus::CMD_STS_T>(!digitalRead(sw1pin))); // send the command frame
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // limit the loop to 10ms
  }
}

void ProcessTask(void *parameter)
{
  while (1)
  {
    // PCF85063_Loop(); // RTC
    // QMI8658_Loop(); // IMU

    CANbus::share_data_t data = CANbus::get_message_data();
    if (CANbus::get_rx_OK())
    {
      lv_label_set_text(ui_COMMS, "COMMS OK");
      char print_str[30];
      char print_str2[30];

      // line 1, Actual Performance
      data.power = data.volts * data.current;
      snprintf(print_str, 30, "%0.1fv %0.1fa %0.2fkw", data.volts, data.current, (data.power / 1000.0f));
      lv_label_set_text(ui_VOLT, print_str);

      // line 2, Allowed Power limits
      snprintf(print_str, 30, "%0.1fkw dsg %0.1fkw crg", data.power_dsg, data.power_crg);
      lv_label_set_text(ui_POWER, print_str);

      // line 3, TEMP average
      snprintf(print_str, 30, "%0.1fdeg c", (data.t_lowest + data.t_highest) / 2.0f);
      lv_label_set_text(ui_TEMP, print_str);

      // line 4, soc
      snprintf(print_str, 30, "%0.1f %", data.soc);
      lv_label_set_text(ui_SOC, print_str);

      // line 5, OP sts
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

      // line 6, BUS sts
      switch (data.bus_sts)
      {
      case CANbus::BUS_STATUS_T::BUS_OFF:
        lv_label_set_text(ui_TEXT2, "BUS OFF");
        break;
      case CANbus::BUS_STATUS_T::BUS_ON:
        lv_label_set_text(ui_TEXT2, "BUS ON");
        break;
      case CANbus::BUS_STATUS_T::PRECHARGE:
        lv_label_set_text(ui_TEXT2, "BUS PRECHARGE");
        break;
      case CANbus::BUS_STATUS_T::STOP_REQ:
        lv_label_set_text(ui_TEXT2, "BUS STOPPING");
        break;
      default:
        lv_label_set_text(ui_TEXT2, "BUS NA");
        break;
      }
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

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void CANbus_Loop()
{
  xTaskCreate(
      CANbusTask,
      "CANbusTask",
      4096,
      NULL,
      3,
      NULL);
}
void Process_Loop()
{
  xTaskCreate(
      ProcessTask,
      "ProcessTask",
      4096,
      NULL,
      4,
      NULL);
}
void setup()
{
  PWR_Init(); // POWER LATCH/UNLATCH
  BAT_Init(); // BAT VOLTAGE READOUT
  // I2C_Init();      // I2C for RTC and IMU comms
  // PCF85063_Init(); // RTC
  // QMI8658_Init();  // IMU

  CANbus::init();
  pinMode(sw1pin, INPUT_PULLUP);
  pinMode(sw2pin, INPUT_PULLUP);
  // pinMode(bootBTN, INPUT); // be carefull with this not to interfere with booting

  Backlight_Init(); // ...
  LCD_Init();
  Lvgl_Init();

  ui_init();
  CANbus_Loop();
  Process_Loop();

  Serial.begin(115200); // virtual COM via programmer
}

void loop()
{
  Lvgl_Loop(); // keep the timers working
  PWR_Loop();
  BAT_Get_Volts(); // read battery voltage
  vTaskDelay(pdMS_TO_TICKS(5));
}
