#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include "driver/twai.h"
#include "../include/pin_config.h"

// 16b 0  POW_DSG
// 16b 16 POW_CRG
// 16b 32 VOLT_LVL
// 16b 48 CURR
#define HVESSD1 0x0CF0905B
// 16b 0  SOC
#define HVESSD2 0x0CF0915B
// 16b 0  highest cell temp
// 16b 16 lowest cell temp
#define HVESSD3 0x0CF0925B
// 16b 0  bus voltage
// 8b  32 cool_in
// 8b  40 cool_out
#define HVESSD6 0x0CF0955B
// 2b 20 HVIL_STS
// 4b 40BUS_CON_STS
// 4b 44 OP_STS
#define HVESS1 0x0CF0965B
// 16b 0 Branch_VOLT //just needs to exist
#define HV_VS_P 0x1CFF0D05
// 2b 0 CONN_CMD
#define HESSC1 0x0C1B5B05

#define RX_TIMEOUT_MS 2000

#define LED1_ON digitalWrite(led1pin, LOW)
#define LED1_OFF digitalWrite(led1pin, HIGH)
#define LED2_ON digitalWrite(led2pin, LOW)
#define LED2_OFF digitalWrite(led2pin, HIGH)

namespace CANbus
{
    void init();
    // void can0_handle(CAN_FRAME *frame);
    void read_alerts();
    void rx_handler_task(void *pvParameters);
    void rx_timeout_task(void *pvParameters);

    void sendMessage(uint32_t id, uint8_t data[8]);


    struct can_frame_t
    {
        uint8_t data[8]; // data
        bool newdata;    // New data flag, (ISR sets true when a new frame is copied)
    };

    struct share_data_t
    {
        float power_dsg;
        float power_crg;
        float volts;
        float current;
        float power;
        float soc;
        uint8_t op_sts;
        uint8_t bus_sts;
        float t_highest;
        float t_lowest;
    };

    share_data_t get_message_data();

    enum OP_STATUS_T
    {
        NA0,
        NA1,
        STATUS_OK,
        NA3,
        NA4,
        NA5,
        STATUS_FAULTED,
    };
    enum BUS_STATUS_T
    {
        BUS_OFF,
        BUS_ON,
        PRECHARGE,
        STOP_REQ,
    };
    enum CMD_STS_T
    {
        DISABLE,
        ENABLE,
    };

    OP_STATUS_T decode_op_status();
    BUS_STATUS_T decode_bus_connect_status();
    float decode_power_dsg();
    float decode_power_crg();
    float decode_soc();
    float decode_volts();
    float decode_current();
    float decode_t_highest();
    float decode_t_lowest();
    float calc_power(float amps, float volts);

    // returns wheather a can message has been recived less then 500ms ago
    uint8_t get_rx_OK();

    // required at least ones for the backs to work properly
    void hv_vs_frame();
    // send the correct command frame to keep the pack happy, and control enabling and disabling the pack
    void enable_pack(CMD_STS_T enable);

};
#endif // CAN_H