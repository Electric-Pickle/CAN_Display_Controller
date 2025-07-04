#include <Arduino.h>
#include "../include/CAN.h"

TaskHandle_t RXHandle;
TaskHandle_t RXTimeout;

namespace CANbus
{
    can_frame_t _rx_HVESSD1;
    can_frame_t _rx_HVESSD2;
    can_frame_t _rx_HVESSD3;
    can_frame_t _rx_HVESSD6;
    can_frame_t _rx_HVESS1;
    can_frame_t _tx1_HV_VS_P;
    can_frame_t _tx2_HESSC1;

    uint8_t frame_active = false;
    uint32_t lastRxTime = false;

    // Can Message Receiced Handler, (copy the data, and set a new data flag, want to keep this as short as possible)
    void read_alerts()
    {
        twai_message_t rx_buf;
        if (twai_receive(&rx_buf, pdMS_TO_TICKS(20)) == ESP_OK)
        {
            switch (rx_buf.identifier)
            {
            case HVESSD1: //
                for (int i = 0; i != 8; i++)
                {
                    _rx_HVESSD1.data[i] = rx_buf.data[i];
                }
                _rx_HVESSD1.newdata = true;
                break;
            case HVESSD2: //
                for (int i = 0; i != 8; i++)
                {
                    _rx_HVESSD2.data[i] = rx_buf.data[i];
                }
                _rx_HVESSD2.newdata = true;
                break;
            case HVESSD3: //
                for (int i = 0; i != 8; i++)
                {
                    _rx_HVESSD3.data[i] = rx_buf.data[i];
                }
                _rx_HVESSD3.newdata = true;
                break;

            case HVESSD6: //
                for (int i = 0; i != 8; i++)
                {
                    _rx_HVESSD6.data[i] = rx_buf.data[i];
                }
                _rx_HVESSD6.newdata = true;
                break;

            case HVESS1: //
                for (int i = 0; i != 8; i++)
                {
                    _rx_HVESS1.data[i] = rx_buf.data[i];
                }
                lastRxTime = millis();
                frame_active = true;
                _rx_HVESS1.newdata = true;
                break;

            default:
                // ID's not in list, Do nothing
                break;
            }
        }
    }

    void sendMessage(uint32_t id, uint8_t data[8])
    {
        twai_message_t message;
        message.identifier = id;
        message.extd = 1;
        message.rtr = 0;
        message.data_length_code = 8;
        for (int n = 0; n < 8; n++)
        {
            message.data[n] = data[n];
        }
        esp_err_t error_frames;
        twai_status_info_t status;
        error_frames = twai_transmit(&message, pdMS_TO_TICKS(10));
        // Serial.print(error_frames);
        // esp_err_t ret = twai_get_status_info(&status);
        // if (ret == ESP_OK)
        // {
        //     printf("Bus state: %d\n", status.state);
        //     printf("TX error count: %d\n", status.tx_error_counter);
        //     printf("RX error count: %d\n", status.rx_error_counter);
        //     printf("waiting for tx: %d\n", status.msgs_to_tx);
        //     printf("waiting for rx: %d\n", status.msgs_to_rx);
        // }
        // else
        // {
        //     printf("Failed to get TWAI status: %s\n", esp_err_to_name(ret));
        // }
        // if (error_frames == ESP_OK)
        // {
        //     printf("Message queued for transmission\n");
        // }
        // else if (error_frames == ESP_ERR_TIMEOUT)
        // {
        //     printf("Failed to queue message for transmission timeout\n");
        // }
        // else
        // {
        //     printf("Failed to queue message for transmission\n");
        // }
    }

    void init()
    {

        // Initialize configuration structures using macro initializers
        // CAN_TX and RX come from pin_config.h
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        f_config.acceptance_code =

            twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK;
        twai_start() == ESP_OK;

        //  create RX handler task
        xTaskCreate(
            rx_handler_task, /* Function to implement the task */
            "RXHandle",      /* Name of the task */
            4096,           /* Stack size in words */
            NULL,            /* Task input parameter */
            5,              /* Priority of the task */
            &RXHandle);      /* Task handle. */

        //  create RX timeout task
        xTaskCreate(
            rx_timeout_task, /* Function to implement the task */
            "RXTimeout",     /* Name of the task */
            4096,           /* Stack size in words */
            NULL,            /* Task input parameter */
            6,              /* Priority of the task */
            &RXTimeout);     /* Task handle. */

        // LED1_OFF;
        // LED2_OFF;
    }

    void rx_handler_task(void *pvParameters)
    {
        while (1)
        {
            delayMicroseconds(50);
            CANbus::read_alerts();
        }
    }
    void rx_timeout_task(void *pvParameters)
    {
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            if (millis() - lastRxTime > RX_TIMEOUT_MS)
            {
                frame_active = false;
            }
        }
    }

    OP_STATUS_T decode_op_status()
    {
        return static_cast<OP_STATUS_T>(_rx_HVESS1.data[5] / 16);
    }
    BUS_STATUS_T decode_bus_connect_status()
    {
        return static_cast<BUS_STATUS_T>(_rx_HVESS1.data[5] & 0x0f); // mask off the upper 4 bits
    }
    float decode_power_dsg()
    {
        uint16_t lsb16 = _rx_HVESSD1.data[0];
        uint16_t msb16 = _rx_HVESSD1.data[1] << 8;
        uint16_t power_int = lsb16 | msb16;
        return static_cast<float>(power_int) / 20.0f;
    }
    float decode_power_crg()
    {
        uint16_t lsb16 = _rx_HVESSD1.data[2];
        uint16_t msb16 = _rx_HVESSD1.data[3] << 8;
        uint16_t power_int = lsb16 | msb16;
        return static_cast<float>(power_int) / 20.0f;
    }

    float decode_soc()
    {
        uint16_t lsb16 = _rx_HVESSD2.data[0];
        uint16_t msb16 = _rx_HVESSD2.data[1] << 8;
        return static_cast<float>(lsb16 | msb16) / 640.0f;
    }
    float decode_volts()
    {
        uint16_t lsb16 = _rx_HVESSD1.data[4];
        uint16_t msb16 = _rx_HVESSD1.data[5] << 8;
        uint16_t volt_int = lsb16 | msb16;
        return static_cast<float>(volt_int) / 20.0f;
    }
    float decode_current()
    {
        uint16_t lsb16 = _rx_HVESSD1.data[6];
        uint16_t msb16 = _rx_HVESSD1.data[7] << 8;
        uint16_t current_int = (lsb16 | msb16);
        return (static_cast<float>(current_int) / 20.0f) - 1600.0f;
    }
    float decode_t_highest()
    {
        float t_highest;
        uint16_t lsb16 = _rx_HVESSD3.data[0];
        uint16_t msb16 = _rx_HVESSD3.data[1] << 8;
        uint16_t t_highest_int = (lsb16 | msb16);
        return t_highest = (static_cast<float>(t_highest_int) / 32.0f) - 273.0f;
    }
    float decode_t_lowest()
    {
        float t_lowest;
        uint16_t lsb16 = _rx_HVESSD3.data[2];
        uint16_t msb16 = _rx_HVESSD3.data[3] << 8;
        uint16_t t_lowest_int = (lsb16 | msb16);
        return t_lowest = (static_cast<float>(t_lowest_int) / 32.0f) - 273.0f;
    }

    float calc_power(float amps, float volts)
    {
        return volts * amps / 1000.0f; // scales W to KW
    }

    uint8_t get_rx_OK()
    {
        // Serial.print("\e[7;0"); // move to row x
        // Serial.print("\e[0G"); // move to column x

        // Serial.print("frame");
        // Serial.print(frame_active);
        // Serial.print("\n");
        return frame_active;
    }

    void hv_vs_frame()
    {
        uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        sendMessage(HV_VS_P, data);
    }

    void enable_pack(CMD_STS_T cmd_enable)
    {
        uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        if (cmd_enable)
        {
            data[0] = 0x01;
        }
        else
        {
            data[0] = 0x00;
        }
        sendMessage(HESSC1, data);
    }

    // share_data_t get_message_data()
    share_data_t get_message_data()
    {

        static share_data_t _data;

        // Process New data
        if (_rx_HVESSD1.newdata)
        {
            _rx_HVESSD1.newdata = false;
            _data.power_dsg = decode_power_dsg();
            _data.power_crg = decode_power_crg();
            _data.volts = decode_volts();
            _data.current = decode_current();
            _data.power = calc_power(_data.current, _data.volts);
        }
        if (_rx_HVESSD2.newdata)
        {
            _rx_HVESSD2.newdata = false;
            _data.soc = decode_soc();
        }
        if (_rx_HVESSD3.newdata)
        {
            _rx_HVESSD3.newdata = false;
            _data.t_highest = decode_t_highest();
            _data.t_lowest = decode_t_lowest();
        }
        if (_rx_HVESSD6.newdata)
        {
            _rx_HVESSD6.newdata = false;
            // _data.t_in = decode_t_in();
            // _data.t_out = decode_t_out();
        }
        if (_rx_HVESS1.newdata)
        {
            _rx_HVESS1.newdata = false;
            _data.op_sts = decode_op_status();

            _data.bus_sts = decode_bus_connect_status();
        }
        if (!frame_active)
        {
            Serial.print("\e[H");  // move to home
            Serial.print("\e[2K"); // erase line

            Serial.print("NO COMMS");
            static bool flipflop = false;
            static uint8_t flipflop_div = 0;
            flipflop_div++;
            if (flipflop_div > 50)
            {
                flipflop_div = 0;
                flipflop ^= 1;
            }
            if (flipflop)
            {
                Serial.print(" -");
            }
            else
            {
                Serial.print("  ");
            }
            Serial.print("\e[1B"); // move down x lines
            Serial.print("\e[2K"); // erase line
            Serial.print("\e[1B"); // move down x lines
            Serial.print("\e[2K"); // erase line
            Serial.print("\e[1B"); // move down x lines
            Serial.print("\e[2K"); // erase line
            Serial.print("\e[1B"); // move down x lines
            Serial.print("\e[2K"); // erase line
            Serial.print("\e[1B"); // move down x lines
            Serial.print("\e[2K"); // erase line
            Serial.print("\e[1B"); // move down x lines
            Serial.print("\e[2K"); // erase line
        }
        else
        {
            Serial.print("\e[H");  // move to home
            Serial.print("\e[2K"); // erase line
            Serial.print(_data.current, 0);
            Serial.print("a");

            Serial.print("\e[7G"); // move to column x
            Serial.print(_data.volts, 0);
            Serial.print("v");

            Serial.print("\e[13G"); // move to column x
            Serial.print(_data.power, 1);
            Serial.print("kW act\n");
            Serial.print("\e[0G"); // move to column x

            Serial.print("\e[2K"); // erase line
            Serial.print(_data.power_dsg, 0);
            Serial.print("kW dsg");

            Serial.print("\e[12G"); // move to column x
            Serial.print(_data.power_crg, 0);
            Serial.print("kW crg\n");
            Serial.print("\e[0G"); // move to column x

            Serial.print("\e[2K");                                        // erase line
            Serial.print(((_data.t_highest + _data.t_lowest) / 2.0f), 1); // Take the average of the highest and lowest cell for a single temp value
            Serial.print("c avg");
            Serial.print("\e[0G"); // move to column x

            Serial.print("\e[1B"); // move 1 line down
            Serial.print("\e[2K"); // erase line
            Serial.print(_data.soc, 1);
            Serial.print("% soc");
            Serial.print("\e[0G"); // move to column x

            Serial.print("\e[1B"); // move 1 line down
            Serial.print("\e[2K"); // erase line

            Serial.print("Op sts  ");
            switch (_data.op_sts)
            {
            case STATUS_OK:
                Serial.print("OK\n");
                break;
            case STATUS_FAULTED:
                Serial.print("FAULTED\n");
                break;
            default:
                Serial.print("N/A\n");
                break;
            }
            Serial.print("\e[0G"); // move to column x

            // Serial.print("\e[1B"); // move 1 line down
            Serial.print("\e[2K"); // erase line

            Serial.print("Bus sts ");
            switch (_data.bus_sts)
            {
            case BUS_OFF:
                Serial.print("OFF");
                break;
            case BUS_ON:
                Serial.print("ON");
                break;
            case PRECHARGE:
                Serial.print("PRECHARGE");
                break;
            case STOP_REQ:
                Serial.print("STOP_REQ");
                break;
            default:
                Serial.print("N/A");
                break;
            }
            Serial.print("\e[1B"); // move 1 line down
            Serial.print("\e[0G"); // move to column x
        }

        return _data;
    }

}