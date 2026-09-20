#define PICO_STDIO_USB_RX_BUFFER_SIZE 1024
#define PICO_STDIO_USB_TX_BUFFER_SIZE 1024

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include <cstdio>
#include <string>
#include <sstream>
#include <map>

using namespace std;

#define STATUS_PIN 25

#define UART_ID uart0
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BAUD_RATE 115200 * 2

const float PWM_FREQ_HZ = 50.0f;
const float MIN_PULSE_MS = 0.5f;
const float MAX_PULSE_MS = 2.4f;
const uint32_t WRAP = 65535;

struct Servo
{
    uint slice;
    uint channel;
};

map<int, Servo> servos;

uint16_t angle_to_duty(float angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    float duty_min = WRAP * (MIN_PULSE_MS / 20.0f);
    float duty_max = WRAP * (MAX_PULSE_MS / 20.0f);
    float duty = duty_min + (angle / 180.0f) * (duty_max - duty_min);

    if (duty < 0)
        duty = 0;
    if (duty > WRAP)
        duty = WRAP;

    return (uint16_t)duty;
}

void setup_servos()
{
    float div = (float)clock_get_hz(clk_sys) / (PWM_FREQ_HZ * (WRAP + 1));

    for (int pin = 0; pin < 16; ++pin)
    {
        gpio_set_function(pin, GPIO_FUNC_PWM);

        uint slice = pwm_gpio_to_slice_num(pin);
        uint channel = pwm_gpio_to_channel(pin);

        pwm_set_clkdiv(slice, div);
        pwm_set_wrap(slice, WRAP);
        pwm_set_chan_level(slice, channel, 0);
        pwm_set_enabled(slice, true);

        servos[pin] = {slice, channel};
    }
}

void process_command(const string &line)
{
    try
    {
        stringstream ss(line);
        string part;

        if (!getline(ss, part, ';'))
            return;
        int motor_id = stoi(part);

        if (!getline(ss, part, ';'))
            return;
        float angle = stof(part);

        if (motor_id >= 0 && motor_id <= 15 && servos.count(motor_id))
        {
            auto s = servos[motor_id];

            uint16_t duty = angle_to_duty(angle);

            // printf("motor=%d angle=%.2f duty=%u\n", motor_id, angle, duty);

            pwm_set_chan_level(s.slice, s.channel, duty);
        }
    }
    catch (...)
    {
        printf("Bad command: %s\n", line.c_str());
    }
}

void handle_char(int ch, string &buffer)
{
    if (ch < 0)
        return;

    char c = (char)ch;

    if (c == '\n' || c == '\r')
    {
        if (!buffer.empty())
        {
            gpio_put(STATUS_PIN, 1);
            process_command(buffer);
            buffer.clear();
            gpio_put(STATUS_PIN, 0);
        }
    }
    else
    {
        if (buffer.size() < 128)
        {
            buffer += c;
        }
        else
        {
            buffer.clear();
        }
    }
}

int main()
{
    stdio_usb_init();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    gpio_init(STATUS_PIN);
    gpio_set_dir(STATUS_PIN, GPIO_OUT);
    gpio_put(STATUS_PIN, 0);

    sleep_ms(2000);

    setup_servos();

    string usb_buffer;
    string uart_buffer;

    printf("Ready. Send: motor_id;angle，例如 0;90\n");

    while (true)
    {
        int usb_ch = getchar_timeout_us(0);
        if (usb_ch != PICO_ERROR_TIMEOUT)
        {
            handle_char(usb_ch, usb_buffer);
        }

        // UART0: TX = GPIO16, RX = GPIO17
        while (uart_is_readable(UART_ID))
        {
            int uart_ch = uart_getc(UART_ID);
            handle_char(uart_ch, uart_buffer);
        }

        sleep_ms(1);
    }
}