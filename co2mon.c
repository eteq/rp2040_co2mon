#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "font8x8_basic.h"
#define FONT_WIDTH 8

#define LED_GPIO 13

#define SDA_PIN 2
#define SCL_PIN 3
// assuming here that SCL is consistent
#if ((SDA_PIN/2) % 2)
#define WHICH_I2C i2c1
#else
#define WHICH_I2C i2c0
#endif
#define I2C_KHZ 400

#define DISPLAY_ADDR 0x3c
#define WIDTH 128
#define HEIGHT 64

#define SCD41_ADDR 0x62

#define WAIT_TIME_MS 5000

#define CRC_ERROR -10


bool display_buffer[WIDTH][HEIGHT];

int write_display_buffer() {
    int thisret, ret = 0;

    uint8_t byte_buffer[HEIGHT+1];
    byte_buffer[0] = 0x40;  //control byte, all follow data
    for (int i=0;i<HEIGHT;i++) {
        byte_buffer[i+1] = 0;
    }

    uint8_t reset_pointer_cmds[3] = {0x0, 0x10, 0xb0};

    for (int i=0; i < (WIDTH/8); i++) {
        // reset the byte buffer
        for (int j=0; j<HEIGHT; j++) { byte_buffer[j+1] = 0; }

        reset_pointer_cmds[2] = 0xb0 + i;
        if (i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, reset_pointer_cmds, 3, false) == PICO_ERROR_GENERIC) {return PICO_ERROR_GENERIC;}

        for (int j=0; j < HEIGHT; j++) {
            for (int k=0; k < 8; k++) {
                byte_buffer[j+1] += display_buffer[i*8+k][j] << k;
            }
        }

        thisret = i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, byte_buffer, HEIGHT+1, false);
        if (ret == PICO_ERROR_GENERIC) {
            return ret;
        } else {
            ret += thisret;
        }
    }
    return ret;
}

void clear_buffer() {
    for (int i=0;i<WIDTH;i++) {
        for (int j=0;j<HEIGHT;j++) {
            display_buffer[i][j] = false;
        }
    }
}

int char_to_buffer(char chr, uint x, uint y) {
    char * bmp  = font8x8_basic[chr];
    for (int i=0; i < 8; i++) {
        for (int j=0; j < 8; j++) {
            display_buffer[i+x][j+y] = (bmp[(7-j)] >> i) & 1;
        }
    }
}

// FROM SCD41 DATASHEET - https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9.5_CO2/Sensirion_CO2_Sensors_SCD40_SCD41_Datasheet.pdf
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
uint8_t sensirion_common_generate_crc(const uint8_t *data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
//END FROM DATASHEET 

int scd41_read(uint16_t code, uint16_t * readinto, size_t nwords, uint32_t cmdtime_ms) {
    int ret;

    uint8_t wbuffer[2];
    wbuffer[1] = code & 0xff; // lsb
    wbuffer[0] = code >> 8; // msb

 
    ret = i2c_write_blocking(WHICH_I2C, SCD41_ADDR, wbuffer, 2, true);
    if (ret == PICO_ERROR_GENERIC) { return ret; }
    sleep_ms(cmdtime_ms);
    uint8_t * rbuffer =  malloc(nwords*3);
    ret = i2c_read_blocking(WHICH_I2C, SCD41_ADDR, rbuffer, nwords*3, false);
    if (ret == PICO_ERROR_GENERIC) { free(rbuffer); return ret; }

    for (int i=0; i < nwords; i++) {
        uint8_t bytes[2];
        int j = i*3;

        bytes[0] = rbuffer[j]; // msb
        bytes[1] = rbuffer[j+1]; //lsb
        if (rbuffer[j+2] != sensirion_common_generate_crc(bytes, 2)) { free(rbuffer); return CRC_ERROR; }

        readinto[i] = bytes[1] | (bytes[0] << 8);
    }

    free(rbuffer);
    return ret;
}


int scd41_sendcommand(uint16_t code, uint32_t cmdtime_ms) {
    int ret;
    uint8_t wbuffer[2];

    wbuffer[1] = code & 0xff; // lsb
    wbuffer[0] = code >> 8; // msb
 
    ret = i2c_write_blocking(WHICH_I2C, SCD41_ADDR, wbuffer, 2, true);
    if (cmdtime_ms > 0) {
        sleep_ms(cmdtime_ms);
    }
    return ret;
}


void setup_display() {
    const uint8_t display_on[2] = {0x0, 0xaf};
    const uint8_t display_init_bytes[20] = {0x0,  //control byte - many command follow
                        0xae, // display off
                        0xdc, 0, // start line 0 - default
                        0x81, 0x4f, //contrast
                        0x20, // vertical addressing - default?
                        0xa0,  // down rotation/segment remap=0
                        0xc0, // scan direction - default
                        0xa8, 0x3f, // multiplex=64
                        0xd3, 0x60, // display offset - 0x60 according to featherwing/adafruit sh1107 driver docs?
                        0xd9, 0x22, // pre-charge/dis-charge period mode: 2 DCLKs/2 DCLKs - default
                        0xdb, 0x35, // VCOM deselect level = 0.770 - default
                        0xa4, // normal/disp off - default
                        0xa6 // normal (not reversed) display - default
    };
    const size_t n_init_bytes = 20;

    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    i2c_init(WHICH_I2C, I2C_KHZ * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, display_init_bytes, n_init_bytes, false);
    clear_buffer();
    i2c_write_blocking(WHICH_I2C, DISPLAY_ADDR, display_on, 2, false);
}


int main() {
    bi_decl(bi_program_description("This is a co2 monitor binary."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));

    stdio_init_all();

    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_put(LED_GPIO, 1);

    printf("Getting display Ready\n");
    setup_display();

    //start measuring SCD41
    scd41_sendcommand(0x21b1, 0);
     

    // this indicates startup
    for (int i=0; i < 5; i++) {
        gpio_put(LED_GPIO, 1);
        sleep_ms(250);
        gpio_put(LED_GPIO, 0);
        sleep_ms(250);
    }

    while (true) {
        clear_buffer();

        uint16_t words[3];

        scd41_read(0xec05, words, 3, 1);

        uint16_t co2ppm = words[0];
        float tC = 175. * (float)words[1] / 65536. - 45.;
        float RH = 100. * (float)words[2] / 65536.;

        int nchar;
        char s[40];
        

        nchar = sprintf(s, "CO2: %i ppm", co2ppm);
        for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 41); }

        nchar = sprintf(s, "T: %.1f degC", tC);
        for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 31); }
        
        nchar = sprintf(s, "RH: %.1f %%", RH);
        for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 21); }

        write_display_buffer();

        sleep_ms(WAIT_TIME_MS);

    }

    return 0;
}