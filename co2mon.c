#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <hardware/flash.h>
#include <hardware/sync.h>

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

#define CRC_ERROR -10

#define SLEEP_TIME_MS 5
#define DEBOUNCE_TIME_MS 3
#define MEASUREMENT_PERIOD_MS 5000
#define RAM_DATA_MAX_SIZE 5000  //20 bytes per item, so mutiple these by .02 to get the ram usage in kB

#define FLASH_DATA_STORAGE_OFFSET 0x100000 //1MB


// global state
bool display_buffer[WIDTH][HEIGHT];
bool degc = true;
bool redraw = false;
bool remeasure = false;
int writing = 0;
struct repeating_timer redraw_timer;

int stored_data_idx = 0;

struct co2mon_data {
    uint32_t time;
    float co2ppm;
    float tempC;
    float rh;
    float tdpC;
};
struct co2mon_data stored_data[RAM_DATA_MAX_SIZE + FLASH_PAGE_SIZE/sizeof(struct co2mon_data)]; // extra length is to prevent possible segfault on flash writing

int write_hour = -1; //-1 means dump
int write_min = 0;
uint32_t write_timestamp = 0;


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

float dewpoint_from_t_rh( float tC, float RH) {
    float logrh = log(RH/100);
    float tnumer = 17.625*tC;
    float tdenom = 243.04 + tC;
    // August–Roche–Magnus formula assumption for Es
    return 243.04*(logrh+(tnumer/tdenom))/(17.625-logrh-(tnumer/tdenom));
}

int update_buffer_with_measurements(bool degc,  uint16_t co2ppm, float tC, float RH, float tdp) {
    int res;
    char tchar;
    float temp;

    int nchar;
    char s[40];

    if (degc) {
        tchar = 'C';
        temp = tC;
    } else {
        tchar = 'F';
        temp = tC*9./5. + 32;
        tdp = tdp*9./5. + 32;
    }

    nchar = sprintf(s, "CO2: %i ppm", co2ppm);
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 51); }

    nchar = sprintf(s, "T: %.1f deg%c", temp, tchar);
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 31); }
    
    nchar = sprintf(s, "RH: %.1f %%", RH);
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 21); }

    nchar = sprintf(s, "Tdp: %.1f deg%c", tdp, tchar);
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8+1, 11); }

    if (co2ppm < 1000) { nchar = sprintf(s, "Safe CO2"); }
    else if (co2ppm < 2000) { nchar = sprintf(s, "Iffy CO2?"); }
    else { nchar = sprintf(s, "Unsafe CO2!"); }
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8 + (128 - nchar*8)/2, 41); }

    return res;
}

void update_buffer_with_write_info() {
    int nchar;
    char s[40];

    nchar = sprintf(s, "Write?"); 
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8 + (128 - nchar*8)/2, 31); }
    
    if (write_hour < 0) {
        nchar = sprintf(s, "No, dump."); 
    } else {
        nchar = sprintf(s, "%i:%i", write_hour, write_min); 
    }
    for (int i=0; i<nchar; i++) { char_to_buffer(s[i], i*8 + (128 - nchar*8)/2, 21); }
}


uint32_t last_callback = 0;
void buttons_callback(uint gpio, uint32_t events) {
    uint32_t this_callback = to_ms_since_boot(get_absolute_time());
    if ((this_callback - last_callback) > DEBOUNCE_TIME_MS) {
        // only triggered on falling.
        // 7=C, 8=B, 9=A
        if (writing == 1) {
            switch (gpio) {
            case 7: // C=min
                write_min = (write_min + 1) % 61;
                break;
            case 8: // B=hour
                write_hour = (write_hour + 1) % 25;
                break;
            case 9: // A=proceed
                writing = 2;
                break;
            }
        } else if (gpio == 9) {
            writing = true;
        } else if (gpio == 7) {
            degc = !degc;
            redraw = true;
        }
    }
    last_callback = this_callback;
}

bool redraw_timer_callback(struct repeating_timer *t) {
    redraw = true;
    remeasure = true;
    return true;
}

void write_stored_data_to_flash(int ndata, int hour, int min, int timestamp) {
    uint8_t single_page_buffer[FLASH_PAGE_SIZE];
    int total_bytes = ndata * sizeof(stored_data[0]) + FLASH_PAGE_SIZE;

    uint32_t ints = save_and_disable_interrupts();

    flash_range_erase(FLASH_DATA_STORAGE_OFFSET, (total_bytes/FLASH_SECTOR_SIZE + 1)*FLASH_SECTOR_SIZE);
    
    //write the metadata to the first page
    for (int i=0; i<FLASH_PAGE_SIZE; i++) { single_page_buffer[i] = 0; }
    ((int*)single_page_buffer)[0] = ndata;
    ((int*)single_page_buffer)[1] = hour;
    ((int*)single_page_buffer)[2] = min;
    ((int*)single_page_buffer)[3] = timestamp;
    flash_range_program(FLASH_DATA_STORAGE_OFFSET, single_page_buffer, FLASH_PAGE_SIZE);

    flash_range_program(FLASH_DATA_STORAGE_OFFSET, (uint8_t*) stored_data, (ndata * sizeof(stored_data[0]) / FLASH_PAGE_SIZE + 1)*FLASH_PAGE_SIZE);

    restore_interrupts (ints);
}

int restore_stored_data_from_flash() {
    // load the stored data from the flash into memory for dumping.  Also restore write_min/write_hour and return ndata
    uint8_t* flash_data_ptr = (uint8_t* )XIP_BASE;
    flash_data_ptr += FLASH_DATA_STORAGE_OFFSET;

    // first extract the metadata from the first page
    int ndata = ((int*)flash_data_ptr)[0];
    write_hour = ((int*)flash_data_ptr)[1];
    write_min = ((int*)flash_data_ptr)[2];
    write_timestamp = ((int*)flash_data_ptr)[3];
    // skip forward to the first data byte
    flash_data_ptr += FLASH_PAGE_SIZE;

    for (int i=0; i < ndata; i++) {
        stored_data[i] = ((struct co2mon_data*)flash_data_ptr)[i];
    }

    return ndata;
}

int main() {
    bi_decl(bi_program_description("This is a co2 monitor binary."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));

    
    uint16_t co2ppm;
    float tempC, rh, tdpC;

    stdio_init_all();

    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_put(LED_GPIO, 1);

    // featherwing buttons
    for (int pinnum=7; pinnum<10; pinnum++) {
        gpio_init(pinnum);
        gpio_set_dir(pinnum, GPIO_IN);
        gpio_pull_up(pinnum);
        // FALL is when the button is pushed
        gpio_set_irq_enabled_with_callback(pinnum, GPIO_IRQ_EDGE_FALL, true, &buttons_callback);
    }

    printf("Getting display Ready\n");
    setup_display();
    clear_buffer();
    write_display_buffer();

    // stop measuring SCD41 if it is already
    scd41_sendcommand(0x3f86, 500);
    //re-init just in case it isn't
    scd41_sendcommand(0x3646, 1020);

    //start measuring SCD41
    scd41_sendcommand(0x21b1, 0);
    add_repeating_timer_ms(MEASUREMENT_PERIOD_MS, redraw_timer_callback, NULL, &redraw_timer);

    // this indicates startup
    for (int i=0; i < 5; i++) {
        gpio_put(LED_GPIO, 1);
        sleep_ms(250);
        gpio_put(LED_GPIO, 0);
        sleep_ms(250);
    }

    while (true) {
        switch (writing) {
        case 1:
            clear_buffer();
            update_buffer_with_write_info();
            write_display_buffer();

            break;
        case 2:
            if (write_hour < 0){
                int dump_data_size = restore_stored_data_from_flash();
                printf("dumping last dataset from flash\n");
                for (int i=0; i<dump_data_size; i++) {
                    // TODO: replace with dumping what was written
                    printf("tstampms,CO2ppm,TC,Rh%%,TdpC:%i,%i,%.1f,%.1f,%.1f\n", 
                           stored_data[i].time, stored_data[i].co2ppm, stored_data[i].tempC, stored_data[i].rh, stored_data[i].tdpC);
                }
                printf("timestamp %i at time: %i:%i\n", write_timestamp, write_hour, write_min);
            } else {
                write_timestamp = to_ms_since_boot(get_absolute_time());
                write_stored_data_to_flash(stored_data_idx, write_hour, write_min, write_timestamp);
            }

            writing = 0;
            write_hour = -1;
            write_min = 0;
            break;
        case 0:
            if (remeasure) {
                int res;
                uint16_t words[3];

                res = scd41_read(0xec05, words, 3, 1);
                if (res == PICO_ERROR_GENERIC) {
                    printf("measurement error!");

                    gpio_put(LED_GPIO, 1);
                    sleep_ms(250);
                    gpio_put(LED_GPIO, 0);
                    sleep_ms(250);
                } 

                co2ppm = words[0];
                tempC = 175. * (float)words[1] / 65536. - 45.;
                rh = 100. * (float)words[2] / 65536.;
                tdpC = dewpoint_from_t_rh(tempC, rh);

                uint32_t timestamp = to_ms_since_boot(get_absolute_time());

                if (stored_data_idx >= RAM_DATA_MAX_SIZE) {
                    printf("no space left to store data, outputting\n");
                    printf("tstampms,CO2ppm,TC,Rh%%,TdpC:%i,%i,%.1f,%.1f,%.1f\n", timestamp, co2ppm, tempC, rh, tdpC);
                } else {
                    stored_data[stored_data_idx].time = timestamp;
                    stored_data[stored_data_idx].co2ppm = co2ppm;
                    stored_data[stored_data_idx].tempC = tempC;
                    stored_data[stored_data_idx].rh = rh;
                    stored_data[stored_data_idx].tdpC = tdpC;
                    stored_data_idx++;
                }

                remeasure = false;
            }

            if (redraw) {
                clear_buffer();
                update_buffer_with_measurements(degc, co2ppm, tempC, rh, tdpC);
                write_display_buffer();
                redraw = false;
            }
            break;
        }

        sleep_ms(SLEEP_TIME_MS);
    }

    return 0;
}