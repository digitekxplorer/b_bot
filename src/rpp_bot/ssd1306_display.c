// SSD1306 Display Fuctions
// October 6, 2024

// Includes functions for the SSD1306 display plus support for
// FreeRTOS.  The project combo, under FreeRTOS folder, was used
// to test incorporation of SSD1306 functions into FreeRTOS.

// NOTES:
// In the original design, all SSD1306 funcitons were decleared as 
// "static".
// When static is used in the context of a function, like 
// "static void ClearDisplay()", static means that the function has 
// internal linkage.  In other words, static specifies that the function
// is only visible within the current translation unit (i.e., the 
// current source file) and cannot be called from outside the file.
// The function cannot be called from another file, even if the 
// function is declared in a header file.
// In essence, static makes the function local to the file, and 
// it's not visible to the outside world.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

//
#include "ssd1306_display.h"
#include "../defs.h"


/* Example code to talk to a SSD1306 OLED display, 128 x 64 pixels
   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.
   Connections on Raspberry Pi Pico board, other boards may vary.
*/

// ****************************
// SSD1306 Display
// ****************************
// We need a 0x40 in the byte before our framebuffer
uint8_t _Framebuffer[SSD1306_FRAMEBUFFER_SIZE + 1] = {0x40};
uint8_t *Framebuffer = _Framebuffer+1;

static uint8_t reversed[sizeof(font)] = {0};

// ***************************************************
// Public Functions
// ***************************************************
// Non-static functions that can be called from main.c

void SSD1306_initialize() {
uint8_t init_cmds[]=
    {
    0x00,                // send one command
//    0x80,    // send multiple commands; didn't work????
    SSD1306_DISPLAYOFF,
    SSD1306_SETMULTIPLEX, 0x3f,
    SSD1306_SETDISPLAYOFFSET, 0x00,
    SSD1306_SETSTARTLINE,
    SSD1306_SEGREMAP127,
    SSD1306_COMSCANDEC,
    SSD1306_SETCOMPINS, 0x12,
    SSD1306_SETCONTRAST, 0xff,
    SSD1306_DISPLAYALLON_RESUME,
    SSD1306_NORMALDISPLAY,
    SSD1306_SETDISPLAYCLOCKDIV, 0x80,
    SSD1306_CHARGEPUMP, 0x14,
    SSD1306_DISPLAYON,
    SSD1306_MEMORYMODE, 0x00,   // 0 = horizontal, 1 = vertical, 2 = page
    SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH-1,  // Set the screen wrapping points
    SSD1306_PAGEADDR, 0, 7};

    SendCommandBuffer(init_cmds, sizeof(init_cmds));
}

// Inverted display
void InvertDisplay(bool yes) {
    if (yes)
        SendCommand(SSD1306_INVERTDISPLAY);
    else
        SendCommand(SSD1306_NORMALDISPLAY);
}

// This copies the entire framebuffer to the display.
void UpdateDisplay() {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, _Framebuffer, sizeof(_Framebuffer), false);
}

// Clear display using memset() and UpdateDisplay()
void ClearDisplay() {
    memset(Framebuffer, 0, SSD1306_FRAMEBUFFER_SIZE);
    UpdateDisplay();
}

// Basic Bresenhams.
void DrawLine(int x0, int y0, int x1, int y1, bool on) {
    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true) {
        SetPixel(x0, y0, on);

        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2*err;

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

// Used for big Font
void WriteBigChar(uint x, uint y, uint8_t ch) {
    if (reversed[0] == 0)
        FillReversedCache();

    if (x > SSD1306_LCDWIDTH - 16 || y > SSD1306_LCDHEIGHT - 16)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        uint16_t w = ExpandByte(reversed[idx * 8 + i]);
        Framebuffer[fb_idx] = w & 0x0ff;
        Framebuffer[fb_idx+1] = w & 0x0ff;
        Framebuffer[fb_idx+128] = w >> 8;
        Framebuffer[fb_idx+129] = w >> 8;
        fb_idx+=2;

    }
}

// Write string at location provided
void WriteString(int x, int y, uint8_t *str) {
    // Cull out any string off the screen
    if (x > SSD1306_LCDWIDTH - 8 || y > SSD1306_LCDHEIGHT - 8)
        return;

    while (*str) {
        WriteChar(x,y, *str++);
        x+=8;
    }
}

// Write string at location provided using big Font
void WriteBigString(int x, int y, uint8_t *str) {
    // Cull out any string off the screen
    if (x > SSD1306_LCDWIDTH - 16 || y > SSD1306_LCDHEIGHT - 16)
        return;

    while (*str) {
        WriteBigChar(x,y, *str++);
        x+=16;
    }
}

// Write a single character to location provided
void WriteChar(uint x, uint y, uint8_t ch) {
    if (reversed[0] == 0)
        FillReversedCache();

    if (x > SSD1306_LCDWIDTH - 8 || y > SSD1306_LCDHEIGHT - 8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        Framebuffer[fb_idx++] = reversed[idx * 8 + i];
    }
}


// Short message displayed on SSD1306
// Use FreeRTOS delay function if this function is called after 
// scheduler has been enabled.
void ssd1306_mess_short(bool isFreeRTOS_enabled) {
    //
    DrawLine(0,0,SSD1306_LCDWIDTH-1, 0, true);
    DrawLine(0,0,0,SSD1306_LCDHEIGHT-1, true);
    DrawLine(SSD1306_LCDWIDTH-1, 0,SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, true);
    DrawLine(0,SSD1306_LCDHEIGHT-1,SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, true);
    
    if (isFreeRTOS_enabled) {
        WriteString(26,8,"FreeRTOS");  // send FreeRTOS message
        WriteString(16,22,"is Enabled!");
    } else {
        WriteString(13,8,"HELLO SSD1306");
    }

    for (int ch = '9'; ch>='0';ch--) {
        WriteBigChar(56, 36, ch);
        UpdateDisplay();
        if (isFreeRTOS_enabled) {
            vTaskDelay(TaskSSD1306_DLY);  // FreeRTOS delay
        } else {
            sleep_ms(500);                // Pico SDK delay
        }
    }
    
    // short delay at end of count-down
    if (isFreeRTOS_enabled) {
        vTaskDelay(TaskSSD1306_DLY);  // FreeRTOS delay
    } else {
        sleep_ms(500);                // Pico SDK delay
    }
}

// Long message displayed on SSD1306
// This function is called before FreeRTOS scheduler is enabled so it is okay to
// use sleep_ms( ) function.
void ssd1306_mess_long(void) {
    // Call short message then continue with more
    ssd1306_mess_short(false);   // use Pico SDK delay function
    // Continue with longer message
    WriteBigString(0,35,"BLASTOFF");
    UpdateDisplay();

    for (int i = 0;i<10;i++) {
        InvertDisplay(true);
        sleep_ms(100);
        InvertDisplay(false);
        sleep_ms(100);
    }

    for (int i=0;i<SSD1306_LCDWIDTH;i++) {
        if (i >= SSD1306_LCDWIDTH/2) {
            DrawLine(i,0,i, SSD1306_LCDHEIGHT-1, false);
            DrawLine(SSD1306_LCDWIDTH - i - 1,0,SSD1306_LCDWIDTH - i - 1, SSD1306_LCDHEIGHT-1, false);
        }
        else {
            DrawLine(i,0,i, SSD1306_LCDHEIGHT-1, true);
            DrawLine(SSD1306_LCDWIDTH - i - 1,0,SSD1306_LCDWIDTH - i - 1, SSD1306_LCDHEIGHT-1,  true);
        }
        UpdateDisplay();
    }
}


// Display current temperature and client on SSD1306
// Use FreeRTOS delay function if this function is called after 
// scheduler has been enabled.
//void ssd1306_dsply(bool isFreeRTOS_enabled, int current_temp, char *client_message ) {
void ssd1306_dsply(bool isFreeRTOS_enabled, int current_temp, char *client_message, float batt_volt_flt ) {
	ClearDisplay();               // clear SSD1306 display
	ssd1306_drawborder();

    if (isFreeRTOS_enabled) {
        // print body of message
        ssd1306_printbody(current_temp, client_message, batt_volt_flt);
    } else {
        WriteString(13,8,"HELLO Al B");
        UpdateDisplay();
    }
}

// Draw border around the SSD1306 display
void ssd1306_drawborder(void) {
    DrawLine(0,0,SSD1306_LCDWIDTH-1, 0, true);
    DrawLine(0,0,0,SSD1306_LCDHEIGHT-1, true);
    DrawLine(SSD1306_LCDWIDTH-1, 0,SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, true);
    DrawLine(0,SSD1306_LCDHEIGHT-1,SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, true);
}

// Print body of message to SSD1306 display
void ssd1306_printbody(uint32_t current_temp, char *client_message, float batt_volt_flt) {
    static char temp_str[10];
    static char volt_str[10];
    
    // If we have low battery voltage, display warning.
    // During programming, the Pico will not be connected to a battery so the voltage 
    // will be zero so also check for a non-zero value.
    if (batt_volt_flt > 1.0 & batt_volt_flt < LOW_BATT_WARNING_VAL) {   // warning value defined in defs.h
//    if (batt_volt_flt < LOW_BATT_WARNING_VAL) {       // use to test LOW_BATT_WARNING_VAL only
       WriteString(10,16,"LOW BATTERY");    // display low battery warning
    }
    else {
       // Wrtie to every 8 rows (8 pixels per character)
       // WriteString(Column, Row, Text)
       
	   // display message from client (phone)
	   WriteString(24,8, client_message);
	   // display temperature
       WriteString(10,24,"Pico Temp:");  // send FreeRTOS message
	   sprintf(temp_str, "%d", current_temp) ;   // convert uint32 to string
	   WriteString(90,24, temp_str);
	   // dispaly Battery Voltage
	   WriteString(10,40,"Batt Volt:");
	   snprintf(volt_str, sizeof(volt_str), "%.1f", batt_volt_flt);    // display one digit to right of decimal
	   WriteString(90,40, volt_str);
	}
	
	UpdateDisplay();
}

// ***************************************************
// Private Functions  (static functions)
// ***************************************************
static void SendCommand(uint8_t cmd) {
    uint8_t buf[] = {0x00, cmd};
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, buf, 2, false);
}

static void SendCommandBuffer(uint8_t *inbuf, int len) {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, inbuf, len, false);
}

static void SetPixel(int x,int y, bool on) {
    assert(x >= 0 && x < SSD1306_LCDWIDTH && y >=0 && y < SSD1306_LCDHEIGHT);

    // The calculation to determine the correct bit to set depends on which address
    // mode we are in. This code assumes horizontal

    // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
    // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
    // byte 1 is x = 1, y=0->7 etc

    // This code could be optimised, but is like this for clarity. The compiler
    // should do a half decent job optimising it anyway.

    const int BytesPerRow = 128; // 128 pixels, 1bpp, but each row is 8 pixel high, so (128 / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow   +   x;
    uint8_t byte = Framebuffer[byte_idx];

    if (on)
        byte |=  1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    Framebuffer[byte_idx] = byte;
}

static uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

// Index to font
// The returned value is the row number in ssd1306_font.h
static inline int GetFontIndex(uint8_t ch) {
    // upper case letters
    if (ch >= 'A' && ch <='Z')
        return  ch - 'A' + 1;        // 65 thru 90
    // numbers
    else if (ch >= '0' && ch <='9')  // 48 thru 57
        return  ch - '0' + 27;
    // symbols after numbers added by ab
    // math and more symbols :;<=>?@
    else if (ch >= ':' && ch <='@')  // 58 thru 64
        return  ch - ':' + 37;
    // more symbols 
    else if (ch >= '!' && ch <='/') // 33 thru 47
        return  ch - '!' + 44; 
    // symbol group that includes underscore
    else if (ch >= '[' && ch <='_') // 91 thru 95
        return  ch - '[' + 59;     
    else
        return  0; // Not got that char so space.
}

static void FillReversedCache() {
    // calculate and cache a reversed version of fhe font, because I defined it upside down...doh!
    for (int i=0;i<sizeof(font);i++)
        reversed[i] = reverse(font[i]);
}

static uint16_t ExpandByte(uint8_t b) {
    uint16_t w = 0;
    for (int i=7;i>=0;i--) {
        uint16_t t = (b & (1 << i));
        w |= (t << i);
        w |= (t << (i + 1));
    }
    return w;
}
