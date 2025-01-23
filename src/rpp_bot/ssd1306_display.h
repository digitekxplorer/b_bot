// SSD1306 Display
// October 6, 2024

#ifndef SSD1306_DISPLAY_H
#define SSD1306_DISPLAY_H
// ****************************
// SSD1306 Display
// ****************************
// Public Functions
// Send commands over I2C to intialize SSD1306
void SSD1306_initialize();
// Invert the display
void InvertDisplay(bool yes);
// Copy the entire framebuffer to the display.
//void UpdateDisplay();
void UpdateDisplay();
// Clear SSD1306 Display
void ClearDisplay();
// Basic Bresenhams.
void DrawLine(int x0, int y0, int x1, int y1, bool on);
// Write one Big Character at location x,y
void WriteBigChar(uint x, uint y, uint8_t ch);
// Write string starting at location x,y
void WriteString(int x, int y, uint8_t *str);
// Write Big tring starting at location x,y
void WriteBigString(int x, int y, uint8_t *str);
// Write one character at location x,y
void WriteChar(uint x, uint y, uint8_t ch);
// Short message displayed on SSD1306
void ssd1306_mess_short(bool isFreeRTOS_enabled);
// Long message displayed on SSD1306
void ssd1306_mess_long(void);
// Display current temperature and client on SSD1306
//void ssd1306_dsply(bool isFreeRTOS_enabled, int current_temp, char *client_message );
void ssd1306_dsply(bool isFreeRTOS_enabled, int current_temp, char *client_message, float batt_volt_flt );
// Draw border around the SSD1306 display
void ssd1306_drawborder(void);
// Print body of message to SSD1306 display
//void ssd1306_printbody(uint32_t current_temp, char *client_message);
void ssd1306_printbody(uint32_t current_temp, char *client_message, float batt_volt_flt);

// Private Functions
// Send a command over I2C to SSD1306
static void SendCommand(uint8_t cmd);
// Send the command buffer over I2C to SSD1306
static void SendCommandBuffer(uint8_t *inbuf, int len);
// Access one pixel at the x,y location
static void SetPixel(int x,int y, bool on);
//
static uint8_t reverse(uint8_t b) ;
//
static inline int GetFontIndex(uint8_t ch);
//
static void FillReversedCache();
//
static uint16_t ExpandByte(uint8_t b);

#endif   // SSD1306_DISPLAY_H
