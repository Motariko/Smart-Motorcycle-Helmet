// config_template.h - Template for user configuration
// Instructions: Copy this file, rename it to 'config.h' and fill in your data.

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// [FOR HELMET] The MAC Address of your receiver (Tail Unit)
// Example: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
const uint8_t RECEIVER_MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 

// [FOR TAIL UNIT] Your Server Endpoint URL
const char* SERVER_URL = "http://YOUR_SERVER_IP:PORT/data";

#endif