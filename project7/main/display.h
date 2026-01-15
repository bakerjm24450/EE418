/*
 * Functions for interfacing with LCD display. Also runs the GUI
 */

#pragma once

#include "esp_err.h"

// initialize display, interface, and GUI
esp_err_t displayInit();