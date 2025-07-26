Project Introduction
ssd1306_1315_oled_espidf: Lightweight U8g2 OLED Driver for ESP-IDF 

This ESP-IDF-compatible U8g2  C library provides robust support for SSD1306 and SSD1315-based OLED displays on Espressif ESP32/ESP32-S series microcontrollers . Designed for simplicity and efficiency, it enables seamless integration of monochrome OLED screens (128x64 resolutions) into embedded projects using I²C interfaces.

Key Features:

🎯 Dual Controller Support: Unified API for both SSD1306 and SSD1315 chipsets 

⚡ Optimized Performance: Leverages ESP-IDF's HAL for efficient I²C communication (Esp-Idf ver 5.x)

🖌️ Versatile Graphics: Built-in primitives (text, shapes, bitmaps) + framebuffer control

🔧 Configurable Display: Adjustable addressing modes, offsets, and pin mappings

📦 MIT Licensed: Permissive usage in commercial/personal projects

Ideal For:

Sensor dashboards

IoT device status displays

Low-power UI implementations

Debug consoles & diagnostic interfaces

By abstracting hardware complexities while exposing essential display controls, this driver accelerates development of ESP32-based projects requiring crisp, low-power visual feedback. Its modular design ensures easy adaptation to diverse OLED hardware variants and ESP-IDF versions.
