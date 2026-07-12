# Arduino-Project SilentSleep
### Snore Detection and Haptic Feedback System

## Overview

SilentSleep is an Arduino UNO R4 WiFi-based embedded system that detects snore-like sounds using a microphone, analyzes the audio using Fast Fourier Transform (FFT), and provides gentle haptic feedback through a vibration motor. The system displays its operating state on a TFT display and is designed as a prototype for a non-invasive sleep assistance device.

## Features

- Real-time microphone monitoring
- FFT-based audio frequency analysis
- Snore confidence estimation
- Haptic feedback using a coin vibration motor
- TFT display showing system status
- I2C communication with DRV2605L
- Modular state machine architecture


## Hardware
- Arduino UNO R4 WiFi
- MAX9814 Microphone Amplifier
- DRV2605L Haptic Driver
- Coin ERM Vibration Motor
- 1.54" ST7789 TFT Display
- Breadboard and jumper wires
