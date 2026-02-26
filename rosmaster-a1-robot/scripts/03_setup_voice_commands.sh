#!/bin/bash

# Setup PulseAudio for voice commands
pactl load-module module-alsa-source device=hw:1,0
pactl set-default-source alsa_input.usb-C-Media_USB_Audio_Analog-00.analog-stereo

# Set audio format to 48kHz
pactl set-source-sample-rate alsa_input.usb-C-Media_USB_Audio_Analog-00.analog-stereo 48000

# Install required packages for voice recognition
sudo apt install -y sox libsox-fmt-all

# Install Python dependencies for voice command processing
pip install SpeechRecognition pyaudio

# Confirm setup completion
echo "Voice command setup completed."