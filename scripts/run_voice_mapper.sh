#!/bin/bash
# run_voice_mapper.sh — Voice-controlled mapping explorer (main application)
source /home/jetson/robot_scripts/ros2_env.sh

# Source LLM environment (OpenAI API key)
if [ -f /home/jetson/.rosmaster_llm_config ]; then
    source /home/jetson/.rosmaster_llm_config
fi

cd /home/jetson/robot_scripts
exec python3 voice_mapper.py
