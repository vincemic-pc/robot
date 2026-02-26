#!/bin/bash

# This script configures the LLM provider for the robot.

# Load configuration from the environment
source ~/.rosmaster_llm_config

# Check if the required environment variables are set
if [ -z "$OPENAI_API_KEY" ]; then
    echo "Error: OPENAI_API_KEY is not set. Please set it in ~/.rosmaster_llm_config."
    exit 1
fi

# Install required Python packages for LLM integration
pip install -r requirements.txt

echo "LLM provider setup completed successfully."