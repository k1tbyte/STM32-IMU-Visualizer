#!/bin/bash

echo "Setting up virtual environment for STM32-BMI160-IMU-Visualizer..."

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "Python is not installed! Please install Python 3.x"
    exit 1
fi

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
else
    echo "Virtual environment already exists..."
fi

# Activate virtual environment and install requirements
echo "Activating virtual environment and installing requirements..."
source venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

echo
echo "Setup completed! To run the program:"
echo "1. Make sure your STM32 device is connected"
echo "2. Run: python program.py"
echo
echo "To activate the virtual environment manually, use:"
echo "    source venv/bin/activate"