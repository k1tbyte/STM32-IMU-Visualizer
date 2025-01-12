@echo off
echo Setting up virtual environment for STM32-BMI160-IMU-Visualizer...

:: Check if Python is installed
python --version > nul 2>&1
if errorlevel 1 (
    echo Python is not installed! Please install Python 3.x
    pause
    exit /b 1
)

:: Create virtual environment if it doesn't exist
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
) else (
    echo Virtual environment already exists...
)

:: Activate virtual environment and install requirements
echo Activating virtual environment and installing requirements...
call venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt

echo.
echo Setup completed! To run the program:
echo 1. Make sure your STM32 device is connected
echo 2. Run: python program.py
echo.
echo To activate the virtual environment manually, use:
echo    venv\Scripts\activate
pause