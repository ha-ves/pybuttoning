@echo off
REM Build and Test Script for PyButtoning with Custom RealSense Node
REM This script builds the package and runs basic tests

echo ============================================================
echo PyButtoning Custom RealSense Node - Build and Test
echo ============================================================
echo.

REM Get the workspace root (parent of src directory)
set SCRIPT_DIR=%~dp0
cd /d "%SCRIPT_DIR%\..\..\"
set WORKSPACE_ROOT=%CD%

echo Workspace: %WORKSPACE_ROOT%
echo.

REM Check if we're in the right directory
if not exist "src\pybuttoning" (
    echo ERROR: Cannot find src\pybuttoning directory
    echo Please run this script from the pybuttoning package directory
    pause
    exit /b 1
)

echo ============================================================
echo Step 1: Installing Python dependencies
echo ============================================================
echo.

REM Install pyrealsense2 if not already installed
python -c "import pyrealsense2" 2>nul
if errorlevel 1 (
    echo Installing pyrealsense2...
    pip install pyrealsense2>=2.50.0
    if errorlevel 1 (
        echo ERROR: Failed to install pyrealsense2
        pause
        exit /b 1
    )
) else (
    echo pyrealsense2 is already installed
)

echo.
echo Installing other dependencies from requirements.txt...
pip install -r src\pybuttoning\requirements.txt
if errorlevel 1 (
    echo WARNING: Some dependencies may have failed to install
    echo Continue anyway? Press Ctrl+C to cancel, or
    pause
)

echo.
echo ============================================================
echo Step 2: Building the package
echo ============================================================
echo.

echo Building pybuttoning package...
colcon build --packages-select pybuttoning
if errorlevel 1 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo Build successful!
echo.

echo ============================================================
echo Step 3: Sourcing the workspace
echo ============================================================
echo.

if exist "install\setup.ps1" (
    echo To source the workspace, run:
    echo   .\install\setup.ps1
    echo.
    echo Or for CMD:
    echo   call install\setup.bat
) else (
    echo ERROR: Build output not found!
    pause
    exit /b 1
)

echo.
echo ============================================================
echo Step 4: Testing (after sourcing the workspace)
echo ============================================================
echo.

echo After sourcing the workspace, you can test with:
echo.
echo 1. List installed nodes:
echo    ros2 pkg executables pybuttoning
echo.
echo 2. Launch RealSense node:
echo    ros2 launch pybuttoning realsense.launch.py
echo.
echo 3. Run test node (in another terminal):
echo    ros2 run pybuttoning realsense_test_node
echo.
echo 4. Check topics:
echo    ros2 topic list ^| Select-String camera
echo.
echo 5. Launch complete system:
echo    ros2 launch pybuttoning buttoning_with_realsense.launch.py model_path:="path\to\model"
echo.

echo ============================================================
echo Build Complete!
echo ============================================================
echo.
echo Summary:
echo - Package built successfully
echo - Entry points created:
echo   * realsense_node
echo   * realsense_test_node
echo   * detection_node
echo   * hand_detection_node
echo   * arm_controller_node
echo   * buttoning_controller_node
echo.
echo Next steps:
echo 1. Source the workspace (see Step 3 above)
echo 2. Run tests (see Step 4 above)
echo 3. Check documentation:
echo    - src\pybuttoning\README.md
echo    - src\pybuttoning\docs\REALSENSE_QUICKSTART.md
echo    - src\pybuttoning\docs\REALSENSE_NODE.md
echo.

pause
