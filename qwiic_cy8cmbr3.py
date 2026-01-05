#-------------------------------------------------------------------------------
# qwiic_cy8cmbr3.py
#
# Python library for the SparkFun Qwiic Capacitive Soil Moisture Sensor, available here:
# https://www.sparkfun.com/products/TODO
#-------------------------------------------------------------------------------
# Written by SparkFun Electronics, January 2026
#
# This python library supports the SparkFun Electroncis Qwiic ecosystem
#
# More information on Qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#===============================================================================
# Copyright (c) 2026 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#===============================================================================

"""
qwiic_cy8cmbr3
============
Python module for the [SparkFun Qwiic Capacitive Soil Moisture Sensor](https://www.sparkfun.com/products/TODO)
This is a port of the existing [Arduino Library](https://github.com/sparkfun/SparkFun_CY8CMBR3_Arduino_Library)
This package can be used with the overall [SparkFun Qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)
New to Qwiic? Take a look at the entire [SparkFun Qwiic ecosystem](https://www.sparkfun.com/qwiic).
"""

# The Qwiic_I2C_Py platform driver is designed to work on almost any Python
# platform, check it out here: https://github.com/sparkfun/Qwiic_I2C_Py
import qwiic_i2c

# Define the device name and I2C addresses. These are set in the class defintion
# as class variables, making them avilable without having to create a class
# instance. This allows higher level logic to rapidly create a index of Qwiic
# devices at runtine
_DEFAULT_NAME = "Qwiic CY8CMBR3"

# Some devices have multiple available addresses - this is a list of these
# addresses. NOTE: The first address in this list is considered the default I2C
# address for the device.
_AVAILABLE_I2C_ADDRESS = [0x39] 
# Add the other allowable addresses (0x08 to 0x77) to the end of the list
_AVAILABLE_I2C_ADDRESS.extend(list(range(0x08, 0x78)))

# Define the class that encapsulates the device being created. All information
# associated with this device is encapsulated by this class. The device class
# should be the only value exported from this module.
class QwiicCY8CMBR3(object):
    # Set default name and I2C address(es)
    device_name         = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    # Register Map
    kRegSensorEn = 0x00
    kRegFssEn = 0x02
    kRegToggleEn = 0x04
    kRegLedOnEn = 0x06
    kRegSensitivity0 = 0x08
    kRegSensitivity1 = 0x09
    kRegSensitivity2 = 0x0A
    kRegSensitivity3 = 0x0B
    kRegBaseThreshold0 = 0x0C
    kRegBaseThreshold1 = 0x0D
    kRegFingerThreshold2 = 0x0E
    kRegFingerThreshold3 = 0x0F
    kRegFingerThreshold4 = 0x10
    kRegFingerThreshold5 = 0x11
    kRegFingerThreshold6 = 0x12
    kRegFingerThreshold7 = 0x13
    kRegFingerThreshold8 = 0x14
    kRegFingerThreshold9 = 0x15
    kRegFingerThreshold10 = 0x16
    kRegFingerThreshold11 = 0x17
    kRegFingerThreshold12 = 0x18
    kRegFingerThreshold13 = 0x19
    kRegFingerThreshold14 = 0x1A
    kRegFingerThreshold15 = 0x1B
    kRegSensorDebounce = 0x1C
    kRegButtonHys = 0x1D
    kRegButtonLbr = 0x1F
    kRegButtonNnt = 0x20
    kRegButtonNt = 0x21
    kRegProxEn = 0x26
    kRegProxCfg = 0x27
    kRegProxCfg2 = 0x28
    kRegProxTouchTh0 = 0x2A
    kRegProxTouchTh1 = 0x2C
    kRegProxResolution0 = 0x2E
    kRegProxResolution1 = 0x2F
    kRegProxHys = 0x30
    kRegProxLbr = 0x32
    kRegProxNnt = 0x33
    kRegProxNt = 0x34
    kRegProxPositiveTh0 = 0x35
    kRegProxPositiveTh1 = 0x36
    kRegProxNegativeTh0 = 0x39
    kRegProxNegativeTh1 = 0x3A
    kRegLedOnTime = 0x3D
    kRegBuzzerCfg = 0x3E
    kRegBuzzerOnTime = 0x3F
    kRegGpoCfg = 0x40
    kRegPwmDutyCycleCfg0 = 0x41
    kRegPwmDutyCycleCfg1 = 0x42
    kRegPwmDutyCycleCfg2 = 0x43
    kRegPwmDutyCycleCfg3 = 0x44
    kRegPwmDutyCycleCfg4 = 0x45
    kRegPwmDutyCycleCfg5 = 0x46
    kRegPwmDutyCycleCfg6 = 0x47
    kRegPwmDutyCycleCfg7 = 0x48
    kRegSpoCfg = 0x4C
    kRegDeviceCfg0 = 0x4D
    kRegDeviceCfg1 = 0x4E
    kRegDeviceCfg2 = 0x4F
    kRegDeviceCfg3 = 0x50
    kRegI2cAddr = 0x51
    kRegRefreshCtrl = 0x52
    kRegStateTimeout = 0x55
    kRegSliderCfg = 0x5D
    kRegSlider1Cfg = 0x61
    kRegSlider1Resolution = 0x62
    kRegSlider1Threshold = 0x63
    kRegSlider2Cfg = 0x67
    kRegSlider2Resolution = 0x68
    kRegSlider2Threshold = 0x69
    kRegSliderLbr = 0x71
    kRegSliderNnt = 0x72
    kRegSliderNt = 0x73
    kRegScratchpad0 = 0x7A
    kRegScratchpad1 = 0x7B
    kRegConfigCrc = 0x7E
    kRegGpoOutputState = 0x80
    kRegSensorId = 0x82
    kRegCtrlCmd = 0x86
    kRegCtrlCmdStatus = 0x88
    kRegCtrlCmdErr = 0x89
    kRegSystemStatus = 0x8A
    kRegPrevCtrlCmdCode = 0x8C
    kRegFamilyId = 0x8F
    kRegDeviceId = 0x90
    kRegDeviceRev = 0x92
    kRegCalcCrc = 0x94
    kRegTotalWorkingSns = 0x97
    kRegSnsCpHigh = 0x98
    kRegSnsVddShort = 0x9A
    kRegSnsGndShort = 0x9C
    kRegSnsSnsShort = 0x9E
    kRegCmodShieldTest = 0xA0
    kRegButtonStat = 0xAA
    kRegLatchedButtonStat = 0xAC
    kRegProxStat = 0xAE
    kRegLatchedProxStat = 0xAF
    kRegSlider1Position = 0xB0
    kRegLiftoffSlider1Position = 0xB1
    kRegSlider2Position = 0xB2
    kRegLiftoffSlider2Position = 0xB3
    kRegSyncCounter0 = 0xB9
    kRegDiffCnt0 = 0xBA
    kRegDiffCnt1 = 0xBC
    kRegDiffCnt2 = 0xBE
    kRegDiffCnt3 = 0xC0
    kRegDiffCnt4 = 0xC2
    kRegDiffCnt5 = 0xC4
    kRegDiffCnt6 = 0xC6
    kRegDiffCnt7 = 0xC8
    kRegDiffCnt8 = 0xCA
    kRegDiffCnt9 = 0xCC
    kRegDiffCnt10 = 0xCE
    kRegDiffCnt11 = 0xD0
    kRegDiffCnt12 = 0xD2
    kRegDiffCnt13 = 0xD4
    kRegDiffCnt14 = 0xD6
    kRegDiffCnt15 = 0xD8
    kRegGpoData = 0xDA
    kRegSyncCounter1 = 0xDB
    kRegDebugSensorId = 0xDC
    kRegDebugCp = 0xDD
    kRegDebugDiffCnt0 = 0xDE
    kRegDebugBaseline0 = 0xE0
    kRegDebugRawCnt0 = 0xE2
    kRegDebugAvgRawCnt0 = 0xE4
    kRegSyncCounter2 = 0xE7

    # Note: Currently since the only supported SparkFun products use the CY8CMBR3102, only CS0 and GPO0 are used. 
    # If other sensors or GPOs are used in the future, additional shifts and masks can be added here.
    # Shifts and Masks
    # SENSOR_EN: Capacitive sensor enable/disable configuration.
    kSensorEnShiftCs0 = 0
    kSensorEnMaskCs0 = 0b01

    # TOGGLE_EN: GPO toggle enable/disable.
    kToggleEnShiftGpo0 = 0
    kToggleEnMaskGpo0 = 0b01

    # LED_ON_EN: GPO extended LED ON duration enable/disable.
    kLedOnEnShiftGpo0 = 0
    kLedOnEnMaskGpo0 = 0b01

    # SENSITIVITY0: Sensitivities (units: counts/pF) for button sensors 0 - 3
    kSensitivity0ShiftCs0 = 0
    kSensitivity0MaskCs0 = 0b11

    # GPO_CFG: GPO configuration
    kGpoCfgShiftGpo0GpoCtl = 0
    kGpoCfgMaskGpo0GpoCtl = 1 << kGpoCfgShiftGpo0GpoCtl
    kGpoCfgShiftGpo0GpoPwm = 1
    kGpoCfgMaskGpo0GpoPwm = 1 << kGpoCfgShiftGpo0GpoPwm
    kGpoCfgShiftGpo0DriveMode = 2
    kGpoCfgMaskGpo0DriveMode = 1 << kGpoCfgShiftGpo0DriveMode
    kGpoCfgShiftGpo0ActiveState = 3
    kGpoCfgMaskGpo0ActiveState = 1 << kGpoCfgShiftGpo0ActiveState

    # GPO_OUTPUT_STATE: GPO output state control
    kGpoOutputStateShiftGpo0 = 0
    kGpoOutputStateMaskGpo0 = 0b01

    # GPO_DATA: Current GPO state values
    kGpoDataShiftGpo0 = 0
    kGpoDataMaskGpo0 = 0b01

    # Value Variables 
    # IDs
    kDefaultCY8CMBR3102FamilyID = 0x9A # When polling the FAMILY_ID register, this should be returned on boot.
    kDefaultCY8CMBR3102DeviceID = 0xA01 # When polling the DEVICE_ID register, this should be returned on boot.

    # Sensitivities
    # Sensitivity values (counts per pF)
    kCsSensitivity500CountsPerPf = 0  # 50 counts/0.1pF
    kCsSensitivity250CountsPerPf = 1  # 50 counts/0.2pF
    kCsSensitivity167CountsPerPf = 2  # 50 counts/0.3pF
    kCsSensitivity125CountsPerPf = 3  # 50 counts/0.4pF

    # Refresh Intervals
    kRefreshInterval20ms = 1
    kRefreshInterval40ms = 2
    kRefreshInterval60ms = 3
    kRefreshInterval80ms = 4
    kRefreshInterval100ms = 5
    kRefreshInterval120ms = 6
    kRefreshInterval140ms = 7
    kRefreshInterval160ms = 8
    kRefreshInterval180ms = 9
    kRefreshInterval200ms = 10
    kRefreshInterval220ms = 11
    kRefreshInterval240ms = 12
    kRefreshInterval260ms = 13
    kRefreshInterval280ms = 14
    kRefreshInterval300ms = 15
    kRefreshInterval320ms = 16
    kRefreshInterval340ms = 17
    kRefreshInterval360ms = 18
    kRefreshInterval380ms = 19
    kRefreshInterval400ms = 20
    kRefreshInterval420ms = 21
    kRefreshInterval440ms = 22
    kRefreshInterval460ms = 23
    kRefreshInterval480ms = 24
    kRefreshInterval500ms = 25
    
    def __init__(self, address=None, i2c_driver=None, enableDebug=False):
        """
        Constructor

        :param address: The I2C address to use for the device
            If not provided, the default address is used
        :type address: int, optional
        :param i2c_driver: An existing i2c driver object
            If not provided, a driver object is created
        :type i2c_driver: I2CDriver, optional
        """

        # Use address if provided, otherwise pick the default
        if address in self.available_addresses:
            self.address = address
        else:
            self.address = self.available_addresses[0]

        # Load the I2C driver if one isn't provided
        if i2c_driver is None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c is None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver

        # TODO: Initialize any variables used by this driver

    def is_connected(self):
        """
        Determines if this device is connected

        :return: `True` if connected, otherwise `False`
        :rtype: bool
        """
        # Check if connected by seeing if an ACK is received
        if not self._i2c.isDeviceConnected(self.address):
            return False
        
        # Check ID registers to confirm connected
        family_id = self.get_family_id()
        device_id = self.get_device_id()
        if (family_id != self.kDefaultCY8CMBR3102FamilyID or
            device_id != self.kDefaultCY8CMBR3102DeviceID):
            return False
        
        return True

    connected = property(is_connected)

    def begin(self):
        """
        Initializes this device with default parameters

        :return: Returns `True` if successful, otherwise `False`
        :rtype: bool
        """
        # Confirm device is connected before doing anything
        if not self.is_connected():
            return False
        
        if not self.enable_cs0(True):
            return False
        
        if not self.set_sensitivity_cs0(self.kCsSensitivity500CountsPerPf):
            return False

        if not self.set_refresh_interval(self.kRefreshInterval100ms):
            return False

        if not self.set_gpo_config(controlByHost=True, pwmOutput=False, strongDrive=False, activeHigh=False):
            return False

        if not self.led_off():
            return False

        if not self.set_sensor_id(0):
            return False
        
        return True

    def get_family_id(self):
        """
        Reads and returns the Family ID register

        :return: The Family ID value
        :rtype: int
        """
        # Read the Family ID register
        family_id = self._i2c.read_byte(self.address, self.kRegFamilyId)
        
        return family_id
    
    def get_device_id(self):
        """
        Reads and returns the Device ID register

        :return: The Device ID value
        :rtype: int
        """
        # Read the Device ID register
        device_id = self._i2c.read_word(self.address, self.kRegDeviceId)
        
        return device_id
    
    def enable_cs0(self, enable=True):
        """
        Enables or disables the capacitive sensor 0 (CS0)

        :param enable: If `True`, enables CS0
            If `False`, disables CS0

        :return: Returns `True` if successful, otherwise `False`
        """
        sensorEnVal = self._i2c.read_byte(self.address, self.kRegSensorEn)

        if enable:
            sensorEnVal |= self.kSensorEnMaskCs0
        else:
            sensorEnVal &= ~self.kSensorEnMaskCs0

        # Write the sensor enable value to the SENSOR_EN register
        self._i2c.write_byte(self.address, self.kRegSensorEn, sensorEnVal)

        return True
    
    def set_sensitivity_cs0(self, sensitivity):
        """
        Sets the sensitivity for the capacitive sensor 0 (CS0)

        :param sensitivity: The sensitivity value to set
            Use one of the kCsSensitivity* constants defined in this class

        :return: Returns `True` if successful, otherwise `False`
        """
        if sensitivity < self.kCsSensitivity500CountsPerPf or sensitivity > self.kCsSensitivity125CountsPerPf:
            self.debug_print("Invalid sensitivity value.")
            return False  # Invalid sensitivity value
        
        # Read the current SENSITIVITY register value
        sensVal = self._i2c.read_byte(self.address, self.kRegSensitivity0)

        # Clear the bits associated with the capacitive sensor
        sensVal &= ~self.kSensitivity0MaskCs0

        # Set the new sensitivity value
        sensVal |= (sensitivity << self.kSensitivity0ShiftCs0) & self.kSensitivity0MaskCs0

        # Write the updated sensitivity value back to the register
        self._i2c.write_byte(self.address, self.kRegSensitivity0, sensVal)

        return True
    
    def set_refresh_interval(self, interval):
        """
        Sets the refresh interval for the capacitive sensor

        :param interval: The refresh interval value to set
            Use one of the kRefreshInterval* constants defined in this class

        :return: Returns `True` if successful, otherwise `False`
        """
        if interval < self.kRefreshInterval20ms or interval > self.kRefreshInterval500ms:
            self.debug_print("Invalid refresh interval value.")
            return False  # Invalid refresh interval value
        
        # Write the refresh interval value to the REFRESH_CTRL register
        self._i2c.write_byte(self.address, self.kRegRefreshCtrl, interval)

        return True
    
    def set_gpo_config(self, controlByHost, pwmOutput, strongDrive, activeHigh):
        """
        Configures the GPO0 settings

        :param controlByHost: If `True`, GPO0 is controlled by the host
            If `False`, GPO0 is controlled by the device
        :param pwmOutput: If `True`, GPO0 outputs PWM signal
            If `False`, GPO0 outputs digital signal
        :param strongDrive: If `True`, GPO0 uses strong drive mode
            If `False`, GPO0 uses weak drive mode
        :param activeHigh: If `True`, GPO0 is active high
            If `False`, GPO0 is active low

        :return: Returns `True` if successful, otherwise `False`
        """
        gpoCfgVal = self._i2c.read_byte(self.address, self.kRegGpoCfg)

        # Configure control by host/device
        if controlByHost:
            gpoCfgVal |= self.kGpoCfgMaskGpo0GpoCtl
        else:
            gpoCfgVal &= ~self.kGpoCfgMaskGpo0GpoCtl
        
        # Configure PWM/digital output
        if pwmOutput:
            gpoCfgVal |= self.kGpoCfgMaskGpo0GpoPwm
        else:
            gpoCfgVal &= ~self.kGpoCfgMaskGpo0GpoPwm

        # Configure strong/weak drive mode
        if strongDrive:
            gpoCfgVal |= self.kGpoCfgMaskGpo0DriveMode
        else:
            gpoCfgVal &= ~self.kGpoCfgMaskGpo0DriveMode
        
        # Configure active high/low
        if activeHigh:
            gpoCfgVal |= self.kGpoCfgMaskGpo0ActiveState
        else:
            gpoCfgVal &= ~self.kGpoCfgMaskGpo0ActiveState
       
        # Write the GPO configuration value to the GPO_CFG register
        self._i2c.write_byte(self.address, self.kRegGpoCfg, gpoCfgVal)

        return True
    
    def led_on(self, enable=True):
        """
        Sets the GPO0 output state to turn on/off an LED

        :param enable: If `True`, turns on the LED
            If `False`, turns off the LED

        :return: Returns `True` if successful, otherwise `False`
        """
        gpoOutputVal = self._i2c.read_byte(self.address, self.kRegGpoOutputState)

        if enable:
            gpoOutputVal |= self.kGpoOutputStateMaskGpo0
        else:
            gpoOutputVal &= ~self.kGpoOutputStateMaskGpo0

        # Write the GPO output state value to the GPO_OUTPUT_STATE register
        self._i2c.write_byte(self.address, self.kRegGpoOutputState, gpoOutputVal)

        return True
    
    def led_off(self):
        """
        Turns off the LED connected to GPO0

        :return: Returns `True` if successful, otherwise `False`
        """
        return self.led_on(False)
    
    def set_sensor_id(self, sensor_id=0):
        """
        Sets the Sensor ID register

        :param sensor_id: The Sensor ID value to set

        :return: Returns `True` if successful, otherwise `False`
        """
        # Write the Sensor ID value to the SENSOR_ID register
        self._i2c.write_byte(self.address, self.kRegSensorId, sensor_id)

        return True
    
    def get_capacitance_pf(self):
        """
        Reads and returns the capacitance value from CS0 in picofarads (pF)

        :return: The capacitance value in pF or 0 on error
        :rtype: float
        """
        if not self.set_sensor_id(0):
            return 0.0

        # Read the capacitance value from the DebugCp register
        capacitancePf = self._i2c.read_word(self.address, self.kRegDebugCp)

        return capacitancePf

    def get_diff_count(self):
        """
        Reads and returns the difference count value from CS0

        :return: The difference count value
        :rtype: int
        """
        # Read the difference count value from the DIFF_CNT0 register
        diffCount = self._i2c.read_word(self.address, self.kRegDiffCnt0)

        return diffCount
    
    def get_diff_pf(self):
        """
        Reads and returns the difference count value from CS0 in picofarads (pF)

        :return: The difference count value in pF
        :rtype: float
        """
        # Read the difference count value from the DIFF_CNT0 register
        diffCount = self.get_diff_count()

        # Read the sensitivity setting for CS0
        sensVal = self._i2c.read_byte(self.address, self.kRegSensitivity0)
        sensitivitySetting = (sensVal & self.kSensitivity0MaskCs0) >> self.kSensitivity0ShiftCs0

        # Map sensitivity setting to counts per pF
        if sensitivitySetting == self.kCsSensitivity500CountsPerPf:
            countsPerPf = 500
        elif sensitivitySetting == self.kCsSensitivity250CountsPerPf:
            countsPerPf = 250
        elif sensitivitySetting == self.kCsSensitivity167CountsPerPf:
            countsPerPf = 167
        elif sensitivitySetting == self.kCsSensitivity125CountsPerPf:
            countsPerPf = 125
        else:
            self.debug_print("Invalid sensitivity setting.")
            return 0.0  # Invalid sensitivity setting

        # Calculate capacitance in pF
        capacitancePf = diffCount / countsPerPf

        return capacitancePf
    
    def get_baseline_count(self):
        """
        Reads and returns the baseline count value from CS0

        :return: The baseline count value
        :rtype: int
        """
        if not self.set_sensor_id(0):
            return 0

        # Read the baseline count value from the DebugBaseline0 register
        baselineCount = self._i2c.read_word(self.address, self.kRegDebugBaseline0)

        return baselineCount
    
    def get_raw_count(self):
        """
        Reads and returns the raw count value from CS0

        :return: The raw count value
        :rtype: int
        """
        if not self.set_sensor_id(0):
            return 0

        # Read the raw count value from the DebugRawCnt0 register
        rawCount = self._i2c.read_word(self.address, self.kRegDebugRawCnt0)

        return rawCount

    def debug_print(self, str):
        """
        Prints debug information to the console if debugging is enabled

        :param str: The string to print
        """
        if self.enableDebug:
            print("QwiicCY8CMBR3: " + str)

    


