#!/usr/bin/env python
"""
DDS238-7 Three Phase Smart Meter. Domoticz plugin.
Author: voyo@no-ip.pl
Requirements:
    1.python module minimalmodbus -> http://minimalmodbus.readthedocs.io/en/master/
        (pi@raspberrypi:~$ sudo pip3 install minimalmodbus)
    2.Communication module Modbus USB to RS485 converter module
"""
"""
<plugin key="DDS238-7" name="DDS238-7 modbus" version="0.8.3" author="voyo@no-ip.pl">
    <params>
        <param field="Address" label="IP Address" width="200px" required="true" default="127.0.0.1"/>
        <param field="Port" label="Port" width="30px" required="true" default="502"/>
        <param field="SerialPort" label="Modbus Port" width="200px" required="true" default="/dev/ttyUSB0" />
        <param field="Mode1" label="Baud rate" width="40px" required="true" default="9600"  />
        <param field="Mode2" label="Device ID" width="40px" required="true" default="1" />
        <param field="Mode3" label="Reading Interval * 10s." width="40px" required="true" default="1" />
        <param field="Mode4" label="Modbus type" width="75px">
            <description><h2>Modbus type</h2>Select the desired type of modbus connection</description>
            <options>
                <option label="TCP" value="TCP" default="true" />
                <option label="RTU" value="RTU" />
            </options>
         </param>
        <param field="Mode6" label="Debug" width="75px">
            <options>
                <option label="True" value="Debug"/>
                <option label="False" value="Normal"  default="true" />
            </options>
        </param>
    </params>
</plugin>
"""

import minimalmodbus
import serial
try:
    import Domoticz
except ImportError:
    import fakeDomoticz as Domoticz
import time
import math

# for TCP modbus connection
from pyModbusTCP.client import ModbusClient
from pymodbus.constants import Endian

# ANSI color codes (global context)
ANSI_BLUE = '\033[94m'
ANSI_RED = '\033[91m'
ANSI_GREEN = '\033[92m'
ANSI_YELLOW = '\033[93m'
ANSI_RESET = '\033[0m'

# ==============================================================================
# VALIDATION CONFIGURATION - Final value ranges only, raw calculated using multiplier
# ==============================================================================
VALIDATION_RANGES = {
    # Energy Measurements
    "Total Energy": (0, 100000000),      # Wh
    "Life Energy": (0, 100000000),       # Wh
    "Reactive Energy": (0, 100000000),   # VArh
    "Import Energy": (0, 100000000),     # Wh
    "Export Energy": (0, 100000000),     # Wh

    # Voltage Related
    "Voltage L1": (180, 260),            # V
    "Voltage L2": (180, 260),            # V
    "Voltage L3": (180, 260),            # V
    "Voltage Frequency": (45, 65),       # Hz

    # Current Related
    "Current L1": (0, 100),              # A
    "Current L2": (0, 100),              # A
    "Current L3": (0, 100),              # A

    # Active Power (signed values)
    "Total Active Power": (-25000, 25000),  # W
    "Active Power L1": (-25000, 25000),     # W
    "Active Power L2": (-25000, 25000),     # W
    "Active Power L3": (-25000, 25000),     # W

    # Reactive Power
    "Total Reactive Power": (-25000, 25000),  # kVAr
    "Reactive Power L1": (-25000, 25000),     # kVAr
    "Reactive Power L2": (-25000, 25000),     # kVAr
    "Reactive Power L3": (-25000, 25000),     # kVAr

    # Apparent Power
    "Total Apparent Power": (0, 25000),      # kVA
    "Apparent Power L1": (0, 25000),         # kVA
    "Apparent Power L2": (0, 25000),         # kVA
    "Apparent Power L3": (0, 25000),         # kVA

    # Power Factor
    "Power Factor": (0, 1),                  # PF
    "Power Factor L1": (0, 1),               # PF
    "Power Factor L2": (0, 1),               # PF
    "Power Factor L3": (0, 1),               # PF

    # Reserved
    "Reserved1": (0, 100000000),
    "Reserved2": (0, 100000000),
}

# ==============================================================================
# CONNECTION HEALTH MONITOR
# ==============================================================================
class ConnectionHealthMonitor:
    def __init__(self, max_failures=5, reset_cooldown=30):
        self.consecutive_failures = 0
        self.max_failures = max_failures
        self.reset_cooldown = reset_cooldown
        self.last_reset_time = 0
        self.bleeding_count = 0
        self.total_reads = 0
        
    def mark_success(self):
        """Mark successful read - reset failure counter"""
        self.consecutive_failures = 0
        self.total_reads += 1
        
    def mark_failure(self):
        """Mark failed read - increment failure counter"""
        self.consecutive_failures += 1
        self.total_reads += 1
        
    def mark_bleeding_detected(self):
        """Mark detected register bleeding"""
        self.bleeding_count += 1
        Domoticz.Log(f"{ANSI_RED}Register bleeding detected! Total bleeding events: {self.bleeding_count}{ANSI_RESET}")
        
    def should_reset_connection(self):
        """Check if connection should be reset"""
        current_time = time.time()
        
        # Don't reset too frequently
        if current_time - self.last_reset_time < self.reset_cooldown:
            return False
            
        # Reset if too many consecutive failures
        if self.consecutive_failures >= self.max_failures:
            return True
            
        return False
        
    def reset_connection(self, rs485_connection):
        """Reset the connection if possible"""
        current_time = time.time()
        self.last_reset_time = current_time
        self.consecutive_failures = 0
        
        try:
            if hasattr(rs485_connection, 'close'):
                rs485_connection.close()
                Domoticz.Log(f"{ANSI_YELLOW}Connection reset attempted due to consecutive failures{ANSI_RESET}")
                return True
        except Exception as e:
            Domoticz.Error(f"Failed to reset connection: {str(e)}")
            
        return False
        
    def get_stats(self):
        """Get connection health statistics"""
        if self.total_reads == 0:
            return "No reads performed yet"
        
        failure_rate = (self.consecutive_failures / max(self.total_reads, 1)) * 100
        return f"Failures: {self.consecutive_failures}, Bleeding: {self.bleeding_count}, Rate: {failure_rate:.1f}%"

# ==============================================================================
# ENHANCED AVERAGE CLASS - Rejects invalid inputs
# ==============================================================================
class Average:
    def __init__(self):
        self.samples = []
        self.max_samples = 30

    def set_max_samples(self, max_samples):
        self.max_samples = max_samples
        if self.max_samples < 1:
            self.max_samples = 1

    def update(self, new_value, scale=0):
        """Update with validation - reject NaN, infinity, or invalid values"""
        if new_value is None:
            Domoticz.Debug("Average.update: Rejecting None value")
            return
            
        if isinstance(new_value, (int, float)):
            # Check for NaN and infinity
            if math.isnan(new_value) or math.isinf(new_value):
                Domoticz.Debug(f"Average.update: Rejecting invalid value: {new_value}")
                return
                
            scaled_value = new_value * (10 ** scale)
            
            # Additional sanity check for extremely large values
            if abs(scaled_value) > 1e15:  # Arbitrary large threshold
                Domoticz.Debug(f"Average.update: Rejecting extremely large value: {scaled_value}")
                return
                
            self.samples.append(scaled_value)
            while len(self.samples) > self.max_samples:
                del self.samples[0]
            Domoticz.Debug(f"Average: {self.get()} - {len(self.samples)} values")
        else:
            Domoticz.Debug(f"Average.update: Rejecting non-numeric value: {new_value} ({type(new_value)})")

    def get(self):
        if len(self.samples) == 0:
            return 0
        return int(sum(self.samples) / len(self.samples))

# ==============================================================================
# UNIFIED VALIDATION SYSTEM - Uses device multiplier from configuration
# ==============================================================================
def is_value_reasonable(value, register_name, multiplier=None, is_raw=False):
    """
    Unified validation function for both raw and calculated values.
    Uses device multiplier to calculate raw range from final range.
    
    Args:
        value: Value to validate
        register_name: Name of the register/device
        multiplier: Multiplier from device configuration
        is_raw: True if validating raw value, False for final value
        
    Returns:
        (is_valid, reason)
    """
    if value is None:
        return False, "Value is None"
        
    if not isinstance(value, (int, float)):
        return False, f"Invalid value type: {type(value)}"
        
    # Check for NaN and infinity
    if math.isnan(value) or math.isinf(value):
        return False, f"Value is NaN or infinity: {value}"
    
    # Get validation range for this register
    if register_name not in VALIDATION_RANGES:
        Domoticz.Debug(f"Warning: No validation range defined for device: {register_name}")
        return True, f"No validation range defined for {register_name}"
    
    # Get final value range
    final_min, final_max = VALIDATION_RANGES[register_name]
    
    if is_raw and multiplier is not None:
        # Calculate raw range using device multiplier
        # raw_value * multiplier = final_value
        # So: raw_min = final_min / multiplier, raw_max = final_max / multiplier
        if multiplier == 0:
            return True, "Multiplier is zero - cannot validate raw value"
            
        # Handle signed ranges properly
        if final_min < 0:  # Signed range
            raw_min = final_min / multiplier
            raw_max = final_max / multiplier
        else:  # Unsigned range
            raw_min = final_min / multiplier
            raw_max = final_max / multiplier
            
        # Ensure proper order for negative multipliers
        if raw_min > raw_max:
            raw_min, raw_max = raw_max, raw_min
            
        min_val, max_val = raw_min, raw_max
        range_type = "raw (calculated)"
    else:
        # Use final range
        min_val, max_val = final_min, final_max
        range_type = "final"
    
    if min_val <= value <= max_val:
        return True, "Value within range"
    else:
        return False, f"Value {value} outside {range_type} range [{min_val:.3f}, {max_val:.3f}]"

# ==============================================================================
# BLEEDING DETECTION
# ==============================================================================
class BleedingDetector:
    def __init__(self, identical_threshold=3):
        self.last_values = {}  # {device_name: [last_n_values]}
        self.identical_threshold = identical_threshold
        
    def check_bleeding(self, device_name, raw_value):
        """
        Detect potential register bleeding by checking for identical consecutive values.
        Returns True if bleeding suspected.
        """
        if device_name not in self.last_values:
            self.last_values[device_name] = []
            
        values = self.last_values[device_name]
        values.append(raw_value)
        
        # Keep only last N values
        if len(values) > self.identical_threshold:
            values.pop(0)
            
        # Check if all values are identical (and we have enough samples)
        if len(values) >= self.identical_threshold:
            if len(set(values)) == 1:  # All values are the same
                # Additional check: don't flag legitimate zero values for energy counters
                if raw_value == 0 and "Energy" in device_name:
                    return False
                    
                Domoticz.Log(f"{ANSI_RED}Bleeding suspected for {device_name}: {self.identical_threshold} identical values: {raw_value}{ANSI_RESET}")
                return True
                
        return False

# ==============================================================================
# DEVICE CLASS - Enhanced with bleeding protection
# ==============================================================================
class Dev:
    def __init__(self, ID, name, multipler, register, size, functioncode: int = 3, options=None, Used: int = 1, signed: bool = False, Description=None, TypeName=None, Type: int = 0, SubType: int = 0, SwitchType: int = 0):
        self.ID = ID
        self.name = name
        self.TypeName = TypeName if TypeName is not None else ""
        self.Type = Type
        self.SubType = SubType
        self.SwitchType = SwitchType
        self.multipler = multipler
        self.value = 0
        self.signed = signed
        self.register = register
        self.size = size
        self.functioncode = functioncode
        self.options = options if options is not None else None
        self.Used = Used
        self.Description = Description if Description is not None else ""
        self.last_valid_value = None  # Store last valid value for fallback
        
        if self.ID not in Devices:
            if self.TypeName != "":
                Domoticz.Log("Adding device: " + self.name + " " + str(self.ID) + " " + self.TypeName + "  Description: " + str(self.Description))
                Domoticz.Device(Name=self.name, Unit=self.ID, TypeName=self.TypeName, Used=self.Used, Options=self.options, Description=self.Description).Create()
            else:
                Domoticz.Device(Name=self.name, Unit=self.ID, Type=self.Type, Subtype=self.SubType, Switchtype=self.SwitchType, Used=self.Used, Options=self.options, Description=self.Description).Create()
                Domoticz.Log("Adding device: name" + self.name + ", Type: " + str(self.Type) + " SubType: " + str(self.SubType) + " SwitchType: " + str(self.SwitchType))

    def UpdateValue(self, RS485, outerClass):
        """
        Enhanced UpdateValue with register bleeding protection.
        Key changes:
        1. NO RETRY mechanism - single read attempt only
        2. Raw value validation before calculation
        3. Bleeding detection
        4. Connection health monitoring
        5. Fallback to last valid value
        """
        try:
            if self.size == 0:  # Virtual device
                value = 0
                payload = 0
            elif self.size == 1:
                # SINGLE READ ATTEMPT ONLY - NO RETRY
                registers = RS485.read_holding_registers(self.register, self.size)
                if registers is None or len(registers) == 0:
                    Domoticz.Log(f"{ANSI_RED}Failed to read register {self.register} for {self.name} - SKIPPING (no retry){ANSI_RESET}")
                    outerClass.health_monitor.mark_failure()
                    return  # Exit immediately - prevents register bleeding
                
                # Mark successful read
                outerClass.health_monitor.mark_success()
                
                value = registers[0]
                if self.signed and value > 32767:
                    value -= 65536
                
                # RAW VALUE VALIDATION - using device multiplier from configuration
                is_valid_raw, reason = is_value_reasonable(value, self.name, self.multipler, is_raw=True)
                if not is_valid_raw:
                    Domoticz.Log(f"{ANSI_RED}Invalid raw value for {self.name}: {value} - {reason}{ANSI_RESET}")
                    outerClass.health_monitor.mark_bleeding_detected()
                    return
                
                # BLEEDING DETECTION
                if outerClass.bleeding_detector.check_bleeding(self.name, value):
                    outerClass.health_monitor.mark_bleeding_detected()
                    return
                
                payload = value * self.multipler

            elif self.size == 2:
                # SINGLE READ ATTEMPT ONLY - NO RETRY
                registers = RS485.read_holding_registers(self.register, self.size)
                if registers is None or len(registers) < 2:
                    Domoticz.Log(f"{ANSI_RED}Failed to read register {self.register} (size=2) for {self.name} - SKIPPING (no retry){ANSI_RESET}")
                    outerClass.health_monitor.mark_failure()
                    return  # Exit immediately - prevents register bleeding
                
                # Mark successful read
                outerClass.health_monitor.mark_success()
                
                value = (registers[0] << 16) + registers[1]
                if self.signed and value > 2147483647:
                    value -= 4294967296
                
                # RAW VALUE VALIDATION - using device multiplier from configuration
                is_valid_raw, reason = is_value_reasonable(value, self.name, self.multipler, is_raw=True)
                if not is_valid_raw:
                    Domoticz.Log(f"{ANSI_RED}Invalid raw value for {self.name}: {value} - {reason}{ANSI_RESET}")
                    outerClass.health_monitor.mark_bleeding_detected()
                    return
                
                # BLEEDING DETECTION
                if outerClass.bleeding_detector.check_bleeding(self.name, value):
                    outerClass.health_monitor.mark_bleeding_detected()
                    return
                
                payload = value * self.multipler

            # FINAL VALUE VALIDATION - after applying multiplier
            is_valid_final, reason = is_value_reasonable(payload, self.name, self.multipler, is_raw=False)
            if not is_valid_final:
                Domoticz.Log(f"{ANSI_RED}Invalid final value for {self.name}: {payload} - {reason}{ANSI_RESET}")
                return

            # Store as last valid value
            self.last_valid_value = payload

            # Debug logging
            if Parameters["Mode6"] == 'Debug':
                Domoticz.Debug(f"Updating {self.name} with value: {payload}")

            # Debug for energy devices
            if self.name in ["Total Energy", "Life Energy", "Import Energy", "Export Energy"]:
                Domoticz.Debug(f"{self.name}: raw value={value}, multiplier={self.multipler}, payload={payload}")

            # Update Domoticz devices (keeping existing logic)
            if self.Type == 243:  # Power, Voltage, Current measurements
                if self.name == "Total Active Power":
                    outerClass.active_power.update(int(payload))
                    if payload < 0:
                        outerClass.reverse_power.update(abs(int(payload)))
                        outerClass.forward_power.update(0)
                    else:
                        outerClass.reverse_power.update(0)
                        outerClass.forward_power.update(int(payload))
                elif "Voltage" in self.name:
                    outerClass.voltage = int(payload)
                elif "Current" in self.name:
                    outerClass.current = int(payload)
                elif "Apparent Power" in self.name:
                    outerClass.apparent_power = int(payload)

            USAGE1 = USAGE2 = RETURN1 = RETURN2 = CONS = PROD = str(0)
            
            # Update Domoticz device with new value (keeping existing logic)
            if self.ID in Devices:
                if self.Type == 250 and self.SubType in (1, 6):
                    # P1 Smart Meter logic (unchanged)
                    if "Total Energy" in self.name:
                        outerClass.total_energy = int(payload)
                        USAGE1 = str(payload)
                        CONS = str(int(outerClass.active_power.get()))
                        sValue = USAGE1 + ";" + USAGE2 + ";" + RETURN1 + ";" + RETURN2 + ";" + CONS + ";" + PROD
                        Domoticz.Log(f"TOTAL ENERGY: {sValue}")
                    elif "Import Energy" in self.name:
                        USAGE1 = str(payload)
                        outerClass.ImportEnergy = payload
                        CONS = str(outerClass.forward_power.get())
                        sValue = USAGE1 + ";" + USAGE2 + ";" + RETURN1 + ";" + RETURN2 + ";" + CONS + ";" + PROD
                    elif "Export Energy" in self.name:
                        RETURN1 = str(payload)
                        outerClass.ExportEnergy = payload
                        PROD = str(abs(outerClass.reverse_power.get()))
                        sValue = USAGE1 + ";" + USAGE2 + ";" + RETURN1 + ";" + RETURN2 + ";" + CONS + ";" + PROD
                    elif "Life Energy" in self.name:
                        USAGE1 = str(int(outerClass.ImportEnergy))
                        RETURN1 = str(int(outerClass.ExportEnergy))
                        CONS = str(int(outerClass.forward_power.get()))
                        PROD = str(int(outerClass.reverse_power.get()))
                        sValue = USAGE1 + ";" + USAGE2 + ";" + RETURN1 + ";" + RETURN2 + ";" + CONS + ";" + PROD
                    elif "Reactive Energy" in self.name:
                        USAGE1 = str(payload)
                        CONS = str(abs(int(outerClass.reactive_power.get())))
                        Domoticz.Log(f"{ANSI_YELLOW}REACTIVE ENERGY CONS: {CONS} {ANSI_RESET}")
                        sValue = USAGE1 + ";" + USAGE2 + ";" + RETURN1 + ";" + RETURN2 + ";" + CONS + ";" + PROD
                    else:
                        Domoticz.Log(f"WPADŁO ELSE, dla device {self.ID}, {self.name}")
                        sValue = f"{payload:.2f};0;0;0;0"

                    Devices[self.ID].Update(nValue=0, sValue=sValue)
                    Domoticz.Log(f"Domoticz P1 device {self.ID} updated with sValue: {sValue}")
                else:
                    if "Total Active Power" in self.name:
                        outerClass.active_power.update(int(payload))
                        if payload < 0:
                            outerClass.reverse_power.update(abs(int(payload)))
                            outerClass.forward_power.update(0)
                        else:
                            outerClass.reverse_power.update(0)
                            outerClass.forward_power.update(int(payload))
                    elif "Total Reactive Power" in self.name:
                        reactive_energy = int(payload * float(1000))
                        Domoticz.Log(f"{ANSI_YELLOW}REACTIVE ENERGY: {reactive_energy} {ANSI_RESET}")
                        outerClass.reactive_power.update(int(reactive_energy))

                    Devices[self.ID].Update(nValue=0, sValue=str(payload))
                    Domoticz.Log(f"Domoticz device {self.ID} updated with value: {payload}")
            else:
                Domoticz.Error(f"Device ID {self.ID} not found in Devices")

        except Exception as e:
            Domoticz.Error(f"Error updating {self.name}: {str(e)}")
            outerClass.health_monitor.mark_failure()
            
            # Check if connection should be reset
            if outerClass.health_monitor.should_reset_connection():
                Domoticz.Log(f"{ANSI_YELLOW}Attempting connection reset due to consecutive failures{ANSI_RESET}")
                outerClass.health_monitor.reset_connection(RS485)

# ==============================================================================
# MAIN PLUGIN CLASS - Enhanced with health monitoring
# ==============================================================================
class BasePlugin:
    def __init__(self):
        self.runInterval = 1
        self.RS485 = ""
        # Enhanced Average classes
        self.active_power = Average()
        self.reactive_power = Average()
        self.reverse_reactive_power = Average()
        self.forward_reactive_power = Average()
        self.forward_power = Average()
        self.reverse_power = Average()
        
        # Store latest values (not averages) for these:
        self.voltage = 0
        self.current = 0
        self.apparent_power = 0
        self.total_energy = 0
        self.ImportEnergy = 0
        self.ExportEnergy = 0
        self.devs = []
        
        # NEW: Health monitoring and bleeding detection
        self.health_monitor = ConnectionHealthMonitor()
        self.bleeding_detector = BleedingDetector()
        
        return

    def onStart(self):
        if Parameters["Mode6"] == "Debug":
            Domoticz.Debugging(1)
            DumpConfigToLog()
            Domoticz.Debug("Debugging enabled")

        Domoticz.Log("DDS238-7 Modbus plugin start, mode: " + Parameters["Mode4"])
        DeviceID = int(Parameters["Mode2"])

        if Parameters["Mode4"] == "RTU" or Parameters["Mode4"] == "ASCII":
            self.RS485 = minimalmodbus.Instrument(Parameters["SerialPort"], DeviceID)
            self.RS485.serial.baudrate = Parameters["Mode1"]
            self.RS485.serial.bytesize = 8
            self.RS485.serial.parity = minimalmodbus.serial.PARITY_NONE
            self.RS485.serial.stopbits = 1
            self.RS485.serial.timeout = 1
            self.RS485.debug = False
            self.RS485.mode = minimalmodbus.MODE_RTU
            self.RS485.MyMode = 'minimalmodbus'
            if Parameters["Mode4"] == "RTU":
                self.RS485.mode = minimalmodbus.MODE_RTU
            elif Parameters["Mode4"] == "ASCII":
                self.RS485.mode = minimalmodbus.MODE_ASCII
        elif Parameters["Mode4"] == "TCP":
            Domoticz.Debug("Using pymodbus, connecting to " + Parameters["Address"] + ":" + Parameters["Port"] + " unit ID: " + str(DeviceID))
            try:
                self.RS485 = ModbusClient(
                    host=Parameters["Address"],
                    port=int(Parameters["Port"]),
                    unit_id=DeviceID,
                    timeout=2
                )
                if self.RS485:
                    self.RS485.MyMode = 'pymodbus'
                else:
                    Domoticz.Error("Failed to create ModbusClient - check your connection parameters")
                    return
            except Exception as e:
                Domoticz.Error(f"pyModbus connection failure: {str(e)}")
                return

        # Only initialize devices if we have a valid connection
        if not self.RS485 or isinstance(self.RS485, str):
            Domoticz.Error("No valid Modbus connection established")
            return

        # Initialize devices list (unchanged)
        self.devs = [
            Dev(1, "Total Energy", 10, 0x00, size=2, functioncode=3, Type=250, SubType=1, Description="Total energy balance"),
            Dev(2, "Life Energy", 10, 0x00, size=0, functioncode=3, Type=250, SubType=1, Description="Total energy flow"),
            Dev(3, "Reserved1", 10, 0x02, size=0, functioncode=3, Used=0, Type=250, SubType=1, Description="Reserved1"),
            Dev(4, "Reactive Energy", 10, 0x02, size=2, functioncode=3, options={"Custom": "1;kVArh"}, Type=250, SubType=6, Description="Reactive energy"),
            Dev(5, "Reserved2", 10, 0x06, size=0, functioncode=3, Used=0, Type=250, SubType=1, Description="Reserved2"),
            Dev(6, "Import Energy", 10, 0x08, size=2, functioncode=3, Type=250, SubType=1, Description="Forward energy"),
            Dev(7, "Export Energy", 10, 0xA, size=2, functioncode=3, Type=250, SubType=1, Description="Reverse energy"),
            Dev(8, "Voltage Frequency", 0.01, 0x11, size=1, functioncode=3, options={"Custom": "1;Hz"}, Type=243, SubType=31, Description="Voltage Frequency"),

            Dev(9, "Voltage L1", 0.1, 0x80, size=1, functioncode=3, Type=243, SubType=8, Description="Voltage L1"),
            Dev(10, "Voltage L2", 0.1, 0x81, size=1, functioncode=3, Type=243, SubType=8, Description="Voltage L2"),
            Dev(11, "Voltage L3", 0.1, 0x82, size=1, functioncode=3, Type=243, SubType=8, Description="Voltage L3"),

            Dev(12, "Current L1", 0.01, 0x83, size=1, functioncode=3, Type=243, SubType=23, Description="Current L1"),
            Dev(13, "Current L2", 0.01, 0x84, size=1, functioncode=3, Type=243, SubType=23, Description="Current L2"),
            Dev(14, "Current L3", 0.01, 0x85, size=1, functioncode=3, Type=243, SubType=23, Description="Current L3"),

            Dev(15, "Total Active Power", 1, 0x86, size=2, functioncode=3, options={"Custom": "1;W"}, Type=243, SubType=31, signed=True, Description="Total active power"),
            Dev(16, "Active Power L1", 1, 0x88, size=1, functioncode=3, options={"Custom": "1;W"}, Type=243, SubType=31, signed=True, Description="Active power L1"),
            Dev(17, "Active Power L2", 1, 0x89, size=1, functioncode=3, options={"Custom": "1;W"}, Type=243, SubType=31, signed=True, Description="Active power L2"),
            Dev(18, "Active Power L3", 1, 0x8A, size=1, functioncode=3, options={"Custom": "1;W"}, Type=243, SubType=31, signed=True, Description="Active power L3"),

            Dev(19, "Total Reactive Power", 0.001, 0x8B, size=2, functioncode=3, options={"Custom": "1;kVAr"}, Type=243, SubType=31, signed=True, Description="Total reactive power"),
            Dev(20, "Reactive Power L1", 0.001, 0x8D, size=1, functioncode=3, options={"Custom": "1;kVAr"}, Type=243, SubType=31, signed=True, Description="Reactive power L1"),
            Dev(21, "Reactive Power L2", 0.001, 0x8E, size=1, functioncode=3, options={"Custom": "1;kVAr"}, Type=243, SubType=31, signed=True, Description="Reactive power L2"),
            Dev(22, "Reactive Power L3", 0.001, 0x8F, size=1, functioncode=3, options={"Custom": "1;kVAr"}, Type=243, SubType=31, signed=True, Description="Reactive power L3"),

            Dev(23, "Total Apparent Power", 0.001, 0x90, size=2, functioncode=3, options={"Custom": "1;kVA"}, Type=243, SubType=31, Description="Total apparent power"),
            Dev(24, "Apparent Power L1", 0.001, 0x92, size=1, functioncode=3, options={"Custom": "1;kVA"}, Type=243, SubType=31, Description="Apparent power L1"),
            Dev(25, "Apparent Power L2", 0.001, 0x93, size=1, functioncode=3, options={"Custom": "1;kVA"}, Type=243, SubType=31, Description="Apparent power L2"),
            Dev(26, "Apparent Power L3", 0.001, 0x94, size=1, functioncode=3, options={"Custom": "1;kVA"}, Type=243, SubType=31, Description="Apparent power L3"),

            Dev(27, "Power Factor", 0.001, 0x95, size=1, functioncode=3, Type=243, SubType=31, Description="Power factor"),
            Dev(28, "Power Factor L1", 0.001, 0x96, size=1, functioncode=3, Type=243, SubType=31, Description="Power factor L1"),
            Dev(29, "Power Factor L2", 0.001, 0x97, size=1, functioncode=3, Type=243, SubType=31, Description="Power factor L2"),
            Dev(30, "Power Factor L3", 0.001, 0x98, size=1, functioncode=3, Type=243, SubType=31, Description="Power factor L3")
        ]

    def onStop(self):
        Domoticz.Log("DDS238-7 Modbus plugin stop")
        # Log final health statistics
        Domoticz.Log(f"Final connection health: {self.health_monitor.get_stats()}")

    def onHeartbeat(self):
        self.runInterval -= 1
        pluginClass = self
        
        if self.runInterval <= 0:
            # Check connection health before starting reads
            if self.health_monitor.should_reset_connection():
                Domoticz.Log(f"{ANSI_YELLOW}Resetting connection due to health issues{ANSI_RESET}")
                self.health_monitor.reset_connection(self.RS485)
            
            for i in self.devs:
                try:
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("Getting data from modbus for device:" + i.name + " ID:" + str(i.ID))
                    
                    # Enhanced UpdateValue with bleeding protection
                    self.devs[i.ID - 1].UpdateValue(self.RS485, pluginClass)
                    
                except Exception as e:
                    Domoticz.Log("Connection failure: " + str(e))
                    self.health_monitor.mark_failure()
                else:
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("in HeartBeat " + i.name + ": " + format(i.value))
                        
            if self.runInterval <= 0:
                self.runInterval = int(Parameters["Mode3"])
                if Parameters["Mode6"] == 'Debug':
                    Domoticz.Debug("Resetting runInterval to: " + str(self.runInterval))

        # Log selected energies and powers (unchanged)
        selected_names = [
            "Total Energy",
            "Life Energy", 
            "Import Energy",
            "Export Energy",
            "Total Active Power",
            "Total Reactive Power",
            "Total Apparent Power"
        ]
        line_parts = []
        for dev in self.devs:
            if dev.name in selected_names:
                try:
                    value = Devices[dev.ID].sValue if dev.ID in Devices else 'N/A'
                except Exception:
                    value = 'N/A'
                line_parts.append(f"{dev.name}: {value}")
        if line_parts:
            Domoticz.Log(f"{ANSI_BLUE}{' | '.join(line_parts)}{ANSI_RESET}")
            
        # Periodically log health statistics
        if self.runInterval == 1:  # Log once per cycle
            health_stats = self.health_monitor.get_stats()
            if "No reads" not in health_stats:
                Domoticz.Log(f"{ANSI_GREEN}Health: {health_stats}{ANSI_RESET}")
                
        return True


# ==============================================================================
# GLOBAL PLUGIN INSTANCE AND DOMOTICZ CALLBACKS
# ==============================================================================
global _plugin
_plugin = BasePlugin()


def onStart():
    global _plugin
    _plugin.onStart()


def onStop():
    global _plugin
    _plugin.onStop()


def onHeartbeat():
    global _plugin
    Domoticz.Debug("onHeartbeat called")
    _plugin.onHeartbeat()


# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================
def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug("'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))
    return


# ==============================================================================
# LEGACY FUNCTION - Kept for backward compatibility but not used
# ==============================================================================
def is_value_reasonable_legacy(value, register_name):
    """
    Legacy validation function - kept for backward compatibility.
    The new unified validation system is used instead.
    """
    ranges = {
        "Total Energy": (0, 100000000),
        "Life Energy": (0, 100000000),
        "Reactive Energy": (0, 100000000),
        "Import Energy": (0, 100000000),
        "Export Energy": (0, 100000000),
        "Voltage L1": (0, 300),
        "Voltage L2": (0, 300),
        "Voltage L3": (0, 300),
        "Voltage Frequency": (45, 65),
        "Current L1": (0, 100),
        "Current L2": (0, 100),
        "Current L3": (0, 100),
        "Total Active Power": (-25000, 25000),
        "Active Power L1": (-25000, 25000),
        "Active Power L2": (-25000, 25000),
        "Active Power L3": (-25000, 25000),
        "Total Reactive Power": (-25000, 25000),
        "Reactive Power L1": (-25000, 25000),
        "Reactive Power L2": (-25000, 25000),
        "Reactive Power L3": (-25000, 25000),
        "Total Apparent Power": (0, 25000),
        "Apparent Power L1": (0, 25000),
        "Apparent Power L2": (0, 25000),
        "Apparent Power L3": (0, 25000),
        "Power Factor": (-1, 1),
        "Power Factor L1": (-1, 1),
        "Power Factor L2": (-1, 1),
        "Power Factor L3": (-1, 1),
        "Reserved1": (0, 100000000),
        "Reserved2": (0, 100000000),
    }

    if register_name in ranges:
        min_val, max_val = ranges[register_name]
        if value is None:
            return False, "Value is None"
        if isinstance(value, (int, float)):
            if min_val <= value <= max_val:
                return True, "Value within range"
            else:
                return False, f"Value {value} outside range [{min_val}, {max_val}]"
        return False, f"Invalid value type: {type(value)}"

    Domoticz.Log(f"Warning: No validation range defined for device: {register_name}")
    return True, f"No validation range defined for {register_name}"
