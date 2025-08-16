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
<plugin key="DDS238-7" name="DDS238-7 modbus" version="0.9.0" author="voyo@no-ip.pl">
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
                <option label="False" value="Normal" default="true" />
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

# ==============================================================================
# LOGGER CLASS - Unified logging without threading
# ==============================================================================
class Logger:
    """Unified logger with consistent formatting and debug mode support"""
    
    # ANSI color codes
    BLUE = '\033[94m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RESET = '\033[0m'
    
    def __init__(self):
        self.debug_mode = False
    
    def set_debug_mode(self, enabled):
        """Enable or disable debug mode"""
        self.debug_mode = enabled
    
    def _log(self, message, level="INFO", color=None):
        """Internal logging method"""
        if self.debug_mode and color:
            formatted_msg = f"{color}{message}{self.RESET}"
        else:
            formatted_msg = message
        
        if level == "ERROR":
            Domoticz.Error(formatted_msg)
        elif level == "DEBUG":
            if self.debug_mode:
                Domoticz.Debug(formatted_msg)
        else:
            Domoticz.Log(formatted_msg)
    
    def info(self, message):
        """Log info message"""
        self._log(message, "INFO")
    
    def error(self, message):
        """Log error message"""
        self._log(message, "ERROR", self.RED if self.debug_mode else None)
    
    def debug(self, message):
        """Log debug message (only in debug mode)"""
        if self.debug_mode:
            self._log(message, "DEBUG")
    
    def warning(self, message):
        """Log warning message"""
        self._log(message, "INFO", self.YELLOW if self.debug_mode else None)
    
    def success(self, message):
        """Log success message"""
        self._log(message, "INFO", self.GREEN if self.debug_mode else None)
    
    def status(self, message):
        """Log status message (blue in debug mode)"""
        self._log(message, "INFO", self.BLUE if self.debug_mode else None)

# Global logger instance
logger = Logger()

# ==============================================================================
# VALIDATION CONFIGURATION
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
# CONNECTION HEALTH MONITOR - Without threading
# ==============================================================================
class ConnectionHealthMonitor:
    """Monitor connection health and handle recovery"""
    
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
        logger.warning(f"Register bleeding detected (event #{self.bleeding_count})")
        
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
                logger.warning("Connection reset due to consecutive failures")
                return True
        except Exception as e:
            logger.error(f"Failed to reset connection: {str(e)}")
            
        return False
        
    def get_stats(self):
        """Get connection health statistics"""
        if self.total_reads == 0:
            return "No reads performed yet"
        
        failure_rate = (self.consecutive_failures / max(self.total_reads, 1)) * 100
        return f"Failures: {self.consecutive_failures}, Bleeding: {self.bleeding_count}, Rate: {failure_rate:.1f}%"

# ==============================================================================
# AVERAGE CLASS - Without threading
# ==============================================================================
class Average:
    """Calculate running average with memory limits"""
    
    def __init__(self, max_samples=30):
        self.samples = []
        self.max_samples = min(100, max(1, max_samples))  # Limit between 1-100

    def set_max_samples(self, max_samples):
        """Set maximum number of samples to keep"""
        self.max_samples = min(100, max(1, max_samples))

    def update(self, new_value, scale=0):
        """Update with validation - reject NaN, infinity, or invalid values"""
        if new_value is None:
            logger.debug("Average.update: Rejecting None value")
            return
            
        if not isinstance(new_value, (int, float)):
            logger.debug(f"Average.update: Rejecting non-numeric value: {new_value} ({type(new_value)})")
            return
            
        # Check for NaN and infinity
        if math.isnan(new_value) or math.isinf(new_value):
            logger.debug(f"Average.update: Rejecting invalid value: {new_value}")
            return
            
        scaled_value = new_value * (10 ** scale)
        
        # Additional sanity check for extremely large values
        if abs(scaled_value) > 1e15:
            logger.debug(f"Average.update: Rejecting extremely large value: {scaled_value}")
            return
            
        self.samples.append(scaled_value)
        # Prevent memory leak - enforce max_samples limit
        while len(self.samples) > self.max_samples:
            self.samples.pop(0)
        
        logger.debug(f"Average: {self.get()} - {len(self.samples)} values")

    def get(self):
        """Get current average value"""
        if len(self.samples) == 0:
            return 0
        return int(sum(self.samples) / len(self.samples))

# ==============================================================================
# VALIDATION FUNCTIONS
# ==============================================================================
def is_value_reasonable(value, register_name, multiplier=None, is_raw=False):
    """
    Unified validation function for both raw and calculated values.
    
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
        logger.debug(f"No validation range defined for device: {register_name}")
        return True, f"No validation range defined for {register_name}"
    
    # Get final value range
    final_min, final_max = VALIDATION_RANGES[register_name]
    
    if is_raw and multiplier is not None:
        # Calculate raw range using device multiplier
        if multiplier == 0:
            return True, "Multiplier is zero - cannot validate raw value"
            
        # Handle signed ranges properly
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
# BLEEDING DETECTOR - Without threading
# ==============================================================================
class BleedingDetector:
    """Detect potential register bleeding"""
    
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
                    
                logger.warning(f"Bleeding suspected for {device_name}: {self.identical_threshold} identical values: {raw_value}")
                return True
                
        return False

# ==============================================================================
# DEVICE CLASS - Refactored with separated concerns
# ==============================================================================
class Dev:
    """Device representation with Modbus register mapping"""
    
    def __init__(self, device_id, name, multiplier, register, size, function_code=3, 
                 options=None, used=1, signed=False, description=None, 
                 type_name=None, type_id=0, sub_type=0, switch_type=0):
        self.id = device_id
        self.name = name
        self.type_name = type_name if type_name is not None else ""
        self.type_id = type_id
        self.sub_type = sub_type
        self.switch_type = switch_type
        self.multiplier = multiplier
        self.value = 0
        self.signed = signed
        self.register = register
        self.size = size
        self.function_code = function_code
        self.options = options if options is not None else None
        self.used = used
        self.description = description if description is not None else ""
        self.last_valid_value = None
        
        self._create_domoticz_device()
    
    def _create_domoticz_device(self):
        """Create Domoticz device if it doesn't exist"""
        if self.id not in Devices:
            if self.type_name != "":
                logger.debug(f"Adding device: {self.name} ID:{self.id} Type:{self.type_name} Desc:{self.description}")
                Domoticz.Device(Name=self.name, Unit=self.id, TypeName=self.type_name, 
                              Used=self.used, Options=self.options, Description=self.description).Create()
            else:
                logger.debug(f"Adding device: {self.name} Type:{self.type_id} SubType:{self.sub_type} SwitchType:{self.switch_type}")
                Domoticz.Device(Name=self.name, Unit=self.id, Type=self.type_id, 
                              Subtype=self.sub_type, Switchtype=self.switch_type, 
                              Used=self.used, Options=self.options, Description=self.description).Create()
    
    def read_register(self, rs485_connection):
        """Read raw value from Modbus register"""
        try:
            if self.size == 0:  # Virtual device
                return 0
            elif self.size == 1:
                registers = rs485_connection.read_holding_registers(self.register, self.size)
                if registers is None or len(registers) == 0:
                    raise Exception(f"Failed to read register {self.register}")
                
                value = registers[0]
                if self.signed and value > 32767:
                    value -= 65536
                return value
                
            elif self.size == 2:
                registers = rs485_connection.read_holding_registers(self.register, self.size)
                if registers is None or len(registers) < 2:
                    raise Exception(f"Failed to read register {self.register} (size=2)")
                
                value = (registers[0] << 16) + registers[1]
                if self.signed and value > 2147483647:
                    value -= 4294967296
                return value
                
        except Exception as e:
            logger.debug(f"Error reading register {self.register} for {self.name}: {str(e)}")
            raise
    
    def validate_raw_value(self, raw_value):
        """Validate raw register value"""
        is_valid, reason = is_value_reasonable(raw_value, self.name, self.multiplier, is_raw=True)
        if not is_valid:
            logger.debug(f"Invalid raw value for {self.name}: {raw_value} - {reason}")
        return is_valid
    
    def calculate_final_value(self, raw_value):
        """Calculate final value from raw register value"""
        return raw_value * self.multiplier
    
    def validate_final_value(self, final_value):
        """Validate calculated final value"""
        is_valid, reason = is_value_reasonable(final_value, self.name, self.multiplier, is_raw=False)
        if not is_valid:
            logger.debug(f"Invalid final value for {self.name}: {final_value} - {reason}")
        return is_valid
    
    def update_value(self, rs485_connection, plugin_instance):
        """Update device value - main entry point"""
        try:
            # Step 1: Read raw value
            raw_value = self.read_register(rs485_connection)
            plugin_instance.health_monitor.mark_success()
            
            # Step 2: Validate raw value
            if not self.validate_raw_value(raw_value):
                plugin_instance.health_monitor.mark_bleeding_detected()
                return
            
            # Step 3: Check for bleeding
            if plugin_instance.bleeding_detector.check_bleeding(self.name, raw_value):
                plugin_instance.health_monitor.mark_bleeding_detected()
                return
            
            # Step 4: Calculate final value
            final_value = self.calculate_final_value(raw_value)
            
            # Step 5: Validate final value
            if not self.validate_final_value(final_value):
                return
            
            # Step 6: Store as last valid value
            self.last_valid_value = final_value
            
            # Step 7: Update Domoticz device
            self._update_domoticz_device(final_value, plugin_instance)
            
        except Exception as e:
            logger.debug(f"Error updating {self.name}: {str(e)}")
            plugin_instance.health_monitor.mark_failure()
            
            # Check if connection should be reset
            if plugin_instance.health_monitor.should_reset_connection():
                logger.warning("Attempting connection reset due to consecutive failures")
                plugin_instance.health_monitor.reset_connection(rs485_connection)
    
    def _update_domoticz_device(self, payload, plugin_instance):
        """Update Domoticz device with new value"""
        logger.debug(f"Updating {self.name} with value: {payload}")
        
        # Update plugin averages for power measurements
        if self.type_id == 243:
            if self.name == "Total Active Power":
                plugin_instance.active_power.update(int(payload))
                if payload < 0:
                    plugin_instance.reverse_power.update(abs(int(payload)))
                    plugin_instance.forward_power.update(0)
                else:
                    plugin_instance.reverse_power.update(0)
                    plugin_instance.forward_power.update(int(payload))
            elif "Voltage" in self.name:
                plugin_instance.voltage = int(payload)
            elif "Current" in self.name:
                plugin_instance.current = int(payload)
            elif "Apparent Power" in self.name:
                plugin_instance.apparent_power = int(payload)
            elif "Total Reactive Power" in self.name:
                reactive_energy = int(payload * 1000)
                logger.debug(f"Reactive energy: {reactive_energy}")
                plugin_instance.reactive_power.update(reactive_energy)
        
        # Format value for P1 Smart Meter devices
        if self.type_id == 250 and self.sub_type in (1, 6):
            s_value = self._format_p1_smart_meter_value(payload, plugin_instance)
        else:
            s_value = str(payload)
        
        # Update Domoticz device
        if self.id in Devices:
            Devices[self.id].Update(nValue=0, sValue=s_value)
            logger.debug(f"Domoticz device {self.id} updated with value: {s_value}")
        else:
            logger.error(f"Device ID {self.id} not found in Devices")
    
    def _format_p1_smart_meter_value(self, payload, plugin_instance):
        """Format value for P1 Smart Meter device type"""
        usage1 = usage2 = return1 = return2 = cons = prod = "0"
        
        if "Total Energy" in self.name:
            plugin_instance.total_energy = int(payload)
            usage1 = str(payload)
            cons = str(int(plugin_instance.active_power.get()))
            
        elif "Import Energy" in self.name:
            usage1 = str(payload)
            plugin_instance.import_energy = payload
            cons = str(plugin_instance.forward_power.get())
            
        elif "Export Energy" in self.name:
            return1 = str(payload)
            plugin_instance.export_energy = payload
            prod = str(abs(plugin_instance.reverse_power.get()))
            
        elif "Life Energy" in self.name:
            # Virtual device - combines import and export statistics
            usage1 = str(int(plugin_instance.import_energy))
            return1 = str(int(plugin_instance.export_energy))
            cons = str(int(plugin_instance.forward_power.get()))
            prod = str(int(plugin_instance.reverse_power.get()))
            
        elif "Reactive Energy" in self.name:
            usage1 = str(payload)
            cons = str(abs(int(plugin_instance.reactive_power.get())))
            logger.debug(f"Reactive energy consumption: {cons}")
        
        return f"{usage1};{usage2};{return1};{return2};{cons};{prod}"

# ==============================================================================
# MAIN PLUGIN CLASS
# ==============================================================================
class BasePlugin:
    """Main plugin class for DDS238-7 Modbus meter"""
    
    def __init__(self):
        self.run_interval = 1
        self.rs485_connection = None
        
        # Power averages
        self.active_power = Average()
        self.reactive_power = Average()
        self.reverse_reactive_power = Average()
        self.forward_reactive_power = Average()
        self.forward_power = Average()
        self.reverse_power = Average()
        
        # Latest values (not averages)
        self.voltage = 0
        self.current = 0
        self.apparent_power = 0
        self.total_energy = 0
        self.import_energy = 0
        self.export_energy = 0
        
        self.devices = []
        
        # Health monitoring and bleeding detection
        self.health_monitor = ConnectionHealthMonitor()
        self.bleeding_detector = BleedingDetector()
    
    def onStart(self):
        """Initialize plugin on start"""
        global logger
        
        # Setup debug mode
        if Parameters["Mode6"] == "Debug":
            Domoticz.Debugging(1)
            logger.set_debug_mode(True)
            self._dump_config_to_log()
            logger.debug("Debug mode enabled")
        else:
            Domoticz.Debugging(0)
            logger.set_debug_mode(False)
        
        logger.info(f"DDS238-7 Modbus plugin start, mode: {Parameters['Mode4']}")
        
        # Initialize Modbus connection
        if not self._initialize_connection():
            logger.error("Failed to initialize Modbus connection")
            return
        
        # Initialize devices
        self._initialize_devices()
    
    def _initialize_connection(self):
        """Initialize Modbus connection (RTU or TCP)"""
        device_id = int(Parameters["Mode2"])
        
        try:
            if Parameters["Mode4"] == "RTU":
                self.rs485_connection = minimalmodbus.Instrument(Parameters["SerialPort"], device_id)
                self.rs485_connection.serial.baudrate = Parameters["Mode1"]
                self.rs485_connection.serial.bytesize = 8
                self.rs485_connection.serial.parity = minimalmodbus.serial.PARITY_NONE
                self.rs485_connection.serial.stopbits = 1
                self.rs485_connection.serial.timeout = 1
                self.rs485_connection.debug = False
                self.rs485_connection.mode = minimalmodbus.MODE_RTU
                self.rs485_connection.MyMode = 'minimalmodbus'
                logger.info(f"RTU connection initialized on {Parameters['SerialPort']}")
                
            elif Parameters["Mode4"] == "TCP":
                logger.debug(f"Connecting to {Parameters['Address']}:{Parameters['Port']} unit ID: {device_id}")
                self.rs485_connection = ModbusClient(
                    host=Parameters["Address"],
                    port=int(Parameters["Port"]),
                    unit_id=device_id,
                    timeout=2
                )
                if self.rs485_connection:
                    self.rs485_connection.MyMode = 'pymodbus'
                    logger.info(f"TCP connection initialized to {Parameters['Address']}:{Parameters['Port']}")
                else:
                    return False
            else:
                logger.error(f"Unknown Modbus type: {Parameters['Mode4']}")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Connection initialization failed: {str(e)}")
            return False
    
    def _initialize_devices(self):
        """Initialize device list"""
        self.devices = [
            # Energy measurements
            Dev(1, "Total Energy", 10, 0x00, size=2, function_code=3, type_id=250, sub_type=1, description="Total energy balance"),
            Dev(2, "Life Energy", 10, 0x00, size=0, function_code=3, type_id=250, sub_type=1, description="Total energy flow (virtual)"),
            Dev(3, "Reserved1", 10, 0x02, size=0, function_code=3, used=0, type_id=250, sub_type=1, description="Reserved1"),
            Dev(4, "Reactive Energy", 10, 0x02, size=2, function_code=3, options={"Custom": "1;kVArh"}, type_id=250, sub_type=6, description="Reactive energy"),
            Dev(5, "Reserved2", 10, 0x06, size=0, function_code=3, used=0, type_id=250, sub_type=1, description="Reserved2"),
            Dev(6, "Import Energy", 10, 0x08, size=2, function_code=3, type_id=250, sub_type=1, description="Forward energy"),
            Dev(7, "Export Energy", 10, 0xA, size=2, function_code=3, type_id=250, sub_type=1, description="Reverse energy"),
            
            # Frequency
            Dev(8, "Voltage Frequency", 0.01, 0x11, size=1, function_code=3, options={"Custom": "1;Hz"}, type_id=243, sub_type=31, description="Voltage Frequency"),
            
            # Voltage measurements
            Dev(9, "Voltage L1", 0.1, 0x80, size=1, function_code=3, type_id=243, sub_type=8, description="Voltage L1"),
            Dev(10, "Voltage L2", 0.1, 0x81, size=1, function_code=3, type_id=243, sub_type=8, description="Voltage L2"),
            Dev(11, "Voltage L3", 0.1, 0x82, size=1, function_code=3, type_id=243, sub_type=8, description="Voltage L3"),
            
            # Current measurements
            Dev(12, "Current L1", 0.01, 0x83, size=1, function_code=3, type_id=243, sub_type=23, description="Current L1"),
            Dev(13, "Current L2", 0.01, 0x84, size=1, function_code=3, type_id=243, sub_type=23, description="Current L2"),
            Dev(14, "Current L3", 0.01, 0x85, size=1, function_code=3, type_id=243, sub_type=23, description="Current L3"),
            
            # Active power measurements
            Dev(15, "Total Active Power", 1, 0x86, size=2, function_code=3, options={"Custom": "1;W"}, type_id=243, sub_type=31, signed=True, description="Total active power"),
            Dev(16, "Active Power L1", 1, 0x88, size=1, function_code=3, options={"Custom": "1;W"}, type_id=243, sub_type=31, signed=True, description="Active power L1"),
            Dev(17, "Active Power L2", 1, 0x89, size=1, function_code=3, options={"Custom": "1;W"}, type_id=243, sub_type=31, signed=True, description="Active power L2"),
            Dev(18, "Active Power L3", 1, 0x8A, size=1, function_code=3, options={"Custom": "1;W"}, type_id=243, sub_type=31, signed=True, description="Active power L3"),
            
            # Reactive power measurements
            Dev(19, "Total Reactive Power", 0.001, 0x8B, size=2, function_code=3, options={"Custom": "1;kVAr"}, type_id=243, sub_type=31, signed=True, description="Total reactive power"),
            Dev(20, "Reactive Power L1", 0.001, 0x8D, size=1, function_code=3, options={"Custom": "1;kVAr"}, type_id=243, sub_type=31, signed=True, description="Reactive power L1"),
            Dev(21, "Reactive Power L2", 0.001, 0x8E, size=1, function_code=3, options={"Custom": "1;kVAr"}, type_id=243, sub_type=31, signed=True, description="Reactive power L2"),
            Dev(22, "Reactive Power L3", 0.001, 0x8F, size=1, function_code=3, options={"Custom": "1;kVAr"}, type_id=243, sub_type=31, signed=True, description="Reactive power L3"),
            
            # Apparent power measurements
            Dev(23, "Total Apparent Power", 0.001, 0x90, size=2, function_code=3, options={"Custom": "1;kVA"}, type_id=243, sub_type=31, description="Total apparent power"),
            Dev(24, "Apparent Power L1", 0.001, 0x92, size=1, function_code=3, options={"Custom": "1;kVA"}, type_id=243, sub_type=31, description="Apparent power L1"),
            Dev(25, "Apparent Power L2", 0.001, 0x93, size=1, function_code=3, options={"Custom": "1;kVA"}, type_id=243, sub_type=31, description="Apparent power L2"),
            Dev(26, "Apparent Power L3", 0.001, 0x94, size=1, function_code=3, options={"Custom": "1;kVA"}, type_id=243, sub_type=31, description="Apparent power L3"),
            
            # Power factor measurements
            Dev(27, "Power Factor", 0.001, 0x95, size=1, function_code=3, type_id=243, sub_type=31, description="Power factor"),
            Dev(28, "Power Factor L1", 0.001, 0x96, size=1, function_code=3, type_id=243, sub_type=31, description="Power factor L1"),
            Dev(29, "Power Factor L2", 0.001, 0x97, size=1, function_code=3, type_id=243, sub_type=31, description="Power factor L2"),
            Dev(30, "Power Factor L3", 0.001, 0x98, size=1, function_code=3, type_id=243, sub_type=31, description="Power factor L3")
        ]
        
        logger.info(f"Initialized {len(self.devices)} devices")
    
    def onStop(self):
        """Clean up on plugin stop"""
        logger.info("DDS238-7 Modbus plugin stopping")
        
        # Log final health statistics
        logger.status(f"Final connection health: {self.health_monitor.get_stats()}")
        
        # Close connection if exists
        try:
            if self.rs485_connection and hasattr(self.rs485_connection, 'close'):
                self.rs485_connection.close()
                logger.info("Modbus connection closed")
        except Exception as e:
            logger.error(f"Error closing connection: {str(e)}")
    
    def onHeartbeat(self):
        """Called every 10 seconds by Domoticz"""
        self.run_interval -= 1
        
        if self.run_interval <= 0:
            # Reset interval
            self.run_interval = int(Parameters["Mode3"])
            
            # Check connection
            if not self.rs485_connection:
                logger.error("No Modbus connection available")
                if not self._initialize_connection():
                    return
            
            # Update all devices
            self._update_all_devices()
            
            # Log status summary
            self._log_status_summary()
            
            # Check connection health
            self._check_connection_health()
    
    def _update_all_devices(self):
        """Update all device values"""
        for device in self.devices:
            try:
                device.update_value(self.rs485_connection, self)
            except Exception as e:
                logger.error(f"Failed to update device {device.name}: {str(e)}")
                self.health_monitor.mark_failure()
    
    def _log_status_summary(self):
        """Log summary of key values (only in debug mode)"""
        if not logger.debug_mode:
            return
            
        selected_names = [
            "Total Energy",
            "Life Energy",
            "Import Energy",
            "Export Energy",
            "Total Active Power",
            "Total Reactive Power",
            "Total Apparent Power"
        ]
        
        summary_parts = []
        for device in self.devices:
            if device.name in selected_names:
                try:
                    value = Devices[device.id].sValue if device.id in Devices else 'N/A'
                except Exception:
                    value = 'N/A'
                summary_parts.append(f"{device.name}: {value}")
        
        if summary_parts:
            logger.status(" | ".join(summary_parts))
    
    def _check_connection_health(self):
        """Check and handle connection health"""
        if self.health_monitor.should_reset_connection():
            logger.warning("Resetting connection due to health issues")
            if self.health_monitor.reset_connection(self.rs485_connection):
                # Try to reinitialize connection
                self._initialize_connection()
        
        # Log health statistics periodically
        if self.run_interval == 1:
            health_stats = self.health_monitor.get_stats()
            if "No reads" not in health_stats:
                logger.success(f"Health: {health_stats}")
    
    def _dump_config_to_log(self):
        """Dump configuration to log for debugging"""
        logger.debug("=== Configuration ===")
        for key in Parameters:
            if Parameters[key] != "":
                logger.debug(f"{key}: {Parameters[key]}")
        
        logger.debug(f"Device count: {len(Devices)}")
        for device_id in Devices:
            device = Devices[device_id]
            logger.debug(f"Device {device_id}: {device.Name} - Value: {device.sValue}")

# ==============================================================================
# GLOBAL PLUGIN INSTANCE AND DOMOTICZ CALLBACKS
# ==============================================================================
global _plugin
_plugin = BasePlugin()

def onStart():
    """Domoticz callback: Plugin start"""
    global _plugin
    _plugin.onStart()

def onStop():
    """Domoticz callback: Plugin stop"""
    global _plugin
    _plugin.onStop()

def onHeartbeat():
    """Domoticz callback: Heartbeat"""
    global _plugin
    logger.debug("onHeartbeat called")
    _plugin.onHeartbeat()
