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
<plugin key="DDS238-7" name="DDS238-7 modbus" version="1.0" author="voyo@no-ip.pl">
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
import Domoticz
from time import sleep

# for TCP modbus connection
from pyModbusTCP.client import ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder


# Domoticz shows graphs with intervals of 5 minutes.
# When collecting information from the inverter more frequently than that, then it makes no sense to only show the last value.
#
# The Average class can be used to calculate the average value based on a sliding window of samples.
# The number of samples stored depends on the interval used to collect the value from the inverter itself.
#
# borrowed from https://github.com/xbeaudouin/domoticz-ds238-modbus-tcp
class Average:
    def __init__(self):
        self.samples = []
        self.max_samples = 30

    def set_max_samples(self, max):
        self.max_samples = max
        if self.max_samples < 1:
            self.max_samples = 1

    def update(self, new_value, scale = 0):
        self.samples.append(new_value * (10 ** scale))
        while (len(self.samples) > self.max_samples):
            del self.samples[0]
        Domoticz.Debug("Average: {} - {} values".format(self.get(), len(self.samples)))

    def get(self):
        if len(self.samples) == 0:
            return 0
        return int(sum(self.samples) / len(self.samples))



class Dev:
  def __init__(self,ID,name,multipler ,register,size,functioncode: int = 3,options=None, Used: int = 1, signed: bool = False, Description=None, TypeName=None,Type: int = 0, SubType:int = 0 , SwitchType:int = 0  ):
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
        self.Used=Used
        self.Description = Description if Description is not None else ""
        if self.ID not in Devices:
        
            if self.TypeName != "":
                Domoticz.Log("Adding device: "+self.name+" "+str(self.ID)+" "+self.TypeName+"  Description: "+str(self.Description))        
                Domoticz.Device(Name=self.name, Unit=self.ID, TypeName=self.TypeName,Used=self.Used,Options=self.options,Description=self.Description).Create()
            else:
                Domoticz.Device(Name=self.name, Unit=self.ID,Type=self.Type, Subtype=self.SubType, Switchtype=self.SwitchType, Used=self.Used,Options=self.options,Description=self.Description).Create()
                Domoticz.Log("Adding device: name"+ self.name+", Type: "+str(self.Type)+" SubType: "+str(self.SubType)+" SwitchType: "+str(self.SwitchType))
                      

  def UpdateValue(self, RS485, outerClass):
    try:
        if self.size == 1:
            if RS485.MyMode == 'pymodbus':
                while True:
                    try:
                        # Read the registers
                        registers = RS485.read_holding_registers(self.register, self.size)
                        if registers and not registers.isError():
                            # Convert using the new method
                            value = RS485.convert_from_registers(
                                registers.registers,
                                data_type=RS485.DATATYPE.INT16,  # For 16-bit integers
                                word_order='big'
                            )
                            payload = value * self.multipler  # decimal places
                            
                            # Validate the value
                            is_valid, reason = is_value_reasonable(payload, self.name)
                            if not is_valid:
                                Domoticz.Log(f"Rejecting invalid value: {reason}")
                                return
                                
                        else:
                            raise Exception("Failed to read registers")
                            
                    except Exception as e:
                        Domoticz.Log("Modbus connection failure: "+str(e))
                        Domoticz.Log("retry updating register in 2 s")
                        sleep(2.0)
                        continue
                    break
                    
        elif self.size == 2:
            if RS485.MyMode == 'pymodbus':
                while True:
                    try:
                        # Read the registers
                        registers = RS485.read_holding_registers(self.register, self.size)
                        if registers and not registers.isError():
                            # Convert using the new method
                            value = RS485.convert_from_registers(
                                registers.registers,
                                data_type=RS485.DATATYPE.INT32,  # For 32-bit integers
                                word_order='big'
                            )
                            payload = value * self.multipler
                            
                            # Validate the value
                            is_valid, reason = is_value_reasonable(payload, self.name)
                            if not is_valid:
                                Domoticz.Log(f"Rejecting invalid value: {reason}")
                                return
                                
                        else:
                            raise Exception("Failed to read registers")
                            
                    except Exception as e:
                        Domoticz.Log("Modbus connection failure: "+str(e))
                        Domoticz.Log("retry updating register in 2 s")
                        sleep(2.0)
                        continue
                    break

        # If we get here, the value is valid
        data = payload
        
        # Add debug logging
        if Parameters["Mode6"] == 'Debug':
            Domoticz.Debug(f"Valid value for {self.name}: {data} (raw: {value}, multiplier: {self.multipler})")
        
        # Update Domoticz
        if self.Type == 243 and (self.name == "TotalActivePower"):
            outerClass.active_power.update(int(data))
            if data < 0:
                outerClass.reverse_power.update(int(data))
                outerClass.forward_power.update(0)
            else:
                outerClass.reverse_power.update(0)
                outerClass.forward_power.update(int(data))
                
    except Exception as e:
        Domoticz.Error(f"Error updating value for {self.name}: {str(e)}")


class BasePlugin:
    def __init__(self):
        self.runInterval = 1
        self.RS485 = ""
        # Active power for last 5 minutes
        self.active_power = Average()
        # Reactive power for last 5 minutes
        self.reactive_power = Average()
        # Forward power for last 5 minutes
        self.forward_power = Average()
        self.reverse_power = Average()
        self.devs = []  # Initialize empty device list
        return

    def onStart(self):
        if Parameters["Mode6"] == "Debug":
           Domoticz.Debugging(1)
           DumpConfigToLog()

#           Domoticz.Log("Debugger started, use 'telnet 0.0.0.0 4444' to connect")
           Domoticz.Debug("Debugging enabled")
#           import rpdb
#           rpdb.set_trace()

        Domoticz.Log("DDS238-7 Modbus plugin start, mode: "+Parameters["Mode4"])
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
            Domoticz.Debug("Using pymodbus, connecting to "+Parameters["Address"]+":"+Parameters["Port"]+" unit ID: "+ str(DeviceID))
            try: 
                self.RS485 = ModbusClient(
                    host=Parameters["Address"], 
                    port=int(Parameters["Port"]), 
                    unit_id=DeviceID, 
                    auto_open=True, 
                    auto_close=False,  # Keep connection open
                    timeout=2,
                    retry_on_empty=True,
                    retries=3,         # Number of retries
                    reconnect_delay=1  # Delay between reconnection attempts
                )
                # Only set MyMode if connection was successful
                if self.RS485:
                    self.RS485.MyMode = 'pymodbus'
                else:
                    Domoticz.Error("Failed to create ModbusClient - check your connection parameters")
                    return
            except Exception as e: 
                Domoticz.Error(f"pyModbus connection failure: {str(e)}")
                return  # Exit onStart if we can't establish connection

        # Only initialize devices if we have a valid connection
        if not self.RS485 or isinstance(self.RS485, str):
            Domoticz.Error("No valid Modbus connection established")
            return

        # Initialize devices list
        self.devs = [
                Dev(1,"TotalEnergy",10,0x00,size=2,functioncode=3,Type=250,SubType=1,Description="Total energy balance"),
                Dev(2,"LifeEnergy",1,0x00,size=2,functioncode=3,Type=250,SubType=1,Description="Total energy flow"),
                Dev(3,"Reserved1",1,0x02,size=2,functioncode=3,Used=0,Type=250,SubType=1,Description="Reserved1"),
                Dev(4,"ReactiveEnergy",1,0x04,size=2,functioncode=3,options={"Custom":"1;kVArh"},Type=250,SubType=6,Description="Reactive energy"), # "Custom":"1;kVArh
                Dev(5,"Reserved2",1,0x06,size=2,functioncode=3,Used=0,Type=250,SubType=1,Description="Reserved2"),
                Dev(6,"Import Energy",1,0x08,size=2,functioncode=3,Type=250,SubType=1,Description="Reverse energy"),
                Dev(7,"Export Energy",1,0xA,size=2,functioncode=3,Type=250,SubType=1,Description="Forward energy"),
                Dev(8,"Voltage Frequency",0.01,0x11,size=1,functioncode=3,options={"Custom":"1;Hz"},Type=243,SubType=31,Description="Voltage Frequency"),
             
                Dev(9,"Voltage_L1",0.1,0x80,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L1"),
                Dev(10,"Voltage_L2",0.1,0x81,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L2"),
                Dev(11,"Voltage_L3",0.1,0x82,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L3"),
             
                Dev(12,"Current_L1",0.01,0x83,size=1,functioncode=3,Type=243,SubType=23,Description="Current L1"),
                Dev(13,"Current_L2",0.01,0x84,size=1,functioncode=3,Type=243,SubType=23,Description="Current L2"),
                Dev(14,"Current_L3",0.01,0x85,size=1,functioncode=3,Type=243,SubType=23,Description="Current L3"),
           
                Dev(15,"TotalActivePower",1,0x86,size=2,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True, Description="Total active power"),
                Dev(16,"ActivePower_L1",1,0x88,size=1,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True,Description="Active power L1"),
                Dev(17,"ActivePower_L2",1,0x89,size=1,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True,Description="Active power L2"),
                Dev(18,"ActivePower_L3",1,0x8A,size=1,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True,Description="Active power L3"),
           
                Dev(19,"TotalReactivePower",0.001,0x8B,size=2,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Total reactive power"), # "Custom":"1;kVAr
                Dev(20,"ReactivePower_L1",0.001,0x8D,size=1,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Reactive power L1"), # "Custom":"1;kVAr
                Dev(21,"ReactivePower_L2",0.001,0x8E,size=1,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Reactive power L2"), # "Custom":"1;kVAr
                Dev(22,"ReactivePower_L3",0.001,0x8F,size=1,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Reactive power L3"), # "Custom":"1;kVAr
           
                Dev(23,"TotalApparentPower",0.001,0x90,size=2,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Total apparent power"), # "Custom":"1;kVA
                Dev(24,"ApparentPower_L1",0.001,0x92,size=1,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Apparent power L1"), # "Custom":"1;kVA
                Dev(25,"ApparentPower_L2",0.001,0x93,size=1,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Apparent power L2"), # "Custom":"1;kVA
                Dev(26,"ApparentPower_L3",0.001,0x94,size=1,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Apparent power L3"), # "Custom":"1;kVA
           
                Dev(27,"PowerFactor",0.001,0x95,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor"),
                Dev(28,"PowerFactor_L1",0.001,0x96,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor L1"),
                Dev(29,"PowerFactor_L2",0.001,0x97,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor L2"),
                Dev(30,"PowerFactor_L3",0.001,0x98,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor L3")

                ]
                
    def onStop(self):
        Domoticz.Log("DDS238-7 Modbus plugin stop")

    def onHeartbeat(self):
        self.runInterval -=1;
        pluginClass = self
        if self.runInterval <= 0:
            for i in self.devs:
                try:
                         # Get data from modbus
                        if Parameters["Mode6"] == 'Debug':
                            Domoticz.Debug("Getting data from modbus for device:"+i.name+" ID:"+str(i.ID))
                        self.devs[i.ID-1].UpdateValue(self.RS485,pluginClass)
                except Exception as e:
                        Domoticz.Log("Connection failure: "+str(e))
                else:
                    if Parameters["Mode6"] == 'Debug':
                            Domoticz.Debug("in HeartBeat "+i.name+": "+format(i.value))
                if self.runInterval <= 0:
                    self.runInterval = int(Parameters["Mode3"])
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("Resetting runInterval to: "+str(self.runInterval))
        return True                


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

# Generic helper functions
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

def is_value_reasonable(value, register_name):
    """
    Check if value is within reasonable range for given register type.
    Returns (bool, str) - (is_valid, reason_if_invalid)
    """
    # Define reasonable ranges for each register type
    valid_ranges = {
        # Voltage should be between 180-260V for normal grid power
        "Voltage": (170, 270),
        "Voltage_L1": (170, 270),
        "Voltage_L2": (170, 270),
        "Voltage_L3": (170, 270),
        
        # Current depends on your installation, adjust as needed
        "Current": (0, 100),  # 0-100A
        "Current_L1": (0, 100),
        "Current_L2": (0, 100),
        "Current_L3": (0, 100),
        
        # Power Factor should be between 0 and 1
        "PowerFactor": (0, 1),
        "PowerFactor_L1": (0, 1),
        "PowerFactor_L2": (0, 1),
        "PowerFactor_L3": (0, 1),
        
        # Frequency should be close to 50Hz (or 60Hz depending on your region)
        "Frequency": (45, 55),  # Adjust to (55, 65) for 60Hz systems
        
        # Power readings (in Watts)
        "TotalActivePower": (-25000, 25000),  # ±25kW, adjust based on your installation
        "ActivePower_L1": (-25000, 25000),
        "ActivePower_L2": (-25000, 25000),
        "ActivePower_L3": (-25000, 25000),
        
        # Energy readings (always positive)
        "TotalActiveEnergy": (0, 999999),
        "ForwardActiveEnergy": (0, 999999),
        "ReverseActiveEnergy": (0, 999999),
        
        # Reactive Power
        "TotalReactivePower": (-25000, 25000),
        "ReactivePower_L1": (-25000, 25000),
        "ReactivePower_L2": (-25000, 25000),
        "ReactivePower_L3": (-25000, 25000),
    }

    # If we don't have defined ranges for this register, log a warning
    if register_name not in valid_ranges:
        return False, f"No validation range defined for register {register_name}"
    
    # Check if value is None or not a number
    if value is None or not isinstance(value, (int, float)):
        return False, f"Invalid value type for {register_name}: {type(value)}"
    
    min_val, max_val = valid_ranges[register_name]
    
    if value < min_val or value > max_val:
        return False, f"Value {value} for {register_name} is outside valid range [{min_val}, {max_val}]"
    
    return True, "OK"
