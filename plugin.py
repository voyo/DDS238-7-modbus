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
<plugin key="DDS238-7" name="DDS238-7 modbus" version="0.8.2" author="voyo@no-ip.pl">
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
        # Try to read registers directly - pyModbusTCP will attempt to connect if needed
        if self.size == 1:
            registers = RS485.read_holding_registers(self.register, self.size)
            if registers is None or len(registers) == 0:
                raise Exception("Failed to read registers")
            
            value = registers[0]
            payload = value * self.multipler
            
        elif self.size == 2:
            registers = RS485.read_holding_registers(self.register, self.size)
            if registers is None or len(registers) < 2:
                raise Exception("Failed to read registers")
            
            # Combine two 16-bit registers into 32-bit value
            value = (registers[0] << 16) + registers[1]
            payload = value * self.multipler
        
        # Validate the value
        is_valid, reason = is_value_reasonable(payload, self.name)
        if not is_valid:
            Domoticz.Log(f"Rejecting invalid value for {self.name}: {reason}")
            return

        # Debug logging
        if Parameters["Mode6"] == 'Debug':
            Domoticz.Debug(f"Updating {self.name} with value: {payload}")

        # Debug for energy devices
        if self.name in ["TotalEnergy", "LifeEnergy", "Import Energy", "Export Energy"]:
            Domoticz.Debug(f"{self.name}: raw value={value}, multiplier={self.multipler}, payload={payload}")

        # Update Domoticz devices
        if self.Type == 243:  # Power measurements
            if self.name == "TotalActivePower":
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
            elif "ApparentPower" in self.name:
                outerClass.apparent_power = int(payload)
            elif "TotalEnergy" in self.name:
                outerClass.total_energy = int(payload)
            # Add other device updates as needed
        
        # --- Update Domoticz device with new value ---
        if self.ID in Devices:
            # P1 Smart Meter (Type FA, SubType 1)
            if getattr(Devices[self.ID], "Type", None) == 0xFA and getattr(Devices[self.ID], "SubType", None) == 1:
                # sValue = "<EnergyDelivered>;<EnergyReturned>;<PowerDelivered>;<PowerReturned>;<GasDelivered>"
                if "Import" in self.name:
                    sValue = f"{payload:.2f};0;0;0;0"
                elif "Export" in self.name:
                    sValue = f"0;{payload:.2f};0;0;0"
                else:
                    sValue = f"{payload:.2f};0;0;0;0"
                Devices[self.ID].Update(nValue=0, sValue=sValue)
                Domoticz.Log(f"Domoticz P1 device {self.ID} updated with sValue: {sValue}")
            # Energy counter: sValue must be "<value>;<counter>"
            elif self.Type == 250 and self.SubType == 1:
                sValue = f"{payload:.2f};0"
                Devices[self.ID].Update(nValue=0, sValue=sValue)
                Domoticz.Log(f"Domoticz energy device {self.ID} updated with sValue: {sValue}")
            else:
                Devices[self.ID].Update(nValue=0, sValue=str(payload))
                Domoticz.Log(f"Domoticz device {self.ID} updated with value: {payload}")
        else:
            Domoticz.Error(f"Device ID {self.ID} not found in Devices")
        
    except Exception as e:
        Domoticz.Error(f"Error updating {self.name}: {str(e)}")
        Domoticz.Log("retry updating register in 2 s")
        sleep(2.0)


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
        # Store latest values (not averages) for these:
        self.voltage = 0
        self.current = 0
        self.apparent_power = 0
        self.total_energy = 0
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
                    timeout=2
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
               # columns description, in one line:
               #Dev(ID, Name, Multiplier, Register, Size, FunctionCode, Type, SubType, Description)
                Dev(1,"TotalEnergy",0.1,0x00,size=2,functioncode=3,Type=250,SubType=1,Description="Total energy balance"),
                Dev(2,"LifeEnergy",0.1,0x00,size=2,functioncode=3,Type=250,SubType=1,Description="Total energy flow"),
                Dev(3,"Reserved1",1,0x02,size=2,functioncode=3,Used=0,Type=250,SubType=1,Description="Reserved1"),
                Dev(4,"ReactiveEnergy",0.1,0x04,size=2,functioncode=3,options={"Custom":"1;kVArh"},Type=250,SubType=6,Description="Reactive energy"), # "Custom":"1;kVArh
                Dev(5,"Reserved2",1,0x06,size=2,functioncode=3,Used=0,Type=250,SubType=1,Description="Reserved2"),
                Dev(6,"Import Energy",0.1,0x08,size=2,functioncode=3,Type=250,SubType=1,Description="Reverse energy"),
                Dev(7,"Export Energy",0.1,0xA,size=2,functioncode=3,Type=250,SubType=1,Description="Forward energy"),
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
    Validate if a value is within reasonable ranges for each register type.
    Returns (is_valid, reason)
    """
    ranges = {
        # Energy Measurements (Type 250)
        "TotalEnergy": (0, 1000000),      # kWh
        "LifeEnergy": (0, 1000000),       # kWh
        "ReactiveEnergy": (0, 1000000),   # kVArh
        "Import Energy": (0, 1000000),    # kWh (Reverse energy)
        "Export Energy": (0, 1000000),    # kWh (Forward energy)
        
        # Voltage Related (Type 243, SubType 8)
        "Voltage_L1": (0, 300),           # V
        "Voltage_L2": (0, 300),           # V
        "Voltage_L3": (0, 300),           # V
        "Voltage Frequency": (45, 65),     # Hz
        
        # Current Related (Type 243, SubType 23)
        "Current_L1": (0, 100),           # A
        "Current_L2": (0, 100),           # A
        "Current_L3": (0, 100),           # A
        
        # Active Power (Type 243, SubType 31, signed)
        "TotalActivePower": (-25000, 25000),  # W
        "ActivePower_L1": (-25000, 25000),    # W
        "ActivePower_L2": (-25000, 25000),    # W
        "ActivePower_L3": (-25000, 25000),    # W
        
        # Reactive Power (Type 243, SubType 31, signed)
        "TotalReactivePower": (-25000, 25000),  # kVAr
        "ReactivePower_L1": (-25000, 25000),    # kVAr
        "ReactivePower_L2": (-25000, 25000),    # kVAr
        "ReactivePower_L3": (-25000, 25000),    # kVAr
        
        # Apparent Power (Type 243, SubType 31)
        "TotalApparentPower": (0, 25000),     # kVA
        "ApparentPower_L1": (0, 25000),       # kVA
        "ApparentPower_L2": (0, 25000),       # kVA
        "ApparentPower_L3": (0, 25000),       # kVA
        
        # Power Factor (Type 243, SubType 31)
        "PowerFactor": (-1, 1),               # PF
        "PowerFactor_L1": (-1, 1),            # PF
        "PowerFactor_L2": (-1, 1),            # PF
        "PowerFactor_L3": (-1, 1),            # PF
        
        # Reserved (not used but included for completeness)
        "Reserved1": (0, 1000000),            # Generic range
        "Reserved2": (0, 1000000),            # Generic range
    }
    
    # Exact match instead of partial match for more precise validation
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
    
    # If device name not found in ranges
    Domoticz.Log(f"Warning: No validation range defined for device: {register_name}")
    return True, f"No validation range defined for {register_name}"
