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
<plugin key="DDS238-7 modbus" name="DDS238-7 modbus" version="0.8.1" author="voyo@no-ip.pl">
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
            msg = "Registering device: "+self.name+" "+str(self.ID)+" "+self.TypeName+"  Description: "+str(self.Description);
            Domoticz.Log(msg)        
            if self.TypeName != "":
                 Domoticz.Log("adding Dev with TypeName, "+self.TypeName)
                 Domoticz.Device(Name=self.name, Unit=self.ID, TypeName=self.TypeName,Used=self.Used,Options=self.options,Description=self.Description).Create()
            else:
                 Domoticz.Device(Name=self.name, Unit=self.ID,Type=self.Type, Subtype=self.SubType, Switchtype=self.SwitchType, Used=self.Used,Options=self.options,Description=self.Description).Create()
                 Domoticz.Log("adding Dev with Type, "+str(self.Type))
                      

  def UpdateValue(self,RS485,outerClass):
                if self.size == 1:
                    if RS485.MyMode == 'minimalmodbus':
                        while True:
                            try:
                               value = RS485.read_register(self.register,self.size)
                               payload = value * self.multipler  # decimal places
                            except Exception as e:
                               Domoticz.Log("Modbus connection failure: "+str(e))
                               Domoticz.Log("retry updating register in 2 s")
                               sleep(2.0)
                               continue
                            break
                    elif RS485.MyMode == 'pymodbus':
                            while True:
                                try:
                                    value = BinaryPayloadDecoder.fromRegisters(RS485.read_holding_registers(self.register, self.size), byteorder=Endian.Big, wordorder=Endian.Big).decode_16bit_int()
                                    payload = value * self.multipler  # decimal places
                                except Exception as e:
                                   Domoticz.Log("Modbus connection failure: "+str(e))
                                   Domoticz.Log("retry updating register in 2 s")
                                   sleep(2.0)
                                   continue
                                break
                            
                elif self.size == 2:
                    if RS485.MyMode == 'minimalmodbus':
                         while True:
                           try:
                             payload = RS485.read_long(self.register)                           
                           except Exception as e:
                               Domoticz.Log("Modbus connection failure"+str(e))
                               Domoticz.Log("retry updating register in 2 s")
                               sleep(2.0)
                               continue
                           break
                    elif RS485.MyMode == 'pymodbus':
                            while True:
                                try:
                                   value = BinaryPayloadDecoder.fromRegisters(RS485.read_holding_registers(self.register, self.size), byteorder=Endian.Big, wordorder=Endian.Big).decode_32bit_int()
                                   payload = value * self.multipler # decimal places
                                except Exception as e:
                                   Domoticz.Log("Modbus connection failure: "+str(e))
                                   Domoticz.Log("retry updating register in 2 s")
                                   sleep(2.0)
                                   continue
                                break
                data = payload
                if self.Type == 243 and (self.name == "TotalActivePower"):
                    outerClass.active_power.update(int(data))
                    if data < 0:
                        outerClass.reverse_power.update(int(data))
                        outerClass.forward_power.update(0)
                    else:
                        outerClass.reverse_power.update(0)
                        outerClass.forward_power.update(int(data))


                if self.Type == 243 and (self.name == "TotalReactivePower"):
                    v=int(abs( data*1000))
                    outerClass.reactive_power.update(v)
                if self.Type == 243 and (self.name == "RevEnergy"):
                    v=int(data*1000)
                    outerClass.reverse_power.update(v)
                if self.Type == 243 and (self.name == "FwdEnergy"):
                    v=int(data*1000)
                    outerClass.forward_power.update(v)

                if self.Type == 250 and (self.name == "LifeEnergy"):
# USAGE1= energy usage meter tariff 1, This is an incrementing counter
# USAGE2= energy usage meter tariff 2, This is an incrementing counter
# RETURN1= energy return meter tariff 1, This is an incrementing counter
# RETURN2= energy return meter tariff 2, This is an incrementing counter
# CONS= actual usage power (Watt)
# PROD= actual return power (Watt)
# DATE = date with %Y-%m-%d format (for instance 2019-09-24) to put data in last week/month/year history log, or "%Y-%m-%d %H:%M:%S" format (for instance 2019-10-03 14:00:00) to put data in last days history log
# or
# Devices[Unit].Update(nValue=nValue, sValue="USAGE1;USAGE2;RETURN1;RETURN2;CONS;PROD;COUNTER1;COUNTER2;COUNTER3;COUNTER4;DATE")
                    USAGE1=str(data)
                    CONS = str(int(outerClass.active_power.get()))
                    Devices[self.ID].Update(1, sValue=USAGE1+"0;0;0;0;"+CONS+";0")
                elif self.Type == 250 and (self.name == "Reserved1"):
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("DEBUG in UpdateValue "+self.name+' {0:.3f} V'.format(data))
                elif self.Type == 250 and (self.name == "Reserved2"):
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("DEBUG in UpdateValue "+self.name+' {0:.3f} V'.format(data))
                elif self.Type == 250 and (self.name == "ReactiveEnergy"):
                    USAGE1=str(data)
                    CONS = str(outerClass.reactive_power.get())
                    Devices[self.ID].Update(1, sValue=USAGE1+"0;0;0;0;"+CONS+";0")
                elif self.Type == 250 and (self.name == "RevEnergy"):
                    USAGE1=str(data)
                    CONS = str(outerClass.reverse_power.get())
                    Devices[self.ID].Update(1, sValue=USAGE1+"0;0;0;0;"+CONS+";0")
                elif self.Type == 250 and (self.name == "FwdEnergy"):
                    USAGE1=str(data)
                    CONS = str(outerClass.forward_power.get())
                    Devices[self.ID].Update(1, sValue=USAGE1+"0;0;0;0;"+CONS+";0")
                else:
                    Devices[self.ID].Update(sValue=str(data),nValue=int(data))
                if Parameters["Mode6"] == 'Debug':
                    Domoticz.Debug("DEBUG in UpdateValue "+self.name+' {0:.3f} V'.format(data))
                    


class BasePlugin:
    def __init__(self):
        self.runInterval = 1
        self.RS485 = ""
        # Active power for last 5 minutes
        self.active_power=Average()
        # Reactive power for last 5 minutes
        self.reactive_power=Average()
        # Forward power for last 5 minutes
        self.forward_power=Average()
        self.reverse_power=Average()
        return

    def onStart(self):
        Domoticz.Log("DDS238-7 Modbus plugin start, mode: "+Parameters["Mode4"])
        DeviceID= int(Parameters["Mode2"])

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
            Domoticz.Debug("TCP mode is not supported by minimalmodbus, so we use pymodbus instead")
            Domoticz.Debug("Using pymodbus, connecting to "+Parameters["Address"]+":"+Parameters["Port"]+" unit ID: "+ str(DeviceID))
            try: 
                self.RS485 = ModbusClient(host=Parameters["Address"], port=int(Parameters["Port"]), unit_id=DeviceID, auto_open=True, auto_close=True, timeout=2)
            except: 
                Domoticz.Log("pyMmodbus connection failure")
            self.RS485.MyMode = 'pymodbus'
        else:
            Domoticz.Log("Unknown mode: "+Parameters["Mode4"])   
                
        devicecreated = []

        self.devs = [
                Dev(1,"LifeEnergy",1,0x00,size=2,functioncode=3,Type=250,SubType=1,Description="Total energy consumption"),
                Dev(2,"Reserved1",1,0x02,size=2,functioncode=3,Type=250,SubType=1,Description="Reserved1"),
                Dev(3,"ReactiveEnergy",1,0x04,size=2,functioncode=3,options={"Custom":"1;kVArh"},Type=250,SubType=6,Description="Reactive energy"), # "Custom":"1;kVArh
                Dev(4,"Reserved2",1,0x06,size=2,functioncode=3,Type=250,SubType=1,Description="Reserved2"),
                Dev(5,"RevEnergy",1,0x08,size=2,functioncode=3,Type=250,SubType=1,Description="Reverse energy"),
                Dev(6,"FwdEnergy",1,0xA,size=2,functioncode=3,Type=250,SubType=1,Description="Forward energy"),
                Dev(7,"Voltage Frequency",0.01,0x11,size=1,functioncode=3,options={"Custom":"1;Hz"},Type=243,SubType=31,Description="Voltage Frequency"),
             
                Dev(8,"Voltage_L1",0.1,0x80,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L1"),
                Dev(9,"Voltage_L2",0.1,0x81,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L2"),
                Dev(10,"Voltage_L3",0.1,0x82,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L3"),
             
                Dev(11,"Current_L1",0.01,0x83,size=1,functioncode=3,Type=243,SubType=23,Description="Current L1"),
                Dev(12,"Current_L2",0.01,0x84,size=1,functioncode=3,Type=243,SubType=23,Description="Current L2"),
                Dev(13,"Current_L3",0.01,0x85,size=1,functioncode=3,Type=243,SubType=23,Description="Current L3"),
           
                Dev(14,"TotalActivePower",1,0x86,size=2,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True, Description="Total active power"),
                Dev(15,"ActivePower_L1",1,0x88,size=1,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True,Description="Active power L1"),
                Dev(16,"ActivePower_L2",1,0x89,size=1,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True,Description="Active power L2"),
                Dev(17,"ActivePower_L3",1,0x8A,size=1,functioncode=3,options={"Custom":"1;W"},Type=243,SubType=31,signed=True,Description="Active power L3"),
           
                Dev(18,"TotalReactivePower",0.001,0x8B,size=2,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Total reactive power"), # "Custom":"1;kVAr
                Dev(19,"ReactivePower_L1",0.001,0x8D,size=1,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Reactive power L1"), # "Custom":"1;kVAr
                Dev(20,"ReactivePower_L2",0.001,0x8E,size=1,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Reactive power L2"), # "Custom":"1;kVAr
                Dev(21,"ReactivePower_L3",0.001,0x8F,size=1,functioncode=3,options={"Custom":"1;kVAr"},Type=243,SubType=31,signed=True,Description="Reactive power L3"), # "Custom":"1;kVAr
           
                Dev(22,"TotalApparentPower",0.001,0x90,size=2,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Total apparent power"), # "Custom":"1;kVA
                Dev(23,"ApparentPower_L1",0.001,0x92,size=1,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Apparent power L1"), # "Custom":"1;kVA
                Dev(24,"ApparentPower_L2",0.001,0x93,size=1,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Apparent power L2"), # "Custom":"1;kVA
                Dev(25,"ApparentPower_L3",0.001,0x94,size=1,functioncode=3,options={"Custom":"1;kVA"},Type=243,SubType=31,Description="Apparent power L3"), # "Custom":"1;kVA
           
                Dev(26,"PowerFactor",0.001,0x95,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor"),
                Dev(27,"PowerFactor_L1",0.001,0x96,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor L1"),
                Dev(28,"PowerFactor_L2",0.001,0x97,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor L2"),
                Dev(29,"PowerFactor_L3",0.001,0x98,size=1,functioncode=3,Type=243,SubType=31,Description="Power factor L3")
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
                        Domoticz.Log("Connection failure: "+str(e));
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
