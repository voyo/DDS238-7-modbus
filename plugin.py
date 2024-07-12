#!/usr/bin/env python
"""
Sofar Inverter. Domoticz plugin.
Author: voyo@no-ip.pl
Requirements: 
    1.python module minimalmodbus -> http://minimalmodbus.readthedocs.io/en/master/
        (pi@raspberrypi:~$ sudo pip3 install minimalmodbus)
    2.Communication module Modbus USB to RS485 converter module
"""
"""
<plugin key="Sofar-modbus" name="Sofar-modbus" version="0.2" author="voyo@no-ip.pl">
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
  def __init__(self,ID,name,multipler ,register,size: int = 1,functioncode: int = 3,options=None, Used: int = 1, signed: bool = False, Description=None, TypeName=None,Type: int = 0, SubType:int = 0 , SwitchType:int = 0  ):
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
                               Domoticz.Log("Modbus connection failure 1: "+str(e))
                               Domoticz.Log("retry updating register in 2 s")
                               sleep(2.0)
                               continue
                            break
                    elif RS485.MyMode == 'pymodbus':
                            Domoticz.Log("1 reading for register: "+str(self.register)+" size: "+str(self.size)+" name: "+self.name + " multipler: "+str(self.multipler))
                            while True:
                                try:
                                    G = RS485.read_holding_registers(self.register, self.size)
                                    Domoticz.Log("data: "+str(G) + " of type: "+str(type(G))+"0: "+str(G[0]))
                                    o = "reg ad #0 to 2: %s" % G 
                                    Domoticz.Log("moje o: "+o )
                                    value = BinaryPayloadDecoder.fromRegisters(G, byteorder=Endian.Big, wordorder=Endian.Big).decode_16bit_int()

 #                                  value = BinaryPayloadDecoder.fromRegisters(RS485.read_holding_registers(self.register, self.size), byteorder=Endian.Big, wordorder=Endian.Big).decode_16bit_int()
                                    payload = value * self.multipler  # decimal places
                                    Domoticz.Log("value: "+str(value)+" payload: "+str(payload)+" data: "+str(G) )
                                except Exception as e:
                                   Domoticz.Log("Modbus connection failure 2: "+str(e))
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
                               Domoticz.Log("Modbus connection failure 3"+str(e))
                               Domoticz.Log("retry updating register in 2 s")
                               sleep(2.0)
                               continue
                           break
                    elif RS485.MyMode == 'pymodbus':
                            Domoticz.Log("2 reading for register: "+str(self.register)+" size: "+str(self.size)+" name: "+self.name + " multipler: "+str(self.multipler))
                            while True:
                                try:
#                                   RS485.open()
                                   G = RS485.read_holding_registers(self.register, self.size)
                                   Domoticz.Log("data: "+str(G) + " of type: "+str(type(G)))
                                   o = "reg ad #0 to 2: %s" % G 
                                   Domoticz.Log("moje o: "+o )

                                   value = BinaryPayloadDecoder.fromRegisters(G, byteorder=Endian.Big, wordorder=Endian.Big).decode_32bit_int()
                                   payload = value * self.multipler # decimal places
                                   Domoticz.Log("value: "+str(value)+"multipler:"+str(self.multipler)+" payload: "+str(payload)+" data: "+str(G) )
                                except Exception as e:
                                   Domoticz.Log("Modbus connection failure 4: "+str(e))
                                   Domoticz.Log("retry updating register in 2 s")
                                   sleep(2.0)
                                   continue
                                break
                data = payload
                if self.Type == 243:
                    Domoticz.Log("DEBUG in UpdateValue, 243:     "+self.name+" data: "+str(data))
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
                if (self.name == "Import Energy"):
                    v=int(data*10)
                    outerClass.consumption=v
                    Domoticz.Log("CHECK consumption: "+str(v))
                    
                if (self.name == "Export Energy"):
                    v=int(data*10)
                    outerClass.production=v
                    Domoticz.Log("CHECK production: "+str(v))
       
                if self.Type == 250 and (self.name == "TotalEnergy"):
                    # virtual meter, no need to get value from Modbus. It is calculated from other values.
# USAGE1= energy usage meter tariff 1, This is an incrementing counter
# USAGE2= energy usage meter tariff 2, This is an incrementing counter
# RETURN1= energy return meter tariff 1, This is an incrementing counter
# RETURN2= energy return meter tariff 2, This is an incrementing counter
# CONS= actual usage power (Watt)
# PROD= actual return power (Watt)
# DATE = date with %Y-%m-%d format (for instance 2019-09-24) to put data in last week/month/year history log, or "%Y-%m-%d %H:%M:%S" format (for instance 2019-10-03 14:00:00) to put data in last days history log
# or
# Devices[Unit].Update(nValue=nValue, sValue="USAGE1;USAGE2;RETURN1;RETURN2;CONS;PROD;COUNTER1;COUNTER2;COUNTER3;COUNTER4;DATE")
                    USAGE1=str(0)
                    USAGE2=str(0)
                    RETURN1=str(0)
                    RETURN2=str(0)
                    CONS=str(0)
                    PROD=str(0)
                    DATE=str(0)

                    POWER = int(outerClass.active_power.get())
                    Domoticz.Debug("DEBUG in UpdateValue, "+self.name+" POWER: "+str(POWER))
                    # if power is negative, it is a return power, otherwise it is a usage power
                    # if POWER >= 0:
                    #     CONS=str(abs(POWER))
                    #     PROD=str(0)
                    #     Domoticz.Log("CHECK 1, outerClass.consumption: "+str(outerClass.consumption))
                    #     USAGE1=str(int(outerClass.consumption))
                    #     RETURN1=str(0)
                    # else:
                    #     CONS=str(0) 
                    #     PROD=str(abs(POWER))
                    #     USAGE1=str(0)                        
                    #     Domoticz.Log("CHECK 2, outerClass.production: "+str(outerClass.production))
                    #     RETURN1=str(int(outerClass.production))

                    USAGE1=str(outerClass.consumption)
                    RETURN1=str(outerClass.production)
                    PROD = str(abs(outerClass.reverse_power.get()))
                    CONS = str(abs(outerClass.forward_power.get()))
                    USAGE2=RETURN2=str(0)
                    Devices[self.ID].Update(1, sValue=USAGE1+";"+USAGE2+";"+RETURN1+";"+RETURN2+";"+CONS+";"+PROD)

                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("TUTAJ DEBUG in UpdateValue, "+self.name+" USAGE1: "+USAGE1+" USAGE2: "+USAGE2+" RETURN1: "+RETURN1+" RETURN2: "+RETURN2+" CONS: "+CONS+" PROD: "+PROD)

                    Devices[self.ID].Update(1, sValue=USAGE1+";"+USAGE2+";"+RETURN1+";"+RETURN2+";"+CONS+";"+PROD)
                    
                elif self.Type == 250 and (self.name == "Reserved1"):
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("DEBUG in UpdateValue "+self.name+' {0:.3f} V'.format(data))
                elif self.Type == 250 and (self.name == "Reserved2"):
                    if Parameters["Mode6"] == 'Debug':
                        Domoticz.Debug("DEBUG in UpdateValue "+self.name+' {0:.3f} V'.format(data))
                elif self.Type == 250 and (self.name == "LifeEnergy"):
                    USAGE1=str(data)
                    POWER = str(abs(int(outerClass.active_power.get())))
                    CONS = POWER
                    Devices[self.ID].Update(1, sValue=USAGE1+"0;0;0;0;"+CONS+";0")           
                elif self.Type == 250 and (self.name == "ReactiveEnergy"):
                    USAGE1=str(data*10)
                    CONS = str(outerClass.reactive_power.get())
                    Devices[self.ID].Update(1, sValue=USAGE1+"0;0;0;0;"+CONS+";0")
                elif self.Type == 250 and (self.name == "Import Energy"):
                    USAGE1=str(data*10)
                    CONS = str(outerClass.forward_power.get())
                    USAGE2=RETURN1=RETURN2=PROD=str(0)
                    Devices[self.ID].Update(1, sValue=USAGE1+";"+USAGE2+";"+RETURN1+";"+RETURN2+";"+CONS+";"+PROD)
                elif self.Type == 250 and (self.name == "Export Energy"):
                    RETURN1=str(data*10)
                    PROD = str(abs(outerClass.reverse_power.get())) 
                    USAGE1=USAGE2=RETURN2=CONS=str(0)
                    Devices[self.ID].Update(1, sValue=USAGE1+";"+USAGE2+";"+RETURN1+";"+RETURN2+";"+CONS+";"+PROD)
                else:
                    Devices[self.ID].Update(sValue=str(data),nValue=int(data))
          
                    


class BasePlugin:
    def __init__(self):
        self.runInterval = 60
        self.RS485 = ""
        # Active power for last 5 minutes
        self.active_power=Average()
        # Reactive power for last 5 minutes
        self.reactive_power=Average()
        # Forward power for last 5 minutes
        self.forward_power=Average()
        self.reverse_power=Average()
        self.consumption=0
        self.production=0
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
        Domoticz.Log("DDS238-7 Modbus plugin start, debug: "+Parameters["Mode2"])
        Domoticz.Log("DDS238-7 Modbus plugin start, debug2 ?: "+Parameters["Mode6"])
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
                Domoticz.Debug("Trying to connect to "+Parameters["Address"]+":"+Parameters["Port"]+" unit ID: "+ str(DeviceID))
                self.RS485 = ModbusClient(host=Parameters["Address"], port=int(Parameters["Port"]), unit_id=int(DeviceID), auto_open=True, auto_close=True, timeout=2, debug=True)
                self.RS485.open()
            except: 
                Domoticz.Log("pyMmodbus connection failure")
            self.RS485.MyMode = 'pymodbus'
            Domoticz.Debug("Using pymodbus, connected to "+Parameters["Address"]+":"+Parameters["Port"]+" unit ID: "+ str(DeviceID) + " mode: "+self.RS485.MyMode + " object: " + str(self.RS485))  
        else:
            Domoticz.Log("Unknown mode: "+Parameters["Mode4"])   
                
        devicecreated = []

        # self.devs = [
        #         Dev(1,"TotalEnergy",10,0x00,size=2,functioncode=3,Type=243,SubType=29,Description="Total energy balance"),
        #         Dev(4,"ReactiveEnergy",1,0x04,size=2,functioncode=3,options={"Custom":"1;kVArh"},Type=250,SubType=6,Description="Reactive energy"), # "Custom":"1;kVArh
        #         Dev(8,"Voltage Frequency",0.01,0x11,size=1,functioncode=3,options={"Custom":"1;Hz"},Type=243,SubType=31,Description="Voltage Frequency"),
        #         Dev(9,"Voltage_L1",0.1,0x80,size=1,functioncode=3,Type=243,SubType=8,Description="Voltage L1"),
   
        self.sensors = [
                Dev(1,"Temperature_Env1",1,1048,functioncode=3,TypeName="Temperature",Description="Temperature_Env1",signed=True),
                Dev(2,"Temperature_Env2",1,1049,functioncode=3,TypeName="Temperature",Description="Temperature_Env2",signed=True),
                Dev(3,"Temperature_HeatSink1",1,1050,functioncode=3,TypeName="Temperature",Description="Temperature_HeatSink1",signed=True),
                Dev(4,"Temperature_HeatSink2",1,1051,functioncode=3,TypeName="Temperature",Description="Temperature_HeatSink2",signed=True),
                Dev(5,"Temperature_HeatSink3",1,1052,functioncode=3,TypeName="Temperature",Description="Temperature_HeatSink3",signed=True),
                Dev(6,"Temperature_HeatSink4",1,1053,functioncode=3,TypeName="Temperature",Description="Temperature_HeatSink4",signed=True),
                Dev(7,"Temperature_HeatSink5",1,1054,functioncode=3,TypeName="Temperature",Description="Temperature_HeatSink5",signed=True),
                Dev(8,"Temperature_HeatSink6",1,1055,functioncode=3,TypeName="Temperature",Description="Temperature_HeatSink6",signed=True),
                Dev(9,"Temperature_Inv1",1,1056,functioncode=3,TypeName="Temperature",Description="Temperature_Inv1",signed=True),
                Dev(10,"Temperature_Inv2",1,1057,functioncode=3,TypeName="Temperature",Description="Temperature_Inv2",signed=True),
                Dev(11,"Temperature_Inv3",1,1058,functioncode=3,TypeName="Temperature",Description="Temperature_Inv3",signed=True),
           
                Dev(12,"GenerationTime_Today",1,1062,functioncode=3,TypeName="Counter",SubType=5,Description="GenerationTime_Today"),
                Dev(13,"GenerationTime_Total",1,1063,size=2,functioncode=3,TypeName="Counter",SubType=5,Description="GenerationTime_Total"),
                Dev(14,"ServiceTime_Total",1,1064,size=2,functioncode=3,TypeName="Counter",SubType=5,Description="ServiceTime_Total"),
                Dev(15,"Voltage_Frequency_grid",0.01,1156,functioncode=3,TypeName="Custom",Description="Frequency_grid",options={ "Custom" : "1;Hz"}),
   # total           
                Dev(16,"ActivePower_Output_Total",10,1157,functioncode=3,Type=243,SubType=29,Description="ActivePower_Output_Total",signed=True),
                Dev(17,"ReactivePower_Output_Total",10,1158,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Output_Total",signed=True),
                Dev(18,"ApparentPower_Output_Total",10,1159,functioncode=3,Type=243,SubType=29,Description="ApparentPower_Output_Total"),
                Dev(19,"ActivePower_PCC_Total",10,1160,functioncode=3,Type=243,SubType=29,Description="ActivePower_PCC_Total",signed=True),
                Dev(20,"ReactivePower_PCC_Total",10,1161,functioncode=3,Type=243,SubType=29,Description="ReactivePower_PCC_Total",signed=True),
                Dev(21,"ApparentPower_PCC_Total",10,1162,functioncode=3,Type=243,SubType=29,Description="ApparentPower_PCC_Total"),
   # phase R
                Dev(22,"Voltage_L1_R",0.1,1165,functioncode=3,TypeName="Voltage",Description="Voltage_Phase_R"),
                Dev(23,"Current_Output_R",0.01,1166,functioncode=3,Type=243,SubType=23,Description="Current_Output_R"),
                Dev(24,"ActivePower_Output_R",10,1167,functioncode=3,Type=243,SubType=29,Description="ActivePower_Output_R",signed=True),
                Dev(25,"ReactivePower_Output_R",10,1168,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Output_R",signed=True),
                Dev(26,"PowerFactor_Output_R",0.001,1169,functioncode=3,TypeName="Percentage",Description="PowerFactor_Output_R",signed=True),
                Dev(27,"Current_PCC_R",0.01,1170,functioncode=3,Type=243,SubType=23,Description="Current_PCC_R"),
                Dev(28,"ActivePower_PCC_R",10,1171,functioncode=3,Type=243,SubType=29,Description="ActivePower_PCC_R",signed=True),
                Dev(29,"ReactivePower_PCC_R",10,1172,functioncode=3,Type=243,SubType=29,Description="ReactivePower_PCC_R",signed=True),
                Dev(30,"PowerFactor_PCC_R",0.001,1173,functioncode=3,TypeName="Percentage",Description="PowerFactor_PCC_R",signed=True),
    # phase S
                Dev(31,"Voltage_L2_S",0.1,1176,functioncode=3,TypeName="Voltage",Description="Voltage_Phase_S"),
                Dev(32,"Current_Output_S",0.01,1177,functioncode=3,Type=243,SubType=23,Description="Current_Output_S"),
                Dev(33,"ActivePower_Output_S",10,1178,functioncode=3,Type=243,SubType=29,Description="ActivePower_Output_S",signed=True),
                Dev(34,"ReactivePower_Output_S",10,1179,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Output_S",signed=True),
                Dev(35,"PowerFactor_Output_S",0.001,1180,functioncode=3,TypeName="Percentage",Description="PowerFactor_Output_S",signed=True),
                Dev(36,"Current_PCC_S",0.01,1181,functioncode=3,Type=243,SubType=23,Description="Current_PCC_S" ),
                Dev(37,"ActivePower_PCC_S",10,1182,functioncode=3,Type=243,SubType=29,Description="ActivePower_PCC_S",signed=True),
                Dev(38,"ReactivePower_PCC_S",10,1183,functioncode=3,Type=243,SubType=29,Description="ReactivePower_PCC_S",signed=True),
                Dev(39,"PowerFactor_PCC_S",0.001,1184,functioncode=3,TypeName="Percentage",Description="PowerFactor_PCC_S",signed=True),
    # phase T
                Dev(40,"Voltage_L3_T",0.1,1187,functioncode=3,TypeName="Voltage",Description="Voltage_Phase_T"),
                Dev(41,"Current_Output_T",0.01,1188,functioncode=3,Type=243,SubType=23,Description="Current_Output_T"),
                Dev(42,"ActivePower_Output_T",10,1189,functioncode=3,Type=243,SubType=29,Description="ActivePower_Output_T",signed=True),
                Dev(43,"ReactivePower_Output_T",10,1190,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Output_T",signed=True),
                Dev(44,"PowerFactor_Output_T",0.001,1191,functioncode=3,TypeName="Percentage",Description="PowerFactor_Output_T",signed=True),
                Dev(45,"Current_PCC_T",0.01,1192,functioncode=3,Type=243,SubType=23,Description="Current_PCC_T"),
                Dev(46,"ActivePower_PCC_T",10,1193,functioncode=3,Type=243,SubType=29,Description="ActivePower_PCC_T",signed=True),
                Dev(47,"ReactivePower_PCC_T",10,1194,functioncode=3,Type=243,SubType=29,Description="ReactivePower_PCC_T",signed=True),
                Dev(48,"PowerFactor_PCC_T",0.001,1195,functioncode=3,TypeName="Percentage",Description="PowerFactor_PCC_T",signed=True),
    # ext
                Dev(49,"ActivePower_PV_Ext",10,1198,functioncode=3,Type=243,SubType=29,Description="ActivePower_PV_Ext",signed=True),
                Dev(50,"ActivePower_Load_Sys",10,1199,functioncode=3,Type=243,SubType=29,Description="ActivePower_Load_Sys",signed=True),
    # phase R
                Dev(51,"Voltage_Output_R",0.1,1290,functioncode=3,TypeName="Voltage",Description="Voltage_Output_R"),
                Dev(52,"Current_Load_R",0.01,1291,functioncode=3,Type=243,SubType=23,Description="Current_Load_R",signed=True),
                Dev(53,"ActivePower_Load_R",10,1292,functioncode=3,Type=243,SubType=29,Description="ActivePower_Load_R",signed=True),
                Dev(54,"ReactivePower_Load_R",10,1293,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Load_R",signed=True),
                Dev(55,"ApparentPower_Load_R",10,1294,functioncode=3,Type=243,SubType=29,Description="ApparentPower_Load_R",signed=True),
                Dev(56,"LoadPeakRatio_R",10,1295,functioncode=3,TypeName="Percentage",Description="LoadPeakRatio_R"),
    # phase S
                Dev(57,"Voltage_Output_S",0.1,1298,functioncode=3,TypeName="Voltage",Description="Voltage_Output_S"),
                Dev(58,"Current_Load_S",0.01,1299,functioncode=3,Type=243,SubType=23,Description="Current_Load_S",signed=True),
                Dev(59,"ActivePower_Load_S",10,1300,functioncode=3,Type=243,SubType=29,Description="ActivePower_Load_S",signed=True),
                Dev(60,"ReactivePower_Load_S",10,1301,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Load_S",signed=True),
                Dev(61,"ApparentPower_Load_S",10,1302,functioncode=3,TypeName="Usage",Description="ApparentPower_Load_S",signed=True),
                Dev(62,"LoadPeakRatio_S",10,1303,functioncode=3,TypeName="Percentage",Description="LoadPeakRatio_S"),
    # phase T
                Dev(63,"Voltage_Output_T",0.1,1306,functioncode=3,TypeName="Voltage",Description="Voltage_Output_T"),
                Dev(64,"Current_Load_T",0.01,1307,functioncode=3,Type=243,SubType=23,Description="Current_Load_T",signed=True),
                Dev(65,"ActivePower_Load_T",10,1308,functioncode=3,Type=243,SubType=29,Description="ActivePower_Load_T",signed=True),
                Dev(66,"ReactivePower_Load_T",10,1309,functioncode=3,Type=243,SubType=29,Description="ReactivePower_Load_T",signed=True),
                Dev(67,"ApparentPower_Load_T",10,1310,functioncode=3,Type=243,SubType=29,Description="ApparentPower_Load_T",signed=True),
                Dev(68,"LoadPeakRatio_T",10,1311,functioncode=3,TypeName="Percentage",Description="LoadPeakRatio_T"),
    # PV    
                Dev(69,"Voltage_PV1",0.1,1412,functioncode=3,TypeName="Voltage",Description="Voltage_PV1"),
                Dev(70,"Current_PV1",0.01,1413,functioncode=3,Type=243,SubType=23,Description="Current_PV1"),
                Dev(71,"Power_PV1",10,1414,functioncode=3,Type=243,SubType=29,Description="Power_PV1",signed=True),
                Dev(72,"Voltage_PV2",0.1,1415,functioncode=3,TypeName="Voltage",Description="Voltage_PV2"),
                Dev(73,"Current_PV2",0.01,1416,functioncode=3,Type=243,SubType=23,Description="Current_PV2"),
                Dev(74,"Power_PV2",10,1417,functioncode=3,Type=243,SubType=29,Description="Power_PV2",signed=True),
                Dev(75,"Voltage_PV3",0.1,1418,functioncode=3,TypeName="Voltage",Description="Voltage_PV3"),
                Dev(76,"Current_PV3",0.01,1419,functioncode=3,Type=243,SubType=23,Description="Current_PV3"),
                Dev(77,"Power_PV3",10,1420,functioncode=3,Type=243,SubType=29,Description="Power_PV3",signed=True),
                Dev(78,"Voltage_PV4",0.1,1421,functioncode=3,TypeName="Voltage",Description="Voltage_PV4"),
                Dev(79,"Current_PV4",0.01,1422,functioncode=3,Type=243,SubType=23,Description="Current_PV4"),
                Dev(80,"Power_PV4",10,1423,functioncode=3,Type=243,SubType=29,Description="Power_PV4",signed=True),
    # PV Generation Today
                Dev(81,"PV_Generation_Today",10,1668,size=2,functioncode=3,TypeName="Usage",Description="PV_Generation_Today",signed=False),
    # PV Generation Total
                Dev(82,"PV_Generation_Total",100,1670,size=2,functioncode=3,TypeName="Usage",Description="PV_Generation_Total",signed=False),
    # load consumption
                Dev(83,"Load_Consumption_Today",10,1672,size=2,functioncode=3,TypeName="Usage",Description="Load_Consumption_Today",signed=True),
                Dev(84,"Load_Consumption_Total",100,1674,size=2,functioncode=3,TypeName="Usage",Description="Load_Consumption_Total",signed=True)
                ]



    def onStop(self):
        Domoticz.Log("DDS238-7 Modbus plugin stop")

    def onHeartbeat(self):
        self.runInterval -=1;
        pluginClass = self
        if self.runInterval <= 0:
            for i in self.sensors:
                try:
                         # Get data from modbus
                        if Parameters["Mode6"] == 'Debug':
                            Domoticz.Debug("Getting data from modbus for device:"+i.name+" ID:"+str(i.ID))
                        self.sensors[i.ID-1].UpdateValue(self.RS485,pluginClass)
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
