# -*- coding: utf-8 -*-
"""
Created on Mon Oct 15 16:40:11 2018

@author: Alexander Marsteller
"""

import serial
import serial.tools.list_ports
from PyQt5.QtCore import (QCoreApplication, QObject, QRunnable, QThread,
                          QThreadPool, pyqtSignal, pyqtSlot)
import sys
import time
import numpy as np
import logging

python_version = float(sys.version_info.major)
serial_version = float(serial.__version__)

class LaserCommunicationHandler(object):
    
    
    def __init__(self):
        logging.debug("Initializing laser communitcation handler.")
        #define variables
        self.request_start_delimiter = "#"
        self.response_start_delimiter = "<"
        self.destination_address = "!"
        self.source_address = "@"
        self.end_delimiter = "\r"
        
        
        self.command_dictionary =  {}
        self.command_dictionary["LaserOff"] = "X"
        self.command_dictionary["LaserOn"] = "g"
        self.command_dictionary["RepetitionOn"] = "h"
        self.command_dictionary["BurstOn"] = "j"
        self.command_dictionary["ExtTrigmode"] = "u"
        self.command_dictionary["LaserStop"] = "i"
        self.command_dictionary["SetBurstQuantity"] = "l"
        self.command_dictionary["SetRepetitionFrequency"] = "m"
        self.command_dictionary["SetHV"] = "n"
        self.command_dictionary["IncrementHV"] = "o1"
        self.command_dictionary["DecrementHV"] = "o0"
        self.command_dictionary["SetShutter"] = "z"
        self.command_dictionary["SetStepperPosition"] = "O3"
        self.command_dictionary["SetTransmission"] = "O4"
        self.command_dictionary["SetAttenuationEnergy"] = "O5"
        self.command_dictionary["InitAttenuator"] = "O60000"
        
        self.command_dictionary["GetShortStatus"] = "W"
        self.command_dictionary["GetStat7"] = "UT"
        self.command_dictionary["GetStat8"] = "UU"
        self.command_dictionary["GetVer3"] = "V3"
        self.command_dictionary["GetSernum"] = "US"
        self.command_dictionary["GetAttenuatorStatus"] = "UV"
        self.command_dictionary["GetEnergyValues"] = "P"
        
        
        
        
        self.command_parameter_dictionary = {}
        self.command_parameter_dictionary["LaserOff"] = None
        self.command_parameter_dictionary["LaserOn"] = None
        self.command_parameter_dictionary["RepetitionOn"] = None
        self.command_parameter_dictionary["BurstOn"] = None
        self.command_parameter_dictionary["ExtTrigmode"] = None
        self.command_parameter_dictionary["LaserStop"] = None
        self.command_parameter_dictionary["SetBurstQuantity"] = {"min":0, "max":9999, "length":4}
        self.command_parameter_dictionary["SetRepetitionFrequency"] = {"min":0, "max":99, "length":2}
        self.command_parameter_dictionary["SetHV"] = {"min":0, "max":100, "length":2}
        self.command_parameter_dictionary["IncrementHV"] = None
        self.command_parameter_dictionary["DecrementHV"] = None
        self.command_parameter_dictionary["SetShutter"] = {"min":0, "max":1, "length":1}
        self.command_parameter_dictionary["SetStepperPosition"] = {"min":0, "max":399, "length":4}
        self.command_parameter_dictionary["SetTransmission"] = {"min":0, "max":200, "length":2}
        self.command_parameter_dictionary["SetAttenuationEnergy"] = {"min":0, "max":9999, "length":4}
        self.command_parameter_dictionary["InitAttenuator"] = None
        
        
        self.reply_directory = {}
        self.reply_directory["W"] = "GetShortStatus"
        self.reply_directory["UT"] = "GetStat7"
        self.reply_directory["UU"] = "GetStat8"
        self.reply_directory["US"] = "GetSernum"
        self.reply_directory["UV"] = "GetAttenuatorStatus"
        self.reply_directory["P"] = "GetEnergyValues"
        self.reply_directory["V"] = "GetVer3"
        
        
        #define more stuff
        
        
        self.flag_byte_1 = 4
        #flag_byte_2 = reply[2:4]   # unused
        self.flag_byte_3 = 0
        self.quantity = 0
        self.frequency = 0
        self.hv = 0
        self.energy = 0
        self.flag_byte_4 = 0
        self.flag_byte_5 = 0
        
        self.internal_voltage = 0
        self.temperature1 = 0
        self.temperature2 = 0
        self.energy = 0
        self.quantity_counter = 0
        self.shot_counter_value = 0
        self.stepper_mode = 0
        self.stepper_setpoint = 0
        self.actual_stepper_position = 0
        self.actual_transmission = 0
        
        """
        self.flag_bytes_1 = {0:"Shutter is Open", 2:"Laser is Ready for Operation", 3:"Laser Standby", 
                      4:"LaserMode: Off", 5:"LaserMode: Repetition", 6:"LaserMode: Burst", 7:"LaserMode: External triggering"}
        self.flag_bytes_3 = {0:"Service Mode activated", 5:"EEPROM error", 6:"Watchdog Reset occurred"}
        
        
        self.flag_bytes_4 = {0:"Static error", 1:"Laser head chamber open", 2:"External interlock circuit open (Remote)", 
                        3:"Temperature too high (> 60°C)", 4:"Temperature 1 too high (> 48°C) ",
                        5:"Temperature 2 too high (> 48°C) warning", 6:"Energy monitor error occurred"}
        
        self.flag_bytes_5 = {0:"Operation error: Laser must be switched off", 3:"High voltage supply error or temperature error",
                        4:"Temperature error 1", 5:"Temperature error 2", 6:"Power switch is damaged", 7:"Power supply is too weak"}
        
        self.release_bytes = {0:"Shutter control is not supported", 1:"Attenuation module is supported",
                         3:"High voltage control is supported", 6:"Energy measuring is supported"}
        """
        
        self.energy_values = []
        
        
        self.shutter_open = False
        self.repetition_on = False
        self.burst_on = False
        self.external_trigger_on = False
        self.ready = False
        self.standby = False
        
        self.service_mode_activated = False
        self.eeprom_error = False
        self.cpu_error = False
        
        self.static_error = False
        self.laser_head_open = False
        self.remote = False
        self.temperature_limit = False
        self.temperature_warning_1 = False
        self.temperature_warning_2 = False
        self.energy_monitor_error = False
        
        self.flag_byte_5 = self._flag_byte_decoder(self.flag_byte_5)
        
        self.operation_error = False
        self.hv_error = False
        self.temperature_error_1 = False
        self.temperature_error_2 = False
        self.power_supply_error = False
        self.power_supply_weak = False
        
        
    def _calculate_frame_check_squence(self, telegram):
        logging.debug("Calculating frame check sequence for: {}".format(telegram))
        
        encoded_telegram = telegram.encode("ASCII")
        fcs = hex(sum(bytearray(encoded_telegram)) % 256).upper()[2:]
        
        logging.debug("Calculated frame check sequence: {}".format(fcs))
        
        return fcs
        
    
    def compose_command(self, command_string, command_parameter=None):
        command = self.request_start_delimiter + self.destination_address + self.source_address
        command = command + self.command_dictionary[command_string]
        
        logging.debug("Composing command for: {}".format(command_string))
        
        if command_parameter != None and self.command_parameter_dictionary[command_string] != None:
            logging.debug("with parameter: {}".format(command_parameter))
            if command_parameter <= self.command_parameter_dictionary[command_string]["max"] and command_parameter >= self.command_parameter_dictionary[command_string]["min"]:
                #parameter_string = hex(command_parameter).upper()
                
                parameter_string = "{0:0{1}x}".format(command_parameter,self.command_parameter_dictionary[command_string]["length"]).upper()
                command += parameter_string
            else:
                logging.critical("Incompatible command parameter supplied:".format(command_parameter))
                raise ValueError()
        
        command = command + self._calculate_frame_check_squence(command)
        command = command + self.end_delimiter
        
        logging.debug("Successfully composed command for: {}".format(command_string))
        return command
        
    
    def _flag_byte_decoder(self, flag_byte):
        binary_string = format(flag_byte, '#010b')[2:]
        truth_array = [i=="1" for i in list(binary_string)]
        return truth_array[::-1]
        
        
    def _interprete_response(self, reply):
        
        cleaned_reply = reply.rstrip(self.end_delimiter)
        logging.debug("Interpreting response: {}".format(cleaned_reply))
        
        logging.debug("Checking response integrity with frame check sequence")
        reply_fcs = cleaned_reply[-2:]
        calculated_reply_fcs = self._calculate_frame_check_squence(cleaned_reply[:-2])
        
        if reply_fcs != calculated_reply_fcs:
            logging.debug("Response integrity compromised!")
            raise IOError()
        else:
            logging.debug("Response integrity confirmed")
            cleaned_reply = cleaned_reply.rstrip(reply_fcs)
        
        cleaned_reply = cleaned_reply.lstrip(self.response_start_delimiter).lstrip(self.source_address).lstrip(self.destination_address)
        
        logging.debug("Determining response type.")
        response_type = None
        for key in self.reply_directory.keys():
            if cleaned_reply[:len(key)] == key:
                response_type = self.reply_directory[key]
                logging.debug("Response type determined: {}".format(response_type))
                break
        
        if response_type != None:
            logging.debug("Interpreting {} type response.".format(response_type))
            eval("self._interprete_{}(cleaned_reply)".format(response_type))
            
            return response_type
        else:
            logging.critical("Unknown response type encountered.")
            raise IOError()
        
    def _interprete_GetShortStatus(self, reply):
        cleaned_reply = reply[1:]
        
        response_dict = {"00":"Standby", "01":"Laser is Working", "03":"EEPROM Error",
                         "04":"Energy Monitor Error occured", "05":"Temperature too high (>48°C)",
                         "06":"Static Error", "07":"Operation Error: Laser mus be switched off"}
        
        return response_dict[cleaned_reply]
    
    def _interprete_GetStat7(self, reply):
        cleaned_reply = reply[2:]
        
        self.flag_byte_1 = int(cleaned_reply[:2],16)
        #flag_byte_2 = reply[2:4]   # unused
        self.flag_byte_3 = int(cleaned_reply[4:6],16)
        self.quantity = int(cleaned_reply[6:10],16)
        self.frequency = int(cleaned_reply[10:12],16)
        self.hv = int(cleaned_reply[12:14],16)
        self.energy = int(cleaned_reply[18:22],16)
        
        self.flag_byte_1 = self._flag_byte_decoder(self.flag_byte_1)
        self.flag_byte_3 = self._flag_byte_decoder(self.flag_byte_3)
        
        self.shutter_open = self.flag_byte_1[0]
        self.ready = self.flag_byte_1[2]
        self.standby = self.flag_byte_1[3]
        self.mode_off = self.flag_byte_1[4]
        self.repetition_on = self.flag_byte_1[7]
        self.burst_on = self.flag_byte_1[6]
        self.external_trigger_on = self.flag_byte_1[5]
        
        
        self.service_mode_activated = self.flag_byte_3[0]
        self.eeprom_error = self.flag_byte_3[5]
        self.cpu_error = self.flag_byte_3[6]
    
                               
    def _interprete_GetStat8(self, reply):
        cleaned_reply = reply[2:]
        
        self.flag_byte_4 = int(cleaned_reply[:2],16)
        self.flag_byte_5 = int(cleaned_reply[2:4],16)
        
        self.internal_voltage = int(cleaned_reply[4:6],16)
        self.temperature1 = int(cleaned_reply[6:8],16)
        self.temperature2 = int(cleaned_reply[8:10],16)
        self.energy = int(cleaned_reply[10:14],16)
        self.quantity_counter = int(cleaned_reply[14:18],16)
        self.shot_counter_value = int(cleaned_reply[18:26],16)
        
        
        self.flag_byte_4 = self._flag_byte_decoder(self.flag_byte_4)
        
        self.static_error = self.flag_byte_4[0]
        self.laser_head_open = self.flag_byte_4[1]
        self.remote = self.flag_byte_4[2]
        self.temperature_limit = self.flag_byte_4[3]
        self.temperature_warning_1 = self.flag_byte_4[4]
        self.temperature_warning_2 = self.flag_byte_4[5]
        self.energy_monitor_error = self.flag_byte_4[6]
        
        self.flag_byte_5 = self._flag_byte_decoder(self.flag_byte_5)
        
        self.operation_error = self.flag_byte_5[0]
        self.hv_error = self.flag_byte_5[3]
        self.temperature_error_1 = self.flag_byte_5[4]
        self.temperature_error_2 = self.flag_byte_5[5]
        self.power_supply_error = self.flag_byte_5[6]
        self.power_supply_weak = self.flag_byte_5[7]
        
    def _interprete_GetSernum(self, reply):
        cleaned_reply = reply[2:]
        
        self.stepper_mode = int(cleaned_reply[:2],16)
        self.stepper_setpoint = int(cleaned_reply[2:6],16)
        self.actual_stepper_position = int(cleaned_reply[6:10],16)
        self.actual_transmission = int(cleaned_reply[10:12],16)/2
        
    def _interprete_GetAttenuatorStatus(self, reply):
        cleaned_reply = reply[2:]
        
        self.laser_serial_number = int(cleaned_reply[:8],16)
        self.energy_monitor_serial_number = int(cleaned_reply[8:12],16)
        
        
    def _interprete_GetEnergyValues(self, reply):
        cleaned_reply = reply[1:]
        
        stored_energy_value_count = int(cleaned_reply[:2],16)
        following_energy_values = int(cleaned_reply[2:4],16)
        
        
        for i in range(following_energy_values):
            energy = int(cleaned_reply[4+4*i:4+4*(i+1)],16) * 250/64000 # micro Joule
            
            self.energy_values.append(energy )
        
        
    def _interprete_GetVer3(self, reply):
        cleaned_reply = reply[2:]
        
        self.main_rev_byte = int(cleaned_reply[:2],16)
        self.release_byte = int(cleaned_reply[2:4],16)
        self.type_byte_1 = int(cleaned_reply[4:6],16)
        self.type_byte_2 = int(cleaned_reply[6:8],16)
        self.program_version = cleaned_reply[8:16],16
        following_chars = int(cleaned_reply[16:18],16)
        self.laser_type = cleaned_reply[18:18+following_chars]
        
        
    
class LaserCommunicationThread(QThread):
    
    
    recieved_reply_signal = pyqtSignal(str)
    update_main_window_signal = pyqtSignal()
    
    
    def __init__(self, main_window, com_port="/dev/ttyUSB0", debug=False):
        QThread.__init__(self)
        if debug:
            logging.debug("LaserCommunicationThread is in debug mode")
        
        logging.info("Initializing LaserCommunicationThread")
        logging.info("Looking for available COM ports...")
        self.available_comports = list(serial.tools.list_ports.comports())
        
        if len(self.available_comports) >= 1:
            for p in self.available_comports:
                logging.info(p.device)
        else:
            logging.critical("No COM ports found.")
        
        
        
        logging.debug("Connecting to GUI")
        self.main_window = main_window
        self.recieved_reply_signal.connect(self.process_recieved_message)
        self.update_main_window_signal.connect(self.main_window.display_laser_status)
        
        logging.debug("Creating and attaching laser communication handler")
        self.handler = LaserCommunicationHandler()
        #serial connection parameters
        
        
        logging.debug("Setting up serial connection parameters")
        self.main_window.ui.connection_label.setText("Setting serial connection parameters")
        self.baud_rate = 9600
        self.parity = serial.PARITY_NONE
        self.bytesize = 8
        self.stopbits = 1
        self.rtscts = 1
        self.write_timeout = 5
        self.timeout = 0.2
        
        default_failed = False
        
        if not debug:
            self.used_com_port = None
            if com_port != None:
                logging.info("Trying to connect using specified com port: {}".format(com_port))
                try:
                    self.serial_connection = serial.Serial(com_port, self.baud_rate, parity=self.parity, 
                                                           bytesize=self.bytesize, stopbits=self.stopbits,
                                                           rtscts=self.rtscts, timeout=self.timeout, write_timeout=self.write_timeout)
                    self.main_window.ui.connection_label.setText("Connected to laser on COM port: {}".format(com_port))
                    self.used_com_port = com_port
                except serial.SerialException:
                    logging.critical("No serial connection possible on specified COM port")
                    default_failed = True
    
            if com_port == None or default_failed:
                if default_failed:
                    logging.info("Specified COM port failed, trying to detect Laser...")
                else:
                    logging.info("No COM port specified, trying to detect Laser...")
                for port in self.available_comports:
                    logging.info("Now trying COM port: {}".format(port.device))
                    try:
                        self.serial_connection = serial.Serial(port.device, self.baud_rate, parity=self.parity, 
                                                               bytesize=self.bytesize, stopbits=self.stopbits,
                                                               rtscts=self.rtscts, timeout=self.timeout, write_timeout=self.write_timeout)
                        
                        time.sleep(0.050)
                        self.serial_connection.reset_input_buffer()
                        outgoing_message = self.handler.compose_command("LaserOn").encode("ASCII")
                        self.serial_connection.write(outgoing_message)
                        time.sleep(0.020)
                        outgoing_message = self.handler.compose_command("GetShortStatus").encode("ASCII")
                        self.serial_connection.write(outgoing_message)
                        time.sleep(0.010)
                        
                        if self._waiting_bytes() != 0:
                            recieved_message = self.serial_connection.read_until(self.handler.end_delimiter)
                            
                            if "<@!W" in recieved_message:
                                self.used_com_port = port.device
                                logging.info("Laser found on COM port: {}".format(port.device))
                            
                                self.main_window.ui.connection_label.setText("Connected to laser on COM port: {}".format(port.device))
                                break
                            else:
                                self.serial_connection.close()
                                logging.info("Nothing found on COM port: {}".format(port.device))
                                time.sleep(0.05)
                        else:
                            self.serial_connection.close()
                            print("Nothing found on COM port: {}".format(port.device))
                            time.sleep(0.05)
                            
                    except Exception as e:
                        logging.critical("An error occured while trying to communicate with device on COM port {}".format(port.device))
                        logging.critical("Error: {}".format(e))
                        try:
                            self.serial_connection.close()
                        except Exception as e2:
                            logging.critical("Could not close erroneous serial connection. Maybe wasn't opened")
                                
            if self.used_com_port == None:
                self.main_window.ui.connection_label.setText("Could not detect laser")
                logging.critical("Laser not found over serial interface")
                raise serial.SerialException("Laser not found over serial interface") 
        else:
            self.main_window.ui.connection_label.setText("Connected to DummySerial")
            logging.debug("Connected to DummySerial")
            self.serial_connection = DummySerial(self.handler)
            
        self.command_queue = []
    
        self.alive = True
        self.recieved_messages = []
        self.outgoing_messages = []
        self.message_limit = 1000
        self.status_poll_interval = 0.5
        logging.debug("Setting Thread operating parameters:")
        logging.debug("\tPolling time: {}".format(self.status_poll_interval))
        logging.debug("\tMaximum replies in memory: {}".format(self.message_limit))
        
        """
        outgoing_message = "#!@gEB\r"
        self.serial_connection.write(outgoing_message.encode("ASCII"))
        """
    
    def run(self):
        
        logging.info("Communication Thread started running")
        time.sleep(2.0)
        
        self.last_status_poll_time = datetime.datetime.utcnow()
        
        while(self.alive):
            read_timeout_counter = 10
            
            
            logging.debug("Looking for recieved reply messages from laser")
            while(self._waiting_bytes() != 0 and read_timeout_counter > 0):
                
                logging.debug("Recieved reply message from laser is available")
                recieved_message = self.serial_connection.read_until(self.handler.end_delimiter)
                recieved_message = recieved_message.decode("ASCII")
                messages = recieved_message.split("\r")
                for m in messages:
                        if m == "":
                               continue
                        self.recieved_messages.append(m)
                        logging.debug("Added message to recieved message list: {}".format(m))
                        self.recieved_reply_signal.emit(m)

                        if len(self.recieved_messages) > self.message_limit:
                            logging.debug("Recieved message larger than allowed maximum")
                            dm = self.recieved_messages.pop(0)
                            logging.debug("Popped message from recieved message list: {}".format(dm))
                    
                read_timeout_counter -= 1
            
            
            logging.debug("Looking if there are outgoing messages scheduled")
            if len(self.outgoing_messages) > 0:
                outgoing_message = self.outgoing_messages.pop(0)
                logging.debug("Found outgoing message: {}".format(outgoing_message))
                
                self.serial_connection.write(outgoing_message.encode("ASCII"))
                logging.debug("Sending message to laser: {}".format(outgoing_message))
    
            time.sleep(0.020)
            
            logging.debug("Check if it is time to poll the laser status")
            now = datetime.datetime.utcnow()
            if (now - self.last_status_poll_time).total_seconds() > self.status_poll_interval:
                self.last_status_poll_time = now
                logging.debug("Queing laser status poll")
                self.QueryStatus()
        
        
        logging.info("Communication Thread ended")
    
    def _waiting_bytes(self):
        global python_version
        global serial_version
        
        incoming_byts_in_buffer = 0
            
        if python_version >= 3 and serial_version >=3:
            incoming_byts_in_buffer = self.serial_connection.in_waiting
        else:
            incoming_byts_in_buffer = self.serial_connection.inWaiting()
    
        return incoming_byts_in_buffer
    
    def process_recieved_message(self, message):
        self.handler._interprete_response(message)
        self.update_main_window_signal.emit()
    
    def execute_command(self, command_string, command_parameter=None):
        logging.debug("Queing command: {}".format(command_string))
        if command_parameter != None:
            logging.debug("with parameter: {}".format(command_parameter))
            
        request_string = self.handler.compose_command(command_string, command_parameter)
        
        self.outgoing_messages.append(request_string)
    
    def LaserOn(self):
        self.execute_command("LaserOn")
    
    def LaserOff(self):
        self.execute_command("LaserOff")
    
    def OpenShutter(self):
        self.execute_command("SetShutter", 1)
        
    def CloseShutter(self):
        self.execute_command("SetShutter", 0)
    
    def RepetitionOn(self):
        self.execute_command("RepetitionOn")
    
    def BurstOn(self):
        self.execute_command("BurstOn")
    
    def ExternalTriggerOn(self):
        self.execute_command("ExtTrigmode")
    
    def Stop(self):
        self.execute_command("LaserStop")
    
    def setRepetitionRate(self, frequency):
        self.execute_command("SetRepetitionFrequency", frequency)
    
    def setRepetitionQuantity(self, quantity):
        self.execute_command("SetBurstQuantity", quantity)
        
    
    def ToggleShutter(self):
        if self.handler.shutter_open:
            self.OpenShutter()
        else:
            self.CloseShutter()
            
    def QueryStatus(self):
        self.execute_command("GetShortStatus")
        self.execute_command("GetStat7")
        self.execute_command("GetStat8")
        
    def QueryShortStatus(self):
        self.execute_command("GetShortStatus")

import datetime
class DummySerial(object):

    def __init__(self, handler):
        self.last_called = datetime.datetime.utcnow()
        self.buffer = ""
        self.handler = handler
        self.shutter_status = 0
    @property
    def in_waiting(self):
        """
        s = "<@!UT040003000A14320000000088\r"
        
        delta_t = (datetime.datetime.utcnow() - self.last_called).total_seconds()
        if delta_t > 0.5:
            self.buffer += s
            self.last_called = datetime.datetime.utcnow()
        """
        return len(self.buffer)
            
    def read_until(self, char):
        index = self.buffer.find(char)+1
        reply = self.buffer[:index]
        self.buffer = self.buffer[index:]
        return reply.encode("ASCII")
    
    def write(self, out):
        out = out.decode("ASCII")
        
        if out == self.handler.compose_command("SetShutter", 1):
            self.shutter_status = 1
        if out == self.handler.compose_command("SetShutter", 0):
            self.shutter_status = 0
        
        if out == self.handler.compose_command("GetStat7"):
            if self.shutter_status == 0:
                self.buffer += "<@!UT05000200320A64000000008D\r"
            elif self.shutter_status == 1:
                self.buffer += "<@!UT04000200320A64000000008C\r"
        
        if out == self.handler.compose_command("GetStat8"):
            message = "<@!UU0000D61E2500000000004602E685"#.format(hex(int(temperature))[2:].upper())
            fcs = self.handler._calculate_frame_check_squence(message)
            self.buffer += message + fcs +"\r"
        

	
