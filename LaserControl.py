# -*- coding: utf-8 -*-
"""
Created on Tue Oct 16 00:51:44 2018

@author: manshiro
"""

import sys
from LedIndicatorWidget import *
import LaserControlMainWindow
import laser_communication
from PyQt5.QtWidgets import QApplication, QMainWindow

from PyQt5.QtCore import (QCoreApplication, QObject, QRunnable, QThread,
                          QThreadPool, pyqtSignal, pyqtSlot)
import time

class LaserControl(QMainWindow):
    
    set_repetition_rate_signal = pyqtSignal(int)
    set_repetition_quantity_signal = pyqtSignal(int)
    
    def __init__(self):
        super(self.__class__, self).__init__()

        self.ui = LaserControlMainWindow.Ui_MainWindow()

        self.ui.setupUi(self)
        
        self.laser_communication_thread = laser_communication.LaserCommunicationThread(self)
        
        self.ui.stop_button.clicked.connect(self.laser_communication_thread.Stop)
        self.ui.burst_on_button.clicked.connect(self.laser_communication_thread.BurstOn)
        self.ui.laser_off_button.clicked.connect(self.laser_communication_thread.LaserOff)
        self.ui.standby_button.clicked.connect(self.laser_communication_thread.LaserOn)
        self.ui.toggle_shutter_button.clicked.connect(self.laser_communication_thread.ToggleShutter)
        self.ui.external_trigger_on_button.clicked.connect(self.laser_communication_thread.ExternalTriggerOn)
        self.ui.repetition_on_button.clicked.connect(self.laser_communication_thread.RepetitionOn)
        self.ui.repetition_rate_spinBox.editingFinished.connect(self.repetition_rate_changed)
        self.set_repetition_rate_signal.connect(self.laser_communication_thread.setRepetitionRate)
        self.ui.repetition_quantity_spinBox.editingFinished.connect(self.repetition_quantity_changed)
        self.set_repetition_quantity_signal.connect(self.laser_communication_thread.setRepetitionQuantity)

        self.laser_communication_thread.start()


    def closeEvent(self, event):
    
        self.laser_communication_thread.alive = False
        
        while(self.laser_communication_thread.isFinished() == False):
            time.sleep(0.01)
        
        event.accept()
        
        
    def repetition_rate_changed(self):
        self.set_repetition_rate_signal.emit(self.ui.set_repetition_rate_signal.value())
    def repetition_quantity_changed(self):
        self.set_repetition_quantity_signal.emit(self.ui.repetition_quantity_spinBox.value())
        
        
    def display_laser_status(self):
        
        if self.laser_communication_thread.handler.shutter_status == 1:
            self.ui.shutter_status_led.setChecked(True)
        else:
            self.ui.shutter_status_led.setChecked(False)
            

if __name__ == "__main__":
    app = QApplication(sys.argv)
    form = LaserControl()
    form.show()
    sys.exit(app.exec_())
