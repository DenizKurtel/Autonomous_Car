#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
/**********************************************************************
* Filename    : client.py
* Description : Client for PiCar-V. Use python3 + pyqt5
* Author      : Dream
* Company     : Sunfounder
* E-mail      : service@sunfounder.com
* website     : www.sunfounder.com
* Update      : Dream    2016/09/12
**********************************************************************/
'''
import math
import sys

if sys.version_info.major < 3 or sys.version_info.minor < 4:
    raise RuntimeError('At least Python 3.4 is required')

import sys, time, http.client
from PyQt5 import QtCore, uic, QtWidgets
import icons_rc
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap
from urllib.request import urlopen
import requests
import cv2
import numpy as np

login_screen = "login_screen.ui"
running_screen = "running_screen.ui"
setting_screen = "setting_screen.ui"
calibrate_screen = "calibrate_screen.ui"

# Load the ui files 
Ui_Login_screen, QtBaseClass = uic.loadUiType(login_screen)
Ui_Running_screen, QtBaseClass = uic.loadUiType(running_screen)
Ui_Setting_screen, QtBaseClass = uic.loadUiType(setting_screen)
Ui_Calibrate_screen, QtBaseClass = uic.loadUiType(calibrate_screen)

# Set speed content, and speed level content
MAX_SPEED = 100
MIN_SPEED = 40
SPEED_LEVEL_1 = MIN_SPEED
SPEED_LEVEL_2 = (MAX_SPEED - MIN_SPEED) / 4 * 1 + MIN_SPEED
SPEED_LEVEL_3 = (MAX_SPEED - MIN_SPEED) / 4 * 2 + MIN_SPEED
SPEED_LEVEL_4 = (MAX_SPEED - MIN_SPEED) / 4 * 3 + MIN_SPEED
SPEED_LEVEL_5 = MAX_SPEED
SPEED = [0, SPEED_LEVEL_1, SPEED_LEVEL_2, SPEED_LEVEL_3, SPEED_LEVEL_4, SPEED_LEVEL_5]

HOST = '192.168.0.133'
PORT = '8000'
autologin = 1

# BASE_URL is variant use to save the format of host and port 
BASE_URL = 'http://' + HOST + ':' + PORT + '/'


def __reflash_url__():
    global BASE_URL
    BASE_URL = 'http://' + HOST + ':' + PORT + '/'


def __read_auto_inf__():
    try:
        fp = open("auto_ip.inf", 'r')
        lines = fp.readlines()
        for line in lines:
            if "ip" in line:
                ip = line.replace(' ', '').replace('\n', '').split(':')[1]

            elif "port" in line:
                port = line.replace(' ', '').replace('\n', '').split(':')[1]

            elif "remember_status" in line:
                remember_status = line.replace(' ', '').replace('\n', '').split(':')[1]
        fp.close()
        return ip, port, int(remember_status)
    except IOError:
        return -1


def __write_auto_inf__(ip=None, port=None, rem_status=None):
    fp = open("auto_ip.inf", 'w')
    string = "ip: %s \nport: %s\nremember_status: %s\n" % (ip, port, rem_status)
    fp.write(string)
    fp.close()


class LoginScreen(QtWidgets.QDialog, Ui_Login_screen):
    """Login Screen

    To creat a Graphical User Interface, inherit from Ui_Login_screen. And define functions
    that use for the control.

    Attributes:
        none
    """

    def __init__(self):
        global autologin
        global HOST, PORT

        info = __read_auto_inf__()
        if info == -1:
            HOST = ''
            PORT = ''
            autologin = -1
        else:
            HOST = info[0]
            PORT = info[1]
            autologin = info[2]

        QtWidgets.QDialog.__init__(self)
        Ui_Login_screen.__init__(self)
        self.setupUi(self)
        self.setWindowTitle("Log In - SunFounder PiCar-V Client")
        # Check value of autologin, if True, set text of host line edit with saved host
        if autologin == 1:
            self.lEd_host.setText(HOST)
            self.label_Error.setText("")
            self.pBtn_checkbox.setStyleSheet("border-image: url(./images/check2.png);")
        # not autologin, line edit will fill with blank
        else:
            self.lEd_host.setText("")
            self.label_Error.setText("")
            self.pBtn_checkbox.setStyleSheet("border-image: url(./images/uncheck1.png);")
        # connect the signal and slot
        self.pBtn_checkbox.clicked.connect(self.on_pBtn_checkbox_clicked)

    def on_pBtn_login_clicked(self):
        """Slot for signal that Login button clicked

        The login button clicked, this function will run. This function use for logining,
        first, check the length of text in line edit, if ok, saved them to variable HOST
        and PORT, after that, call function connection_ok(), if get 'OK' return, login
        succeed, close this screen, show running screen

        Args:
            None

        Returns:
            if login succeed return True
            else return False

        """
        global HOST, PORT
        # check whether the length of input host and port is allowable
        if 7 < len(self.lEd_host.text()) < 16:
            HOST = self.lEd_host.text()
            PORT = self.lEd_port.text()
            __reflash_url__()
            self.label_Error.setText("Connecting....")

            # check whethe server is connected
            if connection_ok() == True:  # request respon 'OK', connected
                if autologin == 1:  # autologin checked, record HOST
                    HOST = self.lEd_host.text()
                    PORT = self.lEd_port.text()
                else:
                    self.lEd_host.setText("")
                    self.label_Error.setText("")

                __write_auto_inf__(HOST, PORT, autologin)
                self.label_Error.setText("")
                # login succeed, login1 screen close, running screen show, function start_stream() run

                login1.close()
                running1.start_stream()
                running1.show()
                return True
            # connected failed, set the information
            else:
                self.label_Error.setText("Failed to connect")
                return False
        # the input length in line edit not allowable
        else:
            self.label_Error.setText("Host or port not correct")
            return False
        print("on_pBtn_login_clicked", HOST, PORT, autologin, "\n")

    # pressed and released, each set their stylesheet, so it make a feedback of press
    def on_pBtn_login_pressed(self):
        self.pBtn_login.setStyleSheet("border-image: url(./images/login_button_pressed.png);color: rgb(255, 255, 255);")

    def on_pBtn_login_released(self):
        self.pBtn_login.setStyleSheet(
            "border-image: url(./images/login_button_unpressed.png);color: rgb(255, 255, 255);")

    def on_pBtn_checkbox_clicked(self):
        """Slot for checkbox button clicked signal

        The checkbox button clicked, this function will run. This function use for autologin,
        when clicked, the status of autologin(check or not check) will changed, if autologin
        checked, save HOST and PORT, and next show this screen, line edit will auto fill with
        the saved value

        Args:
            None

        #TODO: save the HOST and PORT to file in local
        """
        global autologin
        # when clicked checkbox button, change value of various autologin
        autologin = -autologin
        print('autolongin = %s' % autologin)
        if autologin == 1:
            # the checked picture
            self.pBtn_checkbox.setStyleSheet("border-image: url(./images/check2.png);")
        else:
            # the unchecked picture
            self.pBtn_checkbox.setStyleSheet("border-image: url(./images/uncheck1.png);")
        print("on_pBtn_checkbox_clicked", HOST, autologin)


class RunningScreen(QtWidgets.QDialog, Ui_Running_screen):
    """Running Screen

    To creat a Graphical User Interface, inherit from Ui_Running_screen. And define functions
    that use for the control.

    Attributes:
        TIMEOUT, how long it time up for QTimer, using to reflash the frame
    """
    TIMEOUT = 50
    LEVEL1_SPEED = 40
    LEVEL5_SPEED = 100

    LEVEL2_SPEED = int((LEVEL5_SPEED - LEVEL1_SPEED) / 4 * 1 + LEVEL1_SPEED)
    LEVEL3_SPEED = int((LEVEL5_SPEED - LEVEL1_SPEED) / 4 * 2 + LEVEL1_SPEED)
    LEVEL4_SPEED = int((LEVEL5_SPEED - LEVEL1_SPEED) / 4 * 3 + LEVEL1_SPEED)

    LEVEL_SPEED = [0, LEVEL1_SPEED, LEVEL2_SPEED, LEVEL3_SPEED, LEVEL4_SPEED, LEVEL5_SPEED]

    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        Ui_Running_screen.__init__(self)
        self.setupUi(self)

        self.speed_level = 0
        # when init, no level button has been pressed, self.speed_level = 0
        self.level_btn_show(self.speed_level)
        self.setWindowTitle("Operation - SunFounder PiCar-V Client")
        self.btn_back.setStyleSheet("border-image: url(./images/back_unpressed.png);")
        self.btn_setting.setStyleSheet("border-image: url(./images/settings_unpressed.png);")

    def start_stream(self):
        """Start to receive the stream

        With this function called, the QTimer start timing, while time up, call reflash_frame() function,
        the frame will be reflashed.

        Args:
            None
        """
        # creat an object queryImage with the HOST
        self.queryImage = QueryImage(HOST)
        self.timer = QTimer(timeout=self.reflash_frame)  # Qt timer, time up, run reflash_frame()
        self.timer.start(RunningScreen.TIMEOUT)  # start timer
        print('start')
        # init the position
        run_action('fwready')
        run_action('bwready')
        run_action('camready')

    def stop_stream(self):
        self.timer.stop()  # stop timer, so the receive of stream also stop

    def transToPixmap(self):
        """Convert the stream data to pixmap data

        First save the queryImage() data, and then creat an object pixmap, call built-in function
        pixmap.loadFromData(data) to store the data

        Args:
            None

        return:
            pixmap, the object of QPixmap()
            if no data, return None
        """
        # use the buile-in function to query image from http, and save in data
        data = self.queryImage.queryImage()
        if not data:
            return None
        pixmap = QPixmap()
        # get pixmap type data from http type data
        pixmap.loadFromData(data)
        return pixmap

    def reflash_frame(self):
        """Reflash frame on widget label_snapshot

        Use the return value of transToPixmap() to reflash the frame on widget label_snapshot

        Args:
            None
        """
        # this pixmap is the received and converted picture
        pixmap = self.transToPixmap()

        print('reflash')

        if pixmap:
            # show the pixmap on widget label_snapshot
            self.label_snapshot.setPixmap(pixmap)
            print('ok')
        else:
            print("frame lost")

    def level_btn_show(self, speed_level):
        """Reflash the view of level_btn

        Whit this function call, all level_btns change to a unpressed status except one that be clicked recently

        Args:
            1~5, the argument speed_level  means the button be clicked recently
        """
        # set all buttons stylesheet unpressed
        self.level1.setStyleSheet("border-image: url(./images/speed_level_1_unpressed.png);")
        self.level2.setStyleSheet("border-image: url(./images/speed_level_2_unpressed.png);")
        self.level3.setStyleSheet("border-image: url(./images/speed_level_3_unpressed.png);")
        self.level4.setStyleSheet("border-image: url(./images/speed_level_4_unpressed.png);")
        self.level5.setStyleSheet("border-image: url(./images/speed_level_5_unpressed.png);")
        if speed_level == 1:  # level 1 button is pressed
            self.level1.setStyleSheet("border-image: url(./images/speed_level_1_pressed.png);")
        elif speed_level == 2:  # level 2 button is pressed
            self.level2.setStyleSheet("border-image: url(./images/speed_level_2_pressed.png);")
        elif speed_level == 3:  # level 3 button is pressed
            self.level3.setStyleSheet("border-image: url(./images/speed_level_3_pressed.png);")
        elif speed_level == 4:  # level 4 button is pressed
            self.level4.setStyleSheet("border-image: url(./images/speed_level_4_pressed.png);")
        elif speed_level == 5:  # level 5 button is pressed
            self.level5.setStyleSheet("border-image: url(./images/speed_level_5_pressed.png);")

    def set_speed_level(self, speed):  # set speed to server
        run_speed(speed)

    def keyPressEvent(self, event):
        """Keyboard press event

        Effective key: W, A, S, D, ↑,  ↓,  ←,  →
        Press a key on keyboard, the function will get an event, if the condition is met, call the function
        run_action().

        Args:
            event, this argument will get when an event of keyboard pressed occured

        """
        key_press = event.key()

        # don't need autorepeat, while haven't released, just run once
        if not event.isAutoRepeat():
            if key_press == Qt.Key_Up:  # up
                run_action('camup')
            elif key_press == Qt.Key_Right:  # right
                run_action('camright')
            elif key_press == Qt.Key_Down:  # down
                run_action('camdown')
            elif key_press == Qt.Key_Left:  # left
                run_action('camleft')
            elif key_press == Qt.Key_W:  # W
                run_action('forward')
            elif key_press == Qt.Key_A:  # A
                run_action('fwleft')
            elif key_press == Qt.Key_S:  # S
                run_action('backward')
            elif key_press == Qt.Key_D:  # D
                run_action('fwright')

    def keyReleaseEvent(self, event):
        """Keyboard released event

        Effective key: W,A,S,D, ↑,  ↓,  ←,  →
        Release a key on keyboard, the function will get an event, if the condition is met, call the function
        run_action().

        Args:
            event, this argument will get when an event of keyboard release occured

        """
        # don't need autorepeat, while haven't pressed, just run once
        key_release = event.key()
        if not event.isAutoRepeat():
            if key_release == Qt.Key_Up:  # up
                run_action('camready')
            elif key_release == Qt.Key_Right:  # right
                run_action('camready')
            elif key_release == Qt.Key_Down:  # down
                run_action('camready')
            elif key_release == Qt.Key_Left:  # left
                run_action('camready')
            elif key_release == Qt.Key_W:  # W
                run_action('stop')
            elif key_release == Qt.Key_A:  # A
                run_action('fwstraight')
            elif key_release == Qt.Key_S:  # S
                run_action('stop')
            elif key_release == Qt.Key_D:  # D
                run_action('fwstraight')

    def on_level1_clicked(self):
        """Slot for signal that level1 button clicked

        The level1 button clicked, this function will run. Call function level_btn_show()
        and function set_speed_level(), level1 set argument '20' to set_speed_level()

        """
        self.speed_level = 1
        self.level_btn_show(self.speed_level)
        self.set_speed_level(str(self.LEVEL_SPEED[self.speed_level]))  # level 1, speed 20

    def on_level2_clicked(self):
        self.speed_level = 2
        self.level_btn_show(self.speed_level)
        self.set_speed_level(str(self.LEVEL_SPEED[self.speed_level]))  # level 2, speed 40

    def on_level3_clicked(self):
        self.speed_level = 3
        self.level_btn_show(self.speed_level)
        self.set_speed_level(str(self.LEVEL_SPEED[self.speed_level]))  # level 3, speed 60

    def on_level4_clicked(self):
        self.speed_level = 4
        self.level_btn_show(self.speed_level)
        self.set_speed_level(str(self.LEVEL_SPEED[self.speed_level]))  # level 4, speed 80

    def on_level5_clicked(self):
        self.speed_level = 5
        self.level_btn_show(self.speed_level)
        self.set_speed_level(str(self.LEVEL_SPEED[self.speed_level]))  # level 5, speed 100

    def on_btn_back_pressed(self):
        self.btn_back.setStyleSheet("border-image: url(./images/back_pressed.png);")

    def on_btn_back_released(self):
        self.btn_back.setStyleSheet("border-image: url(./images/back_unpressed.png);")

    def on_btn_back_clicked(self):
        """Slot for signal that back button clicked

        The back button clicked, this function will run. Close this screen and stop
        stream receive, show login screen

        """
        self.close()
        # close this screen, stop receiving the stream
        self.stop_stream()
        login1.show()

    def on_btn_setting_pressed(self):
        self.btn_setting.setStyleSheet("border-image: url(./images/settings_pressed.png);")

    def on_btn_setting_released(self):
        self.btn_setting.setStyleSheet("border-image: url(./images/settings_unpressed.png);")

    def on_btn_setting_clicked(self):
        """Slot for signal that setting button clicked

        The setting button clicked, this function will run. Close this screen and show setting screen

        """
        self.btn_back.setStyleSheet("border-image: url(./images/back_unpressed.png);")
        self.close()
        setting1.show()


class SettingScreen(QtWidgets.QDialog, Ui_Setting_screen):
    """Setting Screen

    To creat a Graphical User Interface, inherit from Ui_Setting_screen. And define functions
    that use for the control.

    Attributes:
        none
    """

    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        Ui_Setting_screen.__init__(self)
        self.setupUi(self)
        self.setWindowTitle("Calibration - SunFounder PiCar-V Client")
        self.btn_back.setStyleSheet("border-image: url(./images/back_unpressed.png);")

    def on_btn_camera_cali_pressed(self):
        self.btn_camera_cali.setStyleSheet("border-image: url(./images/camera_cali_pressed.png);")

    def on_btn_camera_cali_released(self):
        self.btn_camera_cali.setStyleSheet("border-image: url(./images/camera_cali_unpressed.png);")

    def on_btn_camera_cali_clicked(self):
        calibrate1.calibration_show(1)

    def on_btn_fw_cali_pressed(self):
        self.btn_fw_cali.setStyleSheet("border-image: url(./images/fw_cali_pressed.png);")

    def on_btn_fw_cali_released(self):
        self.btn_fw_cali.setStyleSheet("border-image: url(./images/fw_cali_unpressed.png);")

    def on_btn_fw_cali_clicked(self):
        calibrate1.calibration_show(2)

    def on_btn_bw_cali_pressed(self):
        self.btn_bw_cali.setStyleSheet("border-image: url(./images/bw_cali_pressed.png);")

    def on_btn_bw_cali_released(self):
        self.btn_bw_cali.setStyleSheet("border-image: url(./images/bw_cali_unpressed.png);")

    def on_btn_bw_cali_clicked(self):
        calibrate1.calibration_show(3)

    def on_btn_back_pressed(self):
        self.btn_back.setStyleSheet("border-image: url(./images/back_pressed.png);")

    def on_btn_back_released(self):
        self.btn_back.setStyleSheet("border-image: url(./images/back_unpressed.png);")

    def on_btn_back_clicked(self):
        self.close()
        running1.set_speed_level(str(running1.LEVEL_SPEED[running1.speed_level]))  # level 1, speed 20
        running1.show()


class CalibrateScreen(QtWidgets.QDialog, Ui_Calibrate_screen):
    """Calibrate Screen

    To creat a Graphical User Interface, inherit from Ui_Calibrate_screen. And define functions
    that use for the control.

    Attributes:
        none
    """

    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        Ui_Calibrate_screen.__init__(self)
        self.setupUi(self)
        self.calibration_status = 0
        self.bw_status = 0

        self.btn_test.setStyleSheet("border-image: url(./images/test_unpressed.png);")
        self.btn_ok.setStyleSheet("border-image: url(./images/ok_unpressed.png);")
        self.btn_cancle.setStyleSheet("border-image: url(./images/cancle_unpressed.png);")

    def calibration_show(self, calibration_status):
        """Show calibration screen

        With the argument, show a screen for calibration.argument calibration_status should be 1, 2, or 3, show camera
        calibration, front wheel calibration and back wheel calibration screen and enter the calibration mode

        Args:
            1, 2, 3, will show camera calibration, front wheel calibration and back wheel calibration screen
        """
        self.calibration_status = calibration_status
        if self.calibration_status == 1:  # calibrate camera
            cali_action('camcali')
            self.setWindowTitle("Camera Calibration - SunFounder PiCar-V Client")
            self.label_pic.setStyleSheet("image: url(./images/cali_cam.png);")
            self.label_Cali_Info.setText("Camera")
            self.label_Info_1.setText("Calibrate the camera to the position like above.")
            self.label_Info_2.setText("Use arrow keys or W, A, S, D keys.")
        if self.calibration_status == 2:  # calibrate front wheels
            cali_action('fwcali')
            self.setWindowTitle("Front Wheels Calibration - SunFounder PiCar-V Client")
            self.label_pic.setStyleSheet("image: url(./images/cali_fw.png);")
            self.label_Cali_Info.setText("Front Wheels")
            self.label_Info_1.setText("Calibrate front wheels to the position like above.")
            self.label_Info_2.setText("Use the left and right arrow keys or A and D.")
        if self.calibration_status == 3:  # calibrate back wheels
            cali_action('bwcali')
            run_speed('50')
            run_action('forward')
            self.setWindowTitle("Rear Wheels Calibration - SunFounder PiCar-V Client")
            self.label_pic.setStyleSheet("image: url(./images/cali_bw.png);")
            self.btn_test.hide()
            self.label_Cali_Info.setText("Rear Wheels")
            self.label_Info_1.setText("Calibrate rear wheels to run forward.")
            self.label_Info_2.setText("Use the left and right arrow keys or A and D.")
        self.show()

    def keyPressEvent(self, event):
        """Keyboard press event

        Press a key on keyboard, the function will get an event, if the condition is met, call the function
        run_action().
        In camera calibration mode, Effective key: W,A,S,D, ↑,  ↓,  ←,  →, ESC
        In front wheel calibration mode, Effective key: A, D, ←,  →, ESC
        In back wheel calibration mode, Effective key: A, D, ←,  →, ESC

        Args:
            event, this argument will get when an event of keyboard pressed occured

        """
        key_press = event.key()

        if key_press in (Qt.Key_Up, Qt.Key_W):  # UP
            if self.calibration_status == 1:
                cali_action('camcaliup')
            elif self.calibration_status == 2:
                pass
            elif self.calibration_status == 3:
                pass
        elif key_press in (Qt.Key_Right, Qt.Key_D):  # RIGHT
            if self.calibration_status == 1:
                cali_action('camcaliright')
            elif self.calibration_status == 2:
                cali_action('fwcaliright')
            elif self.calibration_status == 3:
                cali_action('bwcaliright')
        elif key_press in (Qt.Key_Down, Qt.Key_S):  # DOWN
            if self.calibration_status == 1:
                cali_action('camcalidown')
            elif self.calibration_status == 2:
                pass
            elif self.calibration_status == 3:
                pass
        elif key_press in (Qt.Key_Left, Qt.Key_A):  # LEFT
            if self.calibration_status == 1:
                cali_action('camcalileft')
            elif self.calibration_status == 2:
                cali_action('fwcalileft')
            elif self.calibration_status == 3:
                cali_action('bwcalileft')
                cali_action('forward')
        elif key_press == Qt.Key_Escape:  # ESC
            run_action('stop')
            self.close()

    def on_btn_test_pressed(self):
        self.btn_test.setStyleSheet("border-image: url(./images/test_pressed.png);")

    def on_btn_test_released(self):
        self.btn_test.setStyleSheet("border-image: url(./images/test_unpressed.png);")
        if self.calibration_status == 1:
            run_action('camup')
            time.sleep(0.5)
            run_action('camready')
            time.sleep(0.5)
            run_action('camdown')
            time.sleep(0.5)
            run_action('camready')
            time.sleep(0.5)
            run_action('camleft')
            time.sleep(0.5)
            run_action('camready')
            time.sleep(0.5)
            run_action('camright')
            time.sleep(0.5)
            run_action('camready')
        elif self.calibration_status == 2:
            run_action('fwleft')
            time.sleep(0.5)
            run_action('fwready')
            time.sleep(0.5)
            run_action('fwright')
            time.sleep(0.5)
            run_action('fwready')
        elif self.calibration_status == 3:
            pass

    def on_btn_ok_pressed(self):
        self.btn_ok.setStyleSheet("border-image: url(./images/ok_pressed.png);")

    def on_btn_ok_released(self):
        self.btn_ok.setStyleSheet("border-image: url(./images/ok_unpressed.png);")

    def on_btn_ok_clicked(self):
        # if Ok to calibrate, request to save the value
        if self.calibration_status == 1:
            cali_action('camcaliok')
        elif self.calibration_status == 2:
            cali_action('fwcaliok')
        elif self.calibration_status == 3:
            cali_action('bwcaliok')
            cali_action('stop')
        self.close()

    def on_btn_cancle_pressed(self):
        self.btn_cancle.setStyleSheet("border-image: url(./images/cancle_pressed.png);")

    def on_btn_cancle_released(self):
        self.btn_cancle.setStyleSheet("border-image: url(./images/cancle_unpressed.png);")

    def on_btn_cancle_clicked(self):
        # if cancle to calibrate, reset the status
        if self.calibration_status == 1:
            run_action('camready')
        elif self.calibration_status == 2:
            run_action('fwready')
        elif self.calibration_status == 3:
            run_action('bwready')
            cali_action('stop')
        self.close()


class QueryImage:
    """Query Image

    Query images form http. eg: queryImage = QueryImage(HOST)

    Attributes:
        host, port. Port default 8080, post need to set when creat a new object

    """

    def __init__(self, host, port=8080, argv="/?action=snapshot"):
        # default port 8080, the same as mjpg-streamer server
        self.host = host
        self.port = port
        self.argv = argv

    def queryImage(self):
        """Query Image

        Query images form http.eg:data = queryImage.queryImage()

        Args:
            None

        Return:
            returnmsg.read(), http response data
        """
        http_data = http.client.HTTPConnection(self.host, self.port)
        http_data.putrequest('GET', self.argv)
        http_data.putheader('Host', self.host)
        http_data.putheader('User-agent', 'python-http.client')
        http_data.putheader('Content-type', 'image/jpeg')
        http_data.endheaders()
        returnmsg = http_data.getresponse()
        return returnmsg.read()


def connection_ok():
    """Check whetcher connection is ok

    Post a request to server, if connection ok, server will return http response 'ok'

    Args:
        none

    Returns:
        if connection ok, return True
        if connection not ok, return False

    Raises:
        none
    """
    cmd = 'connection_test'
    url = BASE_URL + cmd + "/"
    print('url: %s' % url)
    # if server find there is 'connection_test' in request url, server will response 'Ok'
    try:
        r = requests.get(url)
        if r.text == 'OK':
            return True
    except:
        return False


def __request__(url, times=10):
    for x in range(times):
        try:
            requests.get(url)

            return 0
        except:
            print("Connection error, try again")
    print("Abort")
    return -1


def run_action(cmd):
    """Ask server to do sth, use in running mode

    Post requests to server, server will do what client want to do according to the url.
    This function for running mode

    Args:
        # ============== Back wheels =============
        'bwready' | 'forward' | 'backward' | 'stop'

        # ============== Front wheels =============
        'fwready' | 'fwleft' | 'fwright' |  'fwstraight'

        # ================ Camera =================
        'camready' | 'camleft' | 'camright' | 'camup' | 'camdown'
    """
    # set the url include action information
    url = BASE_URL + 'run/?action=' + cmd
    print('url: %s' % url)
    # post request with url
    __request__(url)


def run_speed(speed):
    """Ask server to set speed, use in running mode

    Post requests to server, server will set speed according to the url.
    This function for running mode.

    Args:
        '0'~'100'
    """
    # Set set-speed url
    url = BASE_URL + 'run/?speed=' + speed
    print('url: %s' % url)
    # Set speed
    __request__(url)


def cali_action(cmd):
    """Ask server to do sth, use in calibration mode

    Post requests to server, server will do what client want to do according to the url.
    This function for calibration mode

    Args:
        # ============== Back wheels =============
        'bwcali' | 'bwcalileft' | 'bwcaliright' | 'bwcaliok'

        # ============== Front wheels =============
        'fwcali' | 'fwcalileft' | 'fwcaliright' |  'fwcaliok'

        # ================ Camera =================
        'camcali' | 'camcaliup' | 'camcalidown' | 'camcalileft' | 'camright' | 'camcaliok'

    """
    # set the url include cali information
    url = BASE_URL + 'cali/?action=' + cmd
    print('url: %s' % url)
    # post request with url
    __request__(url)


class Lane:

    def __init__(self):
        self.lower = np.array([255, 255, 255])
        self.upper = np.array([200, 200, 200])
        self.steering_angle = 90
        self.stabilized_steering_angle = 90

    def drive(self, frame):
        lane_lines, lane_frame = self.detect_lane(frame)
        self.steering_angle = self.compute_steering_angle(lane_frame, lane_lines)
        self.stabilized_steering_angle = self.stabilize_steering_angle(self.stabilized_steering_angle,
                                                                       self.steering_angle,
                                                                       len(lane_lines))

        return self.stabilized_steering_angle

    def detect_lane(self, frame):
        edges = self.detect_edge(frame)
        edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(edges)
        lane_lines = self.average_slope_intercept(frame, line_segments)
        lane_frame = self.display_lines(frame, lane_lines)
        return lane_lines, lane_frame

    def detect_edge(self, frame):
        edge_mask = cv2.inRange(frame, self.upper, self.lower)
        mask = cv2.bitwise_and(frame, frame, mask=edge_mask)
        edges = cv2.Canny(mask, 200, 400)
        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(edges, mask)
        return masked_image

    def detect_line_segments(self, edges):
        rho = 1  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                        maxLineGap=4)
        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []
        height, width, _ = frame.shape
        left_fit = []
        right_fit = []
        boundary = 1 / 3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))
        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))
        return lane_lines

    def compute_steering_angle(self, lane_frame, lane_lines):
        if len(lane_lines) == 0:
            return -90

        height, width, _ = lane_frame.shape
        if len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.02
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg + 90
        return steering_angle

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 * 1 / 2)
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def display_lines(self, frame, lane_lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lane_lines is not None:
            for line in lane_lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        lane_frame = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return lane_frame

    def stabilize_steering_angle(self, stabilized_steering_angle, steering_angle, num_of_lane_line):

        max_angle_deviation_two_lines = 5
        max_angle_deviation_one_lane = 1
        if num_of_lane_line == 2:
            max_angle_deviation = max_angle_deviation_two_lines
        else:
            max_angle_deviation = max_angle_deviation_one_lane
        angle_deviation = steering_angle - stabilized_steering_angle
        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(stabilized_steering_angle
                                            + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = steering_angle
        return stabilized_steering_angle


def drive(self, speed, steering_angle):
    # self.steering_angle = self.Lane.stabilized_steering_angle
    self.steering_angle = steering_angle
    self.front_wheels.turn(self.steering_angle)
    max_turning_angle = 10
    if 70 < steering_angle < 110:
        self.back_wheels.speed = speed
    elif 50 < steering_angle < 70:
        self.back_wheels.speed = speed - 30
    elif 110 < steering_angle < 130:
        self.back_wheels.speed = speed - 30


def main():
    app = QtWidgets.QApplication(sys.argv)

    # creat objects
    login1 = LoginScreen()
    running1 = RunningScreen()
    setting1 = SettingScreen()
    calibrate1 = CalibrateScreen()

    # Show object login1
    login1.show()

    print("All done")
    # Wait  python if there ito exits a exec_() signal
    sys.exit(app.exec_())


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    # creat objects
    login1 = LoginScreen()
    running1 = RunningScreen()
    setting1 = SettingScreen()
    calibrate1 = CalibrateScreen()

    # Show object login1
    login1.show()

    print("All done")
    # Wait to exit python if there is a exec_() signal
    sys.exit(app.exec_())
