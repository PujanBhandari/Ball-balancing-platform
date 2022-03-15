# -*- coding: utf-8 -*-
"""
Created on Fri Aug 13 10:58:12 2021

@author: Pujan Bhandari
"""

""" Used libraries """
import tkinter as tk
from tkinter import tix
import tkinter.messagebox
import cv2
import numpy as np
import time
import imutils
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from csv import writer
import csv
import math

import sys
import glob
import serial
import re

import configparser


""" Used global variables """

global I_switch # variable to check if the integral branch is turned on
global D_switch # variable to check if the derivative branch is turned on
global theWriter # variable to store the reference value
global setpoint # variable to store the reference value
global beam_length # variable to store the length of the a beam(platform) as input form the user


global x_incm #variable to store the position of the ball when converted from pixels into centimeters
global prev_elapsedTime
global prev_integ
global integ_max
global PID
global prev_deriv
global prev_PID
global saturation
global openloop_switch
global time_datatoservo_start # variable for the deadtime of the servomotor
global time_servo_need
global PID
global servo_signal # signal to send to the servomotor/ signal to pass through the serial path
global com_port # neme of the com port where the arduino is connected
global error # variable for error signal
global prev_ball_pos # previous position of the ball in cm
global I_windup_bool # variable to check if the path to avoid integral windup is activated
global LPF_bool # variable to check if the low pass filter is used
global D_kick_bool # variable to check if the bypass path to avoid derivative kick is activated
global FPS
global conv_no
global stepinput_bool
global ballpos_tocalibrate # helping variabel to denote the position of the ball in pixels for minimum and the maximum possible range of the beam
global x_borderMax # maximum position where the ball can reach in pixels
global x_borderMin # minimum position where the ball can reach in pixels
global x_pos  # helping variable to display in the label in GUI
global filename #variable for the name of the .csv file to store the measurements
global tf_factor # a factor which is used to multiply with the derivative time constant and use as the time constat for the low pass filter
global deriv #variable to store the output from the derivative controller

global camera_connected_bool # helping variable to give feedback if camera is connected
global arduino_connected_bool # helping variable to give feedback if arduino is connected
global cam #variable for the name of the frame captured by the camera
global actual_fps # variable to calculate the actual FPS of the camera during operation


global camHeight # height of the frame 
global camWidth # width of the frame


""" Helping variables to make the deflection range of the servomotor adjustabla (Vertival slider under the servomotor block)""" 
global initial_max_angle
global initial_min_angle
global min_angle
global max_angle
global neutral_angle
global deflection_range


global lowerBound_ball_input # variable to save the lower HSV colour range of the ball
global upperBound_ball_input # variable to save the upper HSV colour range of the ball

global camera_no # camera number to use for the demonstration setup goes from 0, 1, 2....
global angle # variable to store the computed angel to send to the serial path

global time_signaltoservo # helping variable to check the interval to send the signal to the serial path
global deadangle 

global prev_angle
global deadband_angle # variable to store the value of the deadband for the angle obtained from the slider in GUI
global deadband_error # variable to store the value of the deadband for the error signal obtained from the slider in GUI

global deadband_angle_switch_bool
global deadband_error_switch_bool

global input_Dsignal
global previous_input_Dsignal
global baudrate # variable to store the baudrate for serial communication

global comports # variable to store all available comports 

global l_h ,l_s,l_v ,u_h,u_s,u_v # helping variables to read each HSV values(upper and lower) from the config file

global config

global resol

"""2D global variables"""

global setpoint_x # variable to store the setpoint in x-direction
global setpoint_y #variable to store the setpoint in y-direction
global x_incm
global prev_elapsedTime
global prev_integ_x
global prev_integ_y

global integ_max
global PID_x
global PID_y

global prev_deriv_x
global prev_deriv_y

global prev_PID_x
global prev_PID_y

global servo_signal_x #sigbal to feed the servo motor  to control the ball in x-direction
global servo_signal_y  #sigbal to feed the servo motor  to control the ball in x-direction

global error_x
global error_y

global prev_ball_pos_x
global prev_ball_pos_y

global deriv_x
global deriv_y

global neutral_angle_A
global neutral_angle_B
global neutral_angle_C
global neutral_angle_D

global angle_x
global angle_y

global x_px
global y_px

global x_cm
global y_cm

global x_length #length of the plate in x-direction
global y_length #length of the plate in y-direction


global origin_x #origin on the plate for x-direction
global origin_y #origin on the plate for y-direction

global x_pxmax 
global y_pxmax


""" needed variables """

config = configparser.ConfigParser()

"""2D variables """
prev_angle_x = 0.0
prev_angle_y = 0.0

previous_error_x =0.0
previous_error_y =0.0

integ_x =0.0
integ_y =0.0


deriv_x = 0.0
deriv_y = 0.0

prev_integ_x = 0.0
prev_integ_y = 0.0


prev_deriv_x = 0.0
prev_deriv_y = 0.0

integer_max = 0.0

PID_x = 0.0
PID_y = 0.0


servo_signal_x = 0.0
servo_signal_y = 0.0


prev_PID_x = 0.0
prev_PID_y = 0.0

error_x = 0.0
error_y = 0.0

angle_x = 0.0
angle_y = 0.0

prev_ball_pos_x = 0
prev_ball_pos_y = 0



x_incm = 0.0
y_incm = 0.0

x_px = 0 
y_px = 0

x_cm = 0
y_cm= 0

setpoint_x = x_cm/2
setpoint_y = y_cm/2

origin_x = 0
origin_y = 0

x_pxmax = 0
y_pxmax = 0

x_length = 0
y_length = 0




prev_angle = 0.0
previous_error=0.0
prev_elapsedTime = 0.0
timenow=0.0
pid_i =0.0
integ =0.0
deriv = 0.0
prev_integ = 0.0
prev_deriv = 0.0
integer_max = 0.0
PID = 0.0
servo_signal = 0.0
prev_PID = 0.0
error = 0.0
saturation = False
prev_ball_pos = 0
conv_no = 5
tf_factor = 0.1
FPS = 30 # setting the FPS at 30 as most USB cameras have 30FPS, this can be checked by using a online FPS checker website.
camHeight = 550 #suitable for the GUI
camWidth = 600 #suitable for the GUI
actual_fps = 0 # this will be used to see the effective FPS of the camera during operation
deadband = 0
time_signaltoservo = 1 #default value for the deadtime of of the servomotor, this is adjustable in the GUI

deadband_angle = 0 # default value for the deadband on the angle going to the servo motor.
deadband_error = 0 # default value for the deadband on the error signal.

l_h = 0
l_s= 0
l_v= 0
u_h= 0
u_s= 0
u_v = 0

I_switch = 0
D_switch = 0
openloop_switch = 1

x_borderMax = 0
x_borderMin = 0
x_pos = 0

beam_length = 90
setpoint = beam_length/2

x_incm = 0.0
time_start  = time.time()

time_start_graph = time.time()
time_datatoservo_start = time.time()
time_servo_need = 0

x_value = []
y_value = []
y_errorvalue = []
y_setpointvalue = []


input_Dsignal = 0.0
previous_input_Dsignal = 0.0
        
initial_min_angle = 0
initial_max_angle = 180
deflection_range = 90
min_angle = 0
max_angle = 180
neutral_angle = 90
angle  = 0

# neutral_angle_A = 0
# neutral_angle_B = 0
# neutral_angle_C = 0
# neutral_angle_D = 0

baudrate = 115200 #this value must match the boudrate that arduino uses for the serial communication
config = configparser.ConfigParser()
com_port =''

""" the main window of GUI"""
main_Window = tix.Tk() # tix is used because it allows placing information as tooltip for various objects in GUI
main_Window.title("PID GUI Ball Balancer") # Titel of the main window
main_Window.geometry("1900x700+5+5") # size of the main window for GUI starting from 5 pixels right and under the topleft corner of the screen
main_Window["bg"]="light blue" # background colour of the mainwindow, not visible in GUI because of background image
main_Window.resizable(0, 0) # the GUI is not resizable because of the placement of different objects in the GUI

"""importing the background image(blockdiagram), resizing it to make it fit in the main window of GUI,
placing a label on the main window and placing the resized image in the label"""
my_pic = Image.open("Background.png")
resized = my_pic.resize((1900,700),Image.ANTIALIAS)
bg = ImageTk.PhotoImage(resized,master = main_Window)
bg_lbl = tk.Label(main_Window,image = bg)
bg_lbl.place(x= 0,y = 0, relwidth = 1, relheight = 1)

""" Window for calibration of the camera"""
cameracalibration_window = tix.Toplevel(main_Window)
cameracalibration_window.geometry('500x400')
cameracalibration_window.title("Camera calibration")
cameracalibration_window["bg"]="white"
cameracalibration_window.resizable(0, 0) 
cameracalibration_window.withdraw()

""" video window to see the video from the camera for visual conformation, can be accessed from
the window for camera calibration"""
videoWindow = tk.Toplevel(cameracalibration_window)
videoWindow.title("Webcam Frame")
videoWindow.resizable(0, 0) 
lmain = tk.Label(videoWindow)
lmain.pack()
videoWindow.withdraw()

""" Window to calibrate the servomotor and arduino """
systemcalibration_window = tix.Toplevel(main_Window)
systemcalibration_window.geometry('500x420')
systemcalibration_window.title("System calibration")
systemcalibration_window["bg"]="white"
systemcalibration_window.resizable(0, 0) 
systemcalibration_window.withdraw()


"""Options to select the Demonstration setup: 1D, 2D with 3motors and 2D with 4 motors"""
var_demosetup = tk.IntVar()
var_demosetup.set(1)
    
Frame_radiobutton = tk.LabelFrame(main_Window, text="select the setup", bg ='White')
Frame_radiobutton.place(x=150,y=5,width=220)

radio_1D = tk.Radiobutton(Frame_radiobutton, text="1D setup", variable = var_demosetup,value=1,bg ='White',command=lambda:select_setup())
radio_1D.pack( anchor = 'w' )

radio_2D_3motors = tk.Radiobutton(Frame_radiobutton, text="2D setup with three motors", variable = var_demosetup,value=2,bg ='White',command=lambda:select_setup())
radio_2D_3motors.pack( anchor = 'w' )

radio_2D_4motors = tk.Radiobutton(Frame_radiobutton, text="2D setup with four motors", variable = var_demosetup,value=3,bg ='White',command=lambda:select_setup())
radio_2D_4motors.pack( anchor = 'w')

""" Imporing and resizing the images which will be placed to indicate if the branch is turned on or off"""
bg_open_horizontal = Image.open("open_horizontal.png")
bg_open_horizontal = bg_open_horizontal.resize((50,25),Image.ANTIALIAS)
bg_open_horizontal = ImageTk.PhotoImage(bg_open_horizontal, master = main_Window)

bg_closed_horizontal = Image.open("closed_horizontal.png")
bg_closed_horizontal = bg_closed_horizontal.resize((50,25),Image.ANTIALIAS)
bg_closed_horizontal = ImageTk.PhotoImage(bg_closed_horizontal, master = main_Window)

bg_open_vertical = Image.open("open_vertical.png")
bg_open_vertical = bg_open_vertical.resize((25,50),Image.ANTIALIAS)
bg_open_vertical = ImageTk.PhotoImage(bg_open_vertical, master = main_Window)

bg_closed_vertical = Image.open("closed_vertical.png")
bg_closed_vertical = bg_closed_vertical.resize((25,50),Image.ANTIALIAS)
bg_closed_vertical = ImageTk.PhotoImage(bg_closed_vertical, master = main_Window)

lbl_opencircuit_clamping = tk.Label(main_Window,image = bg_closed_horizontal,bg = "lightgreen")
lbl_opencircuit_clamping.place(x=1010, y=185)

lbl_opencircuit_LPF= tk.Label(main_Window,image = bg_closed_horizontal,bg = "lightgreen")
lbl_opencircuit_LPF.place(x=1010, y=390)

"Default valurs for various sliders"
setposDefalult = 0
sliderCoefKDefault = 0 
sliderCoefTiDefault = 15 
sliderCoefTdDefault = 0 
sliderCoefTf_Default = 0
slider_deadband_Default = 1
slider_deadtime_Default = 1

""" Placing different objects in the main window of the GUI at suitable place accordingly on the 
block diagram. Firstly all the objects is placed in the main window and the objects for system calibration and camera calibration is placed"""
# Sliders used
slider_setpos = tk.Scale(main_Window, from_=0, to=beam_length, orient="horizontal",bg = "white", length=140, width = 15, resolution=0.1)
slider_setpos.set(setposDefalult)
slider_setpos.place(x =0,y = 150)

sliderCoefK = tk.Scale(main_Window, from_=0, to=100, orient="horizontal",bg = "white", length=150, width = 15, resolution=0.01)
sliderCoefK.set(sliderCoefKDefault)
sliderCoefK.place(x =380,y = 120)

slider_set_x_pos = tk.Scale(main_Window, from_= 0, to=x_length, orient="horizontal",bg = "lavender", length=140, width = 15, resolution=0.1)

slider_set_y_pos = tk.Scale(main_Window, from_= 0, to=y_length, orient="vertical",bg = "DarkSlateGray2", length=150, width = 15, resolution=0.1)

sliderCoefK_x = tk.Scale(main_Window, from_=0, to=10, orient="horizontal",label ='K for x',bg = "lavender", length=150, width = 15, resolution=0.01)

sliderCoefK_y = tk.Scale(main_Window, from_=0, to=10, orient="horizontal",label ='K for y',bg = "DarkSlateGray2", length=150, width = 15, resolution=0.01)


sliderCoefTi = tk.Scale(main_Window, from_=0.01, to=15, orient="horizontal",bg = "white", length=200, width = 15, resolution=0.01)
sliderCoefTi.set(sliderCoefTiDefault)
sliderCoefTi.place(x =690,y = 120)

sliderCoefTd = tk.Scale(main_Window, from_=0, to=15, orient="horizontal", bg = "white",length=200, width = 15, resolution=0.01)
sliderCoefTd.set(sliderCoefTdDefault)
sliderCoefTd.place(x =690,y = 320)

sliderCoefTf_factor = tk.Scale(main_Window, from_=0, to=1, orient="horizontal", bg = "lightgreen",length=200, width = 15, resolution=0.001)
sliderCoefTf_factor.set(sliderCoefTf_Default)
sliderCoefTf_factor.place(x =955,y = 525)

sliderCoefdeadband_error= tk.Scale(main_Window, from_=0, to=5, orient="horizontal", bg = "red",length=130, width = 15, resolution=0.1)
sliderCoefdeadband_error.set(slider_deadband_Default)
sliderCoefdeadband_error.place(x =230,y = 240)


sliderCoefdeadband_angle = tk.Scale(main_Window, from_=0, to=20, orient="horizontal", bg = "red",length=100, width = 15, resolution=0.1)
sliderCoefdeadband_angle.set(slider_deadband_Default)
sliderCoefdeadband_angle.place(x =1400,y = 245)

sliderdeadtime_servo = tk.Scale(main_Window, from_=0, to=1, orient="horizontal", bg = "white",length=100, width = 15, resolution=0.01)
sliderdeadtime_servo.set(1)
sliderdeadtime_servo.place(x =1710,y = 245)

slidermaxdeflection_servo = tk.Scale(main_Window, from_=0, to=1, orient="vertical", bg = "blue",fg = 'white',length=100, width = 15, resolution=0.01)
slidermaxdeflection_servo.set(1)
slidermaxdeflection_servo.place(x =1345,y = 245)

# Buttons used
Btn_StartPID = tk.Button(main_Window,text = "Start" + "\n" + "controller", bg = 'lightgreen', command=lambda:startPID())
Btn_StartPID.place(x=60, y=210)

Btn_Switch_I = tk.Button(main_Window, image = bg_open_horizontal, bg= "red", command = lambda:ISwitch())
Btn_Switch_I.place(x=550, y=185)

Btn_Switch_Iwindup = tk.Button(main_Window,image = bg_open_vertical, bg= "red", command = lambda: I_windupfun())
Btn_Switch_Iwindup.place(x=935 , y=140)


Btn_Switch_D = tk.Button(main_Window,image = bg_open_horizontal, bg= "red", command = lambda: DSwitch())
Btn_Switch_D.place(x=550, y=390)

Btn_Switch_Dervkick = tk.Button(main_Window,image = bg_open_vertical, bg= "red", command = lambda:d_kickfun() )
Btn_Switch_Dervkick.place(x=615, y=450)

Btn_Switch_LPF = tk.Button(main_Window, image = bg_open_vertical, bg= "red", command = lambda:LPF_fun() )
Btn_Switch_LPF.place(x=920, y=420)

Btn_Switch_closedloop = tk.Button(main_Window, image = bg_closed_vertical, bg= "lightgreen", command = lambda: openloop_switchfun())
Btn_Switch_closedloop.place(x=162, y=395)


Btn_systemcalibration_window = tk.Button(main_Window, text = "Calibration", bg = 'white', command=lambda:showsystemcalibration_window())
Btn_systemcalibration_window.place(x=1595, y=200)

Btn_cameracalibration_window = tk.Button(main_Window, text = "Calibration",bg = 'white', command=lambda:showcameracalibration_window())
Btn_cameracalibration_window.place(x=860, y=640)


""" Frame where different buttons are packed together"""
FrameButtons = tk.LabelFrame(main_Window, text="Options", bg ='White')
FrameButtons.place(x=5,y=500,width=130)

Btn_reset = tk.Button(FrameButtons, text = "Reset controller",bg = 'white', command=lambda:reset_fun())
Btn_reset.pack(fill = "both")

btn_startrec = tk.Button(FrameButtons,text = "start recording",bg = 'white',command = lambda:start_rec())
btn_startrec.pack(fill = "both")


Btn_autoconnect = tk.Button(FrameButtons, text="Auto connect",bg = 'white', command=lambda:autoconnectfun())
Btn_autoconnect.pack(fill = "both")

Btn_graph = tk.Button(FrameButtons, text="Show graph",bg = 'white', command=lambda:showGraphWindow())
Btn_graph.pack(fill = "both")

""" Frame where the objects for taking inputs for step response experiment is packed together"""
Frame_stepinput = tk.LabelFrame(main_Window, text="Step input", bg ='White')
Frame_stepinput.place(x=10,y=10,width=120)

lbl_stepsize= tk.Label(Frame_stepinput,text="Step size",bg = 'white',fg = 'black')
lbl_stepsize.grid(row = 0,column = 0)

txtinput_stepinput = tk.Entry(Frame_stepinput,width = 5)
txtinput_stepinput.grid(row = 0,column = 1)

Btn_step_input = tk.Button(Frame_stepinput, text = "Apply",bg = 'white', command =lambda:step_inputfun())
Btn_step_input.grid(row = 1,column = 0)

""" Frame where different labels are packed together, the background colour changes accorindgly to indicate is something is on or off"""
Frame_indicator = tk.LabelFrame(main_Window, text="Indicators", bg ='White')
Frame_indicator.place(x=1700,y=5,width=130)

lbl_indi_controller= tk.Label(Frame_indicator,text="",bg = 'red',width = 5)
lbl_indi_controller.grid(row = 0,column = 0)

lbl_indi_cont =  tk.Label(Frame_indicator,text="Controller",bg = 'white')
lbl_indi_cont.grid(row = 0,column = 1)


lbl_indi_arduino= tk.Label(Frame_indicator,text="",bg = 'red',width = 5)
lbl_indi_arduino.grid(row = 1,column = 0)

lbl_indi_ard =  tk.Label(Frame_indicator,text="Arduino",bg = 'white')
lbl_indi_ard.grid(row = 1,column = 1)


lbl_indi_camera= tk.Label(Frame_indicator,text="",bg = 'red',width = 5)
lbl_indi_camera.grid(row = 2,column = 0)

lbl_indi_cam =  tk.Label(Frame_indicator,text="Camera",bg = 'white')
lbl_indi_cam.grid(row = 2,column = 1)


lbl_indi_recording= tk.Label(Frame_indicator,text="",bg = 'red',width = 5)
lbl_indi_recording.grid(row = 3,column = 0)

lbl_indi_rec =  tk.Label(Frame_indicator,text="Recording",bg = 'white')
lbl_indi_rec.grid(row = 3,column = 1)


"""label used """
lbl_PID_gegevens = tk.Label(main_Window,text="",bg = 'white',fg = 'black')
lbl_PID_gegevens.place(x=1500,y=350)

""" Variables to give a tool-tip for selected objects"""
tip_Iswitch = tix.Balloon(main_Window)
tip_Icamping = tix.Balloon(main_Window)
tip_Dswitch = tix.Balloon(main_Window)
tip_D_kick = tix.Balloon(main_Window)
tip_LPF = tix.Balloon(main_Window)
tip_openloop = tix.Balloon(main_Window)

tip_tf_factor = tix.Balloon(main_Window)
tip_tf_factor.bind_widget(sliderCoefTf_factor, balloonmsg = "Chosen value will be multiplied with the "+"\n"+"timeconstant(Td) of derivative controller.")

tip_deadband_angle = tix.Balloon(main_Window)
tip_deadband_angle.bind_widget(sliderCoefdeadband_angle, balloonmsg = "Within this deadband, the servo will not rotate.")

tip_deadband_error = tix.Balloon(main_Window)
tip_deadband_error.bind_widget(sliderCoefdeadband_error, balloonmsg = "Within this deadband, the output of the controller"+"\n"+"will be null.")

tip_deadtime_servo = tix.Balloon(main_Window)
tip_deadtime_servo.bind_widget(sliderdeadtime_servo, balloonmsg = "This is the time difference between "+"\n"+"two servo pulses.")

tip_maxdeflection = tix.Balloon(main_Window)
tip_maxdeflection.bind_widget(slidermaxdeflection_servo, balloonmsg = "This is the maximum deflection of the "+"\n"+"servoarm in both directions.")



""""dropdown menu for selecting manipulating operation from the signal manipulator block"""
clicked = tk.StringVar()
clicked.set("linear")
drop_block = tk.OptionMenu(main_Window, clicked, "linear", "Derivate", "Integrate")
drop_block.place(x=1207,y = 190)




""" Placing different objects in the window for system calibration """ 
Btn_arduino_refresh= tk.Button(systemcalibration_window, text = "Refresh com ports", command = lambda: refresh_comports())
Btn_arduino_refresh.place(x = 200, y = 20)

Btn_arduino= tk.Button(systemcalibration_window, text = "Connect Arduino", command = lambda: connect_arduinofun())
Btn_arduino.place(x = 20, y = 60)

lbl_arduino_connect = tk.Label(systemcalibration_window,text="Arduino not connected",bg = 'red',fg = 'black')
lbl_arduino_connect.place(x = 150, y = 60)

lbl_neutral_angle = tk.Label(systemcalibration_window,text="neutral" +"\n"+"servo angle = ",bg = 'white',fg = 'black')
lbl_neutral_angle.place(x= 20,y = 95)

txtinput_neutral_angle = tk.Entry(systemcalibration_window)
txtinput_neutral_angle.place(x= 130,y = 115)

lbl_angle_toservo = tk.Label(systemcalibration_window,text="check angle",bg = 'white',fg = 'black')
lbl_angle_toservo.place(x= 300,y = 115)

txtinput_angle_toservo = tk.Entry(systemcalibration_window,width = 10)
txtinput_angle_toservo.place(x= 400,y = 115)


btn_applyangle = tk.Button(systemcalibration_window,text = "Apply",command = lambda:signal_toservo())
btn_applyangle.place(x = 400,y = 140)


lbl_min_angle = tk.Label(systemcalibration_window,text="minimim" +"\n"+"servo angle = ",bg = 'white',fg = 'black')
lbl_min_angle.place(x= 20,y = 140)

txtinput_min_angle = tk.Entry(systemcalibration_window)
txtinput_min_angle.place(x= 130,y = 160)


lbl_max_angle = tk.Label(systemcalibration_window,text="maximum" +"\n"+"servo angle = ",bg = 'white',fg = 'black')
lbl_max_angle.place(x= 20,y = 190)

txtinput_max_angle = tk.Entry(systemcalibration_window)
txtinput_max_angle.place(x= 130,y = 210)


Btn_applyall= tk.Button(systemcalibration_window, text = "Apply", command = lambda:applyall_fun())
Btn_applyall.place(x = 150, y = 300)


Btn_finish_systemcalibration = tk.Button(systemcalibration_window, text = "OK",bg = 'white', command=lambda:finish_systemcalibration())
Btn_finish_systemcalibration.place(x = 20, y = 300)


tip_neutral_angle = tix.Balloon(systemcalibration_window)
tip_neutral_angle.bind_widget(txtinput_neutral_angle, balloonmsg = "At this angle, the platform should be "+"\n"+"absolutely horizontal.")

tip_min_angel = tix.Balloon(systemcalibration_window)
tip_neutral_angle.bind_widget(txtinput_min_angle, balloonmsg = "Enter the minimum effective"+"\n"+"deflection of the servo motor.")

tip_max_angle = tix.Balloon(systemcalibration_window)
tip_max_angle.bind_widget(txtinput_max_angle, balloonmsg = "Enter the minimum effective"+"\n"+"deflection of the servo motor.")

tip_btn_applyangle = tix.Balloon(systemcalibration_window)
tip_btn_applyangle.bind_widget(btn_applyangle, balloonmsg = "Send given angle to the servomotor")


"""Placing different objects on the window for camera calibration"""
lbl_camera_no = tk.Label(cameracalibration_window,text="Camera no. = ",bg = 'white',fg = 'black')
lbl_camera_no.place(x = 20, y = 20)

txtinput_camera_no = tk.Entry(cameracalibration_window)
txtinput_camera_no.place(x = 160, y = 20)

Btn_setup_HSV = tk.Button(cameracalibration_window, text = "Open HSV Window", command = lambda:openHSV_window())
Btn_setup_HSV.place(x = 20, y = 80)


Btn_resetcamera= tk.Button(cameracalibration_window, text = "Refresh camera", command = lambda:refreshcamera())
Btn_resetcamera.place(x = 180, y = 80)

Btn_camera = tk.Button(cameracalibration_window, text = "Connect camera",command = lambda: connect_cemerafun())
Btn_camera.place(x = 20, y = 130)

lbl_camera_connect = tk.Label(cameracalibration_window,text="camera not connected",bg = 'red',fg = 'white')
lbl_camera_connect.place(x = 160, y = 130)

Btn_ShowVideo = tk.Button(cameracalibration_window, text="Show video", command= lambda:showCameraFrameWindow())
Btn_ShowVideo.place(x = 20, y = 170)


lbl_info = tk.Label(cameracalibration_window,text="Please enter the lengths at first"+"\n"+"to convert the unit of position from pixels into cm",bg = 'white',fg = 'black')
lbl_info.place(x=120,y=170)

lbl_x_length = tk.Label(cameracalibration_window,text="x-length",bg = 'white',fg = 'black')
lbl_y_length = tk.Label(cameracalibration_window,text="y-length",bg = 'white',fg = 'black')
txtinput_x_length = tk.Entry(cameracalibration_window)
txtinput_y_length = tk.Entry(cameracalibration_window)
lbl_posinfo = tk.Label(cameracalibration_window,text="",bg = 'white',fg = 'black')
Btn_setorigin = tk.Button(cameracalibration_window, text = "set origin", command = lambda:set_origin_fun())
Btn_setxmax = tk.Button(cameracalibration_window, text = "set x-max", command = lambda:set_max_xpos())
Btn_setymax = tk.Button(cameracalibration_window, text = "set y-max", command = lambda:set_max_ypos() )


lbl_beamlength = tk.Label(cameracalibration_window,text="Platform's"+"\n"+" length [cm]= ",bg = 'white',fg = 'black')
lbl_beamlength.place(x=20,y=240)

txtinput_beamlength = tk.Entry(cameracalibration_window)
txtinput_beamlength.place(x= 130,y = 260)

lbl_posinfo = tk.Label(cameracalibration_window,text="",bg = 'white',fg = 'black')
lbl_posinfo.place(x=250,y=300)

Btn_setminpos = tk.Button(cameracalibration_window, text = "set minpos", command = lambda:set_minposfun() )
Btn_setminpos.place(x = 20, y = 300)

Btn_setmaxpos = tk.Button(cameracalibration_window, text = "set maxpos", command = lambda:set_maxposfun() )
Btn_setmaxpos.place(x = 120, y = 300)

""" Here ends the construction of the GUI. The different function used are constructed and assigned to the objects above. """

"""All the objects are created, however only the suitable objects for each demonstration setup should be displayed. Folling function is used to select the necessary objects
for the selected demonstration setup."""
def select_setup():
    global var_demosetup
    global setposDefalult
    global sliderCoefKDefaultµ
    global lbl_x_length
    global lbl_y_length
    global txtinput_x_length
    global txtinput_y_length
    global lbl_posinfo
    global Btn_setorigin
    global Btn_setxmax
    global Btn_setymax
    global Btn_setymax
    global slider_set_x_pos
    global slider_set_y_pos
    global sliderCoefK_x
    global sliderCoefK_y
    global txtinput_beamlength
    global Btn_setminpos
    global Btn_setmaxpos
    global lbl_beamlength

    
    if var_demosetup.get()==1: #if the demonstration setup is 1D setup
        slider_setpos.set(setposDefalult)
        slider_setpos.place(x =0,y = 150)
        
        sliderCoefK.set(sliderCoefKDefault)
        sliderCoefK.place(x =380,y = 120)
        
        lbl_beamlength = tk.Label(cameracalibration_window,text="Platform's"+"\n"+" length [cm]= ",bg = 'white',fg = 'black')
        lbl_beamlength.place(x=20,y=240)

        txtinput_beamlength = tk.Entry(cameracalibration_window)
        txtinput_beamlength.place(x= 130,y = 260)

        lbl_posinfo = tk.Label(cameracalibration_window,text="",bg = 'white',fg = 'black')
        lbl_posinfo.place(x=250,y=300)

        Btn_setminpos = tk.Button(cameracalibration_window, text = "set minpos", command = lambda:set_minposfun() )
        Btn_setminpos.place(x = 20, y = 300)

        Btn_setmaxpos = tk.Button(cameracalibration_window, text = "set maxpos", command = lambda:set_maxposfun() )
        Btn_setmaxpos.place(x = 120, y = 300)
        
        Btn_graph.pack(fill = "both")

        
        slider_set_x_pos.place_forget()
        slider_set_y_pos.place_forget()
        sliderCoefK_x.place_forget()
        sliderCoefK_y.place_forget()
        
        lbl_x_length.place_forget()
        lbl_y_length.place_forget()
        txtinput_x_length.place_forget()
        txtinput_y_length.place_forget()
        lbl_posinfo.place_forget()
        Btn_setorigin.place_forget()
        Btn_setxmax.place_forget()
        Btn_setymax.place_forget()

        
        
        
    else: #for the 2D setups
        slider_setpos.place_forget()
        sliderCoefK.place_forget()
        txtinput_beamlength.place_forget()
        lbl_posinfo.place_forget()
        Btn_setminpos.place_forget()
        Btn_setmaxpos.place_forget()
        lbl_beamlength.place_forget()
        Btn_graph.pack_forget()


        slider_set_x_pos.set(setposDefalult)
        slider_set_x_pos.place(x =0,y = 150)
        
        slider_set_y_pos.set(setposDefalult)
        slider_set_y_pos .place(x =0,y = 210)
        
        sliderCoefK_x.set(sliderCoefKDefault)
        sliderCoefK_x.place(x =380,y = 100)
        
        sliderCoefK_y.set(sliderCoefKDefault)
        sliderCoefK_y.place(x =380,y = 235)
        
        lbl_x_length = tk.Label(cameracalibration_window,text="x-length",bg = 'white',fg = 'black')
        lbl_x_length.place(x=20,y=220)

        lbl_y_length = tk.Label(cameracalibration_window,text="y-length",bg = 'white',fg = 'black')
        lbl_y_length.place(x=20,y=260)

        txtinput_x_length = tk.Entry(cameracalibration_window)
        txtinput_x_length.place(x= 130,y = 220)

        txtinput_y_length = tk.Entry(cameracalibration_window)
        txtinput_y_length.place(x= 130,y = 260)

        lbl_posinfo = tk.Label(cameracalibration_window,text="",bg = 'white',fg = 'black')
        lbl_posinfo.place(x=300,y=300)

        Btn_setorigin = tk.Button(cameracalibration_window, text = "set origin", command = lambda:set_origin_fun())
        Btn_setorigin.place(x = 20, y = 300)

        Btn_setxmax = tk.Button(cameracalibration_window, text = "set x-max", command = lambda:set_max_xpos())
        Btn_setxmax.place(x = 120, y = 300)

        Btn_setymax = tk.Button(cameracalibration_window, text = "set y-max", command = lambda:set_max_ypos() )
        Btn_setymax.place(x = 220, y = 300)



Btn_finish_cameracalibration = tk.Button(cameracalibration_window, text = "OK",bg = 'white', command=lambda:finish_cameracalibration())
Btn_finish_cameracalibration.place(x =20, y = 340)


"""function to register the origin of the plate"""
def set_origin_fun():

    global x_px
    global y_px
    
    global origin_x
    global origin_y
    
    global x_length
    global y_length
    
    global x_pxmax
    global y_pxmax

    
    try:
        x_length = float(txtinput_x_length.get())
        y_length = float(txtinput_y_length.get())

        slider_set_x_pos["to"] = x_length
        slider_set_y_pos["to"] = y_length
        
        origin_x = x_px
        origin_y = y_px
        
        lbl_posinfo.config(text = "Origin is registerd")
    except:
        lbl_posinfo.config(text = "Error")


"""function to register the maximum position in x-direction  """
def set_max_xpos():

    global x_pxmax
    global x_px
    
    x_pxmax = x_px
    lbl_posinfo.config(text = "x max is registerd")
    



"""function to register the maximum position in x-direction  """
def set_max_ypos():
    global y_pxmax
    global y_px
    
    y_pxmax = y_px
    lbl_posinfo.config(text = "y max is registerd")


""" Function to read all the inputs from the system calibration window"""
def applyall_fun():
    global conv_no
    global show_systemcalibration_window
    global min_angle
    global max_angle
    global neutral_angle
    global deadband_angle
    global deadband_error
    global time_signaltoservo
    global initial_max_angle
    global initial_min_angle
    
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D
    global var_demosetup
    

    try:
        if var_demosetup.get()==1:
            initial_min_angle = int(txtinput_min_angle.get())
            initial_max_angle = int(txtinput_max_angle.get())
            neutral_angle = int(txtinput_neutral_angle.get())

        
        if var_demosetup.get()==2:
            neutral_angles = txtinput_neutral_angle.get()
            print(neutral_angles)
            neutral_angles = neutral_angles.split (",")
            print(neutral_angles)
            print(len(neutral_angles))
        
            neutral_angle_A = int(neutral_angles[0])
            neutral_angle_B = int(neutral_angles[1])
            neutral_angle_C = int(neutral_angles[2])
            
        if var_demosetup.get()==3:
            neutral_angles = txtinput_neutral_angle.get()
            print(neutral_angles)
            neutral_angles = neutral_angles.split (",")
            print(neutral_angles)
            print(len(neutral_angles))
        
            neutral_angle_A = int(neutral_angles[0])
            neutral_angle_B = int(neutral_angles[1])
            neutral_angle_C = int(neutral_angles[2])
            neutral_angle_D = int(neutral_angles[3])
   
    except:
        tk.messagebox.showerror("showerror","Error while improting inputs") 
        
        
""" Function used for resetting the controller. All the sliders are set to its default value."""    
def reset_fun():
    global setpoint
    global x_pos
    global neutral_angle
    global PID
    global integ
    global deriv
    global prev_PID
    global prev_integ
    global prev_deriv
    global error 
    global previous_error
    global elapsedTime
    global prev_angle
    global prev_ball_pos
    global prev_elapsedTime
    
    global setpoint_x
    global setpoint_y
    global x_pos
    global y_pos
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D
    
    global PID_x
    global PID_y
    global integ_x
    global integ_y
    global deriv_x
    global deriv_y
    global prev_PID_x
    global prev_PID_y
    global prev_integ_x
    global prev_integ_y
    global prev_deriv_x
    global prev_deriv_y
    global error_x
    global error_y
    global previous_error_x
    global previous_error_y
    global elapsedTime
    global prev_angle_x
    global prev_angle_y
    global prev_ball_pos_x
    global prev_ball_pos_y
    
    global var_demosetup
    
    global slider_deadtime_Default

    
    previous_error = 0
    prev_elapsedTime = 0
    prev_integ = 0
    prev_deriv = 0
    prev_PID = 0
    prev_ball_pos = 0
    prev_angle = 0
    error = 0
    integ = 0
    deriv = 0
    
    previous_error_x = 0
    prev_elapsedTime = 0
    prev_integ_x = 0
    prev_deriv_x = 0
    prev_PID_x = 0
    prev_ball_pos_x = 0
    prev_angle_x = 0
    error_x = 0
    integ_x = 0
    deriv_x = 0
    
    previous_error_y = 0
    prev_integ_y = 0
    prev_deriv_y = 0
    prev_PID_y = 0
    prev_ball_pos_y = 0
    prev_angle_y = 0
    error_y = 0
    integ_y = 0
    deriv_y = 0

    
    slider_setpos.set(0)
    sliderCoefK.set(sliderCoefKDefault)
    sliderCoefTi.set(sliderCoefTiDefault)
    sliderCoefTd.set(sliderCoefTdDefault) 
    sliderCoefTf_factor.set(sliderCoefTf_Default)
    sliderCoefdeadband_angle.set(slider_deadband_Default)
    sliderCoefdeadband_error.set(slider_deadband_Default)
    sliderdeadtime_servo.set(slider_deadtime_Default)
    
    slider_set_x_pos.set(0)
    slider_set_y_pos.set(0)
    
    
    sliderCoefK_x.set(sliderCoefKDefault)
    sliderCoefK_y.set(sliderCoefKDefault)
    
    sliderCoefTi.set(sliderCoefTiDefault)
    # sliderCoefTi_y.set(sliderCoefTiDefault)
    
    sliderCoefTd.set(sliderCoefTdDefault) 
    # sliderCoefTd_y.set(sliderCoefTdDefault) 
    
    sliderCoefTf_factor.set(sliderCoefTf_Default)
    sliderCoefdeadband_angle.set(slider_deadband_Default)
    sliderCoefdeadband_error.set(slider_deadband_Default)
    
  
    try:
        if var_demosetup.get()==1:
            ser.write(str(chr(int(neutral_angle))).encode())
            ser.flush()
        if var_demosetup.get()==2:
            string_angles =  neutral_angle_A + "," + neutral_angle_B + "," + neutral_angle_C+ ";"
            b_string_to_arduino = string_angles.encode()
            ser.write(b_string_to_arduino)
            ser.flush()
            
        if var_demosetup.get()==3:
            string_angles =  neutral_angle_A + "," + neutral_angle_B + "," + neutral_angle_C +","+ neutral_angle_D + ";"
            b_string_to_arduino = string_angles.encode()
            ser.write(b_string_to_arduino)
            ser.flush()


    except:
        tk.messagebox.showerror("showerror","Error sending signal to arduino!!!")


"""Function to finish system calibration and close the calibration window"""
def finish_systemcalibration():
    global conv_no
    global show_systemcalibration_window
    global min_angle
    global max_angle
    global neutral_angle
    global deadband_angle
    global deadband_error
    global time_signaltoservo
    global initial_max_angle
    global initial_min_angle
    global deflection_range
    
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D

    global var_demosetup
    try:
        if var_demosetup.get()==1:
            initial_min_angle = int(txtinput_min_angle.get())
            initial_max_angle = int(txtinput_max_angle.get())
            neutral_angle = int(txtinput_neutral_angle.get())
            deflection_range = min((neutral_angle - initial_min_angle),(initial_max_angle - neutral_angle))
            print(deflection_range)
            slidermaxdeflection_servo["to"] = deflection_range
            slidermaxdeflection_servo.set(deflection_range)
            
        if var_demosetup.get()==2:
            initial_min_angle = int(txtinput_min_angle.get())
            initial_max_angle = int(txtinput_max_angle.get())
            neutral_angles = txtinput_neutral_angle.get()
            print(neutral_angles)
            neutral_angles = neutral_angles.split (",")
            print(neutral_angles)
            print(len(neutral_angles))
            
            neutral_angle_A = int(neutral_angles[0])
            neutral_angle_B = int(neutral_angles[1])
            neutral_angle_C = int(neutral_angles[2])
        
            deflection_range = (initial_max_angle-initial_min_angle)/2
            print(deflection_range)
            slidermaxdeflection_servo["to"] = deflection_range
            slidermaxdeflection_servo.set(deflection_range)
            
        if var_demosetup.get()==3:
            initial_min_angle = int(txtinput_min_angle.get())
            initial_max_angle = int(txtinput_max_angle.get())
            neutral_angles = txtinput_neutral_angle.get()
            print(neutral_angles)
            neutral_angles = neutral_angles.split (",")
            print(neutral_angles)
            print(len(neutral_angles))
            
            neutral_angle_A = int(neutral_angles[0])
            neutral_angle_B = int(neutral_angles[1])
            neutral_angle_C = int(neutral_angles[2])
            neutral_angle_D = int(neutral_angles[3])
        
            deflection_range = (initial_max_angle-initial_min_angle)/2
            print(deflection_range)
            slidermaxdeflection_servo["to"] = deflection_range
            slidermaxdeflection_servo.set(deflection_range)

        systemcalibration_window.withdraw()
        show_systemcalibration_window = False
        Btn_systemcalibration_window["text"] = "Calibration"
    except:
        tk.messagebox.showerror("showerror","Error while improting inputs") 
        
"""Function to finish camera calibration and close the calibration window"""
        
def finish_cameracalibration():
    global beam_length
    global conv_no
    global show_cameracalibration_window
    global lowerBound_ball_input
    global upperBound_ball_input
    global FPS
    global x_pos
    global setpoint
    global var_demosetup
    try:
        cameracalibration_window.withdraw()
        show_cameracalibration_window = False
        Btn_cameracalibration_window["text"] = "Calibration"
        
        if var_demosetup.get()==1:    
            slider_setpos.set(beam_length/2)
        if var_demosetup.get()==2 or var_demosetup.get()==3:
            slider_set_x_pos.set(x_length/2)
            slider_set_y_pos.set(y_length /2)

            

    except:
        tk.messagebox.showerror("showerror","Error while reading inputs")      
        
"""Function to open the window to extract the HSV colour range of the ball"""
def openHSV_window():
    global camera_no
    try:
        camera_no = int(txtinput_camera_no.get())
        run_all(camera_no)
        lbl_camera_connect.config(text = "Camera not connected")
        lbl_camera_connect["bg"] = "red"
        lbl_indi_camera["bg"] = 'red'
    except:
        tk.messagebox.showerror("showerror","Error while taking camera number.")

"""Function to open the camera calibration window"""
show_cameracalibration_window = False
def showcameracalibration_window():
    global show_cameracalibration_window
    if show_cameracalibration_window == False:
        cameracalibration_window.deiconify()
        show_cameracalibration_window = True
        Btn_cameracalibration_window["text"] = "Close"
    else:
        cameracalibration_window.withdraw()
        show_cameracalibration_window = False
        Btn_cameracalibration_window["text"] = "Calibration"

"""Function to connect the demonstration setup using the previous calibration data stored in the configuretion file"""
def autoconnectfun():
    global FPS
    global camera_no
    global beam_length
    global x_borderMin
    global x_borderMax
    global lowerBound_ball_input
    global upperBound_ball_input
    global neutral_angle
    global min_angle
    global max_angle
    global time_signaltoservo
    global baudrate
    global com_port
    global deadband_error
    global resol
    global ser
    global deadband_angle
    global l_h ,l_s,l_v ,u_h,u_s,u_v
    global initial_min_angle
    global initial_max_angle
    global deflection_range
    
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D
    
    global x_length
    global y_length


    global origin_x
    global origin_y

    global x_pxmax
    global y_pxmax


    config.read('config.INI')

    # resol = config['Camera']['resol']
    
    camera_no = int(config['Camera']['camera_no'])
    txtinput_camera_no.delete(0, 'end') 
    txtinput_camera_no.insert(0,camera_no)
    
    beam_length = float(config['Camera']['platforms_length'])
    
    x_length = float(config['Camera']['x_length'])
    txtinput_x_length.delete(0, 'end') 
    txtinput_x_length.insert(0,x_length) 

    y_length = float(config['Camera']['y_length'])
    txtinput_y_length.delete(0, 'end') 
    txtinput_y_length.insert(0,y_length) 
    
    origin_x = float(config['Camera']['origin_x'])
    origin_y = float(config['Camera']['origin_y'])
    
    x_pxmax = float(config['Camera']['x_pxmax'])
    y_pxmax = float(config['Camera']['y_pxmax'])
    
    txtinput_beamlength.delete(0, 'end') 
    txtinput_beamlength.insert(0,beam_length) 

    x_borderMin = float(config['Camera']['pix_minpos'])
    x_borderMax = float(config['Camera']['pix_maxpos'])

    l_h  = int(config['Camera']['l_h'])   
    print(l_h)                         
    l_s = int(config['Camera']['l_s'])
    print(type(l_s))
    l_v = int(config['Camera']['l_v'])
    u_h  = int(config['Camera']['u_h'])
    u_s = int(config['Camera']['u_s'])
    u_v = int(config['Camera']['u_v'])
          
    lowerBound_ball_input = np.array ([l_h,l_s,l_v])
    print((lowerBound_ball_input))
    print(type(lowerBound_ball_input))
    upperBound_ball_input = np.array([u_h,u_s,u_v])
    print(upperBound_ball_input)
    print(type(upperBound_ball_input))

    neutral_angle = int(config['System']['neutral angle'])
    
    neutral_angle_A = float(config['System']['neutral angle_A'])
    neutral_angle_B = float(config['System']['neutral angle_B']) 
    neutral_angle_C = float(config['System']['neutral angle_C'])
    neutral_angle_D = float(config['System']['neutral angle_D'])
    
    txtinput_neutral_angle.delete(0, 'end') 
    
    if var_demosetup.get()==1:
        
        txtinput_neutral_angle.insert(0,neutral_angle)    
        txtinput_angle_toservo.delete(0, 'end') 
        txtinput_angle_toservo.insert(0,neutral_angle) 
        
    if var_demosetup.get()==2:
        txtinput_neutral_angle.insert(0,str(neutral_angle_A)+","+str(neutral_angle_B)+","+str(neutral_angle_C))    
        txtinput_angle_toservo.delete(0, 'end') 
        txtinput_angle_toservo.insert(0,str(neutral_angle_A)+","+str(neutral_angle_B)+","+str(neutral_angle_C)) 
        
    if var_demosetup.get()==3:
        txtinput_neutral_angle.insert(0,str(neutral_angle_A)+","+str(neutral_angle_B)+","+str(neutral_angle_C)+","+str(neutral_angle_D))    
        txtinput_angle_toservo.delete(0, 'end') 
        txtinput_angle_toservo.insert(0,str(neutral_angle_A)+","+str(neutral_angle_B)+","+str(neutral_angle_C)+","+str(neutral_angle_D)) 

                

    initial_min_angle = int(config['System']['minimum_angle'])
    txtinput_min_angle.delete(0, 'end') 
    txtinput_min_angle.insert(0,initial_min_angle)

    
    initial_max_angle = int(config['System']['maximum_angle'])
    txtinput_max_angle.delete(0, 'end') 
    txtinput_max_angle.insert(0,initial_max_angle)
    
    
    if var_demosetup.get()==1:
        deflection_range = min((neutral_angle - initial_min_angle),(initial_max_angle - neutral_angle))
        slidermaxdeflection_servo["to"] = deflection_range
        slidermaxdeflection_servo.set(deflection_range)
        
    if var_demosetup.get()==2 or var_demosetup.get()==3:
        deflection_range = (initial_max_angle-initial_min_angle)/2
        print(deflection_range)
        slidermaxdeflection_servo["to"] = deflection_range
        slidermaxdeflection_servo.set(deflection_range)

    time_signaltoservo = float(config['System']['deadtime_servo'])

    baudrate = int(config['System']['baudrate'])
 
    com_port = (config['System']['com_port'])
    
    deadband_error = float(config['PID']['deadband_error'])
    
    deadband_angle = float(config['PID']['deadband_angle'])
    
    tf = float(config['PID']['tf_factor'])
    sliderCoefTf_factor.set(tf)
    sliderdeadtime_servo.set(time_signaltoservo)
    sliderCoefdeadband_angle.set(deadband_angle)
    sliderCoefdeadband_error.set(deadband_error)
    slider_setpos["to"] = beam_length

    slider_set_x_pos["to"] = x_length
    slider_set_y_pos["to"] = y_length
    
    slider_set_x_pos.set(x_length/2)
    slider_set_y_pos.set(y_length/2)

    slider_setpos.set(beam_length/2)
    connect_cemerafun()
    connect_arduinofun()
    
    try:
        if var_demosetup.get()==1:
            ser.write(str(chr(int(neutral_angle))).encode())
            ser.flush()
            
        if var_demosetup.get()==2:
            string_angles =  str(neutral_angle_A) + "," + str(neutral_angle_B) + "," + str(neutral_angle_C)+";"
            b_string_to_arduino = string_angles.encode()
            ser.write(b_string_to_arduino)
            ser.flush()
            
        if var_demosetup.get()==3:
            string_angles =  str(neutral_angle_A) + "," + str(neutral_angle_B) + "," + str(neutral_angle_C)+ "," + str(neutral_angle_D) + ";"
            b_string_to_arduino = string_angles.encode()
            ser.write(b_string_to_arduino)
            ser.flush()

    except:
        tk.messagebox.showerror("showerror","Error sending signal to arduino!!!")  
    
""" Function to refresh all the comports available and update the dropdown menu in the system calibration window.
The function is adapted from:
“Serial List Ports in Python - Serial_ports().” Gist,
https://gist.github.com/tekk/5c8d6824108dfb771a1e16aa8bc479f0.

Accessed 25 March 2021.

"""
def refresh_comports():
    global clicked_comport
    global ser
    try:
        
        ser.close()
        def serial_ports():
            if sys.platform.startswith('win'):
                ports = ['COM%s' % (i + 1) for i in range(256)]
            elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                # this excludes your current terminal "/dev/tty"
                ports = glob.glob('/dev/tty[A-Za-z]*')
            elif sys.platform.startswith('darwin'):
                ports = glob.glob('/dev/tty.*')
            else:
                raise EnvironmentError('Unsupported platform')

            result = []
            for port in ports:
                try:
                    s = serial.Serial(port)
                    s.close()
                    result.append(port)
                except (OSError, serial.SerialException):
                    pass
            return result
        comports= serial_ports()
        clicked_comport = tk.StringVar()
        clicked_comport.set("Choose a com port")
        drop_block_comport = tk.OptionMenu(systemcalibration_window, clicked_comport, *comports)
        drop_block_comport.place(x= 20,y = 20)
    except:
        def serial_ports():
            if sys.platform.startswith('win'):
                ports = ['COM%s' % (i + 1) for i in range(256)]
            elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                # this excludes your current terminal "/dev/tty"
                ports = glob.glob('/dev/tty[A-Za-z]*')
            elif sys.platform.startswith('darwin'):
                ports = glob.glob('/dev/tty.*')
            else:
                raise EnvironmentError('Unsupported platform')

            result = []
            for port in ports:
                try:
                    s = serial.Serial(port)
                    s.close()
                    result.append(port)
                except (OSError, serial.SerialException):
                    pass
            return result
            
    
        comports= serial_ports()
        clicked_comport = tk.StringVar()
        clicked_comport.set("Choose a com port")
        try:
            drop_block_comport = tk.OptionMenu(systemcalibration_window, clicked_comport, *comports)
            drop_block_comport.place(x= 20,y = 20)
        except:
            lbl_arduino_connect.config(text = "No arduino found.")
            


"""Function to read the camera number, read the lower and upper HSV colour range, connect the camera and give proper indication"""
show_systemcalibration_window = False
def showsystemcalibration_window():
    global show_systemcalibration_window
    global comports
    if show_systemcalibration_window == False:
        systemcalibration_window.deiconify()
        show_systemcalibration_window = True
        Btn_systemcalibration_window["text"] = "Close"

    else:
        systemcalibration_window.withdraw()
        show_systemcalibration_window = False
        Btn_systemcalibration_window["text"] = "Calibration"

"""Function to read the camera number, read the lower and upper HSV colour range, connect the camera and give proper indication"""
camera_connected_bool = False
def connect_cemerafun():
    global camera_connected_bool
    global FPS
    global cam
    global camHeight
    global camWidth
    global camera_no
    global lowerBound_ball_input
    global upperBound_ball_input
    global config
    global resol
    global showVideoWindow
    try:

        camera_no = int(txtinput_camera_no.get())

        cam = cv2.VideoCapture(camera_no,cv2.CAP_DSHOW)
        cam.set(3,600)
        cam.set(4,550)
        
        cam.set(cv2.CAP_PROP_FPS, FPS)
        camera_connected_bool = True
        print("just before opening configfile.")
        config.read('config.INI')
        print("reading config file")
        l_h  = int(config['Camera']['l_h'])   
        print(l_h)                         
        l_s = int(config['Camera']['l_s'])
        print(type(l_s))
        l_v = int(config['Camera']['l_v'])
        u_h  = int(config['Camera']['u_h'])
        u_s = int(config['Camera']['u_s'])
        u_v = int(config['Camera']['u_v'])
        
        lowerBound_ball_input = np.array ([l_h,l_s,l_v])
        print((lowerBound_ball_input))
        print(type(lowerBound_ball_input))
        upperBound_ball_input = np.array([u_h,u_s,u_v])
        print(upperBound_ball_input)
        print(type(upperBound_ball_input))

        print("camera must be connected by now")

        lbl_camera_connect.config(text = "camera is connected")
        lbl_indi_camera["bg"] = 'lightgreen'
        lbl_camera_connect["bg"] = "lightgreen"
        main()
        
    except:
        lbl_camera_connect.config(text = "Camera connection failed")
        lbl_camera_connect["bg"] = "red"
        lbl_indi_camera["bg"] = 'red'
        
"""Function to read the selected com port and connect the arduino"""
arduino_connected_bool = False
def connect_arduinofun():
    global ser
    global arduino_connected_bool
    global baudrate
    global com_port
    global clicked_comport
    try:
        try:
            
            com_selection =str( clicked_comport.get())
            port = re.sub('\W', '', com_selection)
            com_port = port
            print(port)
            print(type(port))
        except:
            pass
        port = com_port
        ser = serial.Serial(port,baudrate) 
        arduino_connected_bool = True
        lbl_arduino_connect.config(bg = "lightgreen")
        lbl_arduino_connect.config(text = "Arduino connected.")
        
        lbl_indi_arduino.config(bg = "lightgreen")

    except:
        lbl_arduino_connect.config(bg = "red")
        lbl_arduino_connect.config(text = "Arduino connection failed.")
        lbl_indi_arduino["bg"] = 'red'

"""Function to indicate that controller is started sothat the main code calls the function where the controller is implementes """
Start_PID = False
def startPID():
    global Start_PID
    global FPS
    global conv_no
    global beam_length
    global tf_factor
    global setpoint
    global xincm
    global ser
    global neutral_angle
    
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D
    
    global var_demosetup

    

    if Start_PID == False:
        Start_PID = True
        Btn_StartPID["text"] = "Stop" + "\n" + "controller"
        Btn_StartPID['bg'] = "red"
        lbl_indi_controller["bg"] = "lightgreen"
    else:
        Start_PID = False
        Btn_StartPID["text"] = "Start" + "\n" + "controller"
        Btn_StartPID['bg'] = "lightgreen"
        lbl_indi_controller["bg"] = "red"
        try:
            if var_demosetup.get()==1:
                ser.write(str(chr(int(neutral_angle))).encode())
                ser.flush()
            
            if var_demosetup.get()==2:
                string_angles =  str(neutral_angle_A) + "," + str(neutral_angle_B) + "," + str(neutral_angle_C)+";"
                b_string_to_arduino = string_angles.encode()
                ser.write(b_string_to_arduino)
                ser.flush()
                
            if var_demosetup.get()==3:
                string_angles =  str(neutral_angle_A) + "," + str(neutral_angle_B) + "," + str(neutral_angle_C)+ "," + str(neutral_angle_D) + ";"
                b_string_to_arduino = string_angles.encode()
                ser.write(b_string_to_arduino)
                ser.flush()
        except:
            pass

"""The following function are used to indicate if any particular branch is turned on, the boolian variable and the indicators are adjusted accordingly"""
I_switch_bool = False
tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn on the integrator.") 
def ISwitch():
    global I_switch
    global I_switch_bool
    if I_switch_bool == False:
        I_switch = 1
        I_switch_bool = True
        tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn off the integrator.") 
        Btn_Switch_I["image"] = bg_closed_horizontal
        Btn_Switch_I['bg'] = "lightgreen"

    else:
        I_switch = 0
        I_switch_bool = False
        tip_Iswitch.bind_widget(Btn_Switch_I, balloonmsg = "Click here to turn on the integrator.") 
        Btn_Switch_I["image"] =bg_open_horizontal
        Btn_Switch_I['bg'] = "red"



"""Fuction to connect and disconnect the feedback branch of the closed loop system"""
openloop_switch_bool = True
tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for a openloop system.")
def openloop_switchfun():
    global openloop_switch
    global openloop_switch_bool
    if openloop_switch_bool == True:
        openloop_switch = 0
        openloop_switch_bool = False
        tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for a closedloop system.")
        Btn_Switch_closedloop["image"] = bg_open_vertical
        Btn_Switch_closedloop['bg'] = "red"
    else:
        openloop_switch = 1
        openloop_switch_bool = True
        tip_openloop.bind_widget(Btn_Switch_closedloop, balloonmsg = "Click here for a openloop system.")
        Btn_Switch_closedloop["image"] = bg_closed_vertical
        Btn_Switch_closedloop['bg'] = "lightgreen"


stepinput_bool = False
def step_inputfun():
    global setpoint
    
    global setpoint_x
    global setpoint_y
    global var_demosetup

    try:
        step = float(txtinput_stepinput.get())
        if var_demosetup.get() == 1:
            nieuw_setpoint = setpoint + step
            slider_setpos.set(nieuw_setpoint)
        if var_demosetup.get()==2 or var_demosetup.get():
            nieuw_setpoint_x = setpoint_x + step
            nieuw_setpoint_y = setpoint_y + step
            slider_set_x_pos.set(nieuw_setpoint_x)
            slider_set_y_pos.set(nieuw_setpoint_y)

        
    except:
        tk.messagebox.showerror("showerror","Error while taking inputs from user.")
        


I_windup_bool = False
tip_Icamping.bind_widget(Btn_Switch_Iwindup, balloonmsg = "Click here to activate dynamic clamping.") 
def I_windupfun():
    global I_windup_bool
    if I_windup_bool == False:
        I_windup_bool = True
        tip_Icamping.bind_widget(Btn_Switch_Iwindup, balloonmsg = "Click here to deactivate dynamic clamping.") 
        Btn_Switch_Iwindup["image"] = bg_closed_vertical
        Btn_Switch_Iwindup['bg'] ="lightgreen"
        
        lbl_opencircuit_clamping["image"] = bg_open_horizontal
        lbl_opencircuit_clamping["bg"] = "red"
    else:
        I_windup_bool = False
        tip_Icamping.bind_widget(Btn_Switch_Iwindup, balloonmsg = "Click here to activate dynamic clamping.") 
        Btn_Switch_Iwindup["image"] = bg_open_vertical
        Btn_Switch_Iwindup['bg'] ="red"
        lbl_opencircuit_clamping["image"] = bg_closed_horizontal
        lbl_opencircuit_clamping["bg"] = "lightgreen"
        


D_switch_bool = False
tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
def DSwitch():
    global D_switch
    global D_switch_bool
    global LPF_bool
    global D_kick_bool
    if D_switch_bool == False:
        D_switch = 1
        D_switch_bool = True
        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn off the derivative.") 
        Btn_Switch_D["image"] = bg_closed_horizontal
        Btn_Switch_D['bg'] = "lightgreen"
        
        D_kick_bool = False
        tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to avoid derivative kick.")
        Btn_Switch_Dervkick["image"] = bg_open_vertical
        Btn_Switch_Dervkick["bg"] = "red"
        
    else:
        D_switch = 0
        D_switch_bool = False
        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
        Btn_Switch_D["image"] = bg_open_horizontal
        Btn_Switch_D['bg'] = "red"        
        

D_kick_bool = False
tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to avoid derivative kick.")
def d_kickfun():
    global D_switch
    global D_switch_bool
    global LPF_bool
    global D_kick_bool
    global tf_factor
    if D_kick_bool == False:
        D_kick_bool = True
        D_switch = 1

        tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to disconnect this branch.")
        Btn_Switch_Dervkick["image"] = bg_closed_vertical
        Btn_Switch_Dervkick["bg"] = "lightgreen"
        
        D_switch_bool = False
        tip_Dswitch.bind_widget(Btn_Switch_D, balloonmsg = "Click here to turn on the derivative.") 
        Btn_Switch_D["image"] = bg_open_horizontal
        Btn_Switch_D['bg'] = "red"

    else:
        D_kick_bool = False
        D_switch = 0
        tip_D_kick.bind_widget(Btn_Switch_Dervkick, balloonmsg = "Click here to avoid derivative kick.")
        Btn_Switch_Dervkick["image"] = bg_open_vertical
        Btn_Switch_Dervkick["bg"] = "red"
 
        
LPF_bool = False
tip_LPF.bind_widget(Btn_Switch_LPF, balloonmsg = "Click here to activate low pass filter.")
def LPF_fun():
    global D_switch
    global D_switch_bool
    global LPF_bool
    global D_kick_bool
    global tf_factor
    if LPF_bool == False:
        LPF_bool = True
        tip_LPF.bind_widget(Btn_Switch_LPF, balloonmsg = "Click here to deactivate low pass filter.")
        Btn_Switch_LPF["image"] = bg_closed_vertical
        Btn_Switch_LPF['bg'] = "lightgreen"
        
        lbl_opencircuit_LPF["image"] = bg_open_horizontal
        lbl_opencircuit_LPF["bg"] = 'red'
    else:
        LPF_bool = False
        tip_LPF.bind_widget(Btn_Switch_LPF, balloonmsg = "Click here to activate low pass filter.")
        Btn_Switch_LPF["image"] = bg_open_vertical
        Btn_Switch_LPF['bg'] ="red"
        
        lbl_opencircuit_LPF["image"] = bg_closed_horizontal
        lbl_opencircuit_LPF["bg"] = "lightgreen"

        

"""Following two functions are used to take the minimum and maximum value of pixels where the ball 
can roll back and forth. These values are used to convert the pixels into centimeter in the main function"""
def set_minposfun():
    global x_borderMin
    global ballpos_tocalibrate
    global beam_length
    try:
        
        beam_length = float(txtinput_beamlength.get())
        print(x_borderMin)
        x_borderMin = ballpos_tocalibrate
        print(x_borderMin)
        lbl_posinfo.config(text = "min position is taken")
    except:
        lbl_posinfo.config(text = "Error")
        
"""Function to call when the start/stop recording button is clicked"""
def set_maxposfun():
    global x_borderMax
    global ballpos_tocalibrate
    global beam_length
    
    try:
        beam_length = float(txtinput_beamlength.get())
        slider_setpos["to"] = beam_length
        print(x_borderMax)
        x_borderMax = ballpos_tocalibrate 
        print(x_borderMax)          
        lbl_posinfo.config(text = "max position is taken")
    except:
        lbl_posinfo.config(text = "Error")
        

"""Function to call when the start/stop recording button is clicked"""
start_rec_bool = False
def start_rec():
    global start_rec_bool
    global filename
    
    global var_demosetup
    
    if start_rec_bool == False:
        start_rec_bool = True
        lbl_indi_recording["bg"] = "lightgreen"
        btn_startrec["text"] = "Stop recording"
        filename = time.strftime('%Y%m%d-%H%M%S') 
        filename = "data"+filename +".csv"
        
        if var_demosetup.get()==1:
            with open(filename, 'w', newline='') as write_obj:
                csv_writer = writer(write_obj,delimiter=';')
                fieldname = ['time','setpoint','error','controller output', 'manipulated variable','process value']
                csv_writer = csv.DictWriter(write_obj,fieldnames = fieldname,delimiter = ';')
                csv_writer.writeheader()
                
        if var_demosetup.get()==2 or var_demosetup.get()==3:
            with open(filename, 'w', newline='') as write_obj:
                csv_writer = writer(write_obj,delimiter=';')
                fieldname = ['time','setpoint_x','setpoint_y','error_x','error_y','controller output_x','controller output_y', 'manipulated variable_x','manipulated variable_y','process value_x','process value_y']
                csv_writer = csv.DictWriter(write_obj,fieldnames = fieldname,delimiter = ';')
                csv_writer.writeheader()
     
    else:
        start_rec_bool = False
        btn_startrec["text"] = "Start recording"
        lbl_indi_recording["bg"] = "red"

        
"""Function used to send a angle to the servomotor to get a proper maximum and minimum angle
 of the servomotor"""
def signal_toservo():
    global ser
    global var_demosetup
    try:
        if var_demosetup.get()==1:
            angle_toservo = int(txtinput_angle_toservo.get())
            ser.write(str(chr(int(angle_toservo))).encode())
            ser.flush()
            
        if var_demosetup.get()==2:
            angle_toservo = txtinput_angle_toservo.get()
            print(angle_toservo)
            angle_toservo = angle_toservo.split (",")
            print(angle_toservo)
            print(len(angle_toservo))
            
            angle_A = angle_toservo[0]
            angle_B = angle_toservo[1]
            angle_C = angle_toservo[2]
    
            string_angles =  angle_A + "," + angle_B + "," + angle_C+";"
            b_string_to_arduino = string_angles.encode()
            ser.write(b_string_to_arduino)
            ser.flush()
            
        if var_demosetup.get()==3:
            angle_toservo = txtinput_angle_toservo.get()
            print(angle_toservo)
            angle_toservo = angle_toservo.split (",")
            print(angle_toservo)
            print(len(angle_toservo))
            
            angle_A = angle_toservo[0]
            angle_B = angle_toservo[1]
            angle_C = angle_toservo[2]
            angle_D = angle_toservo[3]
    
            string_angles =  angle_A + "," + angle_B + "," + angle_C +"," + angle_D +";"
            b_string_to_arduino = string_angles.encode()
            ser.write(b_string_to_arduino)
            ser.flush()

            

    except:
        tk.messagebox.showerror("showerror","Error sending signal to arduino!!!")       
        
"""function to call to update/ save a .csv file"""
def append_list_as_row(file_name, list_of_elem): 
    # Open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj,delimiter=';')
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)
    
def resetSlider():
    sliderCoefK.set(sliderCoefKDefault)
    sliderCoefTi.set(sliderCoefTiDefault)
    sliderCoefTd.set(sliderCoefTdDefault) 
    
    sliderCoefK_x.set(sliderCoefKDefault)
    sliderCoefK_y.set(sliderCoefKDefault)
    
showVideoWindow = False
def showCameraFrameWindow():
    global showVideoWindow
    global BRetourVideoTxt
    global lowerBound_ball_input
    global upperBound_ball_input
    if showVideoWindow == False:
        try:
            
            videoWindow.deiconify()
            showVideoWindow = True
            Btn_ShowVideo["text"] = "Close Video "
           
        except:
            videoWindow.deiconify()
            showVideoWindow = True
            Btn_ShowVideo["text"] = "Close Video "
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        Btn_ShowVideo["text"] = "Show video"
        
"""Plotting the graph, here the hold on function form matlab is tried to achieved by updating the points in the same graph."""
showGraph = False
def showGraphWindow():
    global showGraph
    global BafficherGraph    
    if showGraph == False:
        showGraph = True
        Btn_graph["text"] = "Close graph "
    else:
        showGraph = False
        Btn_graph["text"] = "Show graph"


def paintGraph():
    global x_pos
    global setpoint
    global setpoint_x
    global setpoint_y
    global x_cm
    global y_cm
    global var_demosetup


    if var_demosetup == 1:
        if showGraph == True:
            if len(x_value) >1 and  len(y_value) > 1:
                del x_value[0]
                del y_value[0]
                
            if len(y_errorvalue) >1:
                del y_errorvalue[0]
                    
            if len(y_setpointvalue) >1:
                del y_setpointvalue[0]
            time_nu =  time.time()-time_start
            y = x_pos
            x_value.append(time_nu)
            y_value.append(y)
            error = setpoint-x_pos
            y_errorvalue.append(error)
            y_setpointvalue.append(setpoint)
            plt.plot(x_value,y_errorvalue,'g')
            plt.draw()
            plt.plot(x_value,y_setpointvalue,'b')
            plt.draw()
            plt.plot(x_value,y_value,'r')    
            # plt.legend['Error',"Setpoint","Position"]
            plt.draw()
        
        else:
            plt.close()
    else:
        pass
        

"""Function to manage the variables which has to be saved in configuration file."""
def datatofile():
    global camera_no
    global FPS
    global resol
    global beam_length
    global x_borderMin
    global x_borderMax
    global lowerBound_ball_input
    global upperBound_ball_input
    
    global neutral_angle
    global min_angle
    global max_angle
    global time_signaltoservo
    global baudrate
    global com_port
    global deadband_error
    global deadband_angle
    global l_h ,l_s,l_v ,u_h,u_s,u_v
    global initial_max_angle
    global initial_min_angle
    
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D
    
    global x_length
    global y_length


    global origin_x
    global origin_y

    global x_pxmax
    global y_pxmax

    
    config['Camera'] = {'FPS':FPS,
                        # 'resol':resol,
                        'Camera_no': camera_no,
                        'Platforms_length': beam_length,
                        
                        'x_length': x_length,
                        'y_length': y_length,
                        'origin_x': origin_x,
                        'origin_y': origin_y,
                        'x_pxmax': x_pxmax,
                        'y_pxmax': y_pxmax,                        

                        
                        
                        'pix_minpos': x_borderMin,
                        'pix_maxpos': x_borderMax,
                        'HSV_upperbound': upperBound_ball_input,
                        'HSV_lowerbound': lowerBound_ball_input,
                        'l_h':l_h ,                            
                        'l_s': l_s,
                        'l_v':l_v ,
                        'u_h':u_h  ,                            
                        'u_s': u_s,
                        'u_v': u_v
                         }
    
    
    
    config['System'] = {'neutral angle': neutral_angle,
                        
                        
                        'neutral angle_A': neutral_angle_A,
                        'neutral angle_B': neutral_angle_B,
                        'neutral angle_C': neutral_angle_C,
                        'neutral angle_D': neutral_angle_D,

                        'minimum_angle': initial_min_angle,
                        'maximum_angle': initial_max_angle,
                        'deadtime_servo': time_signaltoservo,
                        'com_port': com_port,
                        'baudrate': baudrate
                         }
    
    config['PID'] = {'deadband_error': deadband_error,
                        'K_x': sliderCoefK_x.get(),
                        'K_y': sliderCoefK_y.get(),
                        'K': sliderCoefK.get(),
                        'Ti': sliderCoefTi.get(),
                        'Td': sliderCoefTd.get(),
                        'deadband_angle': deadband_angle,
                        'Tf_factor':sliderCoefTf_factor.get(),
                         }
    
    
def save():
    with open('config.ini', 'w') as configfile:
        config.write(configfile)
        
        
"""Function where the PID controller algorithm is implemented for 1D demonstration setup"""
def PID_Controller(ball_pos,set_pos):    
    global previous_error
    global timenow    
    global integ
    global prev_elapsedTime
    global prev_integ
    global prev_deriv
    global integ_max
    global pid_i
    global I_switch
    global D_switch
    global saturation
    global I_switch_bool
    global prev_PID
    global openloop_switch
    global time_datatoservo
    global time_servo_need
    global time_save
    global PID
    global servo_signal
    global error
    global prev_ball_pos
    global conv_no
    global  I_windup_bool
    global LPF_bool
    global D_kick_bool
    global filename
    global start_rec_bool
    global tf_factor
    
    global min_angle
    global max_angle
    global neutral_angle
    global angle
    
    global prev_angle
    
    global time_signaltoservo
    global deadband_angle
    global deadband_error
    global deadband_angle_switch_bool
    global deadband_error_switch_bool
    
    global D_switch_bool
    global LPF_bool   
    global D_kick_bool
    global deriv
    global config
    
    
    
    K = sliderCoefK.get()
    Ti = sliderCoefTi.get()
    Td = sliderCoefTd.get()
    tf_factor = sliderCoefTf_factor.get()
    Tf = tf_factor * Td
    
    deadband_angle = sliderCoefdeadband_angle.get()
    deadband_error = sliderCoefdeadband_error.get()
    max_angle = slidermaxdeflection_servo.get()



#PID calcuation
    error = (set_pos - int((ball_pos * openloop_switch)))
       
      
    time_previous=timenow    
    timenow = time.time()    
    elapsedTime = timenow - time_previous
    # print("K =",K,"Ti =",Ti,"Td = ",Td,"tf_factor = ", tf_factor, "\n","T_deadangle", deadband_angle,"T_deaderror", deadband_error)
    Prop = K * error
    
    integ = ((K* elapsedTime * (error+previous_error)/(2*Ti)) + prev_integ) * I_switch
    print("K =",K,"  elapsedTime =",elapsedTime,"  error = ",error,"  previous_error = ", previous_error, "\n","  Ti=", Ti,"  prev_integ=", prev_integ,"  Iswitch = " , I_switch)
    print("integ = ", integ)

    if D_switch_bool == True or D_kick_bool == True:
        
        if D_kick_bool == True and LPF_bool == True:
            """ Algorithm derivative on PV with low pass filter"""
            deriv = (((2*K * Td* (-ball_pos+prev_ball_pos)) + (prev_deriv * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))#*D_switch
            print("D on PV with LPF")
            
            
            
        elif D_switch_bool == True and LPF_bool == True:
            """ Algorithm derivative on error with LPF"""
            deriv = (((2*K * Td* (error-previous_error)) + (prev_deriv * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))*D_switch
            print("D on error with LPF")
            
            
            
        elif D_kick_bool == True:
            """ Ideal derivative on PV"""
            deriv = (((2*K*Td*(-ball_pos+prev_ball_pos)/elapsedTime) - prev_deriv))*D_switch
            print("Ideal D on PV")
            
        else:
            """Ideal derivative on error"""
            deriv = (((2*K*Td*(error-previous_error)/elapsedTime) - prev_deriv))*D_switch
            print("ideal D on error")
    
 

    PID =  Prop + integ + deriv

    
    print("P=",Prop,'I =', integ , "D =", deriv)
    
    selection = clicked.get()
    if selection == "Normal": 
        angle = 1 *  (PID)
    if selection == "Integrate":
        angle = 1 *  (PID + prev_PID)
    if selection == "Derivate":
        angle =  1 *  (PID - prev_PID)
    
    

    if abs(angle) < deadband_angle :
        angle = 0
    else:
        pass
    
    if abs(error) < deadband_error:
        angle = 0
    
        
    
    
    # time_servo_need = abs(0.0033 * hoek)
    servo_signal = neutral_angle - angle

    servo_signal_beforelimiting = servo_signal
    
    # if servo_signal<=min_angle:
    #     servo_signal=min_angle
    # if servo_signal>=max_angle:
    #      servo_signal=max_angle
         
         
    if servo_signal > neutral_angle + max_angle:
        servo_signal = neutral_angle + max_angle
    if servo_signal < neutral_angle - max_angle:
        servo_signal = neutral_angle - max_angle     
         
    
 
    time_save = round(time.time() - time_start,2)
    row_contents = [time_save,setpoint,(round(error,2)),PID,servo_signal,round(x_incm,2)]
    # Append a list as new line to an old csv file
    if start_rec_bool == True:
        append_list_as_row(filename, row_contents)     
         
    
    """ Dynamic clamping """
    if servo_signal_beforelimiting != servo_signal:
        saturation = True
    if servo_signal_beforelimiting == servo_signal:
        saturation = False
        
    if I_switch == 1 :        
        if saturation == True and np.sign(error) == np.sign(PID):
            if I_windup_bool == True:
                I_switch = 0
            else:
                I_switch = 1
                
        if saturation != True and np.sign(error) != np.sign(PID):
            I_switch = 1
    if I_switch_bool == True and saturation == False:
        I_switch = 1
        
    previous_error = error
    prev_elapsedTime = elapsedTime
    prev_integ = integ
    prev_deriv = deriv
    prev_PID = PID
    prev_ball_pos = ball_pos
    prev_angle = angle
    try:
        ser.write(str(chr(int(servo_signal))).encode())
        ser.flush()
    except:
        tk.messagebox.showerror("showerror","Error while sending a signal to arduino in PID subroutine!!!")        
    config = configparser.ConfigParser()
    datatofile()
    save()
    
    
"""Function where the PID controller algorithm is implemented for 2D demonstration setup"""
def PID_Controller_2D(ball_pos_x,set_pos_x,ball_pos_y,set_pos_y):    
    global previous_error_x
    global previous_error_y
    
    global timenow    
    
    global integ_x
    global integ_y
    
    global prev_elapsedTime
    
    global prev_integ_x
    global prev_integ_y
    
    global prev_deriv_x
    global prev_deriv_y
    
    global integ_max
    global pid_i
    global I_switch
    global D_switch
    global saturation
    global I_switch_bool
    
    global prev_PID_x
    global prev_PID_y
    
    global openloop_switch
    global time_datatoservo
    global time_servo_need
    global time_save
    
    global PID_x
    global PID_y
    
    
    global servo_signal
    global error_x
    global error_y
    
    global prev_ball_pos_x
    global prev_ball_pos_y
    
    global conv_no
    global  I_windup_bool
    global LPF_bool
    global D_kick_bool
    global filename
    global start_rec_bool
    global tf_factor
    
    global min_angle
    global max_angle
    
    global neutral_angle_A
    global neutral_angle_B
    global neutral_angle_C
    global neutral_angle_D
    
    global angle_x
    global angle_y
    
    global prev_angle_x
    global prev_angle_y
    
    global time_signaltoservo
    global deadband_angle
    global deadband_error
    global deadband_angle_switch_bool
    global deadband_error_switch_bool
    
    global D_switch_bool
    global LPF_bool   
    global D_kick_bool
    
    global deriv_x
    global deriv_y
    
    global config
    global Start_PID
    
    global var_demosetup
    
    global servo_signal_motor_A
    global servo_signal_motor_B
    global servo_signal_motor_C
    global servo_signal_motor_D
    
    global angle_x
    global angle_y
 
    K_x = sliderCoefK_x.get()
    Ti = sliderCoefTi.get()
    Td = sliderCoefTd.get()
    
    K_y = sliderCoefK_y.get()
    # Ti_y = sliderCoefTi_y.get()
    # Td_y = sliderCoefTd_y.get()
    
    tf_factor = sliderCoefTf_factor.get()
    Tf = tf_factor * Td
    # Tf_y = tf_factor * Td_y
    
    
    
    deadband_angle = sliderCoefdeadband_angle.get()
    deadband_error = sliderCoefdeadband_error.get()
    max_angle = slidermaxdeflection_servo.get()
    print(max_angle)



#PID calcuation
    error_x = (set_pos_x - int((ball_pos_x * openloop_switch)))
    print("error_x =", error_x)
    error_y = (set_pos_y - int((ball_pos_y * openloop_switch)))
    print("error_y = " , error_y)

    # if abs(error)<deadband_error:
    #     error=0
    # else:
    #     pass    
      
    time_previous=timenow    
    timenow = time.time()    
    elapsedTime = timenow - time_previous
    
    
    Prop_x = K_x * error_x
    print("Prop_x = ", Prop_x)
    Prop_y = K_y * error_y
    print("Prop_y = ", Prop_y)

    
    integ_x = ((K_x* elapsedTime * (error_x+previous_error_x)/(2*Ti)) + prev_integ_x) * I_switch
    print("integ_x = ", integ_x)
    
    integ_y = ((K_y* elapsedTime * (error_y+previous_error_y)/(2*Ti)) + prev_integ_y) * I_switch
    print("integ_y = ", integ_y)

    

    if D_switch_bool == True or D_kick_bool == True:
        
        if D_kick_bool == True and LPF_bool == True:
            """ Algorithm derivative on PV with low pass filter"""
            deriv_x = (((2*K_x * Td * (-ball_pos_x+prev_ball_pos_x)) + (prev_deriv_x * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))#*D_switch
            deriv_y = (((2*K_y * Td* (-ball_pos_y+prev_ball_pos_y)) + (prev_deriv_y * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))#*D_switch

            print("D on PV with LPF")
            
            
        elif D_switch_bool == True and LPF_bool == True:
            """ Algorithm derivative on error with LPF"""
            deriv_x = (((2*K_x * Td* (error_x-previous_error_x)) + (prev_deriv_x * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))*D_switch
            deriv_y = (((2*K_y * Td* (error_y-previous_error_y)) + (prev_deriv_y * ((2*Tf)- elapsedTime))) / ((2*Tf)+elapsedTime))*D_switch
            print("D on error with LPF")
            
            
        elif D_kick_bool == True:
            """ Ideal derivative on PV"""
            deriv_x = (((2*K_x*Td*(-ball_pos_x+prev_ball_pos_x)/elapsedTime) - prev_deriv_x))*D_switch
            deriv_y = (((2*K_y*Td*(-ball_pos_y+prev_ball_pos_y)/elapsedTime) - prev_deriv_y))*D_switch
            print("Ideal D on PV")
            
        else:
            """Ideal derivative on error"""
            deriv_x = (((2*K_x*Td*(error_x-previous_error_x)/elapsedTime) - prev_deriv_x))*D_switch
            deriv_y = (((2*K_y*Td*(error_y-previous_error_y)/elapsedTime) - prev_deriv_y))*D_switch
            print("ideal D on error")


    PID_x = -1*(Prop_x + integ_x + deriv_x)
    print("PID_x = ", PID_x)
    PID_y = -1 *(Prop_y + integ_y + deriv_y)
    print("PID_y = ", PID_y)

    
    
    selection = clicked.get()
    if selection == "Normal": 
        print("normal manipulator")
        angle_x = 1 *  (PID_x)
        angle_y = 1 *  (PID_y)

    if selection == "Integrate":
        print("integrator manipulator")
        angle_x = 1 *  (PID_x + prev_PID_x)
        angle_y = 1 *  (PID_y + prev_PID_y)
        
    if selection == "Derivate":
        print("derivative manipulator")
        angle_x =  1 *  (PID_x - prev_PID_x)
        angle_y =  1 *  (PID_y - prev_PID_y)
        
        print("angle_x = ", angle_x)
        print("angle_y =", angle_y)
    
    if abs(angle_y)< deadband_angle:
        angle_y = 0

    else:
        pass

    if abs(angle_x) < deadband_angle:
        print("in de deadband angle if")
        angle_x = 0
    else:
        pass
    
    
    errorband_vector = math.sqrt(((error_x)**2)+((error_y)**2))
    if abs(errorband_vector) < deadband_error:
        print("in de deadband error if")
        angle_x = 0
        angle_y = 0
        
    else:
        pass
    
    servo_signal_motor_A = neutral_angle_A
    print(neutral_angle_A)
    servo_signal_motor_B = neutral_angle_B
    print(neutral_angle_B)

    servo_signal_motor_C = neutral_angle_C
    print(neutral_angle_B)

    servo_signal_motor_D = neutral_angle_D
    print(neutral_angle_D)

    
    print(var_demosetup.get())
    # time_servo_need = abs(0.0033 * hoek)
    if var_demosetup.get() == 2:
        print("demonstration 3motors")
        servo_signal_motor_A = int(neutral_angle_A) + angle_x + angle_y
        print(servo_signal_motor_A, neutral_angle_A,angle_x,angle_y)
        servo_signal_motor_B=  int(neutral_angle_B) - angle_x + angle_y
        print(servo_signal_motor_B, neutral_angle_B,angle_x,angle_y)
        servo_signal_motor_C = int(neutral_angle_C) - angle_y
        print(servo_signal_motor_C, neutral_angle_C,angle_x,angle_y)
        
        
    
    if var_demosetup.get() == 3:
        print("demonstration 4motors")
        servo_signal_motor_A = int(neutral_angle_A) + angle_x + angle_y
        print(servo_signal_motor_A)
        servo_signal_motor_B = int(neutral_angle_B) - angle_x + angle_y
        print(servo_signal_motor_B)
        servo_signal_motor_C=  int(neutral_angle_C) - angle_x - angle_y
        print(servo_signal_motor_C)
        servo_signal_motor_D= int(neutral_angle_D) + angle_x - angle_y
        print(servo_signal_motor_D)

    print("before max and min angle")
    servo_signal_beforelimiting_A = servo_signal_motor_A
    servo_signal_beforelimiting_B = servo_signal_motor_B
    servo_signal_beforelimiting_C = servo_signal_motor_C
    
    if var_demosetup.get() == 3:
        servo_signal_beforelimiting_D = servo_signal_motor_D

    
    if servo_signal_motor_A > (neutral_angle_A + max_angle):
        servo_signal_motor_A = neutral_angle_A + max_angle
    if servo_signal_motor_A < (neutral_angle_A - max_angle):
        servo_signal_motor_A = neutral_angle_A - max_angle
        
    if servo_signal_motor_B > (neutral_angle_B + max_angle):
        servo_signal_motor_B = neutral_angle_B + max_angle
    if servo_signal_motor_B < (neutral_angle_B - max_angle):
        servo_signal_motor_B = neutral_angle_B - max_angle
        
    if servo_signal_motor_C > (neutral_angle_C + max_angle):
        servo_signal_motor_C = neutral_angle_C + max_angle
    if servo_signal_motor_C < (neutral_angle_C - max_angle):
        servo_signal_motor_C = neutral_angle_C - max_angle
        
    if var_demosetup.get() == 3:
        if servo_signal_motor_D > (neutral_angle_D + max_angle):
            servo_signal_motor_D = neutral_angle_D + max_angle
        if servo_signal_motor_D < (neutral_angle_D - max_angle):
            servo_signal_motor_D = neutral_angle_D - max_angle
    print("after min and max angle")

    time_save = round(time.time() - time_start,2)
    row_contents = [time_save,setpoint_x,setpoint_y,round(setpoint_x-x_cm,2),round(setpoint_y-y_cm,2),PID_x,PID_y,angle_x,angle_y,round(x_cm,2),round(y_cm,2)]

    # Append a list as new line to an old csv file
    if start_rec_bool == True:
        append_list_as_row(filename, row_contents)     
         
    print("before dynamic clamping")
    """ Dynamic clamping """
    if servo_signal_beforelimiting_A != servo_signal_motor_A:
        saturation = True
    if servo_signal_beforelimiting_A == servo_signal_motor_A:
        saturation = False
        
    if servo_signal_beforelimiting_B != servo_signal_motor_B:
        saturation = True
    if servo_signal_beforelimiting_B == servo_signal_motor_B:
        saturation = False
        
    if servo_signal_beforelimiting_C != servo_signal_motor_C:
        saturation = True
    if servo_signal_beforelimiting_C == servo_signal_motor_C:
        saturation = False
        
    if var_demosetup.get() == 3:
        if servo_signal_beforelimiting_D != servo_signal_motor_D:
            saturation = True
        if servo_signal_beforelimiting_D == servo_signal_motor_D:
            saturation = False
        
    if I_switch == 1 :        
        if saturation == True and np.sign(error_x) == np.sign(PID_x):
            if I_windup_bool == True:
                I_switch = 0
            else:
                I_switch = 1
            
        if saturation == True and np.sign(error_y) == np.sign(PID_y):
            if I_windup_bool == True:
                I_switch = 0
            else:
                I_switch = 1
                
        else:
            I_switch = 1
    if I_switch_bool == True and saturation == False:
        I_switch = 1
    
    
    print("after dynamic clamping")
    
    if var_demosetup.get() == 2:
        print
        
        string_angles =  str(int(servo_signal_motor_A) )+ "," + str(int(servo_signal_motor_B)) + "," + str(int(servo_signal_motor_C))+ ";"
        print( string_angles)
        print(type(string_angles))
        b_string_to_arduino = string_angles.encode()
    
        print( b_string_to_arduino)
        print(type(b_string_to_arduino))
        try:
            ser.write(b_string_to_arduino)
            ser.flush()
            
        except:
            Start_PID = False
            Btn_StartPID["text"] = "Start"+"\n"+" Controller"
            Btn_StartPID['bg'] = "lightgreen"
            lbl_indi_controller["bg"] = "red"
            lbl_arduino_connect.config(bg = "red")
            lbl_arduino_connect.config(text = "Arduino connection failed.")
            lbl_indi_arduino["bg"] = 'red'
            tk.messagebox.showerror("showerror","Error while sending a signal to arduino in PID subroutine!!!")

        
    

    if var_demosetup.get() == 3:
        string_angles =  str(int(servo_signal_motor_A) )+ "," + str(int(servo_signal_motor_B)) + "," + str(int(servo_signal_motor_C))+","+str(int(servo_signal_motor_D))+";"
        print( string_angles)
        print(type(string_angles))
        b_string_to_arduino = string_angles.encode()
        print( b_string_to_arduino)
        print(type(b_string_to_arduino))
        try:
            ser.write(b_string_to_arduino)
            ser.flush()
            
        except:
            Start_PID = False
            Btn_StartPID["text"] = "Start"+"\n"+" Controller"
            Btn_StartPID['bg'] = "lightgreen"
            lbl_indi_controller["bg"] = "red"
            lbl_arduino_connect.config(bg = "red")
            lbl_arduino_connect.config(text = "Arduino connection failed.")
            lbl_indi_arduino["bg"] = 'red'
            tk.messagebox.showerror("showerror","Error while sending a signal to arduino in PID subroutine!!!")


    prev_elapsedTime = elapsedTime

    previous_error_x = error_x
    prev_integ_x = integ_x
    prev_deriv_x = deriv_x
    prev_PID_x = PID_x
    prev_ball_pos_x = ball_pos_x
    prev_angle_x = angle_x
    
    previous_error_y = error_y
    prev_integ_y = integ_y
    prev_deriv_y = deriv_y
    prev_PID_y = PID_y
    prev_ball_pos_y = ball_pos_y
    prev_angle_y = angle_y
    
    
    config = configparser.ConfigParser()
    datatofile()
    save()



"""
Following function is adapted from the website:        
OpenCV Track Object Movement-PyImageSearch. “Ball Tracking with OpenCV.” PyImageSearch, 14 Sept. 2015,
https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/.
Accessed 14 October 2020
"""

def main():
    global x 
    global x_pos
    global camWidth 
    global camHeight
    global timeInterval
    global start_time
    global showVideoWindow
    global time_start_graph
    global setpoint
    global beam_length
    global x_incm
    global Start_PID
    global time_datatoservo_start
    global time_servo_need
    global PID
    global servo_signal
    global error
    global stepinput_bool
    global ballpos_tocalibrate
    global x_borderMin
    global x_borderMax
    global filename
    global start_rec_bool
    global cam
    global camera_connected_bool
    global lowerBound_ball_input
    global upperBound_ball_input
    global time_signaltoservo
    global config
    global actual_fps
    
    
    global setpoint_x
    global setpoint_y

    global PID_x
    global PID_y
    global angle_x
    global angle_y

    global x_px
    global y_px
    
    global x_cm
    global y_cm
    
    global x_length
    global y_length
    
    global origin_x
    global origin_y
    
    global x_pxmax
    global y_pxmax
    
    global var_demosetup

    
    setpoint = slider_setpos.get()
    setpoint_x = slider_set_x_pos.get()
    setpoint_y = slider_set_y_pos.get()

    
    
    
    time_signaltoservo = sliderdeadtime_servo.get()
    
   
    if camera_connected_bool == True:
        start_timeFPS = time.time()
        lbl_camera_connect.config(text = "camera is connected")
        _, img=cam.read()
        img = img[0:int(camHeight),int((camWidth-camHeight)/2):int(camWidth-((camWidth-camHeight)/2))] #[Y1:Y2,X1:X2]
        
        img=cv2.rotate(img, cv2.ROTATE_180)


        imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
        lowerBound_ball=np.array(lowerBound_ball_input)
        upperBound_ball=np.array(upperBound_ball_input)
        
        mask_ball = cv2.inRange(imgHSV,lowerBound_ball,upperBound_ball)
        mask_ball = cv2.blur(mask_ball,(6,6))                        
        mask_ball = cv2.erode(mask_ball, None, iterations=2)         
        mask_ball = cv2.dilate(mask_ball, None, iterations=2)
    
        cnts_ball = cv2.findContours(mask_ball.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts_ball = imutils.grab_contours(cnts_ball)

    
        if len(cnts_ball) > 0 :#and len(cnts_borderMax)>0 and len(cnts_borderMin)>0:
            c_ball = max(cnts_ball, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c_ball)
            ballpos_tocalibrate = x
            x_px = int(x)
            y_px = int(y)

            if var_demosetup.get()==1:
                if x_borderMin > 0 and  x_borderMax> 0:
                    x_incm = (beam_length/(x_borderMax-x_borderMin)*(x-x_borderMin)) 
                    x_pos = round(float(x_incm),1)
                    
            if var_demosetup.get()==2 or var_demosetup.get()==3:
                if x_pxmax > 0 and  y_pxmax > 0 and origin_x > 0 and origin_y > 0 and x_length > 0 and y_length > 0 :
                    x_cm = round((x_length/(x_pxmax-origin_x)*(x_px-origin_x)),1) 
                    y_cm = round((y_length/(y_pxmax-origin_y)*(y_px-origin_y)),1)

                
                
            time_save = round(time.time() - time_start,2)
        # time,position,input,error
            if start_rec_bool == True:
                if openloop_switch_bool== False:
                    if var_demosetup.get()==1:
                        row_contents = [time_save,setpoint,setpoint,PID,servo_signal,round(x_incm,2)]
                    if var_demosetup.get()==2 or var_demosetup.get()==3:
                        row_contents = [time_save,setpoint_x,setpoint_y,setpoint_x,setpoint_y,PID_x,PID_y,angle_x,angle_y,round(x_cm,2),round(y_cm,2)]
                              
            # Append a list as new line to an old csv file
                    append_list_as_row(filename, row_contents)
                else:
                    if var_demosetup.get()==1:
                        row_contents = [time_save,setpoint,round(error,2),PID,servo_signal,round(x_incm,2)]
                        
                    if var_demosetup.get()==2 or var_demosetup.get()==3:
                        row_contents = [time_save,setpoint_x,setpoint_y,round(setpoint_x-x_cm,2),round(setpoint_y-y_cm,2),PID_x,PID_y,angle_x,angle_y,round(x_cm,2),round(y_cm,2)]
            # Append a list as new line to an old csv file
                    append_list_as_row(filename, row_contents)
                
             
            if Start_PID == True:
                # print("time taken = ", round((time_servo_need),2))
                
                if time.time()- time_datatoservo_start > time_signaltoservo: #time_servo_need:
                    # print("in de if loop")
                    if var_demosetup.get()==1:
                       PID_Controller(x_incm, setpoint) 
                        
                    if var_demosetup.get()==2 or var_demosetup.get()==3:
                        PID_Controller_2D(x_cm, setpoint_x , y_cm , setpoint_y)

                        
                    time_datatoservo_start = time.time()
    
            if radius > 10:
                if var_demosetup.get()==1:
                    if x_pos >0: 
                    # cv2.putText(img,str(int(x_incm)) + ";" + str(int(y)).format(0, 0),(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                        cv2.putText(img,str(round((x_pos),1)) ,(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                        cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    
                    else:
                        cv2.putText(img,str(round((x),1)) ,(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                        cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                
                if var_demosetup.get()==2 or var_demosetup.get()==3:
                    if x_cm > 0 and y_cm > 0:
                # cv2.putText(img,str(int(x_incm)) + ";" + str(int(y)).format(0, 0),(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                        cv2.putText(img,str(x_cm)+ ";" + str(y_cm).format(0, 0) ,(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                        cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                        
                    else:
                        print("in the else condition")
                        cv2.putText(img,str(x_px)+ ";" + str(y_px).format(0, 0) ,(int(x)-50, int(y)-50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2)
                        cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)


        if time.time() - time_start_graph >=1 :
            if showGraph == True:
                paintGraph()
            time_start_graph = time.time()
            if var_demosetup.get()==1:
                lbl_PID_gegevens.config(text="Position = "+str(round(x_incm,1))+"\n"+"Set point = "+str(setpoint)+"\n"+"Error = "+str(round(setpoint-x_incm,1))+"\n"+"time  =  "+str(round(time.time() - time_start,2)) +"\n"+"FPS =  "+str(actual_fps) )
            if var_demosetup.get()==2 or var_demosetup.get()==3:
                lbl_PID_gegevens.config(text="Position = "+str(x_cm)+";" + str(y_cm)+"\n"+"Error = "+str(round(setpoint_x-x_cm,1))+";"+str(round(setpoint_y-y_cm,1))+"\n"+"FPS =  "+str(actual_fps) )
                
        if showVideoWindow == True:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=img)
            lmain.imgtk = imgtk
            lmain.configure(image=imgtk)
        lmain.after(5, main)
        actual_fps = round(1.0 / (time.time() - start_timeFPS),2)
        
        
        
def refreshcamera():
    global camera_no
    # Initializing the webcam feed.
    lbl_camera_connect.config(text = "Camera not connected")
    lbl_camera_connect["bg"] = "red"
    lbl_indi_camera["bg"] = 'red'
    cap = cv2.VideoCapture(camera_no,cv2.CAP_DSHOW)
    cap.set(3,600)
    cap.set(4,550)
    while True:
        ret, frame = cap.read()
        break

    cap.release()
    cv2.destroyAllWindows()         


"""The following function is retrieved from:
Praveen. “How to Find HSV Range of an Object for Computer Vision Applications?” Programming_fever, 28 July 2020,
https://medium.com/programming-fever/how-to-find-hsv-range-of-an-object-for-computer-vision-applications-254a8eb039fc.

Accessed 18 October 2020

"""

def run_all(camera):
    
    def nothing(x):
        pass
    global l_h ,l_s,l_v ,u_h,u_s,u_v

    # Initializing the webcam feed.
    cap = cv2.VideoCapture(camera,cv2.CAP_DSHOW)
    cap.set(3,600)
    cap.set(4,550)
    
    # Create a window named trackbars.
    cv2.namedWindow("Trackbars")
    
    # Now create 6 trackbars that will control the lower and upper range of 
    # H,S and V channels. The Arguments are like this: Name of trackbar, 
    # window name, range,callback function. For Hue the range is 0-179 and
    # for S,V its 0-255.
    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
     
    while True:
        
        # Start reading the webcam feed frame by frame.
        ret, frame = cap.read()
        if not ret:
            break
        # Flip the frame horizontally (Not required)
        frame = cv2.flip( frame, 1 ) 
        
        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get the new values of the trackbar in real time as the user changes 
        # them
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
     
        # Set the lower and upper HSV range according to the value selected
        # by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])
        
        # Filter the image and get the binary mask, where white represents 
        # your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)
     
        # You can also visualize the real part of the target color (Optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Converting the binary mask to 3 channel image, this is just so 
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # stack the mask, orginal frame and the filtered result
        stacked = np.hstack((mask_3,frame,res))
        
        # Show this stacked frame at 40% of the size.
        cv2.imshow('Trackbars',cv2.resize(stacked,None,fx=0.9,fy=0.9))
        
        # If the user presses ESC then exit the program
        key = cv2.waitKey(1)
        if key == 27:
            break
        if cv2.getWindowProperty("Trackbars", cv2.WND_PROP_VISIBLE) <1:
            lower = [l_h,l_s,l_v]
            print(lower)
            upper = [u_h, u_s, u_v]
            print(upper)
            config['Camera'] = {                
                                'l_h':l_h ,                            
                                'l_s': l_s,
                                'l_v':l_v ,
                                'u_h':u_h  ,                            
                                'u_s': u_s,
                                'u_v': u_v 
                                
                         }
            save()
            # thearray = [[l_h,l_s,l_v],[u_h, u_s, u_v]]
            # np.save('hsv_value',thearray)
            break
      
    
    cap.release()
    cv2.destroyAllWindows()



def exit_function():
    global ser
    main_Window.destroy()
    ser.close()


def closevideo():
    global showVideoWindow
    videoWindow.withdraw()
    showVideoWindow = False
    Btn_ShowVideo["text"] = "Show video"
    
main_Window.protocol('WM_DELETE_WINDOW', exit_function)
videoWindow.protocol("WM_DELETE_WINDOW",closevideo)
systemcalibration_window.protocol("WM_DELETE_WINDOW",showsystemcalibration_window)
cameracalibration_window.protocol("WM_DELETE_WINDOW",showcameracalibration_window)

main()
tk.mainloop()

