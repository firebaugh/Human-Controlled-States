#!/usr/bin/env python

"""
Cara Takemoto
Caitlyn Clabaugh
Meagan Neal
Melanie Shafer
Kathryn Rath

Soccer tool!! 


"""

import math
import numpy as np
import scipy as sp
import Image as im
import shm
import tkMessageBox, tkFont, tkSimpleDialog, tkFileDialog, sys, os, ImageTk
from Tkinter import *

import readFSM

class MainWindow: 
    
	def __init__(self, fsmInput, naoTeam, naoID):
		
		self.naoTeam = naoTeam
                self.naoID = naoID 
		
		self.root = Tk()

		self.root.title("Team "+self.naoTeam+" player "+self.naoID)

		self.states = [None] * 3

		for fsm in range(len(fsmInput)):
			self.states[fsm] = fsmInput[fsm]

		self.background = Canvas(self.root, width=500, height=300) 
		
		self.background.grid(row=0, column=0) 

		self.robotView = Toplevel(master=self.root)
		#Create canvas
                self.rvCanvas = Canvas(self.robotView, width=160, height=120)
                self.rvCanvas.pack()

		#Create buttons on window
		self.draw_fsm()

		#Get user's ID
                usr = str(os.getenv('USER'))

		#Make shared memory segment
                viSegName = 'vcmImage' +str(self.naoTeam)+str(self.naoID)+usr
		self.vcmImage = shm.ShmWrapper(viSegName)

		#create event that is constantly renewed to get new data
                #self.root.after(0, self.update)

		self.root.mainloop()

	def update(self):

		#show live image stream of what robot is seeing
                self.dispVideo()

		#create event that is constantly renewed to get new data
                self.root.after(200, self.update)

            
	def draw_fsm(self):
		#create attribute to hold buttons
		self.stateB = [None] * len(self.states)
		#Go through the fsm machines and their states and figure out
		#where to display them
		col_num = 0
		for fsm in range(len(self.states)):
			row_num = 0
			self.stateB[fsm] = [None] * len(self.states[fsm])
			for state in range(len(self.states[fsm])):
				self.stateB[fsm][state] = Button(self.background, text=str(self.states[fsm][state]), command=changeState,  width=20).grid(row=row_num, column=col_num)

				row_num += 1

			col_num += 1
			  

	def changeState(self, event):
		print "change the state!!!"

	def highlight_current_state(self):
		state_name = 'b' # This will be grabbed from shared memory, using the global variable memory_segment set earlier
		if state_name in self.states:
			background.itemconfigure(self.states[state_name],fill="blue")
		else:
			print "ERROR: UNKNOWN STATE"   

		# highlight_state = Tkinter.Button(root, text='Highlight Current State', command=highlight_current_state)
           	# highlight_state.grid(row=1,column=0)

        def dispVideo(self):
                #Displays the image of the robot 
                yuyv = self.vcmImage.get_yuyv()
                rgbImg = self.yuyv2rgb(yuyv)
                #display img somewhere (you could test with a different image)
                #note rgb image size is (160,120)
                
                #update image to be new image
                self.rvCanvas.config(image=rgbImg)

        #HELPERS
        def yuyv2rgb(self, yuyv):
                #converts from yuyv shared mem format to rgb image
                #from text_image.py
                # data is actually int32 (YUYV format) not float64
                yuyv.dtype = 'uint32'
                # convert to uint8 to seperate out YUYV
                yuyv.dtype = 'uint8'
                # reshape to Nx4
                yuyv_u8 = yuyv.reshape((120, 80, 4))
        
                #convert from yuyv to yuv to rgb
                rgb = []
                for i in range(len(yuyv_u8)):
                        row = []
                        for j in range(len(yuyv_u8[0])):
                                y1 = yuyv_u8[i][j][0]
                                u = yuyv_u8[i][j][1]
                                y2 = yuyv_u8[i][j][2]
                                v = yuyv_u8[i][j][3]
                                rgb1 = self.yuv2rgb(y1, u, v)
                                row.append(rgb1)
                                rgb2 = self.yuv2rgb(y2, u, v)
                                row.append(rgb2)
                        rgb.append(row)
                #convert rgblist of tuples of (r,g,b) to array
                rgbArr = np.asarray(rgb)
                #convert array to image and save
                img = im.fromarray(np.uint8(rgbArr))
                # YOU CAN USE img TO DISPLAY ANYWHERE I THINK!
                #img.save('img.png') #just saved to test out...
                return img

        def yuv2rgb(self, y, u, v):
                c = y - 16
                d = u - 128
                e = v - 128
                R = (298 * c) + (409 * e) + 128
                G = (298 * c) - (100 * d) - (208 * e) + 128
                B = (298 * c) + (516 * d) + 128
                R >>= 8
                G >>= 8
                B >>= 8
                return (self.clip(R), self.clip(G), self.clip(B))
        
        def clip(self,v):
                v = max(v, 0)
                v = min(v, 255)
                return v        


#test = MainWindow(readFSM.fsm_states)
test = MainWindow(readFSM.getBGHfsm(), sys.argv[1], sys.argv[2])
test.run()
