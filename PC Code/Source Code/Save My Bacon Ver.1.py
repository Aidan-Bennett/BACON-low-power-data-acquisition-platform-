'''0.1 to embed a matplotlib graph in tkinter the libraries use the pack manager so I need to change to that
    This is a minimum viable product
'''

import serial
import winreg
import itertools
import time
from tkinter import *
from tkinter import ttk
from tkinter import filedialog
import fixedint
import winsound


class GUI(Frame):

    currentCom='-'
    fname=''
    def __init__(self, parent):
        Frame.__init__(self, parent, background = "white")
        self.parent = parent        
        self.parent.title("Save My BACON") #window title
        self.pack(fill=BOTH, expand=1)
        self.currentCom= None
        self.comStringVar = StringVar()
        self.dataType = "main"

        
        combutton= Button(self, text="Scan COM port", width=20, command=self.findCom)
        combutton.grid(row=3, column = 0,  pady = 10, padx=10)
        self.savebutton= Button(self, text="Save Main Data", width=20, command=self.asksavefile)
        self.savebutton.grid(row=3, column=1, pady = 1, padx=5)
        self.savebutton2= Button(self, text="Save CO2 Data", width=20, command=self.asksavefileCO2)
        self.savebutton2.grid(row=3, column=2, pady = 1, padx=5)
        
        lab= Label(self, background = "white", text = "Current COM port:")
        lab.grid(row=0, column=0, pady=10, padx=10, sticky="W")

        self.comList = ttk.Combobox(self, textvariable=self.comStringVar, width=20, state='readonly')
        self.comList.grid(row=0, column=1, sticky='W')


        
        
        statusMessage= Label(self, background = "white", text = "Status:")
        statusMessage.grid(row=1, column=0,  pady = 1, padx=10, sticky="W")
        self.status= Label(self, background = "white", text= "-")
        self.status.grid(row=1, column=1,  sticky="W")

        self.sampleMessage= Label(self, background = "white", text = " ")
        self.sampleMessage.grid(row=2, column=0, padx=10, sticky="W")
        self.sample= Label(self, background = "white", text= "  ")
        self.sample.grid(row=2, column=1,  sticky=W)
        
        
        # defining options for opening a directory
        self.dir_opt = options = {}
        #options['initialdir'] = 'C:\\'
        options['mustexist'] = False
        options['parent'] = parent
        options['title'] = 'This is a title'

        # define options for opening or saving a file
        self.file_opt = options = {}
        options['defaultextension'] = '.csv'
        options['filetypes'] = [('Comma Seperated Value', '.csv'), ('text files', '.txt')]
        options['initialfile'] = self.fname
        options['parent'] = parent
        options['title'] = 'Save as'

        
    def asksavefileCO2(self):
        self.dataType = "CO2"
        self.asksavefile();
        self.dataType = "main"
        

    # asks user to create/overwrite a file and then start saving data from device
    def asksavefile(self):
        self.setcurrentComList()

        if(self.currentCom == None or self.currentCom == ""):
            self.status.config(text="Please chose a COM Port!")
            self.parent.update()
  
            winsound.PlaySound('SystemAsterisk', winsound.SND_ALIAS | winsound.SND_ASYNC )
            
            return
        try:
            self.filename = filedialog.asksaveasfile(mode='w', **self.file_opt)

        except PermissionError:
            self.status.config(text="Open File Error: Permission Denied")
            self.parent.update()
            return
            
        if(self.filename == None):
            self.status.config(text="-")
            self.parent.update()
            return
        
        
        self.status.config(text="Opening Serial port")
        self.parent.update()
        self.setup_serial()
        self.status.config(text="Saving Data")
        self.parent.update()

        self.savebutton.grid_remove()
        self.savebutton2.grid_remove()
        
        self.cancelbutton= Button(self, text="Cancel", width=20, command=lambda:self.cancelSave())
        self.cancelbutton.grid(row=3, column=1, padx=10)
        
        self.read_data() #start saving data from device
        self.cancelbutton.grid_remove()
        savebutton= Button(self, text="Save Main Data", width=20, command=self.asksavefile)
        savebutton.grid(row=3, column=1, pady = 1, padx=5)
        savebutton2= Button(self, text="Save CO2 Data", width=20, command=self.asksavefileCO2)
        savebutton2.grid(row=3, column=2, pady = 1, padx=5)

        

        # catch if file and serial were closed already
        if(not self.filename == None):
            self.filename.close()
            
        if(not self.ser == None):
            self.ser.close()
        
        self.update()
       

        

    def findCom(self):
        """this is a popup window"""

        
        comPorts=[]
        for i in enumerate_serial_ports():
            if (comPorts.count(i[0]) == 0):
##                print("not there")
##                print(i[0])
                comPorts.append(i[0])
                #print(comPorts)


        
                
        if(len(comPorts)==1): #if only one automatically choose that one
            com= comPorts[0]
            
            self.comList['values']= com
            self.comList.current(0)
            self.setcurrentComList()
            
            self.status.config(text="Selected Only Available COM")
            return

        elif(len(comPorts)==0):
            self.status.config(text="No COM Ports Available")
            self.comList['values']= comPorts
            self.comList.set("")
            self.setcurrentComList()
            return

        self.comList['values']= comPorts
        self.comList.current(0)

        
 
        


    def setcurrentCom(self, com):
        self.currentCom= com
        #print(self.currentCom)
        self.update()
        #self.top.destroy()

    def setcurrentComList(self):
        self.currentCom = self.comList.get()
        if(self.currentCom == ""):
            
            self.currentCom = None
        self.status.config(text="-              ")
        self.update()
        #self.top.destroy()





    def read_data(self):
        # attempt counters which end transmission if too many consecutive errors occur
        attempt = 0
        maxAttempt = 5
        samplecount = 0
        start_time = time.time()
        self.cancel = False

        #dataWriteBuffer = []
        if( self.ser == None):
            self.status.config(text="Error Opening COM Port")
            return
        
        
        # setup and update UI
        self.status.config(text="Waiting for Device Response")
        self.sampleMessage.grid_forget()
        self.sample.grid_forget()

        self.sampleMessage= Label(self, background = "white", text = "Samples recieved:")
        self.sampleMessage.grid(row=2, column=0, padx=10, sticky="W")
        self.sample= Label(self, background = "white", text= "-               ")
        self.sample.grid(row=2, column=1,  sticky=W)
        self.parent.update()

        # attempt to connect to Device
        while(attempt < maxAttempt*2):
            
            if(self.cancel == True):    #allow while loop to break and method to return if user cancels
                self.status.config(text="Saving Canceled")
                
                return
            
            try:
                self.parent.update()
                if(self.dataType == "main"):
                    self.ser.write(bytes("SndData\n",encoding="ascii"))
                elif(self.dataType == "CO2"):
                    self.ser.write(bytes("SndDataCO2\n",encoding="ascii"))
                line = self.ser.readline()
                line = str(line).strip("b'\\r\\n")
                #print(line)
                
                #self.parent.update()
                if(line == "S"):
                    break
                elif(line == "BADEND"):
                    self.status.config(text="Device Error, Try again")
                    
                    return
                else:
                    line = ""
                    attempt += 1
                
                
            except serial.SerialTimeoutException:

                attempt += 1
                if(attempt >=  maxAttempt*2):
                    self.status.config(text="No Response from Device")
                    
                    return
                
            except serial.serialutil.SerialException:
                self.status.config(text="Port Error: Check Port")
                    
                return
                
        if(attempt >= maxAttempt):
            self.status.config(text="No Response from Device")
            
            return

    
        #once connected, recieve data from Device    
        try:
            #update UI messages
            self.status.config(text="Saving Data")
            self.parent.update()

            
            #setup writing recieved sample count on UI

            self.parent.update()
            
            attempt = 0 #reset attempt counter

            #start recieving
            while(attempt < maxAttempt):


                #update status messages etc
                self.sample.config(text= samplecount)
                self.parent.update()
                
                if(self.cancel == True):    #allow while loop to break and method to return if user cancels
                    self.status.config(text="Saving Canceled")
                    self.ser.write(bytes("END\n",encoding="ascii")) # device Sent End command 
                    return
                

                # reset checksum
                checksum = fixedint.MutableUInt32(0)
                # read line from com port
                line = str(self.ser.readline()).strip("b'\\r\\n")
                #print(line)

                # if single sample being sent command recieved, confirm with "OK" and recieve sample
                if(line == "SNGL"):

                    # send acknowledge
                    self.ser.write(bytes("OK\n",encoding="ascii"))
                    # read data being sent by device
                    data = str(self.ser.readline()).strip("b'\\r\\n")

                    #process data and get sent checksum to compare with
                    striped_data = data.split(",", 1)
                    recievedChecksum = fixedint.UInt32(int(striped_data[0]))
                    
                    # calulate checksum for recieved data
                    for c in striped_data[1]:
                        checksum += fixedint.UInt32(ord(c))

                    #compare checksums. if they are the same, send acknowledge. otherwise send "RETRY" to get data resent
                    if( checksum == recievedChecksum):
                        
                        self.filename.write(striped_data[1])
                        self.filename.write('\n')
                        
                        self.ser.write(bytes("CHKOK\n",encoding="ascii"))
                        
                        attempt = 0 #clear attempt counter if successfull
                        samplecount += 1 #increment counter
                                   
                    else:
                        self.ser.write(bytes("RETRY\n",encoding="ascii"))
                        self.status.config(text="checksum Differ!, Retrying.")
                        attempt += 1

                # if "ENDDATA" comand recieved, all samples have been recieved and saving should stop
                elif(line == "DATAEND"):
                    self.ser.write(bytes("OKEND\n",encoding="ascii"))   #send acknowledge comand

                    #update UI
                    self.status.config(text="Data Saved!")
                    self.sample.config(text= samplecount)
                    self.parent.update()
                    
                    
                    print(time.time() - start_time)
                    
                    return True

                # if "END" command is recieved, the device terminated communication, possibly due to an error
                elif(line != "END"):
                    self.status.config(text="Saving Data Failed")
                    self.cancelbutton.grid_remove()
                    winsound.PlaySound('SystemAsterisk', winsound.SND_ALIAS | winsound.SND_ASYNC)
                    return False

                
        # Exception handling
        except self.ser.SerialTimeoutException:
            self.ser.write(bytes("END\n",encoding="ascii"))
            self.status.config(text="Saving Data Failed: Timeout")
            winsound.PlaySound('SystemAsterisk', winsound.SND_ALIAS | winsound.SND_ASYNC)

        except Exception:
            print(Exception)
            #self.T.insert(END,'Error Saving\n')
            self.ser.write(bytes("END\n",encoding="ascii"))
            self.status.config(text="Saving Data Failed: Exception")
            winsound.PlaySound('SystemAsterisk', winsound.SND_ALIAS | winsound.SND_ASYNC)
            

        finally:
            
            self.filename.close()
        
            self.ser.close()
            

    #small method to update a flag if the user wants to cancel the current operation
    def cancelSave(self):
        self.cancel = True
        self.cancelbutton.config(relief=SUNKEN)
        self.status.config(text="Canceling...")
        self.parent.update()
        

    # method to setup the serial port on a given COM port
    def setup_serial(self):
        try:
            self.ser = serial.Serial(
                        port=self.currentCom,
                        baudrate=115200,
                        bytesize=serial.EIGHTBITS,
                        timeout=3
                        )
            #time.sleep(1)
            self.ser.flush()
            return True
        except serial.SerialException:
            #pops up an annoying warning box if serial fails
            etop= Toplevel(self)
            etop.geometry("250x90+800+150")
            etop.title('WARNING')
            message= Message(etop, text= "Serial cannot connect. Check Com port?", width=140, pady=10)
            message.pack()
            exitbutton= Button(etop, text= "Ok", command= lambda: etop.destroy())
            exitbutton.pack()
            winsound.PlaySound('SystemAsterisk', winsound.SND_ALIAS | winsound.SND_ASYNC)
            etop.update()
            
            
    
def enumerate_serial_ports():
    """ Uses the Win32 registry to return a iterator of serial 
        (COM) ports existing on this computer
    """
    
    path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
    except WindowsError:
        raise StopIteration

    for i in itertools.count():
        try:
            val = winreg.EnumValue(key, i)
            yield (str(val[1]), str(val[0]))
        except EnvironmentError:
            break



def main():
    root = Tk()
    root.geometry("500x135+600+100") #this is the size and position of the GUI window
    app = GUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()






