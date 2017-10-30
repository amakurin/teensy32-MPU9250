import visual as vs
import visual.graph as vsg
from pylab import pi, array, mat, deg2rad
import math 
from struct import pack, unpack
import wx
import numpy as np
from utils import RawHIDDevice, TimeCounter
import time
from regdialog import RegDialog
import matplotlib.figure as mfigure
import matplotlib.animation as manim

from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg

class Quaternion(object):
    def __init__(self, q = (1.0,0.0,0.0,0.0)):
        self.q = array(q, dtype=float)

    def conjugate(self):
        q0, q1, q2, q3 = self.q
        return self.__class__((q0, -q1, -q2, -q3))

    def __mul__(self, other):
        a0, a1, a2, a3 = self.q
        b0, b1, b2, b3 = other.q
        q0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3
        q1 = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2
        q2 = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1
        q3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0
        return self.__class__((q0, q1, q2, q3))

    def norm(self):
        q0, q1, q2, q3 = self.q
        return math.sqrt(q0**2+q1**2+q2**2+q3**2)

    def asVector(self):
        return vs.vector(self.q[1], self.q[2], self.q[3])

    def fromCoeffs(self, tupleCoeffs):
        self.q = array(tupleCoeffs)

CMD_START_SENSORS = 0
CMD_STOP = 1
CMD_MAG_CALIB = 2
CMD_READ_REGS = 3

class MainWindow(object):
    def __init__(self):
        self.hid = None
        self.q = Quaternion((1.0,0.0,1.0,1.0))
        self.sensor_data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.run_loop = None

        self.frame_rate_counter = TimeCounter()

        self.w = vs.window(menus=True, title="9DOF quaternion visualizer", x=0, y=0, width=1150, height=600)
        ######################################
        ### WINDOW CONTROLS ##################
        ######################################
        gbs_main = wx.GridBagSizer( 5, 5 )
        gbs_main.SetFlexibleDirection( wx.BOTH )
        gbs_main.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
        
        # Quaternion box #####################
        sbQ = wx.StaticBoxSizer( wx.StaticBox( self.w.panel, wx.ID_ANY, u"Quaternion" ), wx.VERTICAL )
        fsQ = wx.FlexGridSizer( 0, 2, 0, 0 )
        fsQ.SetFlexibleDirection( wx.BOTH )
        fsQ.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
        
        self.m_staticText11 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"q[0]:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_staticText11, 0, wx.ALL, 5 )
        self.m_st_q0 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"0.00000000", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_st_q0, 0, wx.ALL, 5 )
        self.m_staticText13 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"q[1]:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_staticText13, 0, wx.ALL, 5 )
        self.m_st_q1 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"0.00000000", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_st_q1, 0, wx.ALL, 5 )
        self.m_staticText15 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"q[2]:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_staticText15, 0, wx.ALL, 5 )
        self.m_st_q2 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"0.00000000", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_st_q2, 0, wx.ALL, 5 )
        self.m_staticText17 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"q[3]:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_staticText17, 0, wx.ALL, 5 )
        self.m_st_q3 = wx.StaticText( sbQ.GetStaticBox(), wx.ID_ANY, u"0.00000000", wx.DefaultPosition, wx.DefaultSize, 0 )
        fsQ.Add( self.m_st_q3, 0, wx.ALL, 5 )
        
        sbQ.Add( fsQ, 1, wx.EXPAND, 5 )
        
        gbs_main.Add( sbQ, wx.GBPosition( 0, 0 ), wx.GBSpan( 1, 1 ), wx.EXPAND, 5 )
        
        # Frequency box #####################
        sbFrequency = wx.StaticBoxSizer( wx.StaticBox( self.w.panel, wx.ID_ANY, u"Frequency" ), wx.VERTICAL )
        
        fgSizer3 = wx.FlexGridSizer( 0, 2, 0, 0 )
        fgSizer3.SetFlexibleDirection( wx.BOTH )
        fgSizer3.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
        
        self.m_st_filtUpdate = wx.StaticText( sbFrequency.GetStaticBox(), wx.ID_ANY, u"Filter update:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fgSizer3.Add( self.m_st_filtUpdate, 0, wx.ALL, 5 )
        
        self.m_st_filterRate = wx.StaticText( sbFrequency.GetStaticBox(), wx.ID_ANY, u"0000Hz", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_st_filterRate.SetFont( wx.Font( 20, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD, False, wx.EmptyString ) )
        self.m_st_filterRate.SetForegroundColour( wx.Colour( 0, 0, 160 ) )
        
        fgSizer3.Add( self.m_st_filterRate, 0, wx.ALL, 5 )
        
        self.m_st_frameUpdate = wx.StaticText( sbFrequency.GetStaticBox(), wx.ID_ANY, u"Frame update:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fgSizer3.Add( self.m_st_frameUpdate, 0, wx.ALL, 5 )
        
        self.m_st_frameRate = wx.StaticText( sbFrequency.GetStaticBox(), wx.ID_ANY, u"000Hz", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_st_frameRate.SetFont( wx.Font( 20, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD, False, "Arial" ) )
        self.m_st_frameRate.SetForegroundColour( wx.Colour( 100, 82, 74 ) )
        
        fgSizer3.Add( self.m_st_frameRate, 0, wx.ALL, 5 )
               
        sbFrequency.Add( fgSizer3, 1, wx.EXPAND, 5 )
        
        gbs_main.Add( sbFrequency,  wx.GBPosition( 0, 1 ), wx.GBSpan( 1, 1 ), wx.EXPAND, 5 )


        # Settings box #####################
        sbSettings = wx.StaticBoxSizer( wx.StaticBox( self.w.panel, wx.ID_ANY, u"Settings" ), wx.VERTICAL )
        
        fgsSettings = wx.FlexGridSizer( 0, 2, 0, 0 )
        fgsSettings.SetFlexibleDirection( wx.BOTH )
        fgsSettings.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
        
        self.m_st_gyro = wx.StaticText( sbSettings.GetStaticBox(), wx.ID_ANY, u"Gyro scale:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fgsSettings.Add( self.m_st_gyro, 0, wx.ALL, 5 )
        
        m_cho_gyroScaleChoices = []
        self.m_cho_gyroScale = wx.Choice( sbSettings.GetStaticBox(), wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_cho_gyroScaleChoices, 0 )
        self.m_cho_gyroScale.SetSelection( 0 )
        fgsSettings.Add( self.m_cho_gyroScale, 0, wx.ALL, 5 )
        
        self.m_st_accel = wx.StaticText( sbSettings.GetStaticBox(), wx.ID_ANY, u"Accel scale:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fgsSettings.Add( self.m_st_accel, 0, wx.ALL, 5 )
        
        m_cho_accelScaleChoices = []
        self.m_cho_accelScale = wx.Choice( sbSettings.GetStaticBox(), wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_cho_accelScaleChoices, 0 )
        self.m_cho_accelScale.SetSelection( 0 )
        fgsSettings.Add( self.m_cho_accelScale, 0, wx.ALL, 5 )
        
        self.m_st_filter = wx.StaticText( sbSettings.GetStaticBox(), wx.ID_ANY, u"Filter:", wx.DefaultPosition, wx.DefaultSize, 0 )
        fgsSettings.Add( self.m_st_filter, 0, wx.ALL, 5 )
        
        m_cho_filterChoices = []
        self.m_cho_filter = wx.Choice( sbSettings.GetStaticBox(), wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_cho_filterChoices, 0 )
        self.m_cho_filter.SetSelection( 0 )
        fgsSettings.Add( self.m_cho_filter, 0, wx.ALL, 5 )
        
        sbSettings.Add( fgsSettings, 1, wx.EXPAND, 5 )
        
        gbs_main.Add( sbSettings, wx.GBPosition( 0, 2 ), wx.GBSpan( 1, 1 ), wx.EXPAND, 5 )

        # Main layout #####################
        
        gbs_main.AddGrowableCol( 0 )
        gbs_main.AddGrowableCol( 2 )

        self.w.panel.SetSizer( gbs_main )
        self.w.panel.Layout()

        # Main Menu #####################
        self.m_menubar = wx.MenuBar( 0 )
        self.m_menu_main = wx.Menu()
        self.m_mi_regs = wx.MenuItem( self.m_menu_main, wx.ID_ANY, u"Show REGs", wx.EmptyString, wx.ITEM_NORMAL )
        self.m_menu_main.AppendItem( self.m_mi_regs )
        self.m_mi_magcalib = wx.MenuItem( self.m_menu_main, wx.ID_ANY, u"Calibrate Mag", wx.EmptyString, wx.ITEM_NORMAL )
        self.m_menu_main.AppendItem( self.m_mi_magcalib )
        self.m_mi_settings = wx.MenuItem( self.m_menu_main, wx.ID_ANY, u"Settings", wx.EmptyString, wx.ITEM_NORMAL )
        self.m_menu_main.AppendItem( self.m_mi_settings )
        self.m_mi_start = wx.MenuItem( self.m_menu_main, wx.ID_ANY, u"Start Vis", wx.EmptyString, wx.ITEM_NORMAL )
        self.m_menu_main.AppendItem( self.m_mi_start )
        self.m_mi_stop = wx.MenuItem( self.m_menu_main, wx.ID_ANY, u"Stop Vis", wx.EmptyString, wx.ITEM_NORMAL )
        self.m_menu_main.AppendItem( self.m_mi_stop )

        self.m_menubar.Append( self.m_menu_main, u"Tasks" ) 
        
        self.w.win.SetMenuBar( self.m_menubar)
        
        # Bind Events ########################################################################################
        self.w.win.Bind( wx.EVT_CLOSE, self.win_mainClose, id = self.w.win.GetId() )
        self.w.win.Bind( wx.EVT_MENU, self.mi_regsClick, id = self.m_mi_regs.GetId() )
        self.w.win.Bind( wx.EVT_MENU, self.mi_magcalibClick, id = self.m_mi_magcalib.GetId() )
        self.w.win.Bind( wx.EVT_MENU, self.mi_settingsClick, id = self.m_mi_settings.GetId() )
        self.w.win.Bind( wx.EVT_MENU, self.mi_startClick, id = self.m_mi_start.GetId() )
        self.w.win.Bind( wx.EVT_MENU, self.mi_stopClick, id = self.m_mi_stop.GetId() )

        # Plot ###############################################################################################
        self.plot=vsg.gdisplay(window=self.w, x=360, y=134, width=800, height=350, title='N vs. t', background=vs.color.black)
        self.gxs = vsg.gcurve(color=(190./255.,  20./255.,   0./255.), dot=True, size=4)
        self.gys = vsg.gcurve(color=( 22./255., 158./255.,   7./255.), dot=True, size=4)
        self.gzs = vsg.gcurve(color=( 11./255.,  96./255., 232./255.), dot=True, size=4)

        self.axs = vsg.gcurve(color=(255./255.,  76./255.,  50./255.), dot=True, size=4)
        self.ays = vsg.gcurve(color=( 85./255., 242./255.,  29./255.), dot=True, size=4)
        self.azs = vsg.gcurve(color=( 76./255., 171./255., 255./255.), dot=True, size=4)
        self.frame_counter = 0

        # 3D scene ###########################################################################################

        self.scene=vs.display(window=self.w, x=5, y=134, width=350, height=350)

        self.scene.range=1

        self.f = vs.frame()

        col_x = vs.color.red
        col_y = vs.color.green
        col_z = vs.color.blue

        axis_len = 0.7
        # X = vpyZ
        self.lblX = vs.label(frame=self.f, pos=(axis_len*1.05,0,0), text='X', box=0, opacity=0, color=col_x)
        self.arrX = vs.arrow(frame=self.f, axis=(1,0,0), shaftwidth=0.03, fixedwidth=1, length = axis_len, color=col_x)
        # Y = vpyX
        self.lblY = vs.label(frame=self.f, pos=(0,axis_len*1.05,0), text='Y', box=0, opacity=0, color=col_y)
        self.arrY = vs.arrow(frame=self.f, axis=(0,1,0), shaftwidth=0.03, fixedwidth=1, length = axis_len, color=col_y)
        # Z = vpyY
        self.lblZ = vs.label(frame=self.f, pos=(0,0,axis_len*1.05), text='Z', box=0, opacity=0, color=col_z)
        self.arrZ = vs.arrow(frame=self.f, axis=(0,0,1), shaftwidth=0.03, fixedwidth=1, length = axis_len,color=col_z)
        # Model
        self.platform = vs.box(frame=self.f, pos=(0.0, -0.07, 0.0), length=0.55, height=0.83, width=0.09, color=(1,1,1), opacity=0.9)
        self.teensy = vs.box(frame=self.f, pos=(0.05, -0.4+0.18-0.07, 0.045+0.025), length=0.18, height=0.36, width=0.05, color=(0.05,0.2,0.01), opacity=0.9)
        self.mpu = vs.box(frame=self.f, pos=(0, -0.04+0.125-0.07, 0.045+0.025), length=0.16, height=0.25, width=0.05, color=(0.01,0.05,0.2), opacity=0.9)
        
        # Prepare coordinate frame transformation to z-top, y-right, x-front
        sqrt_2 = math.sqrt(2.0)/2.0
        self.rmC = Quaternion((sqrt_2, 0, sqrt_2, 0)) * Quaternion((-sqrt_2, sqrt_2, 0, 0))

        self.initAxis = Quaternion((0, 1, 0, 0))
        self.initUp = Quaternion((0, 0, 1, 0))
        self.realtime_started = False

    def clear_plot(self):
        if self.plot:
            for obj in self.plot.display.objects: 
                obj.visible = False
                del obj
        self.gxs = vsg.gcurve(color=(190./255.,  20./255.,   0./255.), dot=True, size=4)
        self.gys = vsg.gcurve(color=( 22./255., 158./255.,   7./255.), dot=True, size=4)
        self.gzs = vsg.gcurve(color=( 11./255.,  96./255., 232./255.), dot=True, size=4)

        self.axs = vsg.gcurve(color=(255./255.,  76./255.,  50./255.), dot=True, size=4)
        self.ays = vsg.gcurve(color=( 85./255., 242./255.,  29./255.), dot=True, size=4)
        self.azs = vsg.gcurve(color=( 76./255., 171./255., 255./255.), dot=True, size=4)
        self.frame_counter = 0.

    def updateQuaternionLabels(self):
        self.m_st_q0.SetLabelText('%+8f'%self.q.q[0]) 
        self.m_st_q1.SetLabelText('%+8f'%self.q.q[1]) 
        self.m_st_q2.SetLabelText('%+8f'%self.q.q[2]) 
        self.m_st_q3.SetLabelText('%+8f'%self.q.q[3]) 

    def updateFrequencyLabels(self):
        self.frame_rate_counter.update()
        self.m_st_filterRate.SetLabelText('%04.0fHz'%self.sensor_data[-1])
        self.m_st_frameRate.SetLabelText('%03.0fHz'%self.frame_rate_counter.getRate())

    def win_mainClose(self, event):
        if self.hid:
            self.hid.call(CMD_STOP, [], self.CMD_STOP_SENSORS_callback)
            self.hid.close()
        self.run_loop = False
        event.Skip()

    def mi_regsClick(self, event) :
        dialog = RegDialog(self.w.win, self.hid)
        try:
            dialog.ShowModal()
        except Exception as e:
            print e
        finally:
            dialog.Destroy()

    def mi_magcalibClick(self, event ) :
        print '\n----  handle_mi_magcalib'

    def mi_settingsClick(self, event ) :
        print '\n----  handle_mi_settings'

    def mi_startClick(self, event ) :
        self.realtime_started = True
        self.frame_rate_counter.reset()
        self.hid.call(CMD_START_SENSORS, [100, 0], self.CMD_START_SENSORS_callback)

    def mi_stopClick(self, event ) :
        self.hid.call(CMD_STOP, [], self.CMD_STOP_SENSORS_callback)
        self.clear_plot()

    def CMD_START_SENSORS_callback(self, hid, byte_response):
        self.sensor_data = unpack('f' * 15, str(bytearray(byte_response)))  
        self.q.fromCoeffs(self.sensor_data[10:14])

    def CMD_STOP_SENSORS_callback(self, hid, byte_response):
        hid.releaseCallback(CMD_START_SENSORS)
        self.realtime_started = False

    def plot_sensor_data(self):
        self.frame_counter += 1.
        self.gxs.plot(pos=(self.frame_counter,self.sensor_data[0])) 
        self.gys.plot(pos=(self.frame_counter,self.sensor_data[1])) 
        self.gzs.plot(pos=(self.frame_counter,self.sensor_data[2])) 
        self.axs.plot(pos=(self.frame_counter,self.sensor_data[3])) 
        self.ays.plot(pos=(self.frame_counter,self.sensor_data[4])) 
        self.azs.plot(pos=(self.frame_counter,self.sensor_data[5])) 

    def update(self):
        if self.realtime_started:
            self.updateFrequencyLabels()
            self.updateQuaternionLabels()
            res = self.rmC * self.q
            axis = (res*self.initAxis)*res.conjugate()
            up = (res*self.initUp)*res.conjugate()
            self.f.axis = (axis.q[1], axis.q[2], axis.q[3])
            self.f.up = (up.q[1], up.q[2], up.q[3])
            self.plot_sensor_data()

    def loop(self):
        while not self.hid:
            self.hid = RawHIDDevice.tryOpen()
            if not self.hid:
                print "USB HID Device wasn't found. Retry in 5 seconds..."
                time.sleep(5)
        print "USB HID Device found."
        self.run_loop = True
        while self.run_loop:
            self.update()
            vs.sleep(1E-2)  
##    hid.call(CMD_MAG_CALIB, [], CMD_MAG_CALIB_callback)
#    data = []
##        data.append(sensor_data)
##    a = np.asarray(data)
##    np.savetxt("./data/sensor_data_dump.csv", a, delimiter=",")          


def main():
    w = MainWindow()
    w.loop()
    
if __name__ == '__main__':
    main()
