import visual as vs
from pylab import pi, array, mat, deg2rad
import math 
from struct import pack, unpack
import wx
import numpy
from utils import RawHIDDevice, TimeCounter
import time
from regdialog import RegDialog

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
        self.update_frequency = 0
        self.run_loop = None
        self.w = vs.window(menus=True, title="9DOF quaternion visualizer", x=0, y=0, width=800, height=600)
        ######################################
        ### WINDOW CONTROLS ##################
        ######################################
        gbSizer3 = wx.GridBagSizer( 5, 5 )
        gbSizer3.SetFlexibleDirection( wx.BOTH )
        gbSizer3.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
        
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
        
        gbSizer3.Add( sbQ, wx.GBPosition( 0, 1 ), wx.GBSpan( 1, 1 ), wx.EXPAND, 5 )
        
        # Frequency box #####################
        sbFrequency = wx.StaticBoxSizer( wx.StaticBox( self.w.panel, wx.ID_ANY, u"Frequency" ), wx.VERTICAL )       
        self.m_st_freq = wx.StaticText( sbFrequency.GetStaticBox(), wx.ID_ANY, u"100.0Hz", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_st_freq.SetFont( wx.Font( 20, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD, False, wx.EmptyString ) )
        self.m_st_freq.SetForegroundColour( wx.Colour( 0, 0, 160 ) )
        sbFrequency.Add( self.m_st_freq, 0, wx.ALL, 5 )
        
        gbSizer3.Add( sbFrequency, wx.GBPosition( 0, 2 ), wx.GBSpan( 1, 1 ), wx.EXPAND, 5 )
        
        self.w.panel.SetSizer( gbSizer3 )
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

        # 3D scene ###########################################################################################

        self.scene=vs.display(window=self.w, x=5, y=150, width=770, height=410)

        self.scene.range=2

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

    def setQuaternionLabels(self, quaternion):
        self.m_st_q0.SetLabelText('%+8f'%quaternion.q[0]) 
        self.m_st_q1.SetLabelText('%+8f'%quaternion.q[1]) 
        self.m_st_q2.SetLabelText('%+8f'%quaternion.q[2]) 
        self.m_st_q3.SetLabelText('%+8f'%quaternion.q[3]) 

    def setFrequencyLabel(self, freq):
        self.m_st_freq.SetLabelText('%04.0fHz'%freq)


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
        self.hid.call(CMD_START_SENSORS, [100, 0], self.CMD_START_SENSORS_callback)

    def mi_stopClick(self, event ) :
        self.hid.call(CMD_STOP, [], self.CMD_STOP_SENSORS_callback)

    def CMD_START_SENSORS_callback(self, hid, byte_response):
        sensor_data = unpack('f' * 15, str(bytearray(byte_response)))  
        self.q.fromCoeffs(sensor_data[10:14])
        self.update_frequency = sensor_data[-1]  

    def CMD_STOP_SENSORS_callback(self, hid, byte_response):
        hid.releaseCallback(CMD_START_SENSORS)

    def update(self):
        self.setFrequencyLabel(self.update_frequency)
        self.setQuaternionLabels(self.q)
        res = self.rmC * self.q
        axis = (res*self.initAxis)*res.conjugate()
        up = (res*self.initUp)*res.conjugate()
        self.f.axis = (axis.q[1], axis.q[2], axis.q[3])
        self.f.up = (up.q[1], up.q[2], up.q[3])

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
##    a = numpy.asarray(data)
##    numpy.savetxt("./data/sensor_data_dump.csv", a, delimiter=",")          


def main():
    w = MainWindow()
    w.loop()
    
if __name__ == '__main__':
    main()
