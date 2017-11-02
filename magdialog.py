import wx
import wx.xrc
import matplotlib.figure as mfigure
import matplotlib.animation as animation
import numpy as np
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
from struct import pack, unpack
import os

###########################################################################
## Class MAG Dialog
###########################################################################

class MagDialog ( wx.Dialog ):
    
    def __init__( self, parent, hid, CMD_START_SENSORS, CMD_STOP, CMD_MAG_CALIB):
        wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = wx.EmptyString, pos = wx.DefaultPosition, size = wx.Size( 1000,600 ), style = wx.DEFAULT_DIALOG_STYLE )
        
        self.hid = hid
        self.CMD_START_SENSORS = CMD_START_SENSORS
        self.CMD_STOP = CMD_STOP
        self.CMD_MAG_CALIB = CMD_MAG_CALIB
        self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
        
        bsMain = wx.BoxSizer( wx.VERTICAL )
        
        self.figure = mfigure.Figure(figsize=(10,5), dpi=100, tight_layout=True)
        self.axes = self.figure.add_subplot(111)
        self.canvas = FigureCanvasWxAgg (self, -1, self.figure)
        self.anim = animation.FuncAnimation(self.figure, self.animatePlot, interval=1000)
        self.data = [[0.]*15] # np.genfromtxt('./data/sensor_data_mag_calib.csv', delimiter=',')

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1, wx.LEFT | wx.TOP | wx.GROW)

        bsMain.Add( self.sizer, 0, wx.ALL, 5 )
        
        bsButtons = wx.BoxSizer( wx.HORIZONTAL )
        
        self.m_btn_startCollectData = wx.Button( self, wx.ID_ANY, u"Start collect data", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsButtons.Add( self.m_btn_startCollectData, 0, wx.ALL, 5 )
        self.m_btn_stopCollectData = wx.Button( self, wx.ID_ANY, u"Stop", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsButtons.Add( self.m_btn_stopCollectData, 0, wx.ALL, 5 )
        self.m_btn_saveData = wx.Button( self, wx.ID_ANY, u"Save data", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsButtons.Add( self.m_btn_saveData, 0, wx.ALL, 5 )
        self.m_btn_calibrate = wx.Button( self, wx.ID_ANY, u"Calibrate", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsButtons.Add( self.m_btn_calibrate, 0, wx.ALL, 5 )
                
        bsMain.Add( bsButtons, 1, wx.EXPAND, 5 )
        
        self.SetSizer( bsMain )
        self.Layout()
        
        self.Centre( wx.BOTH )

        self.Bind( wx.EVT_CLOSE, self.winClose, id = self.GetId() )
        self.m_btn_startCollectData.Bind( wx.EVT_BUTTON, self.m_btn_startCollectDataClick )
        self.m_btn_stopCollectData.Bind( wx.EVT_BUTTON, self.m_btn_stopCollectDataClick )
        self.m_btn_saveData.Bind( wx.EVT_BUTTON, self.m_btn_saveDataClick )
        self.m_btn_calibrate.Bind( wx.EVT_BUTTON, self.m_btn_calibrateClick )

    def winClose(self, event):
        self.anim.event_source.stop()
        self.hid.call(self.CMD_STOP, [], self.CMD_STOP_SENSORS_callback)
        event.Skip()

    def m_btn_startCollectDataClick(self, event):
        self.data = [[0.]*15]
        self.hid.call(self.CMD_START_SENSORS, [100, 4], self.CMD_START_SENSORS_callback)
    
    def m_btn_stopCollectDataClick(self, event):
        self.hid.call(self.CMD_STOP, [], self.CMD_STOP_SENSORS_callback)

    def m_btn_saveDataClick(self, event):
        dlg = wx.FileDialog(self, "Save project as...", os.getcwd(), "", "*.csv", \
                    wx.SAVE|wx.OVERWRITE_PROMPT)
        result = dlg.ShowModal()
        path_to_save = dlg.GetPath()
        dlg.Destroy()

        if result == wx.ID_OK: 
            adata = np.asarray(self.data)
            np.savetxt(path_to_save, adata, delimiter=",")          
            print "Data saved to %s"%path_to_save

    def m_btn_calibrateClick(self, event):
        self.hid.call(self.CMD_MAG_CALIB, [], self.CMD_MAG_CALIB_callback)
    
    def animatePlot(self, i):
        adata = np.asarray(self.data)
        self.axes.clear()
        xy, = self.axes.plot(adata[:,6], adata[:,7], 'y.', label='Mxy')
        xz, = self.axes.plot(adata[:,6], adata[:,8], 'r.', label='Mxz')
        yz, = self.axes.plot(adata[:,7], adata[:,8], 'c.', label='Myz')
        self.axes.legend(handles=[xy, xz, yz])
        self.axes.axis('equal')

    def CMD_START_SENSORS_callback(self, hid, byte_response):
        sensor_data = unpack('f' * 15, str(bytearray(byte_response)))  
        self.data.append(sensor_data)

    def CMD_STOP_SENSORS_callback(self, hid, byte_response):
        hid.releaseCallback(self.CMD_START_SENSORS)
        print "Sensors stopped"

    def CMD_MAG_CALIB_callback(self, hid, byte_response):
        result = unpack('f' * 6, str(bytearray(byte_response)))  
        print "Calibration completed"
        print ("Bias: ", result[0:3])
        print ("Scale: ", result[3:])

    def __del__( self ):
        pass


