import wx
import wx.xrc
import wx.dataview

###########################################################################
## Class Reg Dialog
###########################################################################

REG_NAMES = [
'SELF_TEST_X_GYRO  ',
'SELF_TEST_Y_GYRO  ',
'SELF_TEST_Z_GYRO  ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'SELF_TEST_X_ACCEL ',
'SELF_TEST_Y_ACCEL ',
'SELF_TEST_Z_ACCEL ',
'-                 ',
'-                 ',
'-                 ',
'XG_OFFSET_H       ',
'XG_OFFSET_L       ',
'YG_OFFSET_H       ',
'YG_OFFSET_L       ',
'ZG_OFFSET_H       ',
'ZG_OFFSET_L       ',
'SMPLRT_DIV        ',
'CONFIG            ',
'GYRO_CONFIG       ',
'ACCEL_CONFIG      ',
'ACCEL_CONFIG_2    ',
'LP_ACCEL_ODR      ',
'WOM_THR           ',
'-                 ',
'-                 ',
'-                 ',
'FIFO_EN           ',
'I2C_MST_CTRL      ',
'I2C_SLV0_ADDR     ',
'I2C_SLV0_REG      ',
'I2C_SLV0_CTRL     ',
'I2C_SLV1_ADDR     ',
'I2C_SLV1_REG      ',
'I2C_SLV1_CTRL     ',
'I2C_SLV2_ADDR     ',
'I2C_SLV2_REG      ',
'I2C_SLV2_CTRL     ',
'I2C_SLV3_ADDR     ',
'I2C_SLV3_REG      ',
'I2C_SLV3_CTRL     ',
'I2C_SLV4_ADDR     ',
'I2C_SLV4_REG      ',
'I2C_SLV4_DO       ',
'I2C_SLV4_CTRL     ',
'I2C_SLV4_DI       ',
'I2C_MST_STATUS    ',
'INT_PIN_CFG       ',
'INT_ENABLE        ',
'-                 ',
'INT_STATUS        ',
'ACCEL_XOUT_H      ',
'ACCEL_XOUT_L      ',
'ACCEL_YOUT_H      ',
'ACCEL_YOUT_L      ',
'ACCEL_ZOUT_H      ',
'ACCEL_ZOUT_L      ',
'TEMP_OUT_H        ',
'TEMP_OUT_L        ',
'GYRO_XOUT_H       ',
'GYRO_XOUT_L       ',
'GYRO_YOUT_H       ',
'GYRO_YOUT_L       ',
'GYRO_ZOUT_H       ',
'GYRO_ZOUT_L       ',
'EXT_SENS_DATA_00  ',
'EXT_SENS_DATA_01  ',
'EXT_SENS_DATA_02  ',
'EXT_SENS_DATA_03  ',
'EXT_SENS_DATA_04  ',
'EXT_SENS_DATA_05  ',
'EXT_SENS_DATA_06  ',
'EXT_SENS_DATA_07  ',
'EXT_SENS_DATA_08  ',
'EXT_SENS_DATA_09  ',
'EXT_SENS_DATA_10  ',
'EXT_SENS_DATA_11  ',
'EXT_SENS_DATA_12  ',
'EXT_SENS_DATA_13  ',
'EXT_SENS_DATA_14  ',
'EXT_SENS_DATA_15  ',
'EXT_SENS_DATA_16  ',
'EXT_SENS_DATA_17  ',
'EXT_SENS_DATA_18  ',
'EXT_SENS_DATA_19  ',
'EXT_SENS_DATA_20  ',
'EXT_SENS_DATA_21  ',
'EXT_SENS_DATA_22  ',
'EXT_SENS_DATA_23  ',
'-                 ',
'-                 ',
'I2C_SLV0_DO       ',
'I2C_SLV1_DO       ',
'I2C_SLV2_DO       ',
'I2C_SLV3_DO       ',
'I2C_MST_DELAY_CTRL',
'SIGNAL_PATH_RESET ',
'MOT_DETECT_CTRL   ',
'USER_CTRL         ',
'PWR_MGMT_1        ',
'PWR_MGMT_2        ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'-                 ',
'FIFO_COUNTH       ',
'FIFO_COUNTL       ',
'FIFO_R_W          ',
'WHO_AM_I          ',
'-                 ',
'XA_OFFSET_H       ',
'XA_OFFSET_L       ',
'-                 ',
'YA_OFFSET_H       ',
'YA_OFFSET_L       ',
'-                 ',
'ZA_OFFSET_H       ',
'ZA_OFFSET_L       ',
]

class RegDialog ( wx.Dialog ):
    
    def __init__( self, parent, hid, CMD_READ_REGS):

        wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = wx.EmptyString, pos = wx.DefaultPosition, size = wx.Size( 800,400 ), style = wx.DEFAULT_DIALOG_STYLE )

        self.hid = hid
        self.CMD_READ_REGS = CMD_READ_REGS
        self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
        
        bsMain = wx.BoxSizer( wx.VERTICAL )
        
        self.m_dvl_ctrl = wx.dataview.DataViewListCtrl( self, wx.ID_ANY, wx.DefaultPosition, wx.Size( 1000,320 ), 0 )
        self.m_dvl_ctrl.SetMaxSize( wx.Size( -1,320 ) )
        self.m_dvl_col_addr_hex = self.m_dvl_ctrl.AppendTextColumn( u"ADDR Hex", width = 80, align = wx.ALIGN_CENTER)
        self.m_dvl_col_addr_dec = self.m_dvl_ctrl.AppendTextColumn( u"ADDR Dec", width = 80, align = wx.ALIGN_CENTER)
        self.m_dvl_col_regn     = self.m_dvl_ctrl.AppendTextColumn( u"REG"   , width = 120)
        self.m_dvl_col_bit7     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[7]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit6     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[6]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit5     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[5]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit4     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[4]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit3     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[3]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit2     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[2]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit1     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[1]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_bit0     = self.m_dvl_ctrl.AppendTextColumn( u"BIT[0]", width = 50 , align = wx.ALIGN_CENTER)
        self.m_dvl_col_hex      = self.m_dvl_ctrl.AppendTextColumn( u"Hex"   , width = 50 , align = wx.ALIGN_CENTER)
        
        for i in range(127):
            hexaddr = format(i, '02x').upper()
            self.m_dvl_ctrl.AppendItem([hexaddr, str(i), REG_NAMES[i]] + ['-'] * 9)

        bsMain.Add( self.m_dvl_ctrl, 0, wx.ALL, 5 )
        
        bsButtons = wx.BoxSizer( wx.HORIZONTAL )
        
        self.m_btn_regRead = wx.Button( self, wx.ID_ANY, u"Read Regs", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsButtons.Add( self.m_btn_regRead, 0, wx.ALL, 5 )
        self.m_btn_regReadNoSetup = wx.Button( self, wx.ID_ANY, u"Read Regs No Setup", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsButtons.Add( self.m_btn_regReadNoSetup, 0, wx.ALL, 5 )
                
        bsMain.Add( bsButtons, 1, wx.EXPAND, 5 )
        
        self.SetSizer( bsMain )
        self.Layout()
        
        self.Centre( wx.BOTH )

        self.m_btn_regRead.Bind( wx.EVT_BUTTON, self.btn_regReadClick )
        self.m_btn_regReadNoSetup.Bind( wx.EVT_BUTTON, self.m_btn_regReadNoSetupClick )

    def btn_regReadClick(self, event):
        self.hid.call(self.CMD_READ_REGS, [1], self.CMD_READ_REGS_callback)
    
    def m_btn_regReadNoSetupClick(self, event):
        self.hid.call(self.CMD_READ_REGS, [0], self.CMD_READ_REGS_callback)
    
    def CMD_READ_REGS_callback(self, hid, byte_response):
        start_addr = byte_response[0]
        for i in range(len(byte_response)-1):
            addr = i + start_addr
            byte = byte_response[i+1]
            binbyte = format(byte, '08b')
            hexbyte = format(byte, '02x').upper()
            for col in range(3, 11):
                self.m_dvl_ctrl.SetValue(binbyte[col-3], addr, col)
            self.m_dvl_ctrl.SetValue(hexbyte, addr, 11)

    def __del__( self ):
        pass