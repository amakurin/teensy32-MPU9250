import pywinusb.hid as hid
from struct import pack, unpack
from timeit import default_timer as timer
import math 
import numpy as np
from pylab import pi, array, mat, deg2rad

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

class RawHIDDevice(object):
    def __init__(self, deviceDescriptor):
        self.device = deviceDescriptor
        self.async = {}

    @staticmethod
    def tryOpen(vendor_id = 0x16C0, product_id = 0x486, usage_page = 0xFFAB, usage_id = 0x0200):
        raw_hid = None
        devices = hid.HidDeviceFilter(vendor_id = vendor_id, product_id = product_id).get_devices()
        for device in devices:
            try:
                device.open()
                reports = device.find_feature_reports(usage_id = usage_id)
                if len(reports)==0:
                    raw_hid = device
            finally:
                device.close()
        if raw_hid:
            raw_hid.open()
            reports = raw_hid.find_output_reports()
            hid_wrapper = RawHIDDevice(raw_hid)
            hid_wrapper.setRawDataHandler(RawHIDDevice.asyncDataHandler)
            return hid_wrapper
        else:
            return None

    def sendRawData(self, buffer64):
        buf_len = len(buffer64)
        if buf_len > 64:
            raise Exception('Can\'t send raw data: buffer64 has length greater then 64, (len = %s)'%buf_len)
        if buf_len < 64:
            buffer64 += ([0] * ( 64 - buf_len))
        reports = self.device.find_output_reports()
        if len(reports) == 0:
            raise Exception('Can\'t send raw data: No output reports found')
        buffer = [0] + buffer64
        reports[0].set_raw_data(buffer)
        reports[0].send()

    def setRawDataHandler(self, handler, args=[], kwargs={}):
        def rawDataHandler(data):
            _args = [self, data] + list(args)
            handler(*_args, **kwargs)
        self.device.set_raw_data_handler(rawDataHandler)

    def call(self, cmd_id, cmd_data, callback, args=[], kwargs={}):       
        data_len = len(cmd_data)
        if len(cmd_data)>62:
            raise Exception('Too much data during command (%s) call. Max len is 64, %s given'.format(cmd_id, data_len))
        self.async[cmd_id] = [callback, args, kwargs]
        data_buffer = [cmd_id, data_len] + cmd_data
        self.sendRawData(data_buffer)

    def releaseCallback(self, cmd_id):
        self.async.pop(cmd_id, None)

    @staticmethod
    def asyncDataHandler(hid, cmd_resp_data):
        cmd_id = cmd_resp_data[1]
        data_len = cmd_resp_data[2]
        final_packet = cmd_resp_data[3]
        data = cmd_resp_data[4:data_len+4]
        callback_config = hid.async.get(cmd_id)
        if callback_config is None:
            print 'Unknown command response. cmd_id = %s'%cmd_id
            return
        if final_packet == 1:
            hid.releaseCallback(cmd_id)

        callback, args, kwargs = callback_config
        args = [hid, data] + args
        callback(*args, **kwargs)

    def close(self):
        self.device.close()    


class TimeCounter(object):
    def __init__(self, avgThre = 100.):
        self.start = timer()
        self.cur = 0.
        self.avg = 0.
        self.cnt = 0.
        self.avgThre = avgThre

    def reset(self):
        self.start = timer()
        self.cur = 0.
        self.avg = 0.
        self.cnt = 0.

    def update(self):
        end = timer()
        self.avg += (end - self.start)
        self.cnt += 1.
        if self.cnt > self.avgThre:
            self.cur = (self.cur + self.avg/self.cnt)*0.5
            self.avg = 0.
            self.cnt = 0.
        self.start = end

    def getRate(self):
        return 1/self.cur if self.cur > 0. else 0