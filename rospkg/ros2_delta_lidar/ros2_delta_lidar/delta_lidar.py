#!/usr/bin/env python3
# coding: utf-8

import serial
import time
import threading

# frame data
class delta_frame:
    _header       = 0   # (1 byte) Frame Header
    _frame_length = 0   # (2 byte) Frame Length : from frame header to checksum (excluded)
    _proc_ver     = 0   # (1 byte) Protocol Version
    _frame_type   = 0   # (1 byte) Frame Type
    _com_word     = 0   # (1 byte) Command Word : identifier to distinguish parameters
    _param_length = 0   # (2 byte) Parameter Length : length of the parameter field
    _param        = []  # (N byte) Parameter Field
    _checksum     = 0   # (2 byte) Checksum

# range data
class delta_rangedata:
    _rssi         = []  # (N byte) Parameter Field
    _range        = []  # (N byte) Parameter Field




# Delta LiDAR 2G
class delta_lidar(object):

    # constructor
    def __init__(self, port:str='/dev/ttyUSB0', baud:int=115200, use_ctrl:bool=False):
        self._port     = port
        self._baud     = baud
        self._use_ctrl = use_ctrl
        self._frame    = delta_frame()
        self._range    = delta_rangedata()
        self._serial   = None
        self._stat     = 0
        self._callback_range = None


    # destructor
    def __del__(self):
        pass


    def _parse_frame(self, frame):
        if( frame._com_word == 0xAE ):
            # device information
            _rpm = frame._param[0] * 3.0
            #print('RPM : {0}'.format(_rpm))
        elif( frame._com_word == 0xAD ):
            # range data
            _rpm = frame._param[0] * 3.0
            #print('RPM : {0}'.format(_rpm))

            # 2nd: Zero Offset angle (2 bytes)
            _offset_angle = (frame._param[1] << 8) + frame._param[2]
            _offset_angle = _offset_angle * 0.01

            #3rd: Start angle of current data freame (2 bytes)
            _start_angle  = (frame._param[3] << 8) + frame._param[4]
            _start_angle  = _start_angle * 0.01

            #Calculate number of samples in current frame
            _samp_cnt     = int((frame._param_length - 5) / 3)

            #Calculate current angle index of a full frame: For Delta-2G each full rotation has 15 frames
            _frame_index  = int( _start_angle / 24.0 )

            # start data
            if( _frame_index == 0 ):
                self._range._rssi  = []
                self._range._range = []

            for i in range(_samp_cnt):
                _rssi  = frame._param[5 + (i * 3)]
                _range = (frame._param[5 + (i * 3) + 1] << 8) + frame._param[5 + (i * 3) + 2]
                self._range._rssi.append(float(_rssi))
                self._range._range.append(_range*0.00025)

            if( _frame_index == 14 ):
                if( self._callback_range != None ):
                    self._callback_range( self._range )



    # process receive
    def _proc_recv(self):
        # start serial
        self._serial = serial.Serial( self._port, self._baud, rtscts=False )
        _checksum    = 0

        while(True):
            # read data
            _rx = self._serial.read(100)

            # frame check
            for _d in _rx:

                # frame header
                if( self._stat == 0 ):
                    if( _d == 0xAA ):
                        self._frame._header = _d
                        self._frame._param  = []
                        _checksum  = 0
                        self._stat = 1

                # frame length H
                elif( self._stat == 1 ):
                    self._frame._frame_length = _d * 256
                    self._stat = 2

                # frame length L
                elif( self._stat == 2 ):
                    # frame length L
                    self._frame._frame_length += _d
                    self._stat = 3

                # protocol version
                elif( self._stat == 3 ):
                    self._frame._proc_ver = _d
                    if( _d == 0x01 ):
                        self._stat = 4
                    else:
                        print('protocol version error')
                        self._stat = 0

                # frame type
                elif( self._stat == 4 ):
                    self._frame._frame_type = _d
                    if( _d == 0x61 ):
                        self._stat = 5
                    else:
                        print('frame type error')
                        self._stat = 0

                # command word
                elif( self._stat == 5 ):
                    self._frame._com_word = _d
                    self._stat = 6

                # parameter length H
                elif( self._stat == 6 ):
                    self._frame._param_length = _d * 256
                    self._stat = 7

                # parameter length L
                elif( self._stat == 7 ):
                    self._frame._param_length += _d
                    self._stat = 8

                # parameters
                elif( self._stat == 8 ):
                    self._frame._param.append(_d)
                    if( len(self._frame._param) == self._frame._param_length ):
                        self._stat = 9

                # checksum H
                elif( self._stat == 9 ):
                    self._frame._checksum = _d * 256
                    self._stat = 10

                # checksum L
                elif( self._stat == 10 ):
                    self._frame._checksum += _d
                    self._stat = 0
                    if( _checksum == self._frame._checksum ):
                        self._parse_frame( self._frame )

                # calc checksum
                if( self._stat < 10 ):
                    _checksum = ( _checksum + _d ) % 0xFFFF

        self._serial.close()


    # start
    def start(self, callback=None):
        # callback function
        self._callback_range = callback
        
        # control
        if( not self._use_ctrl ):
            self.ctrl_lidar(True)
        else:
            self.ctrl_lidar(False)        
        
        
        # start receive thread
        self._th_recv = threading.Thread( target=self._proc_recv )
        self._th_recv.setDaemon(True)
        self._th_recv.start()
    
    
    # activate or deactivate lidar (RTS pin control)
    def ctrl_lidar(self, active:bool=True):
        if( self._serial and self._use_ctrl ):
            self._serial.rts=active




# callback function for range
def callback_range( range ):
    print( range._range )


# main function
def main(args=None):
    try:
        lidar = delta_lidar()
        lidar.start( callback = callback_range )

        while(True):
            pass

    except KeyboardInterrupt:
        print('exit')


# main function
if(__name__ == '__main__'):
    main()

