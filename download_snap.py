#!/usr/bin/env python2.7
#--------------------------------------------
# capture jpeg from openmv module
# you can use mjpeg-streamer to stream the captured jpg file:
#
# cd /home/xx/mjpeg-streamer
# export LD_LIBRARY_PATH=./
# ./mjpeg-streamer -i "input_file.so -r -f ./ -d 30" -o "output_http.so -p 8090 -w ./www"
#
#--------------------------------------------
# This file is part of the OpenMV project.
#
# Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
# Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
# Copyright (c) 2020 Aixi Wang <aixi.wang@hotmail.com>
#
# This work is licensed under the MIT license, see the file LICENSE for details.
#
# Openmv module.

import struct
import sys,time
import serial
import platform
import numpy as np
#from PIL import Image

__serial = None
__FB_HDR_SIZE   =12

# USB Debug commands
__USBDBG_CMD            = 48
__USBDBG_FW_VERSION     = 0x80
__USBDBG_FRAME_SIZE     = 0x81
__USBDBG_FRAME_DUMP     = 0x82
__USBDBG_ARCH_STR       = 0x83
__USBDBG_SCRIPT_EXEC    = 0x05
__USBDBG_SCRIPT_STOP    = 0x06
__USBDBG_SCRIPT_SAVE    = 0x07
__USBDBG_SCRIPT_RUNNING = 0x87
__USBDBG_TEMPLATE_SAVE  = 0x08
__USBDBG_DESCRIPTOR_SAVE= 0x09
__USBDBG_ATTR_READ      = 0x8A
__USBDBG_ATTR_WRITE     = 0x0B
__USBDBG_SYS_RESET      = 0x0C
__USBDBG_FB_ENABLE      = 0x0D
__USBDBG_TX_BUF_LEN     = 0x8E
__USBDBG_TX_BUF         = 0x8F

ATTR_CONTRAST   =0
ATTR_BRIGHTNESS =1
ATTR_SATURATION =2
ATTR_GAINCEILING=3

__BOOTLDR_START         = 0xABCD0001
__BOOTLDR_RESET         = 0xABCD0002
__BOOTLDR_ERASE         = 0xABCD0004
__BOOTLDR_WRITE         = 0xABCD0008

def init(port, baudrate=921600, timeout=0.3):
    global __serial
    # open CDC port
    __serial =  serial.Serial(port, baudrate=baudrate, timeout=timeout)

def disconnect():
    global __serial
    try:
        if (__serial):
            __serial.close()
            __serial = None
    except:
        pass

def set_timeout(timeout):
    __serial.timeout = timeout

def fb_size():
    # read fb header
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_FRAME_SIZE, __FB_HDR_SIZE))
    return struct.unpack("III", __serial.read(12))

def fb_dump():
    size = fb_size()

    if (not size[0]):
        # frame not ready
        return None

    if (size[2] > 2): #JPEG
        num_bytes = size[2]
    else:
        num_bytes = size[0]*size[1]*size[2]

    # read fb data
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_FRAME_DUMP, num_bytes))
    buff = __serial.read(num_bytes)

    if size[2] == 1:  # Grayscale
        y = np.fromstring(buff, dtype=np.uint8)
        buff = np.column_stack((y, y, y))
    elif size[2] == 2: # RGB565
        arr = np.fromstring(buff, dtype=np.uint16).newbyteorder('S')
        r = (((arr & 0xF800) >>11)*255.0/31.0).astype(np.uint8)
        g = (((arr & 0x07E0) >>5) *255.0/63.0).astype(np.uint8)
        b = (((arr & 0x001F) >>0) *255.0/31.0).astype(np.uint8)
        buff = np.column_stack((r,g,b))
    else: # JPEG
        pass
        #try:
        #    buff = np.asarray(Image.frombuffer("RGB", size[0:2], buff, "jpeg", "RGB", ""))
        #except Exception as e:
        #    print ("JPEG decode error (%s)"%(e))
        #    return None

    if (buff.size != (size[0]*size[1]*3)):
        return None

    return (size[0], size[1], buff.reshape((size[1], size[0], 3)))

def exec_script(buf):
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_SCRIPT_EXEC, len(buf)))
    __serial.write(buf.encode())

def stop_script():
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_SCRIPT_STOP, 0))

def script_running():
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_SCRIPT_RUNNING, 4))
    return struct.unpack("I", __serial.read(4))[0]

def save_template(x, y, w, h, path):
    buf = struct.pack("IIII", x, y, w, h) + path
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_TEMPLATE_SAVE, len(buf)))
    __serial.write(buf)

def save_descriptor(x, y, w, h, path):
    buf = struct.pack("HHHH", x, y, w, h) + path
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_DESCRIPTOR_SAVE, len(buf)))
    __serial.write(buf)

def set_attr(attr, value):
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_ATTR_WRITE, 8))
    __serial.write(struct.pack("<II", attr, value))

def get_attr(attr):
    __serial.write(struct.pack("<BBIh", __USBDBG_CMD, __USBDBG_ATTR_READ, 1, attr))
    return __serial.read(1)

def reset():
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_SYS_RESET, 0))

def bootloader_start():
    __serial.write(struct.pack("<I", __BOOTLDR_START))
    return struct.unpack("I", __serial.read(4))[0] == __BOOTLDR_START

def bootloader_reset():
    __serial.write(struct.pack("<I", __BOOTLDR_RESET))

def flash_erase(sector):
    __serial.write(struct.pack("<II", __BOOTLDR_ERASE, sector))

def flash_write(buf):
    __serial.write(struct.pack("<I", __BOOTLDR_WRITE) + buf)

def tx_buf_len():
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_TX_BUF_LEN, 4))
    return struct.unpack("I", __serial.read(4))[0]

def tx_buf(bytes):
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_TX_BUF, bytes))
    return __serial.read(bytes)

def fw_version():
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_FW_VERSION, 12))
    return struct.unpack("III", __serial.read(12))

def enable_fb(enable):
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_FB_ENABLE, 4))
    __serial.write(struct.pack("<I", enable))

def arch_str():
    __serial.write(struct.pack("<BBI", __USBDBG_CMD, __USBDBG_ARCH_STR, 64))
    return __serial.read(64).split('\0', 1)[0]
    
    
#===============================================================================
# download script to openmv module
#===============================================================================
import time,sys,serial, struct

port = '/dev/ttyACM0'




while True:
    print('prepare to download script ...')
    
    buf = '''import sensor, image, time, ustruct
from pyb import USB_VCP
usb = USB_VCP()
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
while(True):
    cmd = usb.recv(4, timeout=1000)
    if (cmd == b'snap'):
        img = sensor.snapshot().compress()
        usb.send(ustruct.pack("<L", img.size()))
        usb.send(img)
'''
    print('1.1')
    try:
        disconnect()
    except:
        time.sleep(1)
        continue

    print('1.2')
    try:
        init(port)
        stop_script()
        exec_script(buf)
        tx_len = tx_buf_len()
        time.sleep(0.250)
        
        print('1.3')
        if (tx_len):
            print('1.4')
            print(tx_buf(tx_len).decode())
            print('1.5')
        print('download script succesfully!')    
        print('1.6')
    except Exception as e:
        print('download script exceptoin! It may be caused by re-download')
        time.sleep(1)
        pass
    
    print('1.7')
    try:
        disconnect()
    except:
        time.sleep(1)
        continue
        pass
    print('1.8')

    #===============================================================================
    # read image from openmv
    #===============================================================================
    try:
        print('2.1')
        sp = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=3.0, dsrdtr=True)
        sp.setDTR(True) # dsrdtr is ignored on Windows.
    except:
        time.sleep(1)
        continue
        
    print('2.2')
    n = 0
    while True:
        try:
            #print('2.3')
            t1 = time.time()
            sp.write("snap")
            sp.flush()
            size = struct.unpack('<L', sp.read(4))[0]
            img = sp.read(size)
            if len(img) == size:
                with open("img.jpg", "w") as f:            
                    f.write(img)            
                time.sleep(0.01)
                n += 1
                t2 = time.time()
                print('------->',n,1/(t2-t1), 'fps')
                #print('2.4')
            else:
                print('2.6')
                break
        except:
            try:
                sp.close()
            except:
                pass
                
            print('2.5')
            time.sleep(1)
            break
            