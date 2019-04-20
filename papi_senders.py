#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
papi_senders

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

revision 0.1  2017/abr  mlabru
initial release (Linux/Python)
"""
__version__ = "$revision: 0.1$"
__author__ = "Milton Abrunhosa"
__date__ = "2017/04"

# < imports >--------------------------------------------------------------------------------------

# python library
import base64
import IN
import logging
import queue
import random
import serial
import socket
import sys
import threading
import time

# numPy
import numpy as np

# openCV
import cv2

# model
import model.pc_data as gdat

import model.pc_sns_altimeter as salt
import model.pc_sns_barometer as sbar
import model.pc_sns_gps as sgps
import model.pc_sns_thermometer as sthr

# control
import control.pc_defs as gdefs
import control.pc_config as gcfg

# < module data >----------------------------------------------------------------------------------

# logger
M_LOG = logging.getLogger(__name__)
M_LOG.setLevel(logging.DEBUG)

# -------------------------------------------------------------------------------------------------
def get_mtu_size():
    """
    get MTU max size
    """
    # logger
    M_LOG.info("get_mtu_size >>")

    # create dummy socket
    li_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # try connect
    li_sock.connect(("23.26.29.40", 4061))

    # set options
    li_sock.setsockopt(socket.IPPROTO_IP, IN.IP_MTU_DISCOVER, IN.IP_PMTUDISC_DO)

    try:
        # try send big packet
        li_sock.send('#' * 61970)  #1473)

    # em caso de erro...
    except socket.error:
        pass

    # return
    # return li_sock.getsockopt(socket.IPPROTO_IP, getattr(IN, "IP_MTU", 14))
    return 65000

# -------------------------------------------------------------------------------------------------
def send_all(f_sock, fs_msg, fi_pckt_sz, ft_udp_addr):
    """
    send message in packets

    @param fs_msg: message
    @param fi_pckt_sz: packet size
    """
    # logger
    M_LOG.info("send_all >>")

    # header
    ls_header = "{}#{}#".format(gdefs.D_MSG_VRS, gdefs.D_MSG_I00)
    M_LOG.debug("header sz = {}".format(len(ls_header)))

    # tamanho da mensagem
    li_msg_len = len(fs_msg)

    # envia o tamanho da string 
    f_sock.sendto("{}#{}#{}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_SIZ, len(ls_header) + li_msg_len), ft_udp_addr)
    M_LOG.debug("{}#{}#{}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_SIZ, len(ls_header) + li_msg_len))

    # calc number of packets
    li_nmes = int(li_msg_len / fi_pckt_sz)
    M_LOG.debug("li_nmes = {}".format(li_nmes))

    # index
    li_ndx = -1

    # tamanho da mensagem excede o tamanho máximo de MTU ?
    if li_nmes > 0:
        # for all packets
        for li_ndx in range(li_nmes):
            ls_str = "{}#{}#{}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_I00, fs_msg[li_ndx * fi_pckt_sz : (li_ndx + 1) * fi_pckt_sz])
            
            # send message
            f_sock.sendto("{}#{}#{}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_I00, fs_msg[li_ndx * fi_pckt_sz : (li_ndx + 1) * fi_pckt_sz]), ft_udp_addr)
            M_LOG.debug("{}#{}#{} len:{}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_I00, ft_udp_addr, len(ls_str)))

    # envia a string
    f_sock.sendto("{}#{}#{}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_I99, fs_msg[(li_ndx + 1) * fi_pckt_sz :]), ft_udp_addr)

# -------------------------------------------------------------------------------------------------
def send_cam(f_queue, fsck_ccc, ft_ccc_addr):
    """
    sender camera thread
    """
    # logger
    M_LOG.info("send_cam >>")

    # init number of frames sended
    li_num_frames = 0
    
    # tupla network address
    lt_img_addr = (gdat.G_DCT_CONFIG["net.gcs"], gdat.G_DCT_CONFIG["net.img"])
    M_LOG.debug("lt_img_addr = {}".format(lt_img_addr))

    # get packet size (MTU:? - TCP_header:28 - msg_header:10)
    li_pckt_sz = get_mtu_size() - 28 - gdefs.D_HDR_SIZ
    M_LOG.debug("mtu_size = {}".format(get_mtu_size()))
    M_LOG.debug("li_pckt_sz = {}".format(li_pckt_sz))

    # cria o soket
    lsck_cam = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    assert lsck_cam

    # inicia a captura do vídeo
    l_capture = cv2.VideoCapture(gdat.G_DCT_CONFIG["glb.video"])
    assert l_capture

    l_capture.set(cv2.CAP_PROP_FRAME_WIDTH, gdefs.D_VID_HORZ)
    l_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, gdefs.D_VID_VERT)

    # start time
    ll_start = time.time()

    # para todo o sempre...
    while gdat.G_KEEP_RUN:
        # obtém um frame
        l_ret, l_frame = l_capture.read()

        # encode em jpeg
        l_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        # faz o encode
        l_result, l_img_encode = cv2.imencode(".jpg", l_frame, l_encode_param)

        # converte em um array
        l_data = np.array(l_img_encode)

        # encode base64 ?
        if gdefs.D_B64:
            # converte em string
            ls_str_data = base64.b64encode(l_data.tostring())
            M_LOG.debug("len(64): {}".format(len(ls_str_data)))

        # senão,...
        else:
            # converte em string
            ls_str_data = l_data.tostring()
            M_LOG.debug("len(st): {}".format(len(ls_str_data)))
        
        # send message
        send_all(lsck_cam, ls_str_data, li_pckt_sz, lt_img_addr)

        # increment number of frames captured/sended
        li_num_frames += 1

        # time for stats ?
        if (0 == (li_num_frames % gdefs.D_CAM_NFRAMES)):
            # elapsed time
            ll_elapsed = time.time() - ll_start

            # calculate frames per second
            lf_fps = float(li_num_frames) / float(ll_elapsed)

            # envia fps 
            fsck_ccc.sendto("{}#{}#{:3.1f}".format(gdefs.D_MSG_VRS, gdefs.D_MSG_FPS, lf_fps), ft_ccc_addr)

    # fecha o socket
    lsck_cam.close()

    # release video
    l_capture.release()

# -------------------------------------------------------------------------------------------------
def send_sensors(f_queue, fsck_ccc, ft_ccc_addr):
    """
    sender sensors thread
    """
    # logger
    M_LOG.info("send_sensors >>")

    # create altimeter
    l_alt = salt.CAltimeter(None, gdat.G_DCT_CONFIG["net.gcs"], gdat.G_DCT_CONFIG["net.alt"])
    assert l_alt

    # create barometer
    l_bar = sbar.CBarometer(None, gdat.G_DCT_CONFIG["net.gcs"], gdat.G_DCT_CONFIG["net.bar"])
    assert l_bar

    # create gps
    l_gps = sgps.CGPS(None, gdat.G_DCT_CONFIG["net.gcs"], gdat.G_DCT_CONFIG["net.gps"])
    assert l_gps

    # create thermometer
    l_thr = sthr.CThermometer(None, gdat.G_DCT_CONFIG["net.gcs"], gdat.G_DCT_CONFIG["net.thr"])
    assert l_thr

    # while keep running...
    while gdat.G_KEEP_RUN:
        # block until get message
        ls_msg = f_queue.get()

        # invalid ?
        if not ls_msg:
            # logger
            l_log = logging.getLogger("papi_sender::send_sensors")
            l_log.setLevel(logging.WARNING)
            l_log.warning("<E01: queue empty.")

            # next message
            continue
 
        # split message
        llst_msg = ls_msg.split('#')
        M_LOG.debug("llst_msg: {}".format(llst_msg))

        try:
            # mensagem de ccc ?
            if "!@CCC" == llst_msg[0]:
                if len(llst_msg) > 3:
                    # send ccc message (<vrs>#<tipo>#<[conteúdo,...]>#<ts>)
                    fsck_ccc.sendto("{}#{}#{}#{}".format(gdefs.D_MSG_VRS, int(llst_msg[1]), llst_msg[2], float(llst_msg[3])), ft_ccc_addr)

            # mensagem de altímetro ?
            elif "!@ALT" == llst_msg[0]:
                if len(llst_msg) > 3:
                    # send altimeter message (alt1, alt2, ts)
                    l_alt.send_data(float(llst_msg[1]), float(llst_msg[2]), float(llst_msg[3]))

            # mensagem de GPS ?
            elif "!@GPS" == llst_msg[0]:
                if len(llst_msg) > 6:
                    # send gps message (lat, lng, alt, sats, hdop, ts)
                    l_gps.send_data(float(llst_msg[1]), float(llst_msg[2]), float(llst_msg[3]), int(llst_msg[4]), int(llst_msg[5]), float(llst_msg[6]))

            # mensagem de barômetro ?
            elif "!@BAR" == llst_msg[0]:
                if len(llst_msg) > 3:
                    # send barometer message (bar1, bar2, ts)
                    l_bar.send_data(float(llst_msg[1]), float(llst_msg[2]), float(llst_msg[3]))

            # mensagem de termômetro ?
            elif "!@THR" == llst_msg[0]:
                if len(llst_msg) > 3:
                    # send thermometer message (tmp1, tmp2, ts)
                    l_thr.send_data(float(llst_msg[1]), float(llst_msg[2]), float(llst_msg[3]))

        # em caso de erro...
        except Exception as l_err:
            # logger
            l_log = logging.getLogger("papi_sender::send_sensors")
            l_log.setLevel(logging.WARNING)
            l_log.warning("<E02: send data error: {}".format(l_err))
             
# -------------------------------------------------------------------------------------------------
def ser_read(f_queue):
    """
    serial reader thread
    """
    # logger
    M_LOG.info("ser_read >>")

    # open serial port
    l_ser = serial.Serial(gdat.G_DCT_CONFIG["ser.port"], gdat.G_DCT_CONFIG["ser.baud"])
    assert l_ser

    # while keep running...
    while gdat.G_KEEP_RUN:
        # read serial line        
        ls_line = l_ser.readline()
        M_LOG.debug("ls_line: {}".format(ls_line))

        # queue message
        f_queue.put(ls_line[:-2])

# -------------------------------------------------------------------------------------------------
def main():
    """
    REAL PROGRAM MAIN
    """
    # logger
    M_LOG.info("main >>")

    # load config
    gcfg.load_config("papi.cfg")

    # cria o soket
    lsck_ccc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    assert lsck_ccc

    # tupla network address
    lt_ccc_addr = (gdat.G_DCT_CONFIG["net.gcs"], gdat.G_DCT_CONFIG["net.ccc"])

    # create read queue
    l_queue = queue.Queue()
    assert l_queue

    # start application
    gdat.G_KEEP_RUN = True

    # create sender camera thread
    lthr_cam = threading.Thread(target=send_cam, args=(l_queue, lsck_ccc, lt_ccc_addr))
    assert lthr_cam

    # start sender camera thread
    lthr_cam.start()

    # debug mode ?
    if gdat.G_DCT_CONFIG["glb.debug"]:
        # import fake serial
        import control.ser_fake as sfk

        # fake gcs address 
        gdat.G_DCT_CONFIG["net.gcs"] = "192.168.1.182"

        # create serial read thread
        lthr_ser = threading.Thread(target=sfk.ser_fake, args=(l_queue,))
        assert lthr_ser

    # senão, real mode...
    else:    
        # create serial read thread
        lthr_ser = threading.Thread(target=ser_read, args=(l_queue,))
        assert lthr_ser
    
    # start serial read thread
    lthr_ser.start()

    # create sender sensors thread
    lthr_sns = threading.Thread(target=send_sensors, args=(l_queue, lsck_ccc, lt_ccc_addr))
    assert lthr_sns

    # start sender sensors thread
    lthr_sns.start()

    # aguarda as threads
    lthr_cam.join()
    lthr_ser.join()
    lthr_sns.join()

# -------------------------------------------------------------------------------------------------
# this is the bootstrap process

if "__main__" == __name__:

    # logger
    logging.basicConfig(level=logging.DEBUG)

    # disable logging
    # logging.disable(sys.maxsize)

    # run application
    main()

# < the end >--------------------------------------------------------------------------------------
