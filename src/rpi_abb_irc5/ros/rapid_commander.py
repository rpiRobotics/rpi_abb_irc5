# Copyright (c) 2017, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Rensselaer Polytechnic Institute, or Wason 
#       Technology LLC, nor the names of its contributors may be used to 
#       endorse or promote products derived from this software without 
#       specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from collections import namedtuple

from ..srv import \
    RapidStart, RapidStartRequest, RapidStartResponse, \
    RapidStop, RapidStopRequest, RapidStopResponse, \
    RapidGetStatus, RapidGetStatusRequest, RapidGetStatusResponse, \
    RapidGetDigitalIO, RapidGetDigitalIORequest, RapidGetDigitalIOResponse, \
    RapidSetDigitalIO, RapidSetDigitalIORequest, RapidSetDigitalIOResponse, \
    RapidReadEventLog, RapidReadEventLogRequest, RapidReadEventLogResponse

from ..msg import RapidEventLogMessage

class RAPIDCommander(object):
    
    CYCLE_ASIS=RapidGetStatusResponse.CYCLE_ASIS
    CYCLE_ONCE=RapidGetStatusResponse.CYCLE_ONCE
    CYCLE_ONCE_DONE=RapidGetStatusResponse.CYCLE_ONCE_DONE
    CYCLE_FOREVER=RapidGetStatusResponse.CYCLE_FOREVER
    OPMODE_INIT=RapidGetStatusResponse.OPMODE_INIT           #State init
    OPMODE_AUTO_CH=RapidGetStatusResponse.OPMODE_AUTO_CH     #State change request for automatic mode
    OPMODE_MANF_CH=RapidGetStatusResponse.OPMODE_MANF_CH     #State change request for manual mode & full speed
    OPMODE_MANR=RapidGetStatusResponse.OPMODE_MANR           #State manual mode & reduced speed
    OPMODE_MANF=RapidGetStatusResponse.OPMODE_MANF           #State manual mode & full speed
    OPMODE_AUTO=RapidGetStatusResponse.OPMODE_AUTO           #State automatic mode
    OPMODE_UNDEFINED=RapidGetStatusResponse.OPMODE_UNDEFINED #State undefined
    CTRLSTATE_INIT=RapidGetStatusResponse.CTRLSTATE_INIT
    CTRLSTATE_MOTORON=RapidGetStatusResponse.CTRLSTATE_MOTORON
    CTRLSTATE_MOTOROFF=RapidGetStatusResponse.CTRLSTATE_MOTOROFF
    CTRLSTATE_GUARDSTOP=RapidGetStatusResponse.CTRLSTATE_GUARDSTOP
    CTRLSTATE_EMERGENCYSTOP=RapidGetStatusResponse.CTRLSTATE_EMERGENCYSTOP
    CTRLSTATE_EMERGENCYSTOPRESET=RapidGetStatusResponse.CTRLSTATE_EMERGENCYSTOPRESET
    CTRLSTATE_SYSFAIL=RapidGetStatusResponse.CTRLSTATE_SYSFAIL
    
    LOG_MSG_TYPE_INFO = RapidEventLogMessage.MSG_TYPE_INFO
    LOG_MSG_TYPE_WARNING = RapidEventLogMessage.MSG_TYPE_WARNING
    LOG_MSG_TYPE_ERROR = RapidEventLogMessage.MSG_TYPE_ERROR
    
    
    
    def __init__(self, ns='rapid'):
        self.ns=ns
        
        self._start_srv=rospy.ServiceProxy(rospy.names.ns_join(ns,'start'), \
                                                    RapidStart)
        
        self._stop_srv=rospy.ServiceProxy(rospy.names.ns_join(ns,'stop'), \
                                                    RapidStop)
        
        self._get_digital_io_srv=rospy.ServiceProxy(rospy.names.ns_join(ns,'get_digital_io'), \
                                                    RapidGetDigitalIO)
        self._set_digital_io_srv=rospy.ServiceProxy(rospy.names.ns_join(ns,'set_digital_io'), \
                                                    RapidSetDigitalIO)
        
        self._get_status_srv=rospy.ServiceProxy(rospy.names.ns_join(ns,'status'), \
                                                    RapidGetStatus)
        
        self._read_event_log_srv=rospy.ServiceProxy(rospy.names.ns_join(ns,'read_event_log'), \
                                                    RapidReadEventLog)
        
    def start(self, reset_pp=True, cycle="asis"):
        
        req=RapidStartRequest()
        req.reset_pp=reset_pp
        req.cycle=cycle
        
        self._start_srv.wait_for_service(1.0)        
        res=self._start_srv(req)
        
        if not res.success:
            raise Exception("RAPID Start Failed")
        
    def stop(self):
        
        req=RapidStopRequest()
                
        self._stop_srv.wait_for_service(1.0)        
        res=self._stop_srv(req)
        
        if not res.success:
            raise Exception("RAPID Stop Failed")

    def get_digital_io(self, signal):
        req=RapidGetDigitalIORequest()
        req.signal=signal
        
        self._get_digital_io_srv.wait_for_service(1.0)        
        res=self._get_digital_io_srv(req)
        
        if not res.success:
            raise Exception("RAPID Get Digital IO Failed")
        
        return res.lvalue
    
    def set_digital_io(self, signal, lvalue):
        req=RapidSetDigitalIORequest()
        req.signal=signal
        req.lvalue=lvalue
        
        self._set_digital_io_srv.wait_for_service(1.0)        
        res=self._set_digital_io_srv(req)
        
        if not res.success:
            raise Exception("RAPID Set Digital IO Failed")

    def get_status(self):
        req=RapidGetStatusRequest()
        
        self._get_status_srv.wait_for_service(1.0)        
        res=self._get_status_srv(req)
        
        if not res.success:
            raise Exception("RAPID Get Status Failed")
        
        return RAPIDNodeStatus(res.running, res.cycle, res.opmode, res.ctrlstate)
    
    def read_event_log(self):
        req=RapidReadEventLogRequest()
        
        self._read_event_log_srv.wait_for_service(1.0)        
        res=self._read_event_log_srv(req)
        
        if not res.success:
            raise Exception("RAPID Read Event Log Failed")
        
        ret=[]
        for e in res.messages:
            ret.append(RAPIDEventLogEntry(e.msgtype, e.code, e.tstamp, e.args, e.title, e.desc, e.conseqs, \
                                          e.causes, e.actions))
        
        return ret
        
RAPIDNodeStatus=namedtuple('RAPIDNodeStatus', ['running','cycle','opmode','ctrlstate'], verbose=False)
RAPIDEventLogEntry=namedtuple('RAPIDNodeEventLogEntry', ['msgtype', 'code', 'tstamp', 'args', 'title', \
                                                         'desc', 'conseqs', 'causes', 'actions'])

        
