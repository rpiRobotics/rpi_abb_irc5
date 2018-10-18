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

from __future__ import absolute_import

import socket
import select
from . import egm_pb2
import requests
from BeautifulSoup import BeautifulSoup
import traceback
from websocket import create_connection
from collections import namedtuple
import numpy as np
from datetime import datetime
import errno

class EGM(object):

    def __init__(self, port=6510):

        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('',port))
        self.send_sequence_number=0
        self.egm_addr=None
        self.count=0

    def receive_from_robot(self, timeout=0):

        s=self.socket
        s_list=[s]
        try:
            res=select.select(s_list, [], s_list, timeout)
        except select.error as err:
            if err.args[0] == errno.EINTR:
                return False, None
            else:
                raise

        if len(res[0]) == 0 and len(res[2])==0:
            return False, None
        try:
            (buf, addr)=s.recvfrom(65536)
        except:
            self.egm_addr=None
            return False, None

        self.egm_addr=addr

        robot_message=egm_pb2.EgmRobot()
        robot_message.ParseFromString(buf)

        joint_angles=None
        rapid_running=False
        motors_on=False

        if robot_message.HasField('feedBack'):
            joints=robot_message.feedBack.joints.joints
            joint_angles=np.array(list(joints))
        if robot_message.HasField('rapidExecState'):
            rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        if robot_message.HasField('motorState'):
            motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON

        return True, EGMRobotState(joint_angles, rapid_running, motors_on, robot_message)

    def send_to_robot(self, joint_angles):

        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned

        if joint_angles is not None:
            joint_angles2 = list(np.rad2deg(joint_angles))
            planned.joints.joints.extend(joint_angles2)

        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True

EGMRobotState=namedtuple('EGMRobotState', ['joint_angles', 'rapid_running', 'motors_on', 'robot_message'], verbose=False)

class RAPID(object):

    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics'):
        self.base_url=base_url
        self.auth=requests.auth.HTTPDigestAuth(username, password)

    def _do_get(self, relative_url):
        url="/".join([self.base_url, relative_url])
        res=requests.get(url, auth=self.auth)
        return self._process_response(res)

    def _do_post(self, relative_url, payload=None):
        url="/".join([self.base_url, relative_url])
        res=requests.post(url, data=payload, auth=self.auth)
        return self._process_response(res)

    def _process_response(self, response):        
        soup=BeautifulSoup(response.text)

        if (response.status_code == 500):
            raise Exception("Robot returning 500 Internal Server Error")
    

        if (response.status_code == 200 or response.status_code==204):
            return soup.body
        
        error_code=int(soup.find('span', attrs={'class':'code'}).text)
        error_message1=soup.find('span', attrs={'class': 'msg'})
        if (error_message1 is not None):
            error_message=error_message1.text
        else:
            error_message="Received error from ABB robot: " + str(error_code)

        raise ABBException(error_message, error_code)

    def start(self, cycle='asis'):
        payload={"regain": "continue", "execmode": "continue" , "cycle": cycle, "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"}
        res=self._do_post("rw/rapid/execution?action=start", payload)

    def stop(self):
        payload={"stopmode": "stop"}
        res=self._do_post("rw/rapid/execution?action=stop", payload)

    def resetpp(self):
        res=self._do_post("rw/rapid/execution?action=resetpp")

    def get_execution_state(self):
        soup = self._do_get("rw/rapid/execution")
        ctrlexecstate=soup.find('span', attrs={'class': 'ctrlexecstate'}).text
        cycle=soup.find('span', attrs={'class': 'cycle'}).text
        return RAPIDExecutionState(ctrlexecstate, cycle)
    
    def get_controller_state(self):
        soup = self._do_get("rw/panel/ctrlstate")
        return soup.find('span', attrs={'class': 'ctrlstate'}).text
    
    def get_operation_mode(self):
        soup = self._do_get("rw/panel/opmode")        
        return soup.find('span', attrs={'class': 'opmode'}).text
    
    def get_digital_io(self, signal, network='Local', unit='DRV_1'):
        soup = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)        
        state = soup.find('span', attrs={'class': 'lvalue'}).text
        return int(state)
    
    def set_digital_io(self, signal, value, network='Local', unit='DRV_1'):
        lvalue = '1' if bool(value) else '0'
        payload={'lvalue': lvalue}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)
    
    def get_rapid_variable(self, var):
        soup = self._do_get("rw/rapid/symbol/data/RAPID/T_ROB1/" + var)        
        state = soup.find('span', attrs={'class': 'value'}).text
        return state
    
    def set_rapid_variable(self, var, value):
        payload={'value': value}
        res=self._do_post("rw/rapid/symbol/data/RAPID/T_ROB1/" + var + "?action=set", payload)
        
    def read_event_log(self, elog=0):
        o=[]
        soup = self._do_get("rw/elog/" + str(elog) + "/?lang=en")
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        for li in ul.findAll('li'):
            def find_val(v):
                return li.find('span', attrs={'class': v}).text
            msg_type=int(find_val('msgtype'))
            code=int(find_val('code'))
            tstamp=datetime.strptime(find_val('tstamp'), '%Y-%m-%d T  %H:%M:%S')
            title=find_val('title')
            desc=find_val('desc')
            conseqs=find_val('conseqs')
            causes=find_val('causes')
            actions=find_val('actions')
            args=[]
            nargs=int(find_val('argc'))
            for i in xrange(nargs):
                arg=find_val('arg%d' % (i+1))
                args.append(arg)
            
            o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o

RAPIDExecutionState=namedtuple('RAPIDExecutionState', ['ctrlexecstate', 'cycle'], verbose=False)
RAPIDEventLogEntry=namedtuple('RAPIDEventLogEntry', ['msgtype', 'code', 'tstamp', 'args', 'title', 'desc', 'conseqs', 'causes', 'actions'])

class ABBException(Exception):
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code
