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
from collections import namedtuple
import numpy as np
from datetime import datetime
import errno
import re
from ws4py.client.threadedclient import WebSocketClient
import threading
import time
import random

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
            joint_angles=np.array(np.deg2rad(list(joints)))
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
            joint_angles2 = list(np.rad2deg(np.rad2deg(joint_angles)))
            planned.joints.joints.extend(joint_angles2)

        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True

EGMRobotState=namedtuple('EGMRobotState', ['joint_angles', 'rapid_running', 'motors_on', 'robot_message'], verbose=False)
JointTarget=namedtuple('JointTarget', ['robax', 'extax'])
RobTarget=namedtuple('RobTarget', ['trans','rot','robconf','extax'])

class RAPID(object):

    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics'):
        self.base_url=base_url
        self.auth=requests.auth.HTTPDigestAuth(username, password)
        self._session=requests.Session()
        self._rmmp_session=None
        self._rmmp_session_t=None
        
    def _do_get(self, relative_url):
        url="/".join([self.base_url, relative_url])
        res=self._session.get(url, auth=self.auth)
        try:            
            return self._process_response(res)
        finally:
            res.close()
    

    def _do_post(self, relative_url, payload=None):
        url="/".join([self.base_url, relative_url])
        res=self._session.post(url, data=payload, auth=self.auth)
        try:
            return self._process_response(res)
        finally:
            res.close()

    def _process_response(self, response):        
        soup=BeautifulSoup(response.text)

        if (response.status_code == 500):
            raise Exception("Robot returning 500 Internal Server Error")
    
        if (response.status_code == 200 or response.status_code == 201  \
            or response.status_code==202 or response.status_code==204):
            
            return soup.body
        
        if soup.body is None:
            raise Exception("Robot returning HTTP error " + str(response.status_code))
        
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
    
    def get_jointtarget(self, mechunit="ROB_1"):
        soup=self._do_get("rw/motionsystem/mechunits/" + mechunit + "/jointtarget")
        state=str(soup.find('li', attrs={'class': 'ms-jointtarget'}))
        robjoint=np.zeros((6,))
        i=0
        for match in re.finditer('class=\"rax_(\\d)">(-?\\d+(?:\\.\\d+)?)',state):
            j=int(match.groups()[0])
            assert i+1==j
            a=float(match.groups()[1])
            robjoint[j-1]=np.deg2rad(a)
            i+=1
            
        #robjoint=np.array([np.deg2rad(float(state.find('span', attrs={'class': 'rax_' + str(i+1)}).text)) for i in xrange(6)]) 
        #extjoint=np.array([np.deg2rad(float(state.find('span', attrs={'class': 'eax_' + chr(i)}).text)) for i in xrange(ord('a'),ord('g'))])
        extjoint=None
        return JointTarget(robjoint,extjoint)
        
    def get_robtarget(self, mechunit='ROB_1', tool='tool0', wobj='wobj0', coordinate='Base'):
        soup=self._do_get("rw/motionsystem/mechunits/" + mechunit + "/robtarget?tool=%s&wobj=%s&coordinate=%s" % (tool, wobj, coordinate))
        state=soup.find('li', attrs={'class': 'ms-robtargets'})
        trans=np.array([(float(state.find('span', attrs={'class': i}).text)/1000.0) for i in 'xyz'])
        rot=np.array([(float(state.find('span', attrs={'class': 'q' + str(i+1)}).text)) for i in xrange(4)])
        robconf=np.array([(float(state.find('span', attrs={'class': i}).text)) for i in ['cf1','cf4','cf6','cfx']])
        extax=np.array([np.deg2rad(float(state.find('span', attrs={'class': 'eax_' + chr(i)}).text)) for i in xrange(ord('a'),ord('g'))])
        return RobTarget(trans,rot,robconf,extax)
    
    def _rws_value_to_jointtarget(self, val):
        v1=re.match('^\\[\\[([^\\]]+)\\],\\[([^\\]]+)\\]',val)
        robax = np.deg2rad(np.fromstring(v1.groups()[0],sep=','))
        extax = np.deg2rad(np.fromstring(v1.groups()[1],sep=','))
        return JointTarget(robax,extax)
    
    def _jointtarget_to_rws_value(self, val):
        assert np.shape(val[0]) == (6,)
        assert np.shape(val[1]) == (6,)
        robax=','.join([format(x, '.4f') for x in np.rad2deg(val[0])])
        extax=','.join([format(x, '.4f') for x in np.rad2deg(val[1])])
        rws_value="[[" + robax + "],[" + extax + "]]"
        return rws_value
    
    def get_rapid_variable_jointtarget(self, var):
        v = self.get_rapid_variable(var)
        return self._rws_value_to_jointtarget(v)
    
    def set_rapid_variable_jointtarget(self,var,value):
        rws_value=self._jointtarget_to_rws_value(value)
        self.set_rapid_variable(var, rws_value)
            
    def _rws_value_to_jointtarget_array(self,val):
        m1=re.match('^\\[(.*)\\]$',val)
        if len(m1.groups()[0])==0:
            return []
        arr=[]
        val1=m1.groups()[0]
        while len(val1) > 0:
            m2=re.match('^(\\[\\[[^\\]]+\\],\\[[^\\]]+\\]\\]),?(.*)$',val1)            
            val1 = m2.groups()[1]
            arr.append(self._rws_value_to_jointtarget(m2.groups()[0]))
        
        return arr       
    
    def _jointtarget_array_to_rws_value(self, val):
        return "[" + ','.join([self._jointtarget_to_rws_value(v) for v in val]) + "]"
    
    def get_rapid_variable_jointtarget_array(self, var):
        v = self.get_rapid_variable(var)
        return self._rws_value_to_jointtarget_array(v)
    
    def set_rapid_variable_jointtarget_array(self,var,value):
        rws_value=self._jointtarget_array_to_rws_value(value)
        self.set_rapid_variable(var, rws_value)

    def get_rapid_variable_num(self, var):
        return float(self.get_rapid_variable(var))
    
    def set_rapid_variable_num(self, var, val):
        self.set_rapid_variable(var, str(val))
        
    def get_rapid_variable_num_array(self, var):
        val1=self.get_rapid_variable(var)
        m=re.match("^\\[([^\\]]*)\\]$", val1)
        val2=m.groups()[0].strip()
        return np.fromstring(val2,sep=',')
    
    def set_rapid_variable_num_array(self, var, val):
        self.set_rapid_variable(var, "[" + ','.join([str(s) for s in val]) + "]")
    
    
    def read_ipc_message(self, queue_name, timeout=0):
        
        o=[]
        
        timeout_str=""
        if timeout > 0:
            timeout_str="&timeout=" + str(timeout)
        
        soup=self._do_get("rw/dipc/" + queue_name + "/?action=dipc-read" + timeout_str)
                            
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        for li in ul.findAll('li'):
            def find_val(v):
                return li.find('span', attrs={'class': v}).text
            msgtype=find_val('dipc-msgtype')
            cmd=int(find_val('dipc-cmd'))
            userdef=int(find_val('dipc-userdef'))
            data=find_val('dipc-data')
            
            o.append(RAPIDIpcMessage(data,userdef,msgtype,cmd))
            
            #o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o
    
    def send_ipc_message(self, target_queue, data, queue_name="rpi_abb_irc5", cmd=111, userdef=1, msgtype=1 ):
        payload={"dipc-src-queue-name": queue_name, "dipc-cmd": str(cmd), "dipc-userdef": str(userdef), \
                 "dipc-msgtype": str(msgtype), "dipc-data": data}
        res=self._do_post("rw/dipc/" + target_queue + "?action=dipc-send", payload)
    
    def get_ipc_queue(self, queue_name):
        res=self._do_get("rw/dipc/" + queue_name + "?action=dipc-read")
        return res
    
    def try_create_ipc_queue(self, queue_name, queue_size=4440, max_msg_size=444):
        try:
            payload={"dipc-queue-name": queue_name, "dipc-queue-size": str(queue_size), "dipc-max-msg-size": str(max_msg_size)}
            self._do_post("rw/dipc?action=dipc-create", payload)
            return True
        except ABBException as e:
            if e.code==-1073445879:
                return False
            raise
    
    def request_rmmp(self, timeout=5):
        t1=time.time()
        self._do_post('users/rmmp', {'privilege': 'modify'})
        while time.time() - t1 < timeout:
            
            soup=self._do_get('users/rmmp/poll')
            status=soup.find('span', {'class': 'status'}).text
            if status=="GRANTED":
                self.poll_rmmp()
                return
            elif status!="PENDING":
                raise Exception("User did not grant remote access")                               
            time.sleep(0.25)
        raise Exception("User did not grant remote access")
    
    def poll_rmmp(self):
        
        # A "persistent session" can only make 400 calls before
        # being disconnected. Once this connection is lost,
        # the grant will be revoked. To work around this,
        # create parallel sessions with copied session cookies
        # to maintain the connection.
        
        url="/".join([self.base_url, 'users/rmmp/poll'])
        
        old_rmmp_session=None
        if self._rmmp_session is None:
            self._do_get(url)
            self._rmmp_session=requests.Session()
            self._rmmp_session_t=time.time()            
            
            for c in self._session.cookies:
                self._rmmp_session.cookies.set_cookie(c) 
        else:
            if time.time() - self._rmmp_session_t > 30:
                old_rmmp_session=self._rmmp_session
                rmmp_session=requests.Session()
                
                for c in self._session.cookies:
                    rmmp_session.cookies.set_cookie(c)
        
        rmmp_session=self._rmmp_session        
                
        res=rmmp_session.get(url, auth=self.auth)
        soup=self._process_response(res)
                
        if old_rmmp_session is not None:
            self._rmmp_session=rmmp_session
            self._rmmp_session_t=time.time()
            try:
                old_rmmp_session.close()
            except:
                pass
        
        return soup.find('span', {'class': 'status'}).text == "GRANTED"
    
    def subscribe_controller_state(self, callback, closed_callback=None):
        payload = {'resources':['1'],             
             '1':'/rw/panel/ctrlstate',
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDControllerStateSubscription, callback, closed_callback)
     
    def subscribe_operation_mode(self, callback, closed_callback=None):
        payload = {'resources':['1'],             
             '1':'/rw/panel/opmode',
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDOpmodeSubscription, callback, closed_callback)
    
    def subscribe_execution_state(self, callback, closed_callback=None):
        payload = {'resources':['1'],             
             '1':'/rw/rapid/execution;ctrlexecstate',
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDExecutionStateSubscription, callback, closed_callback)
     
    def subscribe_rapid_pers_variable(self, var, callback, closed_callback=None):
        payload = {'resources':['1'],             
             '1':'/rw/rapid/symbol/data/RAPID/T_ROB1/' + var + ';value',
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDPersVarSubscription, callback, closed_callback)
     
    def subscribe_ipc_queue(self, queue_name, callback, closed_callback=None):
        payload = {'resources':['1'],             
             '1':'/rw/dipc/' + queue_name,
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDIpcQueueSubscription, callback, closed_callback)
     
    def subscribe_event_log(self, callback, closed_callback=None):
        payload = {'resources':['1'],             
             '1':'/rw/elog/0',
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDElogSubscription, callback, closed_callback)
     
    def subscribe_digital_io(self, signal, network='Local', unit='DRV_1', callback=None, closed_callback=None):
        payload = {'resources':['1'],             
             '1': '/rw/iosystem/signals/' + network + '/' + unit + '/' + signal + ';state',
             '1-p':'1'}
        
        return self._subscribe(payload, RAPIDSignalSubscription, callback, closed_callback)
     
    
    
    def _subscribe(self, payload, ws_type, callback, closed_callback):    
        
        session=requests.Session()
        
        url="/".join([self.base_url, "subscription"])
        res1=session.post(url, data=payload, auth=self.auth)
        try:
            res=self._process_response(res1)
        finally:
            res1.close()        
        
        ws_url=res.find("a", {"rel": "self"})['href']
        cookie = 'ABBCX={0}'.format(session.cookies['ABBCX'])
        header=[('Cookie',cookie), ('Authorization', self.auth.build_digest_header("GET", ws_url))]
        ws=ws_type(ws_url, ['robapi2_subscription'], header, callback, closed_callback, session)
        ws.connect()        
        return ws

RAPIDExecutionState=namedtuple('RAPIDExecutionState', ['ctrlexecstate', 'cycle'], verbose=False)
RAPIDEventLogEntry=namedtuple('RAPIDEventLogEntry', ['msgtype', 'code', 'tstamp', 'args', 'title', 'desc', 'conseqs', 'causes', 'actions'])
RAPIDIpcMessage=namedtuple('RAPIDIpcMessage',['data','userdef','msgtype','cmd'])
RAPIDSignal=namedtuple('RAPIDSignal',['name','lvalue'])


class ABBException(Exception):
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code

class RAPIDSubscriptionClient(WebSocketClient):
    
    def __init__(self, ws_url, protocols, headers, callback, closed_callback,session):
        super(RAPIDSubscriptionClient,self).__init__(ws_url, protocols=protocols, headers=headers)
        self._callback=callback
        self._closed_callback=closed_callback
        self._session=session
    
    def opened(self):
        pass
    
    def closed(self, code, reason=None):
        if self._closed_callback is not None:
            self._closed_callback()
    
    def received_message(self, event_xml):        
        if event_xml.is_text:           
            soup=BeautifulSoup(event_xml.data)
            data=self.extract_data(soup)
            self._callback(data)
        else:
            print "Received Illegal Event " + str(event_xml)
            
    def extract_data(self, soup):
        return None
            
class RAPIDControllerStateSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        return soup.find("span", attrs={"class": "ctrlstate"}).text

class RAPIDOpmodeSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        return soup.find("span", attrs={"class": "opmode"}).text
    
class RAPIDExecutionStateSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        ctrlexecstate=soup.find('span', attrs={'class': 'ctrlexecstate'}).text        
        return ctrlexecstate

class RAPIDPersVarSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        o=[]
        
        for li in ul.findAll('li'):
            url=li.find('a')['href']
            m=re.match('^/rw/rapid/symbol/data/RAPID/T_ROB1/(.+);value$', url)
            o.append(m.groups()[0])
        return o
    
class RAPIDIpcQueueSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        o=[]
        
        for li in ul.findAll('li'):
            data=li.find('span', attrs={'class': 'dipc-data'}).text
            
            o.append(data)
        return o

class RAPIDElogSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        o=[]
        
        for li in ul.findAll('li'):
            data=int(li.find('span', attrs={'class': 'seqnum'}).text)
            
            o.append(data)
        return o
    
class RAPIDSignalSubscription(RAPIDSubscriptionClient):
    def extract_data(self, soup):
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')

        o=[]        
        for li in ul.findAll('li'):
            name=li["title"]
            lvalue=float(li.find('span', attrs={'class': 'lvalue'}).text)
                        
            o.append(RAPIDSignal(name,lvalue))
        return o
