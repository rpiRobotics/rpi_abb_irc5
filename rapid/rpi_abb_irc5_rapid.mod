MODULE rpi_abb_irc5_rapid

    PERS jointtarget JointTrajectory_0{10};
    PERS jointtarget JointTrajectory_1{10};
    PERS jointtarget JointTrajectory_2{10};
    PERS jointtarget JointTrajectory_3{10};
    PERS jointtarget JointTrajectory_4{10};
    PERS jointtarget JointTrajectory_5{10};
    PERS jointtarget JointTrajectory_6{10};
    PERS jointtarget JointTrajectory_7{10};
    PERS jointtarget JointTrajectory_8{10};
    PERS jointtarget JointTrajectory_9{10};    
    PERS num JointTrajectoryCount:=0;
    PERS num JointTrajectoryTime{100};
    PERS num CurrentJointTrajectoryCount:=0;    
    
    VAR intnum joint_trajectory_count_intnum:=-1;
    VAR intnum joint_trajectory_time_intnum:=-1;
    VAR intnum joint_trajectory_0_intnum:=-1;
    VAR intnum joint_trajectory_1_intnum:=-1;
    VAR intnum joint_trajectory_2_intnum:=-1;
    VAR intnum joint_trajectory_3_intnum:=-1;
    VAR intnum joint_trajectory_4_intnum:=-1;
    VAR intnum joint_trajectory_5_intnum:=-1;
    VAR intnum joint_trajectory_6_intnum:=-1;
    VAR intnum joint_trajectory_7_intnum:=-1;
    VAR intnum joint_trajectory_8_intnum:=-1;
    VAR intnum joint_trajectory_9_intnum:=-1;    
            
    VAR egmident egmID1;
    VAR egmstate egmSt1;
    
    CONST egm_minmax egm_minmax_joint1:=[-0.5,0.5];
                
    PROC main()
        VAR num j;
        VAR num k;
        VAR jointtarget target;
        VAR zonedata zone;
        VAR rmqslot rmq;        
        VAR jointtarget joints;
                
        CurrentJointTrajectoryCount:=0;
        ConnectInterrupts;
        
        
        ! "Move" a tiny amount to start EGM
        joints:= CJointT();
        !joints.robax.rax_6 := joints.robax.rax_6 + .0001;
        MoveAbsj joints, v100, fine, tool0;
        
        StartEGM;
        
        IF JointTrajectoryCount = -1000 THEN
                            
            CurrentJointTrajectoryCount:=-1000;
            
            EGMActJoint egmID1 \Tool:=tool0 \WObj:=wobj0, \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1
            \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1 \LpFilter:=100 \Samplerate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=1000;
                      
            EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=2000000 \RampInTime:=0.05 \PosCorrGain:=1;
            
            CurrentJointTrajectoryCount:=0;
            
            ExitCycle;            
        ENDIF
        
        IF JointTrajectoryCount <= 0 THEN
            
            WaitUntil FALSE;
            
            ExitCycle;
        ENDIF
        
        FOR i FROM 0 TO JointTrajectoryCount - 1 DO
            CurrentJointTrajectoryCount:=i+1;
            j:=i DIV 10;
            k:=(i MOD 10) + 1;
            TEST j
                CASE 0:
                    target:=JointTrajectory_0{k};
                CASE 1:
                    target:=JointTrajectory_1{k};
                CASE 2:
                    target:=JointTrajectory_2{k};
                CASE 3:
                    target:=JointTrajectory_3{k};
                CASE 4:
                    target:=JointTrajectory_4{k};
                CASE 5:
                    target:=JointTrajectory_5{k};
                CASE 6:
                    target:=JointTrajectory_6{k};
                CASE 7:
                    target:=JointTrajectory_7{k};
                CASE 8:
                    target:=JointTrajectory_8{k};
                CASE 9:
                    target:=JointTrajectory_9{k};
                DEFAULT:
                    JointTrajectoryCount:=0;
            ENDTEST
            
            IF i < JointTrajectoryCount-2 THEN
                zone:= z100;
            ELSE
                zone:= fine;
            ENDIF
            
            MoveAbsJ target, v1000, \T:=JointTrajectoryTime{i+1}, zone, tool0;
            
        ENDFOR
        
        CurrentJointTrajectoryCount:=0;
        JointTrajectoryCount:=0;
            
        
    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "EGM UDP Command Timeout, Restarting!";
            WaitTime 1;
            ExitCycle;
        ELSE            
            RAISE;
        ENDIF
    ENDPROC
    
    LOCAL PROC ConnectInterrupts()
        
        IDelete joint_trajectory_count_intnum;
        IDelete joint_trajectory_time_intnum;
        IDelete joint_trajectory_0_intnum;
        IDelete joint_trajectory_1_intnum;
        IDelete joint_trajectory_2_intnum;
        IDelete joint_trajectory_3_intnum;
        IDelete joint_trajectory_4_intnum;
        IDelete joint_trajectory_5_intnum;
        IDelete joint_trajectory_6_intnum;
        IDelete joint_trajectory_7_intnum;
        IDelete joint_trajectory_8_intnum;
        IDelete joint_trajectory_9_intnum;
          
        CONNECT joint_trajectory_count_intnum WITH JointTrajectoryCountChanged;
        CONNECT joint_trajectory_time_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_0_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_1_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_2_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_3_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_4_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_5_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_6_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_7_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_8_intnum WITH JointTrajectoryChanged;
        CONNECT joint_trajectory_9_intnum WITH JointTrajectoryChanged;
                       
        IPers JointTrajectoryCount, joint_trajectory_count_intnum;
        IPers JointTrajectoryTime, joint_trajectory_time_intnum;
        IPers JointTrajectory_0, joint_trajectory_0_intnum;
        IPers JointTrajectory_1, joint_trajectory_1_intnum;
        IPers JointTrajectory_2, joint_trajectory_2_intnum;
        IPers JointTrajectory_3, joint_trajectory_3_intnum;
        IPers JointTrajectory_4, joint_trajectory_4_intnum;
        IPers JointTrajectory_5, joint_trajectory_5_intnum;
        IPers JointTrajectory_6, joint_trajectory_6_intnum;
        IPers JointTrajectory_7, joint_trajectory_7_intnum;
        IPers JointTrajectory_8, joint_trajectory_8_intnum;
        IPers JointTrajectory_9, joint_trajectory_9_intnum;
               
    ENDPROC
    
    PROC ResetProgram()
        CurrentJointTrajectoryCount:=0;
        JointTrajectoryCount:=0;        
        ExitCycle;
    ENDPROC
    
    PROC StartEGM()
        
        !This call to EGMReset seems to be problematic.
        !It is shown in all documentation. Is it really necessary?
        !EGMReset egmID1;
        
        EGMGetId egmID1;
        egmSt1 := EGMGetState(egmID1);        
        
        IF egmSt1 <= EGM_STATE_CONNECTED THEN            
            EGMSetupUC ROB_1, egmID1, "conf1", "UCdevice:" \Joint \CommTimeout:=100;
        ENDIF
        
        EGMStreamStart egmID1;
    ENDPROC
    
    TRAP JointTrajectoryCountChanged
        CurrentJointTrajectoryCount:=0;
        ExitCycle;
    ENDTRAP
    
    TRAP JointTrajectoryChanged
        IF JointTrajectoryCount > 0 THEN
            JointTrajectoryCount:=0;
            ExitCycle;
        ENDIF
    ENDTRAP        
        
ENDMODULE

  
        
