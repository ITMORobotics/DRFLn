#include <doosan_control.h>

#include <iostream>
#include <string>
#include <unistd.h>

#include <algorithm>  // std::copy
#include <cassert>
#include <cstring>
#include <cmath>

#include <DRFLEx.h>
#include <DRFC.h>
#include <DRFL.h>

int         rate    = 100;
int         standby = 5000;
std::string name    = "dsr01"; // May bo dont needed
std::string host    = "192.168.88.7";
int         port    = 12345;
std::string model   = "a0509";
std::string mode    = "real";

DRAFramework::CDRFLEx Drfl;

bool g_bHasControlAuthority = false;
bool g_bTpInitailizingComplted = false;
bool g_bHommingCompleted = false;

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

int g_nAnalogOutputModeCh1;
int g_nAnalogOutputModeCh2;

#define STABLE_BAND_JNT     0.05
#define DSR_CTL_PUB_RATE    100  //[hz] 10ms <----- 퍼블리싱 주기, but OnMonitoringDataCB() 은 100ms 마다 불려짐을 유의!


const char* GetRobotStateString(int nState)
{
    switch(nState)
    {
    case STATE_INITIALIZING:    return "(0) INITIALIZING";
    case STATE_STANDBY:         return "(1) STANDBY";
    case STATE_MOVING:          return "(2) MOVING";
    case STATE_SAFE_OFF:        return "(3) SAFE_OFF";
    case STATE_TEACHING:        return "(4) TEACHING";
    case STATE_SAFE_STOP:       return "(5) SAFE_STOP";
    case STATE_EMERGENCY_STOP:  return "(6) EMERGENCY_STOP";
    case STATE_HOMMING:         return "(7) HOMMING";
    case STATE_RECOVERY:        return "(8) RECOVERY";
    case STATE_SAFE_STOP2:      return "(9) SAFE_STOP2";
    case STATE_SAFE_OFF2:       return "(10) SAFE_OFF2";
    case STATE_RESERVED1:       return "(11) RESERVED1";
    case STATE_RESERVED2:       return "(12) RESERVED2";
    case STATE_RESERVED3:       return "(13) RESERVED3";
    case STATE_RESERVED4:       return "(14) RESERVED4";
    case STATE_NOT_READY:       return "(15) NOT_READY";

    default:                  return "UNKNOWN";
    }
    return "UNKNOWN";
}



DoosanRobot::DoosanRobot()
{
    g_nAnalogOutputModeCh1 = -1;
    g_nAnalogOutputModeCh2 = -1;
}
DoosanRobot::~DoosanRobot()
{
    Drfl.close_connection();
}


void DoosanRobot::read()
{
    // joints.pos, vel, eff should be update
    LPROBOT_POSE pose = Drfl.GetCurrentPose();
    for(int i = 0; i < NUM_JOINT; i++){
        std::cout << "[DRHWInterface::read] " << i << "-pos: " << pose->_fPosition[i] << std::endl;
        joints[i].pos = deg2rad(pose->_fPosition[i]);
    }
}
void DoosanRobot::write()
{
        static int count = 0;
        // joints.cmd is updated
        std::array<float, NUM_JOINT> tmp;
        for(int i = 0; i < NUM_JOINT; i++){
            std::cout << "[write]::write " <<
                i << "-pos: " << joints[i].pos << " " <<
                i << "-vel: " << joints[i].vel << " " <<
                i << "-cmd: " << joints[i].cmd << std::endl;

            tmp[i] = joints[i].cmd;
        }
        if( !bCommand_ ) return;
}
bool DoosanRobot::init()
{
    Drfl.set_on_tp_initializing_completed(OnTpInitializingCompletedCB);
    Drfl.set_on_homming_completed(OnHommingCompletedCB);
    Drfl.set_on_program_stopped(OnProgramStoppedCB);
    Drfl.set_on_monitoring_modbus(OnMonitoringModbusCB);
    Drfl.set_on_monitoring_data(OnMonitoringDataCB);           // Callback function in M2.4 and earlier
    Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);       // Callback function in M2.4 and earlier
    Drfl.set_on_monitoring_state(OnMonitoringStateCB);
    Drfl.set_on_monitoring_access_control(OnMonitoringAccessControlCB);
    Drfl.set_on_log_alarm(OnLogAlarm);
    Drfl.set_on_tp_popup(OnTpPopupCB);
    Drfl.set_on_tp_log(OnTpLogCB);
    Drfl.set_on_tp_get_user_input(OnTpGetUserInputCB);

    if(Drfl.open_connection(host, port))
    {
        //--- connect Emulator ? ------------------------------
        if(host == "127.0.0.1") m_bIsEmulatorMode = true;
        else                    m_bIsEmulatorMode = false;

        //--- Get version -------------------------------------
        SYSTEM_VERSION tSysVerion = {'\0', };
        assert(Drfl.get_system_version(&tSysVerion));

        //--- Get DRCF version & convert to integer  ----------
        m_nVersionDRCF = 0;
        int k = 0;
        for(int i=std::strlen(tSysVerion._szController); i > 0; i--)
            if(tSysVerion._szController[i] >= '0' && tSysVerion._szController[i] <= '9')
                m_nVersionDRCF += (tSysVerion._szController[i]-'0')*std::pow(10.0,k++);
        if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000;

        std::cout << "_______________________________________________" << std::endl;
        if(m_bIsEmulatorMode) std::cout << "    Emulator Mode" << std::endl;
        else                  std::cout << "    Real Robot Mode" << std::endl;
        std::cout << "    DRCF version = " << tSysVerion._szController << std::endl;
        std::cout << "    DRFL version = " << Drfl.get_library_version() << std::endl;
        std::cout << "    m_nVersionDRCF = " << m_nVersionDRCF << std::endl;  //ex> M2.40 = 120400, M2.50 = 120500
        std::cout << "_______________________________________________" << std::endl;

        if(m_nVersionDRCF >= 120500)    //M2.5 or later
        {
            Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);      //Callback function in version 2.5 and higher
            Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);  //Callback function in version 2.5 and higher
            Drfl.setup_monitoring_version(1);                        //Enabling extended monitoring functions
        }

        //--- Check Robot State : STATE_STANDBY ---
        int delay = 5000; // Default
        while ((Drfl.get_robot_state() != STATE_STANDBY)) {
            usleep(delay);
        }

        //--- Set Robot mode : MANUAL or AUTO
        assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
        // assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));

        //--- Set Robot mode : virual or real
        ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
        if(mode == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
        assert(Drfl.set_robot_system(eTargetSystem));

        // to compare with joints[].cmd
        for(int i = 0; i < NUM_JOINT; ++i){
            std::cout << "[init]::read " << i << "-pos: " << joints[i].cmd << std::endl;
            cmd_[i] = joints[i].cmd;
        }
        return true;
    }
    return false;
}


void DoosanRobot::OnHommingCompletedCB()
{
    g_bHommingCompleted = TRUE;
    // Only work within 50msec
    std::cout << "[callback OnHommingCompletedCB] homming completed" << std::endl;

    g_stDrState.bHommingCompleted = TRUE;
}
void DoosanRobot::OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause)
{
    std::cout << "[callback OnProgramStoppedCB] Program Stop: " << (int)iStopCause << std::endl;
    g_stDrState.bDrlStopped = TRUE;
}
void DoosanRobot::OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    //ROS_INFO("DRHWInterface::OnMonitoringDataCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){
            // joint
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }
}
void DoosanRobot::OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    //ROS_INFO("DRHWInterface::OnMonitoringDataExCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){
            // joint
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < NUM_JOINT; i++){
        g_stDrState.fActualW2B[i] = pData->_tCtrl._tWorld._fActualW2B[i];
        g_stDrState.fCurrentVelW[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fWorldETT[i] = pData->_tCtrl._tWorld._fActualETT[i];
        g_stDrState.fTargetPosW[i] = pData->_tCtrl._tWorld._fTargetPos[i];
        g_stDrState.fTargetVelW[i] = pData->_tCtrl._tWorld._fTargetVel[i];
        g_stDrState.fCurrentVelU[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fUserETT[i] = pData->_tCtrl._tUser._fActualETT[i];
        g_stDrState.fTargetPosU[i] = pData->_tCtrl._tUser._fTargetPos[i];
        g_stDrState.fTargetVelU[i] = pData->_tCtrl._tUser._fTargetVel[i];
    }

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            g_stDrState.fCurrentPosW[i][j] = pData->_tCtrl._tWorld._fActualPos[i][j];
            g_stDrState.fCurrentPosU[i][j] = pData->_tCtrl._tUser._fActualPos[i][j];
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            g_stDrState.fRotationMatrixWorld[j][i] = pData->_tCtrl._tWorld._fRotationMatrix[j][i];
            g_stDrState.fRotationMatrixUser[j][i] = pData->_tCtrl._tUser._fRotationMatrix[j][i];
        }
    }

    g_stDrState.iActualUCN = pData->_tCtrl._tUser._iActualUCN;
    g_stDrState.iParent    = pData->_tCtrl._tUser._iParent;
    //-------------------------------------------------------------------------
}
void DoosanRobot::OnTpInitializingCompletedCB()
{
    // request control authority after TP initialized
    std::cout << "[callback OnTpInitializingCompletedCB] tp initializing completed" << std::endl;
    g_bTpInitailizingComplted = TRUE;
    //Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
    Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    g_stDrState.bTpInitialized = TRUE;
}


void DoosanRobot::OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
{
    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];
        }
    }
}
void DoosanRobot::OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO)
{
    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < 3; i++)
        g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

    for (int i = 0; i < 2; i++){
        g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
        g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
        g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
        g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
        g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
        g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
        g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
        g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
    }
    //-------------------------------------------------------------------------
}
void DoosanRobot::OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
{
    g_stDrState.nRegCount = pModbus->_iRegCount;
    for (int i = 0; i < pModbus->_iRegCount; i++){
        std::cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< std::endl;
        g_stDrState.strModbusSymbol[i] = pModbus->_tRegister[i]._szSymbol;
        g_stDrState.nModbusValue[i]    = pModbus->_tRegister[i]._iValue;
    }
}
void DoosanRobot::OnMonitoringStateCB(const ROBOT_STATE eState)
{
        //This function is called when the state changes.
        //ROS_INFO("DRHWInterface::OnMonitoringStateCB");
        // Only work within 50msec
        std::cout << "On Monitor State" << std::endl;
        switch((unsigned char)eState)
        {
#if 0 // TP initializing logic, Don't use in API level. (If you want to operate without TP, use this logic)
        case eSTATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
            break;
        case eSTATE_INITIALIZING:
            // add initalizing logic
            if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
            break;
#endif
        case STATE_EMERGENCY_STOP:
            // popup
            break;
        case STATE_STANDBY:
        case STATE_MOVING:
        case STATE_TEACHING:
            break;
        case STATE_SAFE_STOP:
            if (g_bHasControlAuthority) {
                Drfl.set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT);
                Drfl.set_robot_control(CONTROL_RESET_SAFET_STOP);
            }
            break;
        case STATE_SAFE_OFF:
            if (g_bHasControlAuthority){
                Drfl.set_robot_control(CONTROL_SERVO_ON);
				Drfl.set_robot_mode(ROBOT_MODE_MANUAL);   //Idle Servo Off 후 servo on 하는 상황 발생 시 set_robot_mode 명령을 전송해 manual 로 전환. add 2020/04/28
            }
            break;
        case STATE_SAFE_STOP2:
            if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_STOP);
            break;
        case STATE_SAFE_OFF2:
            if (g_bHasControlAuthority) {
                Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
            }
            break;
        case STATE_RECOVERY:
            Drfl.set_robot_control(CONTROL_RESET_RECOVERY);
            break;
        default:
            break;
        }

        std::cout << "[callback OnMonitoringStateCB] current state: " << GetRobotStateString((int)eState) << std::endl;
        g_stDrState.nRobotState = (int)eState;
        strncpy(g_stDrState.strRobotState, GetRobotStateString((int)eState), MAX_SYMBOL_SIZE);
}
void DoosanRobot::OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl)
{
    // Only work within 50msec

    std::cout << "[callback OnMonitoringAccessControlCB] eAccCtrl: " << eAccCtrl << std::endl;
    switch(eAccCtrl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
        //Drfl.TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        std::cout  << "access control granted" << std::endl;
        g_bHasControlAuthority = TRUE;
        OnMonitoringStateCB(Drfl.GetRobotState());
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
        std::cout << "Access control deny !!!!!!!!!!!!!!!" << std::endl;
        break;
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_bTpInitailizingComplted) {
            Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
            //Drfl.TransitControlAuth(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        break;
    default:
        break;
    }
    g_stDrState.nAccessControl = (int)eAccCtrl;
}
void DoosanRobot::OnLogAlarm(LPLOG_ALARM pLogAlarm)
{
        //This function is called when an error occurs.

        switch(pLogAlarm->_iLevel)
        {
        case LOG_LEVEL_SYSINFO:
            std::cout << "[callback OnLogAlarm]" << std::endl;
            std::cout << " level : " << (unsigned int)pLogAlarm->_iLevel << std::endl;
            std::cout << " group : " << (unsigned int)pLogAlarm->_iGroup << std::endl;
            std::cout << " index : " << pLogAlarm->_iIndex << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[0] << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[1] << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[2] << std::endl;
            break;
        case LOG_LEVEL_SYSWARN:
            std::cout << "[callback OnLogAlarm]" << std::endl;
            std::cout << " level : " << (unsigned int)pLogAlarm->_iLevel << std::endl;
            std::cout << " group : " << (unsigned int)pLogAlarm->_iGroup << std::endl;
            std::cout << " index : " << pLogAlarm->_iIndex << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[0] << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[1] << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[2] << std::endl;
            break;
        case LOG_LEVEL_SYSERROR:
        default:
            std::cout << "[callback OnLogAlarm]" << std::endl;
            std::cout << " level : " << (unsigned int)pLogAlarm->_iLevel << std::endl;
            std::cout << " group : " << (unsigned int)pLogAlarm->_iGroup << std::endl;
            std::cout << " index : " << pLogAlarm->_iIndex << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[0] << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[1] << std::endl;
            std::cout << " param : " << pLogAlarm->_szParam[2] << std::endl;
            break;
        }

        g_stDrError.nLevel=(unsigned int)pLogAlarm->_iLevel;
        g_stDrError.nGroup=(unsigned int)pLogAlarm->_iGroup;
        g_stDrError.nCode=pLogAlarm->_iIndex;
        strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE);
        strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE);
        strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE);
}


void DoosanRobot::OnTpPopupCB(LPMESSAGE_POPUP tPopup)
{
    std::cout << "OnTpPopup" << std::endl;
}
void DoosanRobot::OnTpLogCB(const char* strLog)
{
    std::cout << "OnTpLog" << std::endl;
    std::cout << strLog << std::endl;
}
void DoosanRobot::onTpProgressCB(LPMESSAGE_PROGRESS tProgress)
{
    std::cout << "OnTpProgress" << std::endl;
}
void DoosanRobot::OnTpGetUserInputCB(LPMESSAGE_INPUT tInput)
{
    std::cout << "OnTpGetUserInput" << std::endl;
}


void DoosanRobot::OnRTMonitoringDataCB(LPRT_OUTPUT_DATA_LIST tData)
{}


bool DoosanRobot::set_robot_mode(ROBOT_MODE mode)
{
    return Drfl.set_robot_mode(mode);
}
bool DoosanRobot::get_current_posx(std::array<float, NUM_TASK> & pos, COORDINATE_SYSTEM eCoodType)
{
    LPROBOT_TASK_POSE robot_pos = Drfl.get_current_posx(eCoodType);
    for(int i = 0; i < NUM_TASK; ++i) {
        pos[i] = robot_pos->_fTargetPos[i];
    }
    return true;
}
bool DoosanRobot::set_robot_speed_mode(SPEED_MODE eSpeedMode)
{
    return Drfl.set_robot_speed_mode(eSpeedMode);
}


/// Movements

bool DoosanRobot::move_home()
{
    if (!Drfl.move_home(MOVE_HOME_MECHANIC)) {
        return false;
    }

    // wait untill homming
    while (!g_stDrState.bHommingCompleted)
    {
        usleep(100000);
    }

    return true;
}
bool DoosanRobot::move_wait()
{
    return Drfl.mwait();
}
bool DoosanRobot::movej(std::array<float, NUM_JOINT> & jointTarget, float maxSpeed, float maxAcceleration)
{
    return Drfl.movej(jointTarget.data(), maxSpeed, maxAcceleration);
}
bool DoosanRobot::movel(std::array<float, NUM_TASK> & posTarget, std::array<float, 2> maxSpeed, std::array<float, 2> maxAcceleration, MOVE_REFERENCE eMoveReference)
{
    return Drfl.movel(posTarget.data(), maxSpeed.data(), maxAcceleration.data(), 0.0, MOVE_MODE_ABSOLUTE, eMoveReference);
}
bool servol(std::array<float, NUM_TASK> & posTarget, std::array<float, 2> maxSpeed, std::array<float, 2> maxAcceleration)
{
    return Drfl.servol(posTarget.data(), maxSpeed.data(), maxAcceleration.data(), 0.0);
}