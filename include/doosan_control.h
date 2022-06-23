#pragma once

#include <iostream>
#include <string>
#include <unistd.h>
#include <array>

#include <DRFLEx.h>
#include <DRFC.h>
#include <DRFL.h>


#ifndef PI
#define PI 3.14159265359
#endif
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

//_____ defines for Dooan Robot Controller _______________
#define POINT_COUNT         6

// solution space
#define DR_SOL_MIN          0
#define DR_SOL_MAX          7

// posb seg_type
#define DR_LINE             0
#define DR_CIRCLE           1

// move reference
#define DR_BASE             0
#define DR_TOOL             1
#define DR_WORLD            2
#define DR_TC_USER_MIN      101
#define DR_TC_USER_MAX      200

// move mod
#define DR_MV_MOD_ABS       0
#define DR_MV_MOD_REL       1

// move reaction
#define DR_MV_RA_NONE       0
#define DR_MV_RA_DUPLICATE  0
#define DR_MV_RA_OVERRIDE   1

// move command type
#define DR_MV_COMMAND_NORM  0

// movesx velocity
#define DR_MVS_VEL_NONE     0
#define DR_MVS_VEL_CONST    1

// motion state
#define DR_STATE_IDLE       0
#define DR_STATE_INIT       1
#define DR_STATE_BUSY       2
#define DR_STATE_BLEND      3
#define DR_STATE_ACC        4
#define DR_STATE_CRZ        5
#define DR_STATE_DEC        6

// axis
#define DR_AXIS_X           0
#define DR_AXIS_Y           1
#define DR_AXIS_Z           2
#define DR_AXIS_A          10
#define DR_AXIS_B          11
#define DR_AXIS_C          12

// collision sensitivity
#define DR_COLSENS_DEFAULT 20
#define DR_COLSENS_MIN      1   
#define DR_COLSENS_MAX    300

// speed
#define DR_OP_SPEED_MIN     1
#define DR_OP_SPEED_MAX   100

// stop
#define DR_QSTOP_STO        0
#define DR_QSTOP            1
#define DR_SSTOP            2
#define DR_HOLD             3

#define DR_STOP_FIRST       DR_QSTOP_STO
#define DR_STOP_LAST        DR_HOLD

// condition
#define DR_COND_NONE        -10000

// digital I/O
#define DR_DIO_MIN_INDEX    1
#define DR_DIO_MAX_INDEX    16  

// tool digital I/O
#define DR_TDIO_MIN_INDEX   1
#define DR_TDIO_MAX_INDEX   6

// I/O value
#define ON                  1
#define OFF                 0

// Analog I/O mode
#define DR_ANALOG_CURRENT   0
#define DR_ANALOG_VOLTAGE   1

// modbus type
#define DR_MODBUS_DIG_INPUT     0
#define DR_MODBUS_DIG_OUTPUT    1
#define DR_MODBUS_REG_INPUT     2
#define DR_MODBUS_REG_OUTPUT    3
#define DR_DISCRETE_INPUT       0
#define DR_COIL                 1
#define DR_INPUT_REGISTER       2
#define DR_HOLDING_REGISTER     3

#define DR_MODBUS_ACCESS_MAX    32
#define DR_MAX_MODBUS_NAME_SIZE 32

// tp_popup pm_type
#define DR_PM_MESSAGE           0
#define DR_PM_WARNING           1
#define DR_PM_ALARM             2

// tp_get_user_input type
#define DR_VAR_INT              0
#define DR_VAR_FLOAT            1
#define DR_VAR_STR              2
#define DR_VAR_BOOL             3   

// len
#define DR_VELJ_DT_LEN          6
#define DR_ACCJ_DT_LEN          6

#define DR_VELX_DT_LEN          2
#define DR_ACCX_DT_LEN          2

#define DR_ANGLE_DT_LEN         2
#define DR_COG_DT_LEN           3
#define DR_WEIGHT_DT_LEN        3
#define DR_VECTOR_DT_LEN        3
#define DR_ST_DT_LEN            6
#define DR_FD_DT_LEN            6
#define DR_DIR_DT_LEN           6
#define DR_INERTIA_DT_LEN       6
#define DR_VECTOR_U1_LEN        3
#define DR_VECTOR_V1_LEN        3

#define DR_AVOID                0
#define DR_TASK_STOP            1

#define DR_FIFO                 0
#define DR_LIFO                 1

#define DR_FC_MOD_ABS           0
#define DR_FC_MOD_REL           1

#define DR_GLOBAL_VAR_TYPE_BOOL         0
#define DR_GLOBAL_VAR_TYPE_INT          1
#define DR_GLOBAL_VAR_TYPE_FLOAT        2
#define DR_GLOBAL_VAR_TYPE_STR          3
#define DR_GLOBAL_VAR_TYPE_POSJ         4
#define DR_GLOBAL_VAR_TYPE_POSX         5
#define DR_GLOBAL_VAR_TYPE_UNKNOWN      6

#define DR_IE_SLAVE_GPR_ADDR_START      0
#define DR_IE_SLAVE_GPR_ADDR_END       23
#define DR_IE_SLAVE_GPR_ADDR_END_BIT   63

#define DR_DPOS                         0
#define DR_DVEL                         1

#define DR_HOME_TARGET_MECHANIC         0
#define DR_HOME_TARGET_USER             1

#define DR_MV_ORI_TEACH                 0    
#define DR_MV_ORI_FIXED                 1    
#define DR_MV_ORI_RADIAL                2    

#define DR_MV_APP_NONE                  0
#define DR_MV_APP_WELD                  1
//________________________________________________________

typedef struct {
    int	    nLevel;         // INFO =1, WARN =2, ERROR =3 
    int	    nGroup;         // SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
    int	    nCode;          // error code 
    char    strMsg1[MAX_STRING_SIZE];   // error msg 1
    char    strMsg2[MAX_STRING_SIZE];   // error msg 2
    char    strMsg3[MAX_STRING_SIZE];   // error msg 3
} DR_ERROR, *LPDR_ERROR;

typedef struct {
    int     nRobotState;
    char    strRobotState[MAX_SYMBOL_SIZE];
    float   fCurrentPosj[NUM_JOINT];
    float   fCurrentPosx[NUM_TASK];
    float   fCurrentToolPosx[NUM_TASK];

    int     nActualMode;
    int     nActualSpace;
    
    float   fJointAbs[NUM_JOINT];
    float   fJointErr[NUM_JOINT];
    float   fTargetPosj[NUM_JOINT];
    float   fTargetVelj[NUM_JOINT];
    float   fCurrentVelj[NUM_JOINT];

    float   fTaskErr[NUM_TASK];
    float   fTargetPosx[NUM_TASK];
    float   fTargetVelx[NUM_TASK];
    float   fCurrentVelx[NUM_TASK];
    int     nSolutionSpace;
    float   fRotationMatrix[3][3];

    float   fDynamicTor[NUM_JOINT];
    float   fActualJTS[NUM_JOINT];
    float   fActualEJT[NUM_JOINT];
    float   fActualETT[NUM_JOINT];

    double  dSyncTime;
    int     nActualBK[NUM_JOINT];
    int     nActualBT[NUM_BUTTON];
    float   fActualMC[NUM_JOINT];
    float   fActualMT[NUM_JOINT];
    bool    bCtrlBoxDigitalOutput[16];
    bool    bCtrlBoxDigitalInput[16];
    bool    bFlangeDigitalOutput[6];
    bool    bFlangeDigitalInput[6];

    int     nRegCount;
    string  strModbusSymbol[100];
    int     nModbusValue[100];
  
    int     nAccessControl;
    bool    bHommingCompleted;
    bool    bTpInitialized;
    bool    bMasteringNeed;
    bool    bDrlStopped;
    bool    bDisconnected;

    //--- The following variables have been updated since version M2.50 or higher. ---
	//ROBOT_MONITORING_WORLD
	float   fActualW2B[6];
	float   fCurrentPosW[2][6];
	float   fCurrentVelW[6];
	float   fWorldETT[6];
	float   fTargetPosW[6];
	float   fTargetVelW[6];
	float   fRotationMatrixWorld[3][3];

	//ROBOT_MONITORING_USER
	int     iActualUCN;
	int     iParent;
	float   fCurrentPosU[2][6];
	float   fCurrentVelU[6];
	float   fUserETT[6];
	float   fTargetPosU[6];
	float   fTargetVelU[6];
	float   fRotationMatrixUser[3][3];

    //READ_CTRLIO_INPUT_EX
	float   fActualAI[6];
	bool    bActualSW[3];
	bool    bActualSI[2];
	int     iActualAT[2];

	//READ_CTRLIO_OUTPUT_EX
	float   fTargetAO[2];
	int     iTargetAT[2];

	//READ_ENCODER_INPUT
	bool    bActualES[2];
	int     iActualED[2];
	bool    bActualER[2];
    //---------------------------------------------------------------------------------

} DR_STATE, *LPDR_STATE;


class DoosanRobot
{
public:

    DoosanRobot();
    ~DoosanRobot();

    void read();
    void write();
    bool init();

    static void OnHommingCompletedCB();
    static void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause);
    static void OnMonitoringDataCB(const LPMONITORING_DATA pData);
    static void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
    static void OnTpInitializingCompletedCB();

    static void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
    static void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO);
    static void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
    static void OnMonitoringStateCB(const ROBOT_STATE eState);
    static void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
    static void OnLogAlarm(LPLOG_ALARM pLogAlarm);

    static void OnTpPopupCB(LPMESSAGE_POPUP tPopup);
    static void OnTpLogCB(const char* strLog);
    static void onTpProgressCB(LPMESSAGE_PROGRESS tProgress);
    static void OnTpGetUserInputCB(LPMESSAGE_INPUT tInput);

    static void OnRTMonitoringDataCB(LPRT_OUTPUT_DATA_LIST tData);

    /// System
    bool set_robot_mode(ROBOT_MODE mode);
    bool set_robot_speed_mode(SPEED_MODE eSpeedMode);
    bool get_current_posx(std::array<float, NUM_TASK> & pos, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);

    /// Motion control
    bool move_home();

    bool move_wait();

    // motion control: joint move
    // float64[6] pos               # target joint angle list [degree] 
    // float64    vel               # set velocity: [deg/sec]
    // float64    acc               # set acceleration: [deg/sec2]
    // float64    time #= 0.0       # Time [sec] 
    // float64    radius #=0.0      # Radius under blending mode [mm] 
    // int8       mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
    // int8       blendType #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
    // int8       syncType #=0      # SYNC = 0, ASYNC = 1
    bool movej(std::array<float, NUM_JOINT> & jointTarget, float maxSpeed, float maxAcceleration);

    // motion control: linear move
    // float64[6] pos               # target  
    // float64[2] vel               # set velocity: [mm/sec], [deg/sec]
    // float64[2] acc               # set acceleration: [mm/sec2], [deg/sec2]
    // float64    time #= 0.0       # Time [sec] 
    // float64    radius #=0.0      # Radius under blending mode [mm] 
    // int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
    //                              # <DR_WORLD is only available in M2.40 or later> 
    // int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
    // int8       blendType #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
    // int8       syncType #=0      # SYNC = 0, ASYNC = 1
    bool movel(std::array<float, NUM_TASK> & posTarget, std::array<float, 2> maxSpeed, std::array<float, 2> maxAcceleration, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE);

    // float64[6] pos               # position  
    // float64[2] vel               # velocity
    // float64[2] acc               # acceleration
    // float64    time              # f
    bool servol(std::array<float, NUM_TASK> & posTarget, std::array<float, 2> maxSpeed, std::array<float, 2> maxAcceleration);

private:

    int     m_nVersionDRCF;
    bool    m_bIsEmulatorMode;

    std::array<float, NUM_JOINT> cmd_;
    bool bCommand_;
    struct Joint{
        double cmd;
        double pos;
        double vel;
        double eff;
        Joint(): cmd(0), pos(0), vel(0), eff(0) {}
    } joints[NUM_JOINT];

    DR_STATE m_stDrState;
    DR_ERROR m_stDrError;

};