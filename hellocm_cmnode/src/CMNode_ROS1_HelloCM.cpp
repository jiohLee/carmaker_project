/*!
 ******************************************************************************
 **  CarMaker - Version 7.1.2
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 *
 *
 * ToDo:
 * - C++!!!
 * - ROS naming/way/namespaces
 * - parameter: CarMaker read, ROS set by service?
 *   -> ROS parameter mechanism seems better solution!
 * - node/topic/... destruction to allow dynamic load/unload
 *   when TestRun starts instead of initialization at CarMaker startup
 * - New Param_Get() function to read parameters from Infofile
 * - ...
 *
 */


/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */
#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"

// #include "Vehicle.h"
// #include "DrivMan.h"
// #include "Vehicle/Sensor_Object.h"

#include "VehicleControl.h"
#include "DrivMan.h"
#include "Vehicle_Control_UDP.h"
extern tUDP_PC UDP_PC;
extern tUDP_Input UDP_Input;

#include "Car/Car.h" //dk
#include "Vehicle.h"
#include "Vehicle/Sensor_LidarRSI.h"
#include "Vehicle/Sensor_GNav.h"
#include "Vehicle/Sensor_TSign.h" //Speed_Limit
#include "Vehicle/Sensor_Inertial.h" //dk
#include "TrafficLight.h" //dk
#include "Environment.h" //dk
#include "Traffic.h" // chp
#include <Car/Vehicle_Car.h> // chp

//#include <Vehicle/Sensor_Road.h>//9WEEK

#define VLP_16_NUMBER_OF_POINTS 28800
#define VLP_16_POINTS_OF_PACKET 2880
#define VLP_16_NUMBER_OF_PACKET 10

#define OS1_64_NUMBER_OF_POINTS 65536
#define OS1_64_POINTS_OF_PACKET 2900
#define OS1_64_POINTS_OF_LAST_PACKET 1736
#define OS1_64_NUMBER_OF_PACKET 23
#define Max_Intensity 65000

int Lidar_CycleCount;

struct G_VC                     // chp
{                               // chp
    double SteeringWheel;       // chp
    double Ax;                  // chp
    double GearNo;              // chp
    double VC_SwitchOn;         // chp
    int Light_Indicator; // Right: -1, Off: 0, Left: 1  // chp
    int Light_Hazard;           // chp
};                              // chp
G_VC g_vc;                      // chp


/* ROS */
#include "cmrosutils/CMRosUtils.h"    /* Node independent templates, ...*/
#include "cmrosutils/CMRosIF_Utils.h" /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemote.h"      /* Basic service for CarMaker remote from ROS */

#include "sensor_msgs/PointCloud.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/Path.h"
#include <angles/angles.h>
#include <hellocm_msgs/GPS_Out.h>
#include <hellocm_msgs/Speed_Limit.h> //Speed_Limit
#include <sensor_msgs/Imu.h> //dk
#include <hellocm_msgs/TrafficLight.h> //dk
#include <geometry_msgs/Vector3Stamped.h> //dk

#include <hellocm_msgs/GPS_Out.h>           // chp
#include <hellocm_msgs/CarMaker_Out.h>      // chp

// e he ra di ya~
#include "Vehicle/Sensor_Camera.h"
#include "hellocm_msgs/CameraSensor.h"
#include "hellocm_msgs/Ext2CM.h"
#include "hellocm_msgs/CM2Ext.h"

/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 1
#  include "hellocm/ROS1_HelloCM.h"  /* External ROS Node. Topic name, ... */
#else
#  include <hellocm_msgs/Ext2CM.h>
#  include <hellocm_msgs/CM2Ext.h>
#  include <hellocm_msgs/Init.h>
#endif

/*! String and numerical version of this Node
 *  - String:    e.g. <Major>.<Minor>.<Patch>
 *  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
 */
#define CMNODE_VERSION "0.8.0"
#define CMNODE_NUMVER  800


/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  warning "Debug options are enabled!"
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Not beautiful but consistent with external ROS Node
 * where ROS_INFO is used (implicit newline)*/
# define LOG(frmt, ...)  Log(frmt "\n", ##__VA_ARGS__)


/* General switches for CarMaker ROS Node */
typedef enum tCMNode_Mode {
    CMNode_Mode_Disabled  = 0,  /*!< Node is disabled. e.g. don't publish. */
    CMNode_Mode_Default   = 1,  /*!< Node is enabled, spinOnce is used  */
    CMNode_Mode_Threaded  = 2   /*!< Node is enabled, spin in parallel thread
                                     - Messages are received all the time
                                     - Data is updated at defined position, e.g. *_In()
                                     - Currently not implemented! */
} tCMNode_Mode;



/* Managing synchronization between CarMaker Node and other ROS nodes */
typedef enum tCMNode_SyncMode {
    CMNode_SyncMode_Disabled  = 0, /*!< No synchronization on CarMaker side */
    CMNode_SyncMode_Tpc       = 1  /*!< Buffer messages or Spin until external Topics are received */
} tCMNode_SyncMode;



/* Global struct for this Node */
static struct {
    unsigned long  CycleNoRel;  /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct {
        double         Duration;      /*!< Time spent for synchronization task */
        int            nCycles;       /*!< Number of cycles in synchronization loop */
        int            CyclePrepDone; /*!< Last cycle when preparation was done */
        int            CycleJobDone;  /*!< Last cycle when job was done */
        double         SynthDelay;    /*!< Synthetic delay in seconds provided to external node to check sync */
    } Sync; /*!< Synchronization related information */

    struct {
        int CycleNo;      /*!< Cycle number of external ROS Node (only for information) */

        /* For debugging */
        int            CycleLastOut;   /*!< Cycle number when Topic was published */
        int            CycleLastIn;    /*!< Cycle number when Topic from external ROS Node was received */
        int            CycleLastFlush; /*!< Cycle number when data from external ROS Node was provided to model */
    } Model; /*!< Model related information. ROS side! */

    struct {
        struct {
            tRosIF_TpcSub<hellocm_msgs::Ext2CM> Ext2CM; /* For this example also used for Synchronization */
        } Sub; /*!< Topics to be subscribed */

        struct {
            tRosIF_TpcPub<hellocm_msgs::CM2Ext> CM2Ext;

//            tRosIF_TpcPub<sensor_msgs::PointCloud> Lidar_VLP;
            tRosIF_TpcPub<sensor_msgs::PointCloud> Lidar_OS1;

            //            tRosIF_TpcPub<hellocm_msgs::GPS_Out> GPS_Out;
            //            tRosIF_TpcPub<hellocm_msgs::Speed_Limit> Speed_Limit; //Speed_Limit
            //            tRosIF_TpcPub<sensor_msgs::Imu> Imu; //dk
            //            tRosIF_TpcPub<hellocm_msgs::TrafficLight> TrafficLight; //dk
            //            tRosIF_TpcPub<geometry_msgs::Vector3Stamped> Velocity; //dk

            /*!< CarMaker can be working as ROS Time Server providing simulation time
             *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
        /*!< Initialization/Preparation of external ROS Node e.g. when simulation starts */
        tRosIF_Srv<hellocm_msgs::Init>    Init;
        tRosIF_Srv<cmrosutils::CMRemote>  CMRemote; // Trial
    } Services; /*!< ROS Services used by this Node (client and server)*/

    struct {
        boost::shared_ptr<tf2_ros::TransformBroadcaster> br;
        boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> st_br;

//        geometry_msgs::TransformStamped Lidar_VLP;
        geometry_msgs::TransformStamped Lidar_OS1;

    } TF;


    struct {
        int               QueuePub;     /*!< Queue size for Publishers */
        int               QueueSub;     /*!< Queue size for Subscribers */
        int               nCyclesClock; /*!< Number of cycles publishing /clock topic.
                                             CycleTime should be multiple of this value */
        tCMNode_Mode      Mode;
        tCMNode_SyncMode  SyncMode;
        double            SyncTimeMax;  /* Maximum Synchronization time */

        tRosIF_Cfg        Ros;
    } Cfg; /*!< General configuration for this Node */

    struct {
        double *BeamTable;
        int rows = 0;
    } LidarRSI_VLP, LidarRSI_OS1;

} CMNode;



/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
static void
cmnode_HelloCM_CB_TpcIn (const hellocm_msgs::Ext2CM::ConstPtr &msg)
{
    /* Process message only if receive is expected */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
        return;
    
    int rv;
    auto in = &CMNode.Topics.Sub.Ext2CM;

    /* Update receive buffer
     * - No lock for spinOnce necessary?
     */
    in->Msg.header  = msg->header;
    in->Msg.time    = msg->time;
    in->Msg.cycleno = msg->cycleno;

    /* Stopping simulation is only necessary when synchronization is activated */
    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success) {
        LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;

    g_vc.SteeringWheel = msg->SteeringWheel;    // chp
    g_vc.Ax = msg->Ax;    // chp
    g_vc.GearNo = msg->GearNo;    // chp
    g_vc.VC_SwitchOn = msg->VC_SwitchOn;    // chp
    g_vc.Light_Indicator = msg->Light_Indicator; // Right: -1, Off: 0, Left: +1    // chp
    g_vc.Light_Hazard = msg->Light_Hazard;    // chp

    LOG("%s (CMSimTime=%.3fs): External Node is in cycle %lu, Time=%.3fs, Stamp=%.3fs, SeqID=%d",
        ros::this_node::getName().c_str(), SimCore.Time,
        in->Msg.cycleno, msg->time.toSec(), in->Msg.header.stamp.toSec(), in->Msg.header.seq);

}



/*!
 * Description:
 * - Exemplary Service Callback for CarMaker Remote using ROS
 * - e.g. via rqt Service Caller or terminal "rosservice call ..."
 *
 *
 */
static bool
cmnode_HelloCM_CB_SrvCMRemote(cmrosutils::CMRemote::Request &req,
                              cmrosutils::CMRemote::Response &resp)
{

    int rv = -2;
    char sbuf[512];

    LOG("%s: Service '%s' was triggered with type='%s', msg='%s', data='%s'",
        ros::this_node::getName().c_str(),
        CMNode.Services.CMRemote.Srv.getService().c_str(),
        req.type.c_str(), req.msg.c_str(), req.data.c_str());


    /* Commands to CarMaker GUI
     * - Tcl commands!
     * - More information see "ProgrammersGuide"
     */
    if (strcasecmp("guicmd", req.type.c_str()) == 0) {
        /* Default: Evaluate command sent with message */
        if (strcasecmp("eval", req.msg.c_str()) == 0) {
            /* e.g. data = 'LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim' */
            rv = GuiCmd_Eval(req.data.c_str());
        } else {
            if (strcasecmp("start", req.msg.c_str()) == 0) {
                if (req.data.length() == 0)
                    rv = GuiCmd_Eval("LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim");
                else {
                    sprintf(sbuf, "%s; StartSim", req.data.c_str());
                    rv = GuiCmd_Eval(sbuf);
                }
            }
            if (strcasecmp("stop", req.msg.c_str()) == 0)
                rv = GuiCmd_Eval("StopSim");
        }


        /* Commands directly to CarMaker Executable
     * - Warning:
     *   - Information normally provided by CarMaker GUI might be missing
     */
    } else if (strcasecmp("cmd", req.type.c_str()) == 0) {
        if (strcasecmp("start", req.msg.c_str()) == 0) {
            if (req.data.length() == 0) {
                /* Most strings are normally provided by CarMaker GUI */
                SimStart(NULL, ros::this_node::getName().c_str(),
                         "CMRosIF/AdaptiveCruiseControl", NULL, NULL);
            } else {
                /* Most strings are normally provided by CarMaker GUI */
                SimStart(NULL, ros::this_node::getName().c_str(),
                         req.data.c_str(), NULL, NULL);
            }
        }
        if (strcasecmp("stop", req.msg.c_str()) == 0)
            SimStop2(0);
        rv = 0;
    }

    resp.res = rv;
    return true;
}



/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Get versions from shared library
 * - Set the returned Version to 0 if there is no dependency!
 * - Compatibility check should be done by calling procedure
 *   as early as possible(e.g. before CMRosIF_CMNode_Init()
 *
 * Arguments:
 * - CMRosIFVer = CMRosIF shared library version (User defined)
 *                - Initially filled with version of CMRosIF management library
 * - CMNumVer   = CarMaker version used for shared library at compile time (normally CM_NUMVER)
 *                - Initially filled with version of CMRosIF management library
 * - RosVersion = ROS version used for shared library at compile time (normally ROS_VERSION)
 *                - Initially filled with version requested by CMRosIF management library (0 if no request)
 *
 */
int
CMRosIF_CMNode_GetVersion (unsigned long *CMRosIFCMNodeNumVer,
                           unsigned long *CMNumVer,
                           unsigned long *RosNumVer)
{

    *CMRosIFCMNodeNumVer = CMNODE_NUMVER;
    *CMNumVer            = CM_NUMVER;
    *RosNumVer           = ROS_VERSION;

    return 0;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Basic Initialization
 * - e.g. create ROS Node, subscriptions, ...
 * - Return values
 *   - "rv <  0" = Error at initialization, CarMaker executable will stop
 *   - "rv >= 0" = Everything OK, CarMaker executable will continue
 *
 * Arguments:
 * - Argc/Argv  = Arguments normally provided to ROS executable are not be provided
 *                to CM executable directly, but can be set in Infofile for CMRosIF
 *                with key "Node.Args" in "Data/Config/CMRosIFParameters"
 *
 * - CMNodeName = Default CarMaker Node name
 *                - Can be parameterized in Infofile for CMRosIF
 *                - Final node name might be different (argument remapping, ...)
 *
 * - Inf        = Handle to CarMaker Infofile with parameters for this interface
 *                - Please note that pointer may change, e.g. before TestRun begins
 *
 * ToDo:
 * - Possible to create/initialize node/... before each TestRun start instead of CM startup?
 * - New Param_Get() function to read parameters from Infofile
 */
int
CMRosIF_CMNode_Init (int Argc, char **Argv, char *CMNodeName, struct tInfos *Inf)
{

    int rv;
    bool rvb                = false;
    char sbuf[512]          = "";
    char keybuf[256]        = "";
    ros::NodeHandlePtr node = NULL;
    ros::V_string names;


    LOG("Initialize CarMaker ROS Node");
    LOG("  -> Node Version = %05d", CMNODE_NUMVER);
    LOG("  -> ROS Version  = %05d", ROS_VERSION);
    LOG("  -> CM Version   = %05d", CM_NUMVER);

    /* ROS initialization. Name of Node might be different after remapping! */
    if (ros::isInitialized() == false) {
        /* "Remapping arguments" functionality (launchfiles, ...)? */
        ros::init(Argc, Argv, CMNodeName);
    } else {
        //node.reset(); ToDo!
    }

    if (ros::master::check() == false) {
        LogErrF(EC_Init, "Can't contact ROS Master!\n Start roscore or run launch file e.g. via Extras->CMRosIF\n");
        ros::shutdown();
        return -1;
    }

    /* Node specific */
    CMNode.Cfg.Ros.Node = ros::NodeHandlePtr(boost::make_shared<ros::NodeHandle>());
    node                = CMNode.Cfg.Ros.Node;

    /* Publish specific */
    CMNode.Cfg.QueuePub  = iGetIntOpt(Inf, "Node.QueuePub", 1000); /* ToDo: Influence of queue length relevant? */

    /* Prepare the node to provide simulation time. CarMaker will be /clock server */
    strcpy(sbuf, "/use_sim_time");

    if ((rv = node->hasParam(sbuf)) == true) {
        node->getParam(sbuf, rvb);
        LOG("  -> Has param '%s' with value '%d'", sbuf, rvb);
    }

    /* Additional switch to provide simulation Time */
    strcpy(keybuf, "Node.UseSimTime");

    if ((rv = iGetIntOpt(Inf, keybuf, 1)) > 0) {
        /* Parameter must be set before other nodes start
     * - set parameter outside to be independent from execution order?
     */
        LOG("  -> Provide simulation time!");
        node->setParam("/use_sim_time", true); /* enable parameter if not already done */

        CMNode.Cfg.nCyclesClock  = iGetIntOpt(Inf, "Node.nCyclesClock", 1000);

        strcpy(sbuf, "/clock");
        LOG("    -> Publish '%s' every %dms", sbuf, CMNode.Cfg.nCyclesClock);
        CMNode.Topics.Pub.Clock.Pub  = node->advertise<rosgraph_msgs::Clock>(sbuf, CMNode.Cfg.QueuePub);


        /* ToDo: Necessary/Possible to ensure /clock is zeroed? */
        CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
        CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    } else {
        LOG("  -> Don't provide simulation time!");
        CMNode.Cfg.nCyclesClock  = 0;
    }

    strcpy(sbuf, hellocm::tpc_in_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.CM2Ext.Pub         = node->advertise<hellocm_msgs::CM2Ext>(sbuf, CMNode.Cfg.QueuePub);
    CMNode.Topics.Pub.CM2Ext.Job         = CMCRJob_Create("CM2Ext");
    CMNode.Topics.Pub.CM2Ext.CycleTime   = 10; // chp
    CMNode.Topics.Pub.CM2Ext.CycleOffset = 0;

//    strcpy(sbuf, "/pointcloud/vlp");
//    LOG("  -> Publish '%s'", sbuf);
//    CMNode.Topics.Pub.Lidar_VLP.Pub         = node->advertise<sensor_msgs::PointCloud>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
//    CMNode.Topics.Pub.Lidar_VLP.Job         = CMCRJob_Create("pointcloud/vlp");

    strcpy(sbuf, "/pointcloud/os1");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.Lidar_OS1.Pub         = node->advertise<sensor_msgs::PointCloud>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.Lidar_OS1.Job         = CMCRJob_Create("pointcloud/os1");

    //    strcpy(sbuf, "/gps_out");
    //    LOG("  -> Publish '%s'", sbuf);
    //    CMNode.Topics.Pub.GPS_Out.Pub           = node->advertise<hellocm_msgs::GPS_Out>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    //    CMNode.Topics.Pub.GPS_Out.Job           = CMCRJob_Create("gps_out");

    //    strcpy(sbuf, "/speed_limit");
    //    LOG("  -> Publish '%s'", sbuf);
    //    CMNode.Topics.Pub.Speed_Limit.Pub       = node->advertise<hellocm_msgs::Speed_Limit>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    //    CMNode.Topics.Pub.Speed_Limit.Job       = CMCRJob_Create("speed_limit");

    //    strcpy(sbuf, "/gpsimu_driver/imu_data"); //dk
    //    LOG("  -> Publish '%s'", sbuf); //dk
    //    CMNode.Topics.Pub.Imu.Pub                  = node->advertise<sensor_msgs::Imu>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub)); //dk
    //    CMNode.Topics.Pub.Imu.Job                  = CMCRJob_Create("gpsimu_driver/imu_data"); //dk

    //    strcpy(sbuf, "/Traffic_Light"); //dk
    //    LOG("  -> Publish '%s'", sbuf); //dk
    //    CMNode.Topics.Pub.TrafficLight.Pub  = node->advertise<hellocm_msgs::TrafficLight>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub)); //dk
    //    CMNode.Topics.Pub.TrafficLight.Job  = CMCRJob_Create("Traffic_Light"); //dk

    //    strcpy(sbuf, "/Velocity"); //dk
    //    LOG("  -> Publish '%s'", sbuf); //dk
    //    CMNode.Topics.Pub.Velocity.Pub  = node->advertise<geometry_msgs::Vector3Stamped>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub)); //dk
    //    CMNode.Topics.Pub.Velocity.Job  = CMCRJob_Create("Velocity"); //dk

    CMNode.TF.br = boost::make_shared<tf2_ros::TransformBroadcaster>();
    CMNode.TF.st_br = boost::make_shared<tf2_ros::StaticTransformBroadcaster>();


    /* Subscribe specific */
    CMNode.Cfg.QueueSub  = iGetIntOpt(Inf, "Node.QueueSub", 1); /* ToDo: Effect of queue length for subscriber? */


    strcpy(sbuf, hellocm::tpc_out_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Subscribe '%s'", sbuf);
    CMNode.Topics.Sub.Ext2CM.Sub         = node->subscribe(sbuf, CMNode.Cfg.QueueSub, cmnode_HelloCM_CB_TpcIn);
    CMNode.Topics.Sub.Ext2CM.Job         = CMCRJob_Create("Ext2CM_for_Sync");

    /* In this example cycle time might be updated with value of external ROS Node
     * - See CMRosIF_CMNode_TestRun_Start_atBegin() */
    CMNode.Topics.Sub.Ext2CM.CycleTime   = 15000;

    /* Services */
    strcpy(sbuf, hellocm::srv_init_name.c_str());
    LOG("  -> Service Client '%s'", sbuf);
    CMNode.Services.Init.Clnt = node->serviceClient<hellocm_msgs::Init>(sbuf);


    strcpy(sbuf, "CMRemote");
    LOG("  -> Create Service '%s'", sbuf);
    CMNode.Services.CMRemote.Srv = node->advertiseService(
                sbuf, cmnode_HelloCM_CB_SrvCMRemote);


    /* Print general information after everything is done */
    LOG("Initialization of ROS Node finished!");
    LOG("  -> Node Name = '%s'", ros::this_node::getName().c_str());
    LOG("  -> Namespace = '%s'", ros::this_node::getNamespace().c_str());


    /* Advertised Topics */
    ros::this_node::getAdvertisedTopics(names);
    LOG("  -> Advertised Topics (%lu)", names.size());

    auto it = names.begin();
    for (; it != names.end(); ++it)
        LOG("    -> %s", (*it).c_str());


    /* Subscribed Topics */
    names.clear();
    ros::this_node::getSubscribedTopics(names);
    LOG("  -> Subscribed Topics (%lu)", names.size());
    it = names.begin();
    for (; it != names.end(); ++it)
        LOG("    -> %s",  (*it).c_str());

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Add user specific Quantities for data storage
 *   and visualization to DataDictionary
 * - Called once at program start
 * - no realtime conditions
 *
 */
void
CMRosIF_CMNode_DeclQuants (void)
{

    tDDefault *df = DDefaultCreate("CMRosIF.");

    DDefULong   (df, "CycleNoRel",         "ms", &CMNode.CycleNoRel,               DVA_None);
    DDefInt     (df, "Sync.Cycles",        "-",  &CMNode.Sync.nCycles,             DVA_None);
    DDefDouble  (df, "Sync.Time",          "s",  &CMNode.Sync.Duration,            DVA_None);
    DDefInt     (df, "Sync.CyclePrepDone", "-",  &CMNode.Sync.CyclePrepDone,       DVA_None);
    DDefInt     (df, "Sync.CycleJobDone" , "-",  &CMNode.Sync.CycleJobDone,        DVA_None);
    DDefDouble4 (df, "Sync.SynthDelay",     "s", &CMNode.Sync.SynthDelay,          DVA_IO_In);

    DDefUChar   (df, "Cfg.Mode",           "-",  (unsigned char*)&CMNode.Cfg.Mode, DVA_None);
    DDefInt     (df, "Cfg.nCyclesClock",   "ms", &CMNode.Cfg.nCyclesClock,         DVA_None);
    DDefChar    (df, "Cfg.SyncMode",       "-",  (char*)&CMNode.Cfg.SyncMode,      DVA_None);
    DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",  &CMNode.Cfg.SyncTimeMax,          DVA_IO_In);

    DDefInt     (df, "Mdl.CycleNo",        "-",  &CMNode.Model.CycleNo,            DVA_None);
    DDefInt     (df, "Mdl.CycleLastOut",   "ms", &CMNode.Model.CycleLastOut,       DVA_None);
    DDefInt     (df, "Mdl.CycleLastIn",    "ms", &CMNode.Model.CycleLastIn,        DVA_None);
    DDefInt     (df, "Mdl.CycleLastFlush", "ms", &CMNode.Model.CycleLastFlush,     DVA_None);

    DDefaultDelete(df);
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when starting a new TestRun
 * - In separate Thread (no realtime conditions)
 * - After standard Infofiles are read in
 * - Return values
 *   - "rv <  0" = Error, TestRun start will be aborted
 *   - "rv >= 0" = Everything OK
 *
 * Arguments:
 * - Inf = CarMaker Infofile for CMRosIF with content after TestRun start
 *         - Please note that the Infofile provided at initialization might have been updated!
 *
 * ToDo:
 * - New Param_Get() function to read parameters from Infofile
 *
 */
int
CMRosIF_CMNode_TestRun_Start_atBegin (struct tInfos *Inf)
{
    srand(time(NULL));

    Lidar_CycleCount = 0;

    tInfos *Inf_Sensor = nullptr;
    tErrorMsg *err = nullptr;

    //Create infofile handle
    Inf_Sensor = InfoNew();
    iRead2(&err, Inf_Sensor, "Data/Sensor/LidarRSI_VLP_16", "");
    CMNode.LidarRSI_VLP.BeamTable = (double*)malloc(VLP_16_NUMBER_OF_POINTS * 6 * sizeof(double));
    //Read infofile parameters
    iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI_VLP.BeamTable, VLP_16_NUMBER_OF_POINTS * 6, 6, &CMNode.LidarRSI_VLP.rows);
    InfoDelete(Inf_Sensor);

    //Create infofile handle
    Inf_Sensor = InfoNew();
    iRead2(&err, Inf_Sensor, "Data/Sensor/LidarRSI_OS1_64", "");
    CMNode.LidarRSI_OS1.BeamTable = (double*)malloc(OS1_64_NUMBER_OF_POINTS * 6 * sizeof(double));
    //Read infofile parameters
    iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI_OS1.BeamTable, OS1_64_NUMBER_OF_POINTS * 6, 6, &CMNode.LidarRSI_OS1.rows);
    InfoDelete(Inf_Sensor);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Read sensor info from Vehicle InfoFile
    tInfos *Inf_Vehicle = nullptr;
    Inf_Vehicle = InfoNew();
    
    const char *FName;
    FName = InfoGetFilename(SimCore.Vhcl.Inf);

    Log("FName = %s\n", FName);

    int VehicleInfo_Err = iRead2(&err, Inf_Vehicle, FName, "SensorReadCode");

    if (VehicleInfo_Err == 0) {

        tf2::Quaternion q;
        double* position;
        double* rotation;
        double tmp[3] = {0, 0, 0};

//        //Lidar Sensor
//        position = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.0.pos", tmp, 3, 1);
//        rotation = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.0.rot", tmp, 3, 1);
//        q.setRPY(rotation[0], rotation[1], rotation[2]);
//        CMNode.TF.Lidar_VLP.transform.rotation = tf2::toMsg(q);
//        CMNode.TF.Lidar_VLP.transform.translation = tf2::toMsg(tf2::Vector3(position[0], position[1], position[2]));
//        CMNode.TF.Lidar_VLP.child_frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.name", "LIR00");
//        CMNode.TF.Lidar_VLP.header.frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.Mounting", "Fr1A");
//        CMNode.Topics.Pub.Lidar_VLP.CycleTime = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.0.CycleTime", 100);
//        CMNode.Topics.Pub.Lidar_VLP.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.0.nCycleOffset", 0);

        //Lidar Sensor
        position = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.0.pos", tmp, 3, 1);
        rotation = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.0.rot", tmp, 3, 1);
        q.setRPY(rotation[0], rotation[1], rotation[2]);
        CMNode.TF.Lidar_OS1.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_OS1.transform.translation = tf2::toMsg(tf2::Vector3(position[0], position[1], position[2]));
        CMNode.TF.Lidar_OS1.child_frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.name", "LIR00");
        CMNode.TF.Lidar_OS1.header.frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.Mounting", "Fr1A");
        CMNode.Topics.Pub.Lidar_OS1.CycleTime = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.0.CycleTime", 100);
        CMNode.Topics.Pub.Lidar_OS1.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.0.nCycleOffset", 0);

        //        //GNSS
        //        CMNode.Topics.Pub.GPS_Out.CycleTime     = (int)(1000 / iGetIntOpt(Inf_Vehicle, "Sensor.GNav.UpdRate", 10));
        //        CMNode.Topics.Pub.GPS_Out.CycleOffset   = iGetIntOpt(Inf_Vehicle, "Sensor.GNav.nCycleOffset", 0);

        //        //Speed_Limit
        //        CMNode.Topics.Pub.Speed_Limit.CycleTime     = (int)(1000 / iGetIntOpt(Inf_Vehicle, "Sensor.TSign.0.UpdRate", 10));
        //        CMNode.Topics.Pub.Speed_Limit.CycleOffset   = iGetIntOpt(Inf_Vehicle, "Sensor.TSign.0.nCycleOffset", 0);

        //        //IMU //dk
        //        CMNode.Topics.Pub.Imu.CycleTime = (int)(1000 / iGetIntOpt(Inf_Vehicle, "Sensor.Inertial.IMU.UpdRate", 1000)); //dk
        //        CMNode.Topics.Pub.Imu.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.Inertial.IMU.nCycleOffset", 0); //dk

        //        //Traffic Light //dk
        //        CMNode.Topics.Pub.TrafficLight.CycleTime = (int)(1000 / iGetIntOpt(Inf_Vehicle, "TrfLight.UpdRate", 10)); //dk
        //        CMNode.Topics.Pub.TrafficLight.CycleOffset = iGetIntOpt(Inf_Vehicle, "TrfLight.nCycleOffset", 0); //dk

        //        //Velocity //dk
        //        CMNode.Topics.Pub.Velocity.CycleTime = (int)(1000 / iGetIntOpt(Inf_Vehicle, "Sensor.Inertial.IMU.UpdRate", 1000)); //dk
        //        CMNode.Topics.Pub.Velocity.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.Inertial.IMU.nCycleOffset", 0); //dk
    }

    int errCode = InfoDelete(Inf_Vehicle);
    Log("errCode = %d\n", errCode);
    

    /* Node can be disabled via Infofile */
    tCMNode_Mode     *pmode     = &CMNode.Cfg.Mode;
    tCMNode_SyncMode *psyncmode = &CMNode.Cfg.SyncMode;

    if (Inf != NULL) {
        *pmode     =     (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode",      CMNode_Mode_Disabled);
        *psyncmode = (tCMNode_SyncMode)iGetIntOpt(Inf, "Node.Sync.Mode", CMNode_SyncMode_Disabled);
    }

    if (SimCore.CycleNo == 0 || Inf == NULL || *pmode == CMNode_Mode_Disabled) {
        *pmode = CMNode_Mode_Disabled;
        LOG("CarMaker ROS Node is disabled!");
        return 0;
    }

    char sbuf[512];
    char key[256];
    char *str        = NULL;
    int rv           = 0;
    bool rvb                = false;

    int cycletime           = 0;
    int *pcycletime         = NULL;
    int cycleoff            = 0;
    tCMCRJob *job           = NULL;
    auto srv         = &CMNode.Services.Init;

    LOG("CarMaker ROS Node is enabled! Mode=%d, SyncMode=%d", *pmode, *psyncmode);
    LOG("  -> Node Name = %s", ros::this_node::getName().c_str());


    /* Update synchronization */
    if (*psyncmode != CMNode_SyncMode_Disabled && *psyncmode != CMNode_SyncMode_Tpc) {
        LogErrF(EC_Sim, "CMNode: Invalid synchronization mode '%d'!",*psyncmode);
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    CMNode.Cfg.SyncTimeMax = iGetDblOpt(Inf, "Node.Sync.TimeMax", 1.0);


    /* Reset for next cycle */
    CMNode.CycleNoRel           =  0;
    CMNode.Sync.Duration        =  0.0;
    CMNode.Sync.nCycles         = -1;
    CMNode.Sync.CycleJobDone    = -1;
    CMNode.Sync.CyclePrepDone   = -1;
    CMNode.Model.CycleNo        = -1;
    CMNode.Model.CycleLastIn    = -1;
    CMNode.Model.CycleLastOut   = -1;
    CMNode.Model.CycleLastFlush = -1;


    /* Allow an update of the clock only if it was enabled before! */
    if (CMNode.Cfg.nCyclesClock > 0) {
        if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
            CMNode.Cfg.nCyclesClock = rv;
    }

    /* Necessary to ensure /clock is zeroed here?
     * ToDo: Create function? */
    if (CMNode.Cfg.nCyclesClock > 0) {
        LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
        CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
        CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }


    /* Prepare external node for next simulation */
    if (!srv->Clnt.exists()) {
        // ToDo: possible to get update if external ROS Node name changes?
        LogErrF(EC_Sim, "ROS Service is not ready! Please start external ROS Node providing Service '%s'!",
                srv->Clnt.getService().c_str());
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    LOG("  -> Send Service Request");

    /* ToDo: Async?*/
    if (!srv->Clnt.call(srv->Msg)) {
        LogErrF(EC_Sim, "ROS Service error!");
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    /* Update cycle time with information of external node */
#if 1
    /* Variant 1:
     * - Receiving parameters via ROS Parameter Server
     * - Parameter may be set externally e.g. by other node or arguments to command
     * - ROS parameters are more flexible than ROS services!
     */
    strcpy(sbuf, hellocm::prm_cycletime_name.c_str());
    if ((rv = CMNode.Cfg.Ros.Node->hasParam(sbuf)) == true)
        CMNode.Cfg.Ros.Node->getParam(sbuf, rv);
#else
    /* Variant 2:
     * - Receiving parameters from external Node via Service
     * - Services might be too "static"
     * - Not recommended!
     */
    rv = srv->Msg.response.cycletime;
#endif

    pcycletime = &CMNode.Topics.Sub.Ext2CM.CycleTime;

    if (*pcycletime != rv) {
        LOG("  -> Cycle time of external node changed from %dms to %dms", *pcycletime, rv);
        *pcycletime = rv;
    }


    /* Plausibility check for Cycle Time */
    if (CMNode.Cfg.nCyclesClock > 0 && (*pcycletime < CMNode.Cfg.nCyclesClock
                                        || *pcycletime%CMNode.Cfg.nCyclesClock != 0)) {

        LogErrF(EC_Sim, "Ext. ROS Node has invalid cycle time! Expected multiple of %dms but got %dms",
                CMNode.Cfg.nCyclesClock, *pcycletime);

        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    
    
    /* Prepare Jobs for publish and subscribe
     * - Special use case:
     *   - Topic in and Topic out use same cycle time with relative shift!
     */

    /* Start to publish when simulation starts */
    job       = CMNode.Topics.Pub.CM2Ext.Job;
    cycletime = CMNode.Topics.Pub.CM2Ext.CycleTime;
    cycleoff  = CMNode.Topics.Pub.CM2Ext.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
    
//    job       = CMNode.Topics.Pub.Lidar_VLP.Job;
//    cycletime = CMNode.Topics.Pub.Lidar_VLP.CycleTime;
//    cycleoff  = CMNode.Topics.Pub.Lidar_VLP.CycleOffset;

//    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

    job       = CMNode.Topics.Pub.Lidar_OS1.Job;
    cycletime = CMNode.Topics.Pub.Lidar_OS1.CycleTime;
    cycleoff  = CMNode.Topics.Pub.Lidar_OS1.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

    //    job       = CMNode.Topics.Pub.GPS_Out.Job;
    //    cycletime = CMNode.Topics.Pub.GPS_Out.CycleTime;
    //    cycleoff  = CMNode.Topics.Pub.GPS_Out.CycleOffset;

    //    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

    //    job       = CMNode.Topics.Pub.Speed_Limit.Job;
    //    cycletime = CMNode.Topics.Pub.Speed_Limit.CycleTime;
    //    cycleoff  = CMNode.Topics.Pub.Speed_Limit.CycleOffset;

    //    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

    //    job       = CMNode.Topics.Pub.Imu.Job; //dk
    //    cycletime = CMNode.Topics.Pub.Imu.CycleTime; //dk
    //    cycleoff  = CMNode.Topics.Pub.Imu.CycleOffset; //dk

    //    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext); //dk

    //    job       = CMNode.Topics.Pub.TrafficLight.Job; //dk
    //    cycletime = CMNode.Topics.Pub.TrafficLight.CycleTime; //dk
    //    cycleoff  = CMNode.Topics.Pub.TrafficLight.CycleOffset; //dk

    //    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext); //dk

    //    job       = CMNode.Topics.Pub.Velocity.Job; //dk
    //    cycletime = CMNode.Topics.Pub.Velocity.CycleTime; //dk
    //    cycleoff  = CMNode.Topics.Pub.Velocity.CycleOffset; //dk

    //    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext); //dk

    /* Synchronization with external node
     * - external node provides cycle time (see service above)
     * - other parameterization methods (e.g. ROS parameter, ...) are possible!
     * - Expect sync Topic are delayed (communication time, ...)
     * - This example shows sync via ROS Timer in external node
     *   - Therefore "/clock" topic needs to be published by CarMaker!
     *   - Other mechanism, e.g. data triggered on external node side
     *     via publishing Topic directly inside subscription callback is also possible!
     * - time=0.0 can't be detected by external node, therefore
     *   first receive needs to start after expected cycle time
     *   of external ROS node
     */

    job       = CMNode.Topics.Sub.Ext2CM.Job;
    cycletime = CMNode.Topics.Sub.Ext2CM.CycleTime;
    cycleoff  = CMNode.Topics.Sub.Ext2CM.CycleOffset = 0; /* No offset allowed if ROS Timer is used for sync!*/


    /* Create the synchronization jobs */
    if (*psyncmode == CMNode_SyncMode_Tpc) {
        CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Ext);

        LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
            CMNode.Topics.Sub.Ext2CM.Sub.getTopic().c_str(), cycletime, cycleoff);

    } else
        CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Default);

    std::vector<geometry_msgs::TransformStamped> transforms;
//    transforms.push_back(CMNode.TF.Lidar_VLP);
    transforms.push_back(CMNode.TF.Lidar_OS1);
    CMNode.TF.st_br->sendTransform(transforms);



    LOG("External ROS Node is ready to simulate");

    return 1;
}



/*!
 * ToDo:
 * - Put everything to TestRun_Start_atBegin?
 *
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Repeating call for several CarMaker cycles until return value is 1
 * - May be called even previous return value was 1
 * - See "User.c:User_TestRun_RampUp()"
 *
 */
int
CMRosIF_CMNode_TestRun_RampUp (void)
{

    /* Return immediately if node is disabled */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
        return 1;

    /* Put your code here */
    //if (NotReady) return 0;


    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when TestRun ends (no realtime conditions)
 * - See "User.c:User_TestRun_End()"
 *
 */
int
CMRosIF_CMNode_TestRun_End (void)
{


    /* Put your code here */

    /* Disable after simulation has finished */
    CMNode.Cfg.Mode = CMNode_Mode_Disabled;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called at very beginning of CarMaker cycle
 * - Process all topics/services using different modes or synchronization modes
 * - See "User.c:User_In()"
 *
 * ToDo:
 * - Additional spin mechanism
 *   - e.g. for HIL
 *   - e.g. spinning in new thread, copying incoming data here, ...
 *
 */
int
CMRosIF_CMNode_In (void)
{

    int rv                   = 0;
    int rx_done              = 0;
    const char *job_name     = NULL;
    tCMCRJob *job            = NULL;
    ros::WallTime tStart     = ros::WallTime::now();
    ros::WallDuration tDelta = ros::WallDuration(0.0);
    CMNode.Sync.nCycles      = 0;
    CMNode.Sync.Duration     = 0.0;

    switch (CMNode.Cfg.Mode) {
    case CMNode_Mode_Disabled:
        /* Comment next line if no messages/services
         * shall be processed in disabled Node state
         */
        ros::spinOnce();
        break;

    case CMNode_Mode_Default:

        if (CMNode.Cfg.SyncMode != CMNode_SyncMode_Tpc) {
            /* Process messages in queue, but do not block */
            ros::spinOnce();

        } else {
            /* Synchronization based on expected Topics
         * - Blocking call (process publish and wait for answer)
         * - Stop simulation if maximum time is exceeded
         */
            do {
                ros::spinOnce();

                /* Only do anything if simulation is running */
                if (SimCore.State != SCState_Simulate) {
                    rx_done = 1;
                    break;
                }

                rx_done = 0;

                /* Check all jobs if preparation is done */
                job      = CMNode.Topics.Sub.Ext2CM.Job;

                if ((rv = CMCRJob_DoPrep(job, CMNode.CycleNoRel, 0, NULL, NULL)) < CMCRJob_RV_OK) {
                    LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(job), CMCRJob_RVStr(rv));
                    rx_done = 0;
                    break;
                }

                /* If job is not done, remember name and prevent loop to finish */
                job_name = (rv != CMCRJob_RV_DoSomething ? NULL : CMCRJob_GetName(job));
                rx_done  = rv == CMCRJob_RV_DoNothing ? 1 : 0;

                if (rx_done == 1)
                    break;

                /* Wait a little that data can arrive. WallTime, NOT ROS time!!!*/
                ros::WallDuration(0.0).sleep();
                tDelta = ros::WallTime::now() - tStart;
                CMNode.Sync.nCycles++;

            } while (ros::ok() && rx_done == 0 && tDelta.toSec() < CMNode.Cfg.SyncTimeMax);

            /* Final calculation to get duration including last cycle before receive */
            tDelta = ros::WallTime::now() - tStart;

            CMNode.Sync.Duration = tDelta.toSec();

            if (rx_done != 1 && CMNode.Cfg.SyncTimeMax > 0.0 && tDelta.toSec() >= CMNode.Cfg.SyncTimeMax)
                LogErrF(EC_Sim, "CMNode: Synchronization error! tDelta=%.3f, Last invalid Job='%s'\n", tDelta.toSec(), job_name);
        }

        break;

    case CMNode_Mode_Threaded:
        /* ToDo
         * - Spinning in parallel thread started before
         * - Lock variables!
         * - e.g. for HIL
         */
        break;

    default:
        /* Invalid!!! */;
    }

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after driving maneuver calculation
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_DrivManCalc()"
 */
int
CMRosIF_CMNode_DrivMan_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
            || SimCore.State != SCState_Simulate)
        return 0;

    /* Put your code here */

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after CMRosIF_CMNode_DrivManCalc
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_VehicleControl_Calc()"
 */
int
CMRosIF_CMNode_VehicleControl_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
            || SimCore.State != SCState_Simulate)
        return 0;

    /* Put your code here */
    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after vehicle model has been calculated
 * - See "User.c:User_Calc()"
 */
int
CMRosIF_CMNode_Calc (double dt)
{

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
            || SimCore.State != SCState_Simulate)
        return 0;

    /* Put your code here
     * - Update model parameters here?
     * - Do some calculation...
     */

    /* Update model with values from external node only in specific cycle?
     * - This data handling is optionl, but necessary for deterministic behaviour
     * - if synchronization is active, incoming data remains in msg buffer until correct cycle
     */
    int rv;
    auto sync = &CMNode.Topics.Sub.Ext2CM;

    if ((rv = CMCRJob_DoJob(sync->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
        /* Something to do in sync cycle? */
        //CMCRJob_Info(in->Job, CMNode.CycleNoRel, "CMNode: Do Something for Sync: ");

        /* Update model parameters here? */
        CMNode.Model.CycleNo = CMNode.Topics.Sub.Ext2CM.Msg.cycleno;


        /* Remember cycle for debugging */
        CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
        CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
    }

    /* Do some calculation... */

    if (SimCore.State == SCState_Simulate) {
//        //Lidar RSI --> PointCloud
//        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Lidar_VLP.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
//            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Lidar_VLP.CycleTime, CMNode.Topics.Pub.Lidar_VLP.CycleOffset, CMNode.CycleNoRel);
//            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Lidar_VLP.Job), CMCRJob_RVStr(rv));
//        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Lidar_VLP.CycleTime == 0) {
//            geometry_msgs::Point32 points;
//            sensor_msgs::ChannelFloat32 channels;
//            channels.name = "intensity";

//            //clearing vector data to avoid overflows
//            CMNode.Topics.Pub.Lidar_VLP.Msg.points.clear();
//            CMNode.Topics.Pub.Lidar_VLP.Msg.channels.clear();
//            channels.values.clear();

//            for (int i = 0; i < LidarRSI[0].nScanPoints; i++) {

//                const int beam_id = LidarRSI[0].ScanPoint[i].BeamID;
//                const double azimuth = angles::from_degrees(CMNode.LidarRSI_VLP.BeamTable[4*CMNode.LidarRSI_VLP.rows + beam_id]);
//                const double elevation = angles::from_degrees(CMNode.LidarRSI_VLP.BeamTable[5*CMNode.LidarRSI_VLP.rows + beam_id]);
//                const double ray_length = 0.5 * LidarRSI[0].ScanPoint[i].LengthOF; // length of flight is back and forth

//                //XYZ-coordinates of scan point
//                points.x = ray_length * cos(elevation) * cos(azimuth);
//                points.y = ray_length * cos(elevation) * sin(azimuth);
//                points.z = ray_length * sin(elevation);

//                CMNode.Topics.Pub.Lidar_VLP.Msg.points.push_back(points);
//                channels.values.push_back(LidarRSI[0].ScanPoint[i].Intensity);

//            }
//            CMNode.Topics.Pub.Lidar_VLP.Msg.channels.push_back(channels);
//            CMNode.Topics.Pub.Lidar_VLP.Msg.header.frame_id = CMNode.TF.Lidar_VLP.child_frame_id;
//            CMNode.Topics.Pub.Lidar_VLP.Msg.header.stamp = ros::Time(LidarRSI[0].ScanTime); //dk caption

//        }

        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Lidar_OS1.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Lidar_OS1.CycleTime, CMNode.Topics.Pub.Lidar_OS1.CycleOffset, CMNode.CycleNoRel);
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Lidar_OS1.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Lidar_OS1.CycleTime == 0) {
            geometry_msgs::Point32 points;
            sensor_msgs::ChannelFloat32 channels;
            channels.name = "intensity";

            //clearing vector data to avoid overflows
            CMNode.Topics.Pub.Lidar_OS1.Msg.points.clear();
            CMNode.Topics.Pub.Lidar_OS1.Msg.channels.clear();
            channels.values.clear();

            //Lidar Quantity processing
            for (int i = 0; i < LidarRSI[0].nScanPoints; i++) {

                const int beam_id = LidarRSI[0].ScanPoint[i].BeamID;
                const double azimuth = angles::from_degrees(CMNode.LidarRSI_OS1.BeamTable[4*CMNode.LidarRSI_OS1.rows + beam_id]);
                const double elevation = angles::from_degrees(CMNode.LidarRSI_OS1.BeamTable[5*CMNode.LidarRSI_OS1.rows + beam_id]);
                const double ray_length = 0.5 * LidarRSI[0].ScanPoint[i].LengthOF; // length of flight is back and forth

                //XYZ-coordinates of scan point
                points.x = ray_length * cos(elevation) * cos(azimuth);
                points.y = ray_length * cos(elevation) * sin(azimuth);
                points.z = ray_length * sin(elevation);

                CMNode.Topics.Pub.Lidar_OS1.Msg.points.push_back(points);
                channels.values.push_back(LidarRSI[0].ScanPoint[i].Intensity);

            }
            CMNode.Topics.Pub.Lidar_OS1.Msg.channels.push_back(channels);
            CMNode.Topics.Pub.Lidar_OS1.Msg.header.frame_id = CMNode.TF.Lidar_OS1.child_frame_id;
            CMNode.Topics.Pub.Lidar_OS1.Msg.header.stamp = ros::Time(LidarRSI[0].ScanTime); //dk caption

        }

        //        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.GPS_Out.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
        //            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.GPS_Out.CycleTime, CMNode.Topics.Pub.GPS_Out.CycleOffset, CMNode.CycleNoRel);
        //            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.GPS_Out.Job), CMCRJob_RVStr(rv));
        //        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.GPS_Out.CycleTime == 0) {
        //            CMNode.Topics.Pub.GPS_Out.Msg.cycleno      = CMNode.CycleNoRel;
        //            CMNode.Topics.Pub.GPS_Out.Msg.time         = ros::Time(SimCore.Time);
        //            CMNode.Topics.Pub.GPS_Out.Msg.synthdelay   = CMNode.Sync.SynthDelay;

        //            ros::WallTime wtime = ros::WallTime::now();
        //            CMNode.Topics.Pub.GPS_Out.Msg.header.stamp.sec  = wtime.sec;
        //            CMNode.Topics.Pub.GPS_Out.Msg.header.stamp.nsec = wtime.nsec;

        //            unsigned int noise_deg = 150;
        //            double pi = 3.1415926536897932384626433832795028841971;
        //            CMNode.Topics.Pub.GPS_Out.Msg.latitude          = GNavSensor.Receiver.UserPosLlhTsa[0] * 180 / pi + ((double)(rand() % (noise_deg * 2 + 1)) - noise_deg) / 10000000;
        //            CMNode.Topics.Pub.GPS_Out.Msg.longitude         = GNavSensor.Receiver.UserPosLlhTsa[1] * 180 / pi + ((double)(rand() % (noise_deg * 2 + 1)) - noise_deg) / 10000000;
        //            CMNode.Topics.Pub.GPS_Out.Msg.altitude          = GNavSensor.Receiver.UserPosLlhTsa[2];

        //        }

        //        //Speed_Limit --> hellocm_msgs/Speed_Limit
        //        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Speed_Limit.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
        //            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Speed_Limit.CycleTime, CMNode.Topics.Pub.Speed_Limit.CycleOffset, CMNode.CycleNoRel);
        //            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Speed_Limit.Job), CMCRJob_RVStr(rv));
        //        } else {
        //            CMNode.Topics.Pub.Speed_Limit.Msg.cycleno      = CMNode.CycleNoRel;
        //            CMNode.Topics.Pub.Speed_Limit.Msg.time         = ros::Time(SimCore.Time);
        //            CMNode.Topics.Pub.Speed_Limit.Msg.synthdelay   = CMNode.Sync.SynthDelay;

        //            ros::WallTime wtime = ros::WallTime::now();
        //            CMNode.Topics.Pub.Speed_Limit.Msg.header.stamp.sec  = wtime.sec;
        //            CMNode.Topics.Pub.Speed_Limit.Msg.header.stamp.nsec = wtime.nsec;

        //            if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Speed_Limit.CycleTime == 0 && TSignSensorCount > 0) {
        //                if (TSignSensor[0].nSign > 0) {
        //                    CMNode.Topics.Pub.Speed_Limit.Msg.distance          = TSignSensor[0].Sign[0].ds_p;
        //                    CMNode.Topics.Pub.Speed_Limit.Msg.speed_limit       = TSignSensor[0].Sign[0].main.val[0];
        //                } else {
        //                    CMNode.Topics.Pub.Speed_Limit.Msg.distance          = 0;
        //                    CMNode.Topics.Pub.Speed_Limit.Msg.speed_limit       = 0;
        //                }

        //            } else {
        //                CMNode.Topics.Pub.Speed_Limit.Msg.distance          = 0;
        //                CMNode.Topics.Pub.Speed_Limit.Msg.speed_limit       = 0;
        //            }

        //        }

        //        //IMU--> sensor_msgs/Imu //dk
        //        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Imu.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) { //dk
        //            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Imu.CycleTime, CMNode.Topics.Pub.Imu.CycleOffset, CMNode.CycleNoRel); //dk
        //            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Imu.Job), CMCRJob_RVStr(rv)); //dk
        //        } else { //dk
        ////            ros::WallTime wtime = ros::WallTime::now(); //dk
        ////            CMNode.Topics.Pub.Imu.Msg.header.stamp.sec  = wtime.sec; //dk
        ////            CMNode.Topics.Pub.Imu.Msg.header.stamp.nsec = wtime.nsec; //dk
        //            CMNode.Topics.Pub.Imu.Msg.header.stamp = ros::Time(SimCore.Time); //dk

        //            tf2::Quaternion rotation; //dk
        //            rotation.setRPY(Car.Roll, Car.Pitch, Car.Yaw - 0.03194); //dk

        //            CMNode.Topics.Pub.Imu.Msg.orientation = tf2::toMsg(rotation); //dk
        //            CMNode.Topics.Pub.Imu.Msg.linear_acceleration.x = InertialSensor[0].Acc_B[0]; //dk
        //            CMNode.Topics.Pub.Imu.Msg.linear_acceleration.y = InertialSensor[0].Acc_B[1]; //dk
        //            CMNode.Topics.Pub.Imu.Msg.linear_acceleration.z = InertialSensor[0].Acc_B[2]; //dk
        //            CMNode.Topics.Pub.Imu.Msg.angular_velocity.x = InertialSensor[0].Omega_B[0]; //dk
        //            CMNode.Topics.Pub.Imu.Msg.angular_velocity.y = InertialSensor[0].Omega_B[1]; //dk
        //            CMNode.Topics.Pub.Imu.Msg.angular_velocity.z = InertialSensor[0].Omega_B[2]; //dk

        //        } //dk

        //        //Traffic Light--> hellocm_msgs::TrafficLight //dk
        //        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.TrafficLight.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) { //dk
        //            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.TrafficLight.CycleTime, CMNode.Topics.Pub.TrafficLight.CycleOffset, CMNode.CycleNoRel); //dk
        //            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.TrafficLight.Job), CMCRJob_RVStr(rv)); //dk
        //        } else { //dk
        //            double distance; //dk
        //            int TL_id = TrfLight_GetDist2NextObj(Env.Route.ObjId, Vehicle.sRoad, &distance); //dk
        //            int state = TrfLight.Objs[TL_id].State; //dk

        //            CMNode.Topics.Pub.TrafficLight.Msg.cycleno      = CMNode.CycleNoRel; //dk
        //            CMNode.Topics.Pub.TrafficLight.Msg.time         = ros::Time(SimCore.Time); //dk
        //            CMNode.Topics.Pub.TrafficLight.Msg.synthdelay   = CMNode.Sync.SynthDelay; //dk

        //            ros::WallTime wtime = ros::WallTime::now(); //dk
        //            CMNode.Topics.Pub.TrafficLight.Msg.header.stamp.sec  = wtime.sec; //dk
        //            CMNode.Topics.Pub.TrafficLight.Msg.header.stamp.nsec = wtime.nsec; //dk

        //            CMNode.Topics.Pub.TrafficLight.Msg.status = state; //dk
        //            CMNode.Topics.Pub.TrafficLight.Msg.distance = distance; //dk

        //        } //dk

        //        //Velocity--> geometry_msgs/Vector3Stamped //dk
        //        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Velocity.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) { //dk
        //            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Velocity.CycleTime, CMNode.Topics.Pub.Velocity.CycleOffset, CMNode.CycleNoRel); //dk
        //            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Velocity.Job), CMCRJob_RVStr(rv)); //dk
        //        } else { //dk
        ////            ros::WallTime wtime = ros::WallTime::now(); //dk
        ////            CMNode.Topics.Pub.Velocity.Msg.header.stamp.sec  = wtime.sec; //dk
        ////            CMNode.Topics.Pub.Velocity.Msg.header.stamp.nsec = wtime.nsec; //dk
        //            CMNode.Topics.Pub.Velocity.Msg.header.stamp = ros::Time(SimCore.Time); //dk

        //            CMNode.Topics.Pub.Velocity.Msg.vector.x = Car.ConBdy1.v_1[0]; //dk
        //            CMNode.Topics.Pub.Velocity.Msg.vector.y = Car.ConBdy1.v_1[1]; //dk
        //            CMNode.Topics.Pub.Velocity.Msg.vector.z = Car.ConBdy1.v_1[2]; //dk

        //        } //dk

        Lidar_CycleCount++;

    }

    UDP_Input.DriveCont.SteeringWheel = g_vc.SteeringWheel;     // chp
    UDP_Input.DriveCont.Ax = g_vc.Ax;     // chp
    UDP_Input.DriveCont.GearNo = g_vc.GearNo;     // chp
    UDP_PC.VC_SwitchOn = g_vc.VC_SwitchOn;     // chp
    DrivMan.Lights.Indicator = g_vc.Light_Indicator;     // chp
    DrivMan.Lights.Hazard = g_vc.Light_Hazard;     // chp

    //UDP_Input.DriveCont.SteeringWheel = 1;
    //UDP_Input.DriveCont.Ax = 1;
    //UDP_Input.DriveCont.GearNo = 1;
    //UDP_PC.VC_SwitchOn = 1;
    
    //DrivMan.Lights.Hazard = 1;
    //DrivMan.Lights.Indicator = 1;
    
    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called close to end of CarMaker cycle
 * - See "User.c:User_Out()"
 */
int
CMRosIF_CMNode_Out (void)
{
    ros::WallTime wtime = ros::WallTime::now();

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
            || SimCore.State != SCState_Simulate)
        return 0;

    int rv;
    auto out = &CMNode.Topics.Pub.CM2Ext;

    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

        out->Msg.cycleno      = CMNode.CycleNoRel;
        out->Msg.time         = ros::Time(SimCore.Time);
        out->Msg.synthdelay   = CMNode.Sync.SynthDelay;

        /* Header stamp and frame needs to be set manually! */

        /* provide system time close to data is sent */
        wtime = ros::WallTime::now();
        out->Msg.header.stamp.sec  = wtime.sec;
        out->Msg.header.stamp.nsec = wtime.nsec;

        //////////////////////////////////////////////////////////////// 9WEEK

        hellocm_msgs::RoadSensorPath tmp;
        out->Msg.RoadPath.clear();
        int nRoadSensors = 10;
        for(int i = 0; i < nRoadSensors; i++)
        {
            tmp.x = RoadSensor[i].Path.P_0[0]; // path.tx
            tmp.y = RoadSensor[i].Path.P_0[1]; // path.ty
            out->Msg.RoadPath.push_back(tmp);
        }

        //////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////// e he ra di ya~


        out->Msg.camearaSensor.cameraSensorObj.clear();
        out->Msg.camearaSensor.nObj = CameraSensor->nObj;
        out->Msg.camearaSensor.cameraSensorObj.resize(out->Msg.camearaSensor.nObj);
        for (int i = 0; i < out->Msg.camearaSensor.nObj; i++)
        {
            out->Msg.camearaSensor.cameraSensorObj[i].ObjID = CameraSensor->Obj[i].ObjID;
            out->Msg.camearaSensor.cameraSensorObj[i].Confidence = CameraSensor->Obj[i].Confidence;
            out->Msg.camearaSensor.cameraSensorObj[i].Facing = CameraSensor->Obj[i].Facing;
            out->Msg.camearaSensor.cameraSensorObj[i].LightState = CameraSensor->Obj[i].LightState;
            out->Msg.camearaSensor.cameraSensorObj[i].nVisPixels = CameraSensor->Obj[i].nVisPixels;
            out->Msg.camearaSensor.cameraSensorObj[i].classType = CameraSensor->Obj[i].Type;

            out->Msg.camearaSensor.cameraSensorObj[i].bottom_left_x = CameraSensor->Obj[i].MBR[0][0];
            out->Msg.camearaSensor.cameraSensorObj[i].bottom_left_y = CameraSensor->Obj[i].MBR[0][1];
            out->Msg.camearaSensor.cameraSensorObj[i].bottom_left_z = CameraSensor->Obj[i].MBR[0][2];

            out->Msg.camearaSensor.cameraSensorObj[i].top_right_x = CameraSensor->Obj[i].MBR[1][0];
            out->Msg.camearaSensor.cameraSensorObj[i].top_right_y = CameraSensor->Obj[i].MBR[1][1];
            out->Msg.camearaSensor.cameraSensorObj[i].top_right_z = CameraSensor->Obj[i].MBR[1][2];
        }


        /////////////////////////////
        out->Msg.ego_GK_x = Car.FARoadSensor.P_0[0]; //RoadSensor[0].Path.P_0[0];
        out->Msg.ego_GK_y = Car.FARoadSensor.P_0[1]; //RoadSensor[0].Path.P_0[1];

        
        out->Msg.CMOut.Car_Roll_rad = Car.Roll;    // chp
        out->Msg.CMOut.Car_Pitch_rad = Car.Pitch;    // chp
        out->Msg.CMOut.Car_Yaw_rad = Car.Yaw;    // chp
        out->Msg.CMOut.Car_vx_mps = Car.ConBdy1.v_1[0];    // chp
        out->Msg.CMOut.Car_vy_mps = Car.ConBdy1.v_1[1];    // chp
        out->Msg.CMOut.Car_vz_mps = Car.ConBdy1.v_1[2];    // chp
        out->Msg.CMOut.Car_RollVel_rps = Car.RollVel;    // chp
        out->Msg.CMOut.Car_PitchVel_rps = Car.PitchVel;    // chp
        out->Msg.CMOut.Car_YawVel_rps = Car.YawRate;    // chp
        out->Msg.CMOut.Car_ax_mpss = Car.ConBdy1.a_1[0];    // chp
        out->Msg.CMOut.Car_ay_mpss = Car.ConBdy1.a_1[1];    // chp
        out->Msg.CMOut.Car_az_mpss = Car.ConBdy1.a_1[2];    // chp
        out->Msg.CMOut.Car_RollAcc_rpss = Car.RollAcc;    // chp
        out->Msg.CMOut.Car_PitchAcc_rpss = Car.PitchAcc;    // chp
        out->Msg.CMOut.Car_YawAcc_rpss = Car.YawAcc;    // chp
        out->Msg.CMOut.Steer_WhlAng_rad = Steering.IF.Ang;    // chp
        out->Msg.CMOut.VC_Gas = VehicleControl.Gas;    // chp
        out->Msg.CMOut.VC_Brake = VehicleControl.Brake;    // chp
        out->Msg.CMOut.VC_SelectorCtrl = VehicleControl.SelectorCtrl;    // chp

        out->Msg.CMOut.x_lin_vel_no_gravity_mps = InertialSensor[0].Vel_B[0];       // m/s   ( = Car.ConBdy1.v_1[0] ) ( current_velocity )    // chp
        out->Msg.CMOut.y_lin_vel_no_gravity_mps = InertialSensor[0].Vel_B[1];       // m/s    // chp
        out->Msg.CMOut.z_lin_vel_no_gravity_mps = InertialSensor[0].Vel_B[2];       // m/s    // chp
        out->Msg.CMOut.y_lin_vel_with_gravity_mps = InertialSensor[1].Vel_B[1];       // m/s    // chp
        out->Msg.CMOut.z_lin_vel_with_gravity_mps = InertialSensor[1].Vel_B[2];       // m/s    // chp
        out->Msg.CMOut.x_lin_vel_with_gravity_mps = InertialSensor[1].Vel_B[0];       // m/s    // chp


        out->Msg.CMOut.x_lin_acc_no_gravity_mpss = InertialSensor[0].Acc_B[0];       // m/s2    // chp
        out->Msg.CMOut.y_lin_acc_no_gravity_mpss = InertialSensor[0].Acc_B[1];       // m/s2    // chp
        out->Msg.CMOut.z_lin_acc_no_gravity_mpss = InertialSensor[0].Acc_B[2];       // m/s2    // chp
        out->Msg.CMOut.y_lin_acc_with_gravity_mpss = InertialSensor[1].Acc_B[1];       // m/s2    // chp
        out->Msg.CMOut.z_lin_acc_with_gravity_mpss = InertialSensor[1].Acc_B[2];       // m/s2    // chp
        out->Msg.CMOut.x_lin_acc_with_gravity_mpss = InertialSensor[1].Acc_B[0];       // m/s2    // chp


        out->Msg.CMOut.x_rot_vel_no_gravity_rps = InertialSensor[0].Omega_B[0];     // rad/s    // chp
        out->Msg.CMOut.y_rot_vel_no_gravity_rps = InertialSensor[0].Omega_B[1];     // rad/s    // chp
        out->Msg.CMOut.z_rot_vel_no_gravity_rps = InertialSensor[0].Omega_B[2];     // rad/s ( yaw_rate )    // chp
        out->Msg.CMOut.x_rot_vel_with_gravity_rps = InertialSensor[1].Omega_B[0];   // rad/s    // chp
        out->Msg.CMOut.y_rot_vel_with_gravity_rps = InertialSensor[1].Omega_B[1];   // rad/s    // chp
        out->Msg.CMOut.z_rot_vel_with_gravity_rps = InertialSensor[1].Omega_B[2];   // rad/s    // chp

        out->Msg.CMOut.x_rot_acc_no_gravity_rps = InertialSensor[0].Alpha_B[0];     // rad/s2    // chp
        out->Msg.CMOut.y_rot_acc_no_gravity_rps = InertialSensor[0].Alpha_B[1];     // rad/s2    // chp
        out->Msg.CMOut.z_rot_acc_no_gravity_rps = InertialSensor[0].Alpha_B[2];     // rad/s2    // chp
        out->Msg.CMOut.x_rot_acc_with_gravity_rps = InertialSensor[1].Alpha_B[0];   // rad/s2    // chp
        out->Msg.CMOut.y_rot_acc_with_gravity_rps = InertialSensor[1].Alpha_B[1];   // rad/s2    // chp
        out->Msg.CMOut.z_rot_acc_with_gravity_rps = InertialSensor[1].Alpha_B[2];   // rad/s2    // chp


        out->Msg.ego_orientation = Car.Yaw;    // chp

        out->Pub.publish(out->Msg);

        /* Remember cycle for debugging */
        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

//    auto out_lidar_vlp = &CMNode.Topics.Pub.Lidar_VLP;

//    /* Communicate to External ROS Node in this cycle?
//     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
//     */
//    if ((rv = CMCRJob_DoJob(out_lidar_vlp->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
//            && rv != CMCRJob_RV_DoSomething) {
//        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_lidar_vlp->Job), CMCRJob_RVStr(rv));
//    } else if (rv == CMCRJob_RV_DoSomething) {

//        out_lidar_vlp->Pub.publish(out_lidar_vlp->Msg);

//        /* Remember cycle for debugging */
//        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
//    }

    auto out_lidar_os1 = &CMNode.Topics.Pub.Lidar_OS1;

    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out_lidar_os1->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_lidar_os1->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

        out_lidar_os1->Pub.publish(out_lidar_os1->Msg);

        /* Remember cycle for debugging */
        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

    //    auto out_gps_out = &CMNode.Topics.Pub.GPS_Out;

    //    /* Communicate to External ROS Node in this cycle?
    //     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
    //     */
    //    if ((rv = CMCRJob_DoJob(out_gps_out->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
    //            && rv != CMCRJob_RV_DoSomething) {
    //        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_gps_out->Job), CMCRJob_RVStr(rv));
    //    } else if (rv == CMCRJob_RV_DoSomething) {

    //        out_gps_out->Pub.publish(out_gps_out->Msg);

    //        /* Remember cycle for debugging */
    //        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    //    }

    //    //Send out Speed_Limit
    //    auto out_speed_limit = &CMNode.Topics.Pub.Speed_Limit;

    //    /* Communicate to External ROS Node in this cycle?
    //     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
    //     */
    //    if ((rv = CMCRJob_DoJob(out_speed_limit->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
    //            && rv != CMCRJob_RV_DoSomething) {
    //        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_speed_limit->Job), CMCRJob_RVStr(rv));
    //    } else if (rv == CMCRJob_RV_DoSomething) {

    //    out_speed_limit->Pub.publish(out_speed_limit->Msg);

    //    /* Remember cycle for debugging */
    //    CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    //    }

    //    //Send out IMU //dk
    //    auto out_imu = &CMNode.Topics.Pub.Imu; //dk

    //    /* Communicate to External ROS Node in this cycle? //dk
    //     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle //dk
    //     */ //dk
    //    if ((rv = CMCRJob_DoJob(out_imu->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing //dk
    //            && rv != CMCRJob_RV_DoSomething) { //dk
    //        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_imu->Job), CMCRJob_RVStr(rv)); //dk
    //    } else if (rv == CMCRJob_RV_DoSomething) { //dk

    //    out_imu->Pub.publish(out_imu->Msg); //dk

    //    /* Remember cycle for debugging */ //dk
    //    CMNode.Model.CycleLastOut = CMNode.CycleNoRel; //dk
    //    } //dk

    //    //Send out Traffic Light //dk
    //    auto out_trafficLight = &CMNode.Topics.Pub.TrafficLight; //dk

    //    /* Communicate to External ROS Node in this cycle? //dk
    //     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle //dk
    //     */ //dk
    //    if ((rv = CMCRJob_DoJob(out_trafficLight->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing //dk
    //            && rv != CMCRJob_RV_DoSomething) { //dk
    //        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_trafficLight->Job), CMCRJob_RVStr(rv)); //dk
    //    } else if (rv == CMCRJob_RV_DoSomething) { //dk

    //    out_trafficLight->Pub.publish(out_trafficLight->Msg); //dk

    //    /* Remember cycle for debugging */ //dk
    //    CMNode.Model.CycleLastOut = CMNode.CycleNoRel; //dk
    //    } //dk

    //    //Send out Velocity //dk
    //    auto out_velocity = &CMNode.Topics.Pub.Velocity; //dk

    //    /* Communicate to External ROS Node in this cycle? //dk
    //     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle //dk
    //     */ //dk
    //    if ((rv = CMCRJob_DoJob(out_velocity->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing //dk
    //            && rv != CMCRJob_RV_DoSomething) { //dk
    //        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_velocity->Job), CMCRJob_RVStr(rv)); //dk
    //    } else if (rv == CMCRJob_RV_DoSomething) { //dk

    //    out_velocity->Pub.publish(out_velocity->Msg); //dk

    //    /* Remember cycle for debugging */ //dk
    //    CMNode.Model.CycleLastOut = CMNode.CycleNoRel; //dk
    //    } //dk

    /* Publish "/clock" topic after all other other topics are published
     * - Is the order of arrival in other node identical? */
    if (CMNode.Cfg.nCyclesClock > 0 && CMNode.CycleNoRel%CMNode.Cfg.nCyclesClock == 0) {
        CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(SimCore.Time);
        CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }

    /* ToDo: When increase? */
    CMNode.CycleNoRel++;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called one Time when CarMaker ends
 * - See "User.c:User_End()"
 */
int
CMRosIF_CMNode_End (void)
{
//    CMNode.Topics.Pub.Lidar_VLP.Msg.points.clear();
//    CMNode.Topics.Pub.Lidar_VLP.Msg.channels.clear();

    CMNode.Topics.Pub.Lidar_OS1.Msg.points.clear();
    CMNode.Topics.Pub.Lidar_OS1.Msg.channels.clear();

    free(CMNode.LidarRSI_VLP.BeamTable);
    free(CMNode.LidarRSI_OS1.BeamTable);

    LOG("%s: End", __func__);

    if (ros::isInitialized()) {

        /* Needs to be called before program exists, otherwise
     * "boost" error due to shared library and default deconstructor */
        CMNode.Cfg.Ros.Node->shutdown();

        /* ToDo:
     * - Blocking call? Wait until shutdown has finished?
     * - Optional? */
        ros::shutdown();
    }

    return 1;
}



/*!
 * Important:
 * - NOT automatically called by CMRosIF extension
 *
 * Description:
 * - Example of user generated function
 * - Can be accessed in other sources, e.g. User.c
 * - Use "CMRosIF_GetSymbol()" to get symbol (see "lib/CMRosIF.h")
 *
 */
int
CMRosIF_CMNode_MyFunc (char *LogMsg)
{

    LOG("%s: %s",  __func__, LogMsg);
    return 1;
}

#ifdef __cplusplus
}
#endif

