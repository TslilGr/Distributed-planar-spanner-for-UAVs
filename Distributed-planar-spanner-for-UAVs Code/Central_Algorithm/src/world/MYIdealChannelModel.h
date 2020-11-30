//
// Copyright (C) 2013 OpenSim Ltd
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//
// author: Zoltan Bojthe
//

#ifndef __INET_IDEALCHANNELMODEL_H
#define __INET_IDEALCHANNELMODEL_H


#include "INETDefs.h"

#include "Coord.h"

// Forward declarations
class MYIdealAirFrame;
class MYIdealRadio;

#define INF_DRONE  30              // make sure number of drones will not pass the number. or just increment it
#define INF_HOSTS 30                // real number of host should not exceed this number
#define INF_MessageID_Numner 10     // make
#define DRONE_BUFFER 2048
#define NO_Neighbours -2
#define SOLDEIR_NUMBER 10



/**
 * This class represent an ideal channel model (aka channelControl)
 *
 * Stores infos about all registered radios.
 * Forward messages to all other radios in max transmission range
 */
class MYIdealChannelModel : public cSimpleModule
{

  public:
    enum{
            PRIORITY_REQUEST=1,
            PRIORITY_REPLY,
            PRIORITY_MOVE

        }PRIORITY_MESSAGES;
    struct RadioEntry
    {
        cModule *radioModule;           // the module that registered this radio interface
        cGate *radioInGate;             // gate on host module used to receive airframes
        Coord pos;                      // cached radio position
        bool isActive;                  // radio module is active
        bool myisActive;                // radio module is active
        bool isFault=false;             // radio module is fault
        bool faultHandler;              // radio module is in fault handler
        int moveID;                     //radio id that is need to move
        const char *linker ;            // defines the owner for host and linker for drones
        double transmissionRange;       //transmissionRange
        const char *nameCoordFrameName; //send when drone has 0 neighbors and invoked to a new location
        double bestCurrentRmin;
        int ownerSize;                  // how much host, the drone is covering (if this is a cover drone)
        int myCoverSum;                 // MOSHE: sum of priority
        int ID;                         // the Drone/Host ID number.
        int linkerID;                   // linker of the drone in integer form. only used for the implying of the central algorithm
        int seconds_Of_Operation;       // for statistics - how long this drone has been operate
        int num_Of_Hops;                // how far this drone from its CC
        bool is_Drone;                  // does this module is a drone or not
        bool is_Host;                   // does this module is a host or not
        bool central_Algo_Succeeded;    // does the central algorithm had done and its valid
        bool drone_In_BFS_Mode;         // is the drone still waits for replay messages?
        bool is_In_CentralAlgorithm_Procedure;       // Is drone currently participate in central algorithm procedure?
        bool sendDroneToCC;             // This flag is a patch to fix bug when a not-needed linker drone is shut-down by another drone and needs to go to CC (when drones finished implying the central algorithm)
        bool sendDroneToNameCoord;      //moveing unit
        bool isReplaced;                //for fault
        bool isFirstEstablish;          //flag for randomizing R_u for first establish message in each drone
        const char* nextlinker;         //for fault
        double currCost;                //TSLIL & HILA to set my linker with the max cost
        int lastSender;                 //from whom i recived the max currcost
        bool stisticsFlag;
        Coord soldierArray[SOLDEIR_NUMBER];
   };

    bool CENTRAL_ALGORITHM_ON;          // this flag indicate if the user choose to work in this simulation with the central algorithm as part of it.

    int drone_imply_centralAlgo_turn[INF_DRONE];  // This array will give a drone indication for whether it needs to implies the central algorithm(!=-1), and if so, its priority in line

    int drone_imply_centralAlgo_turn_Duplicat[INF_DRONE];  // duplicate of the "drone_imply_centralAlgo_turn" so a drone will know it was implying the central algorithm

    int noSEC_CountDown[INF_DRONE];               //after a drone implied central algorithm - use this counter to avoid the drone to move the SEC point for couple of times.

    int inFault=4;

    int recognizeID=-1;

    // statistics:
    simtime_t simulationTimeOfOperation_new;
    simtime_t simulationTimeOfOperation_old;



    cDoubleHistogram *Total_Number_Of_Drones22;
    cOutVector *Total_Number_Of_Drones22_Vec;



    double Drone_criticalStrip;     // being initiliaze in MYConstSpeedMobilityDrone.cc

    int pass ;

    /**used in MYIdealRadio to mimic radios and use it to set the next position */
    typedef std::list<RadioEntry> myRadioList;

    myRadioList radios2;    //must be public in order to be able to return radios to MYConstSpeedMobilityDrone

    myRadioList radios3;    //This list will use to pass relocations of the central algorithm to the "MYConstSpeedMobilityDrone" module.


  protected:
    typedef std::list<RadioEntry> RadioList;
    RadioList radios;    // list of registered radios

    friend std::ostream& operator<<(std::ostream&, const RadioEntry&);

    /** the biggest transmission range in the network.*/
    double maxTransmissionRange;

  protected:
    /** Reads init parameters and calculates a maximal interference distance*/
    virtual void initialize();


    /** recalculate the largest transmission range in the network.*/
    virtual void recalculateMaxTransmissionRange();

  public:
    MYIdealChannelModel();
    virtual ~MYIdealChannelModel();

    virtual int send_Find_Message(RadioEntry *srcRadio, MYIdealAirFrame *airFrame,int lastSender );

    virtual void send_Reply_Message(RadioEntry *srcRadio, MYIdealAirFrame *airFrame,int lastSender);

    virtual int detachNumFromFullName2(const char *senderName);

    virtual void update_My_Hops_Counts(RadioEntry re1);

    virtual RadioEntry find_Radio_from_radios3_by_ID(int drone_ID);

    virtual bool isDrone(const char *senderName);

    virtual bool isHost(const char *senderName);

    //    /** Returns the "handle" of a previously registered radio. The pointer to the registering (radio) module must be provided */
    virtual RadioEntry *lookupRadio(cModule *radioModule);


    // Search if this drone priority of imply the central algorithm is the biggest - if so, then it will imply the new position and covering and then will past the priority to the second biggest one
    virtual bool does_drone_imply_centralAlgoritm(int drone_ID);

    // update the radios list with the results of the central algorithm towards this specific drone
    virtual void update_radios(int droneServer_ID);

    // Notify to the "MYConstSpeedMobilityDrone", through the radios2 list, that this procedure has now over and it can go on with its regular flow or to apply the changes made by the central algorithm
    virtual void Notify_Algorithm_isOver(bool isValid,int drone_ID);

    virtual bool check_Linker_In_Range(int droneID);

    virtual bool is_Drone_Last_ToRelocate(int myID);

    virtual bool does_Drone_Needs_To_Wait();

    virtual void end_Central_Algorithm_Procedure(int droneID);

    virtual void reset_Central_Algorithm_Procedure(int droneID);

    virtual bool did_Drone_Arrived_To_Target_Position(int droneID,Coord droneCurrPostion,Coord new_TargetPosition);

    virtual void Notify_All_Drones_To_Abort_Central_Procedure();

    virtual int findMyHostId(const char *me);

    virtual void update_Hosts_Linker();

    virtual void try_Swap_linker_With_Cover_Linker();

    virtual void setDroneOperateTime(int myID, bool take_Only_numbe_of_drones);

    virtual void Set_DroneServers_To_NotUse_SEC();

    virtual void swap_Hosts_Ownership_while_waiting(int DroneID);

    virtual int swap_Linker_Ownership_while_waiting(int DroneID);

    virtual int number_Of_Active_Drones();

    virtual void setNotActiveDroneToMoveToCC(int droneID, bool flag);

    virtual void setsendDroneToNameCoord(int droneID, bool flag);

    virtual void setisReplaced(int droneID, bool flag);

    virtual bool multiple_Drones_Cover_closed_Group_Of_Hosts(int droneID);

    virtual bool isDroneLeaf(int droneID);

    virtual int dronePriority(int droneID);

    void finish();


    //
    /** Returns the "handle" of a previously registered radio. The pointer to the registering (radio) module must be provided */
    //virtual RadioEntry *lookupRadio(cModule *radioModule);

    /** Registers the given radio. If radioInGate==NULL, the "radioIn" gate is assumed */
    virtual RadioEntry * registerRadio(cModule *radioModule, cGate *radioInGate = NULL);

    /** Unregisters the given radio */
    virtual void unregisterRadio(RadioEntry *r);

    /** To be called when the host moved; updates proximity info */
    virtual void setRadioPosition(RadioEntry *r, const Coord& pos);

    //just a TMP for the real function to set a new owner(linker) of the hosts
    //we want to use it from mobility
    virtual void setmyisActive(const char *targetName, const bool& myisActive);

    virtual void setnextlinker(const char *targetName,const char *nextLinker);

    virtual const char * getnextlinker(const char *targetName);

    virtual void setisFault(const char *targetName, const bool& isFault);

    virtual void setfaultHandler(const char *targetName, const bool& isFault);

    virtual void setmoveID(const char *targetName, int id);

    virtual void setNameCoordFrameName(const char *currfullname,const char *nameCoordFrameName);

    virtual void setNewHostOwner(const char *hostFullName,const char *droneFullName);

    virtual void setDroneOwnedSize(const char *currfullname,int size);

    virtual int getDroneOwnedSize(const char * currfullname);

    virtual void setMyCoverSum(const char *currfullname,int size);

    virtual int getMyCoverSum(const char * currfullname);

    virtual void getAndSetExtraDroneLinker(const char *currfullname,const char * newLinkerFullName);

    virtual const char * findAndSetCCLinker(const char *currfullname);

    virtual Coord& getHostPosition(const char * currfullname);

    virtual const char * getLinker(const char * currfullname);

    virtual double getTransmissionRange(const char * currfullname);

    virtual void setBestCurrentRmin(const char * currfullname, double currBestRmin);

    virtual double getBestCurrentRmin(const char * currfullname);

    /** Called from MYIdealChannelModelAccess, to transmit a frame to the radios in range, on the frame's channel */
    virtual void sendToChannel(RadioEntry * srcRadio, MYIdealAirFrame *airFrame);

    //my send to channel that sends simple direct msg to spesific dest(mine needed cuz no network layer is used)
    virtual int sendToMYLinker(RadioEntry *srcRadio, MYIdealAirFrame *airFrame);

    /** Disable the reception in the reference module */
    virtual void disableReception(RadioEntry *r) { r->isActive = false; };

    /** Enable the reception in the reference module */
    virtual void enableReception(RadioEntry *r) { r->isActive = true; };

    /** returns speed of signal in meter/sec */
    virtual double getSignalSpeed() { return SPEED_OF_LIGHT; }

    virtual bool isMYRadioOwnerHost(const char *name);

    ///me trying to pass back to MYIdealRadio radios.(and if yes then just the relevant once)
    virtual int getFromICM();

    virtual myRadioList getRadios();

    virtual RadioEntry lookupRadioByName(const char *name);

    virtual const char * lookupNameByName(const char *name);

    // Enable the waiting drone at cc request in the reference module
    virtual void activateExtraDrone(RadioEntry *r) { r->myisActive = true; };

    //virtual double getT() { return SPEED_OF_LIGHT; }

};

#endif      // __INET_IDEALCHANNELMODEL_H

