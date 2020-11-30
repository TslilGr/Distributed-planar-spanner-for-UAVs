
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


#ifndef __INET_IDEALCHANNELMODELACCESS_H
#define __INET_IDEALCHANNELMODELACCESS_H
#define DISTANCE_FROM_COMMANDER 20

#include "INETDefs.h"
#include <crng.h>
#include "BasicModule.h"
#include "MYIdealChannelModel.h"

// Forward declarations
class MYIdealAirFrame;


/**
 * This class contains functions that cooperate with MYIdealChannelModel.
 * Subscribes to position change and updates cached radio position if
 * position change signal arrived.
 *
 * The radio module has to be derived from this class!
 */
class MYIdealChannelModelAccess : public BasicModule, protected cListener
{
  protected:
    static simsignal_t mobilityStateChangedSignal;
    static simsignal_t extraDroneToCCSignal;
    static simsignal_t droneToDrone;
    static simsignal_t faultSignal;
    static simsignal_t establishSubGraph;

    MYIdealChannelModel *cc;  // Pointer to the MYIdealChannelModel module
    MYIdealChannelModel::RadioEntry *myRadioRef;  // Identifies this radio in the MYIdealChannelModel module
    cModule *hostModule;    // the host that contains this radio model
    Coord radioPos;  // the physical position of the radio (derived from display string or from mobility models)
    bool positionUpdateArrived;
    bool isWaitingForConnectivity;
    char linkerWhenCalledConnectivity[20];

    int min_currDepthLvl_Receveid;    // this value will help to know what is the shortest path which a replay message has reach to this drone.


  public:

    //************************* Tamir & Adi****************************************************************/
      public:
        MYIdealChannelModel::myRadioList hostNeigborsList;       //drones neighbors vector
        MYIdealChannelModel *cc2;
        int priority_sums [20]={0};
        int my_priority_sum=0;
        bool priority_case=false;
       virtual bool IsLeafDrone();
       // virtual Coord GetSrcLocation();
       int detachNumFromFullName3(const char *senderName);

    enum{
        PRIORITY_REQUEST=1,
        PRIORITY_REPLY,
        PRIORITY_MOVE

    }PRIORITY_MESSAGES;

    struct DroneStockDataBase       // to stock the receiving databases from the replyMessages- until sent it,Re-attached, back to the initiative drone.
    {
        int messageID;              // the ID of the message that its database shell be collect.
        int messageCounter;         // to how much drones this drone has pass the findMessage and will need to wait for them to reply with replyMessages.
        char databaseArray[DRONE_BUFFER];   // This database will be used to reassemble all the data from the other reply messages.
    };

    struct Host_Info
    {
        int host_ID;
        int coverDrone_ID;
        Coord host_Pos;
        bool isDealeted;
    };

    struct Drone_Info
    {
        int coverHost[INF_HOSTS];
        int droneLinker_ID;
        int drone_ID;
        int num_Of_Hosts_Cover;     // how much hosts this drone is cover.
        int number_Of_Hops;         // Drone's Number of hops from the CC.
        int droneIndex;             // the drone number of being pushed into the list. first 1 will be 0
        bool isCover;               // Is this a covering drone?
        bool useAsServer;           // Does this drone use as a drone server for the Central algorithm?
        Coord drone_Cord;
        Coord droneLinker_Cord;
        double droneRadios;         // The drone's transmission radios
        double currCost;            //Tslil & HILA //M_x
    };

    struct Host_Drone_Dist
    {
        int hostID;
        int droneID;
        double host_drone_Distance;
    };


    typedef std::list<Host_Info> HostList;
    typedef std::list<Drone_Info> DronesList;

    HostList hostList;              // List of all the hosts and their info ,received by the drone's replyFrame
    DronesList droneList;           // List of all the drones and their info ,received by the drone's replyFrame
    DronesList droneServerList;     // List of the cover drones that we want to replace their locations.

    DronesList final_List;          /* This list is only use in and for the final changes in the and of the central algorithm. it will hold all of the drones
                                       in the "droneList", with the changes made in the "droneServerList"  */


    DroneStockDataBase databaseArray[INF_MessageID_Numner];

    int droneArray[INF_DRONE];
    int numOfDrones;
    int numOfHosts;

    MYIdealChannelModelAccess() : cc(NULL), myRadioRef(NULL), hostModule(NULL) {}
    virtual ~MYIdealChannelModelAccess();

    //Received a find message to pass it towards my neighbors and then wait for their replies.
    virtual void receivedFindMessage(MYIdealAirFrame *airframe);

    //Received the Reply messages from all the drones that sent a Find messages, reassemble the database from all of them and send a reply message to the drone which sent to me a find message
    virtual void receivedReplyMessage(MYIdealAirFrame *airframe);

    //This function will detach the drone number from the drone name (as a string) and will return it as an integer.
    virtual int detachNumFromFullName();

    virtual int detachNumFromFullName(const char * currfullname);

    // This function will return the Lines of the rectangular that covers the hosts. Note that the (0,0) point is the most UPPER-LEFT
    virtual void Rectangular_Cover_Hosts(int* RecHorizontal_UpperLine, int* RecHorizontal_DownLine, int* RecVertical_LeftLine, int* RecVertical_RightLine);

    // Return the drone__id that most close to a given point. also set the drone's coordinations to the new ones.
    virtual int find_Best_Drone_Replacment(Coord midRec);

    // Remove from "hostList" all the host from the given drone, by its radios.
    virtual void remove_Hosts_From_Drone_Cover(int droneID,int droneServer_Counter);

    // Count how many hosts that not deleted are left in the "hostList"
    virtual int find_HostGroup_size();

    // Find the most far distance between each pair of <Drone,Host> (drone from the "droneServerList" drones)
    virtual void Find_Far_Distance_Pair(int *droneServer_ID, int *mostFarHost);

    /* Add to the server drone that have the longest distance to any host, a second host - on its circumference - directing that host.
     * Also add to that new drone - all the host it may been cover and delete that host for the host's group : "hostList"     */
    virtual int add_New_ServerDrone(int newServerDrone,int mostFarHost);

    // Clean and reset all data-structure - but(!!), only for information related to the relevant message ID.
    virtual void reset_All_Structures(int msgID);


    virtual void imply_CentralAlgorithm_Changes();

    /* Create new list of drones- all the drones from the "droneServerList" will be added and also all the other drones from the "droneList" that
       didn't showed up in  "droneServerList" - those will be linkers or just expendable drones - so we will add them and only change their
       "num_Of_Hosts_Cover" to zero.*/
    virtual void set_Final_droneList();

    virtual int howManyDigits(const char *fullName,int index);

    // find the cell which has the info on the received Message. if didn't find:return -1
    virtual int findCellIndex(int farmeID);

    virtual int findOpenCell();

    virtual int findMessageSender(int frameID);
    virtual void buildDataStructure(int msgID);

    virtual bool is_New_Replacments_Are_Better();


    /**
     * @brief Called by the signaling mechanism to inform of changes.
     *
     * MYIdealChannelModelAccess is subscribed to position changes.
     */
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

  protected:
    /** Sends a message to all radios in range */
    virtual void sendToChannel(MYIdealAirFrame *msg);

    /** Pass to MYIdealChannelModel to MYIdealChannelModel::sendToMYChannel*/
    virtual void sendToMYChannel(MYIdealAirFrame *msg,int SignalID);

    virtual void centralAlgorithm(int msgID);

    // get drones linker
    virtual const char * getLinker(const char * currfullname);

    // update the replyMessage with your locations, your hosts locations(if there are) and your linker location
    virtual void set_Drone_Database(MYIdealAirFrame *airframe,bool lastPazel);

    // update the drone's database with each ReplyFrame received and if needed, with my own database (only when all reply messages arrived)
    virtual void Update_Drone_Database(MYIdealAirFrame *airframe,int index, bool useMyDatabase,bool isLeaf, bool is_Sender);

    // get host position
    virtual Coord& getHostPosition(const char * currfullname);

    virtual void setmyisActive(const char *currfullname, const bool& myisActive);

    virtual int changeCoordTOString(MYIdealAirFrame *airframe,int startIdx, Coord pos);

    virtual void setNameCoordFrameName(const char *currfullname,const char *nameCoordFrameName);

    virtual void setsendToNameCoordFrameName(const char *currfullname,bool send);

    virtual void setExtraDroneLinker(const char *currfullname,const char * newLinkerFullName);


    virtual int fill_Drone_Linker_data(MYIdealAirFrame *airframe,const char* moduleType,int index,const char *fullName,Coord pos);

    // set drones linker
    virtual void setCCLinker(const char *currfullname);

    // Check if the frame with the frameID exist in the database
    virtual bool isFrmaeExist(int frameID);

    virtual cPar& getChannelControlPar(const char *parName) { return (cc)->par(parName); }
    const Coord& getRadioPosition() const { return radioPos; }
    cModule *getHostModule() const { return hostModule; }

    /** Register with ChannelControl and subscribe to hostPos*/
    virtual void initialize(int stage);
    virtual int numInitStages() const { return 3; }

    virtual void AtachNumToFullName(int senderID, char * senderName);

    //TSLIL & HILA
    virtual double culc_beta();
    virtual double culc_R_u();
    virtual bool check_cycles(const char * start);
    virtual void print_statistics_time();
    virtual void update_soldier_array();
    virtual void update_farest_soldier();
};

#endif

