/*
 * MYConstSpeedMobilityDrone.h
 *
 *  Created on: May 17, 2014
 *      Author: Miker
 */

#ifndef MYCONSTSPEEDMOBILITYDRONE_H_
#define MYCONSTSPEEDMOBILITYDRONE_H_

//#include "INETDefs.h"

#include "MYLineSegmentsMobilityBase.h"
#include "MYIdealChannelModel.h"

//
#include "MYIdealChannelModelAccess.h"
#include "MYIdealRadio.h"

/**
 * @brief Moves along a line with constant speed to a randomly chosen target.
 * When the target is reached it selects randomly a new one.
 *
 * @ingroup mobility
 * @author Steffen Sroka, Marc Loebbers, Daniel Willkomm
 */
class MYConstSpeedMobilityDrone : public MYLineSegmentsMobilityBase
{
protected:
    /** @brief Speed parameter. */

    double speed;

    double RneighborhelpstripDelta;
    double criticalStrip;
    double RwhenExDroneOnTheWay;
    double strengthStripDelta;
    double criticalConnectivityStrip;
    double maxOnTheWayDistX;
    double maxOnTheWayDistY;
    double transmissionRange;

    int ccChoosingMode;

    simtime_t droneTimeOfOperation_old;
    simtime_t droneTimeOfOperation_new;
    simtime_t timeOf_Last_ShotDown;

    int faultStage=4;
    bool isInCritical;

    double distToLinkerTag;

    //list for position calculation
    typedef std::list<double> PosList;
    PosList posList;

    static simsignal_t extraDroneToCCSignal;
    static simsignal_t droneToDrone;
    static simsignal_t faultSignal;
    static simsignal_t establishSubGraph;
    static simsignal_t hopCountSignal;


    cModule *hostModule;    // the host that contains this radio model

    MYIdealChannelModel::myRadioList myradios2;

    MYIdealChannelModel::myRadioList hostNeigborsList;       //drones neighbors vector

    MYIdealChannelModel::myRadioList droneNeigborsList;       //drones neighbors vector

    MYIdealChannelModel::myRadioList ccNeigborsList;

    MYIdealChannelModel::myRadioList criticalContenders;

    MYIdealChannelModel::RadioEntry myRadioRef2;

    MYIdealChannelModel *cc2;  // Pointer to the MYIdealChannelModel module (like in MYIdealChannelModelAccess) - these two must know each other

    double onTheWaySpeed;

    const char *curFullName ;

    struct DroneHostRminSet
    {
        const char *droneHelping ;     // the bestPair drone member, the Neigbor that can help
        const char *hostHelped ;       // the bestPair host member, the one is targeted to be transfered
        double newTogetherRminsqrt;        //the new Rmin that droneHelping have when adding hostHelped to its ownership
    };

    struct waitingForCriticalSet
    {
        bool isWaiting;
        const char *onTheWaylastCalledExtraDrone ;
        const char *lastHelpedHost ;
        Coord lastHelpedHostPos;
    };

    waitingForCriticalSet waitingForCriticalSet;
    struct faultOrganize{
        bool imFault;
        bool isWaiting;
        int faultNum;
        const char *faulted ;
        const char *recognize ;
        Coord lastFaultedPlace;
    };
    faultOrganize faultInfo;
    const char *curName ;
    bool keep_Position_Alone;         // this flag is only for use when all hosts are with the same drone(not at the initial stage!!! but when all host gather to a single point )
    int mystage;



protected:
    virtual int numInitStages() const { return 3; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage);

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition();

    virtual const char* swapAllToRedundanteLinker();

    virtual Coord setDroneOff(const char * curFullName);

    virtual int findTransferredHostId(const char *hostFullName);

    virtual bool isAnyCriticalContendorInSameArea(Coord preCallHostPos);

    virtual const char *getClosestCriticalContendorFromSameArea(const char *preCallHostFullName, Coord preCallHostPos);

    virtual const char *findFARTHESTTransferredCriticalHost();

    virtual const char *findRANDOMTransferredCriticalHost();

    virtual MYIdealChannelModel::myRadioList canMyNeighborDronesHelp();

    virtual bool isDroneCanHelpWithHostCover(DroneHostRminSet currDronesBestNeighborSet);

    virtual const char *canPassHostToNeighborDrone(const char *testedOneHostToPassFullName);

    virtual const char *getClosestNeighborDroneToHost(MYIdealChannelModel::myRadioList canHelpSizeMoreOneNeighborDroneContendorList, const char*testedOneHostToPassFullName);

    virtual void emitExtraDroneToCCSignal(long hostNum);

    virtual void emitFaultSignal(long hostNum);

    virtual void emitDroneToDroneSignal(long hostNum);

    virtual void emitEstablishSubGraphSignal(const char * droneAndLinker);

    virtual Coord calcSECPosition();

    virtual MYIdealChannelModel::myRadioList checkPairs();

    virtual MYIdealChannelModel::myRadioList checkTriplets();

    virtual bool isThisPairCoveringAllPairs(Coord currMidPoint, double currR, cModule *itradioModule, cModule *it2radioModule);

    virtual bool isThisTripletCoveringAllTriplets(Coord currMidPoint, double currR, cModule *itradioModule, cModule *it2radioModule, cModule *it3radioModule);

    virtual int checkScenario(double itX,double itY,  double it2X,double it2Y,  double it3X,double it3Y);

    virtual bool isExtraDroneTypeConnectivity(const char *nameCoordFrameName);

    virtual bool isReplacedrone(const char *nameCoordFrameName);

    virtual Coord detachCoordFromNameCoordFrame(const char *nameCoordFrameName);

    virtual void setDroneNeigborsList();

    virtual MYIdealChannelModel::myRadioList::iterator getDroneNeighborsList();

    virtual void updateConnectivityTag();

    virtual Coord calculate_Linker_Next_Corrd();

    virtual bool is_ServerDrone_On_Action();

    virtual bool is_Drone_Is_Linker_Waiting_For_Relocation();

    // Check if all the hosts are in the cover zone of a single drone, and if the all close to the same point in the map.
    virtual bool all_Hosts_In_One_Spot_Under_One_Drone();

    // Find the middle of a very close group of host, covered by the same drone.
    virtual Coord find_CenterPoint();

    virtual double euclidean_distance(int x1, int y1, int x2, int y2);

    virtual bool checkIfDroneIsBlue();

    virtual void AtachNumToFullName(int senderID, char * senderName);
    virtual void print_statistics_time();
    virtual void print_statistics_hop_count();
    virtual void change_true_statistics();

    //    virtual Coord calculate_TP_Around_NFZ(int numOfNFZ);

    //virtual double median(bool isXmedian);
public:
    MYConstSpeedMobilityDrone();
    int    myCoverSum;
    virtual int handleFault();

};




#endif /* MYCONSTSPEEDMOBILITYDRONE_H_ */
