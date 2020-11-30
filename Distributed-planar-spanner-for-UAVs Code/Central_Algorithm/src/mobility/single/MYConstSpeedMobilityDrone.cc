/*
 * MYConstSpeedMobilityDrone.cc
 *
 *  Created on: May 17, 2014
 *      Author: Miker
 */


#include "MYConstSpeedMobilityDrone.h"
#include "FWMath.h"
#include "stdlib.h"
#include <string>
#include <math.h>
#include <fstream>

#include "MYIdealChannelModel.h"
#include "MYIdealChannelModelAccess.h"

#include <algorithm>
#include <vector>

//MOSHE: enum for fault cases
enum fault{leafFall=1,leafDiscover,ReplacedByOther,noFault,endFault};

using std::vector;
using std::sort;

//send signal
simsignal_t MYConstSpeedMobilityDrone::extraDroneToCCSignal = cComponent::registerSignal("extraDroneToCC");
simsignal_t MYConstSpeedMobilityDrone::droneToDrone = cComponent::registerSignal("msgToDrone");
simsignal_t MYConstSpeedMobilityDrone::faultSignal = cComponent::registerSignal("msgFaultDrone");
simsignal_t MYConstSpeedMobilityDrone::establishSubGraph = cComponent::registerSignal("msgEstablishSubGraph");// Tslil & Hila
simsignal_t MYConstSpeedMobilityDrone::hopCountSignal = cComponent::registerSignal("hopCount");// Tslil & Hila

bool firstDroneOnly_For_Statistics = true;
bool firstTImeOnly_To_create_critical_stage = true;
bool firstFault=true;
Coord faultTarget;
Define_Module(MYConstSpeedMobilityDrone);

MYConstSpeedMobilityDrone::MYConstSpeedMobilityDrone()
{
    speed = 0;
}


void MYConstSpeedMobilityDrone::initialize(int stage)
{
    MYLineSegmentsMobilityBase::initialize(stage);

    EV_TRACE << "initializing MYConstSpeedMobilityDrone stage " << stage << endl;

    if (stage == 0)
    {
        curFullName = this->getParentModule()->getFullName();  //setting full name of current object
        curName = this->getParentModule()->getName();  //setting name of current object
        mystage = 0;

        criticalStrip = par("criticalStrip");
        RneighborhelpstripDelta = par("RneighborhelpstripDelta");
        RwhenExDroneOnTheWay = par("RwhenExDroneOnTheWay");
        strengthStripDelta = par("strengthStripDelta");
        criticalConnectivityStrip = par("criticalConnectivityStrip");
        ccChoosingMode = par("ccChoosingMode");

        maxOnTheWayDistX = par("maxOnTheWayDistX");
        maxOnTheWayDistY = par("maxOnTheWayDistY");
        transmissionRange = par("transmissionRange");

        droneTimeOfOperation_old = 0;
        droneTimeOfOperation_new = 0;

        //    keep_Position_Alone = false;
        speed = par("speed");

        /*
         the bigger the strip -> the dist is smaller -> and we want to use the minimal dist to pass until on the way drone arrives, minimal couse onTheWay speed need to be faster
         ONLY TRUE IF ${isMINcriticalStrip1}>${isMINcriticalConnectivityStrip2}} HOLDS
            onTheWayspeed = speed*maxOnTheWayDist = sqrt(maxOnTheWayDistX^2+maxOnTheWayDistY^2)/isMINcriticalStrip1
             ELSE
            onTheWayspeed = ${speed}*${maxOnTheWayDist = sqrt(${maxOnTheWayDistX}*${maxOnTheWayDistX}+${maxOnTheWayDistY}*${maxOnTheWayDistY})}/${isMINcriticalStrip2}
         */
        double maxOnTheWayDist = speed*(sqrt(maxOnTheWayDistX*maxOnTheWayDistX+maxOnTheWayDistY*maxOnTheWayDistY));

        if(criticalStrip>criticalConnectivityStrip)
            onTheWaySpeed = maxOnTheWayDist/(transmissionRange - criticalStrip);
        else
            onTheWaySpeed = maxOnTheWayDist/(transmissionRange - criticalConnectivityStrip);

        if (strcmp(curFullName,"drone[0]")!=0)
            speed = onTheWaySpeed;

        stationary = speed == 0;
        isInCritical = false;
        timeOf_Last_ShotDown = 0;

        distToLinkerTag = 0;
        WATCH(distToLinkerTag);


        waitingForCriticalSet.isWaiting = false;
        waitingForCriticalSet.onTheWaylastCalledExtraDrone = "NULL";
        waitingForCriticalSet.lastHelpedHost = "NULL";
        waitingForCriticalSet.lastHelpedHostPos = Coord::ZERO;

        faultInfo.imFault=false;
        faultInfo.isWaiting = false;
        faultInfo.faultNum = noFault;
        faultInfo.faulted="NULL" ;
        faultInfo.recognize="NULL" ;
        faultInfo.lastFaultedPlace =Coord::ZERO ;


    }

    //same as cc = MYIdealChannelModel::get(); (in DD)
    cc2 = dynamic_cast<MYIdealChannelModel *>(simulation.getModuleByPath("channelControl"));
    if (!cc2)
        throw cRuntimeError("Could not find MYIdealChannelModel module with name 'channelControl' in the toplevel network.222222222222222222222");
    cc2->Drone_criticalStrip = criticalStrip;

}

//MOSHE: Fault drone recognize check cases
int MYConstSpeedMobilityDrone::handleFault(){
    bool thereIsFault=false;
    int returnValue=noFault;
    MYIdealChannelModel::RadioEntry fault, replacer;
    EV_INFO << "MMMMMMMM MOSHE MMMMMMMM" << endl;
    EV_INFO << "MMMMMMMM fault check start MMMMMMMM" << endl;
    //MOSHE looking for my linker or who i'm linking in order to determine if it is a leaf or not
    for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++){
        if((strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0) && (it->isFault) && (!(strcmp(it->radioModule->getParentModule()->getParentModule()->getFullName(),curFullName) == 0 ))){
            //MOSHE: check if it is drones, not me, fault or off
            if(strcmp(it->radioModule->getParentModule()->getParentModule()->getFullName(),myRadioRef2.linker) == 0 ){
                //MOSHE: check if it is my linker
                thereIsFault=true;
                EV_INFO << "MMMMMMMM it=myRadioRef2.linker MMMMMMMM" << endl;
                bool isLeaf=true;
                MYIdealChannelModel::myRadioList::iterator it2;
                for (it2 = myradios2.begin(); isLeaf && it2 != myradios2.end(); it2++) //check if no other drone use it as linker
                    if(strcmp(it2->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0 && strcmp(it2->linker,myRadioRef2.radioModule->getParentModule()->getParentModule()->getFullName()) == 0 )
                        isLeaf=false;
                if (isLeaf){  //MOSHE: the fault one is leaf ask from CC
                    faultTarget.x=it->pos.x;
                    faultTarget.y=it->pos.y;
                    faultTarget.z=it->pos.z;
                    cc2->setNewHostOwner(myRadioRef2.radioModule->getParentModule()->getParentModule()->getFullName(),it->linker);
                    cc2->recognizeID=myRadioRef2.ID;
                    returnValue=leafDiscover;
                    this->getParentModule()->bubble(" DRONE FAULT RECOGNIZE AS LINKER LEAF");
                    for (it2 = myradios2.begin(); it2 != myradios2.end(); it2++)
                        if(strcmp(it2->radioModule->getParentModule()->getParentModule()->getFullName(),myRadioRef2.radioModule->getParentModule()->getParentModule()->getFullName()) == 0 )
                            it2->linker=it->linker;
                    it->linker="none";
                    EV_INFO << "MMMMMMMM leaf discover MMMMMMMM" << endl;
                }
                else{   //MOSHE: not a leaf find a replacement in its tree
                    if(cc2->inFault==noFault){
                        emitFaultSignal(myRadioRef2.ID);
                        //this->getParentModule()->bubble(" DRONE FAULT RECOGNIZE AS LINKER");
                        EV_INFO << "MMMMMMMM looking for replacement MMMMMMMM" << endl;
                    }
                    cc2->recognizeID=it->ID;
                    returnValue=ReplacedByOther;
                }
            }
            else if (strcmp(it->linker,curFullName) == 0 ){   //it is linked by me
                bool isLeaf=true;
                MYIdealChannelModel::myRadioList::iterator it2;
                for (it2 = myradios2.begin(); isLeaf && it2 != myradios2.end(); it2++){
                    if(strcmp(it2->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0 && strcmp(it2->linker,it->radioModule->getParentModule()->getParentModule()->getFullName()) == 0 )
                        isLeaf=false;
                }
                if (isLeaf){}  //it is leaf otherwise will be notice by its linked by
            }
        }
    }
    EV_INFO << "MMMMMMMM fault check found: " << thereIsFault << " MMMMMMMM" << endl;
    return returnValue;
}

//this is the implementation of the drone position
void MYConstSpeedMobilityDrone::setTargetPosition(){
    Coord SECcenter, currentPosition,newTargetPosition;
    int result= -1;
    bool keepConnectivityOnly = false, coolDownTime =false, goStage1 = false, isDroneOnlyLinker = false;
    int size = -1 ,helpsize,count=0;
    MYIdealChannelModel::myRadioList helpNeighborDroneHostList;
    MYIdealChannelModel::RadioEntry radioEntry_radio3;
    const char *hostFullName,*droneFullName;
    int myID = cc2->detachNumFromFullName2(curFullName);

    mystage++;
    if (mystage<=1){     //BUG FIX
        myRadioRef2.faultHandler=false;
        myRadioRef2.isReplaced=false;
    }
    else{
        setDroneNeigborsList();
        myradios2 = cc2->getRadios();
        myRadioRef2 = cc2->lookupRadioByName(curFullName);
    }

    if(myRadioRef2.ID==0){
        EV_INFO<<endl;
    }

    if(faultStage==noFault && myRadioRef2.is_Drone && mystage > 1 && myRadioRef2.myisActive ){
        myradios2 = cc2->getRadios();
        myRadioRef2 = cc2->lookupRadioByName(curFullName);
        faultStage=handleFault();
    }
    cc2->inFault=faultStage;
    //EV_INFO<<" MY NAME: "<<curFullName<<" my linker: "<<myRadioRef2.linker<<" namecoordframe name: "<<myRadioRef2.nameCoordFrameName<<endl;
    if (myRadioRef2.isReplaced){ //MOSHE: if isreplaced we need to let it go to its new place
        //cc2->setisReplaced(myRadioRef2.ID,false);
        cc2->inFault=endFault;
        faultStage=endFault;
        cc2->setisReplaced(myRadioRef2.ID,false);
        emitExtraDroneToCCSignal(-10);
        cc2->setNameCoordFrameName(curFullName,"NULL");
        targetPosition=cc2->getHostPosition(curFullName);
    }
    //MOSHE: set in next place
    else if(myRadioRef2.isFault || (simulation.getSimTime() >= 90  && strcmp("drone[3]",myRadioRef2.radioModule->getParentModule()->getParentModule()->getFullName()) == 0)) //MOSHE: THIS SHOULD BE THE FIRST BECAUSE FAULT ONE CAN'T DO NOTHING
    {
        faultInfo.imFault=true;
        cc2->setisFault(curFullName, true);
        EV_INFO << "MMMMMMMMMMMMMMMMMMMMMM FAULT!!!!!!!!!!! MMMMMMMMMMMMMMMMMMM"<<endl;
        this->getParentModule()->setDisplayString("p=35,155;i=misc/node,#FF0000;is=vs"); //set new size and color red
        faultInfo.imFault=true;
        cc2->setBestCurrentRmin(curFullName,0);
        cc2->setNameCoordFrameName(curFullName,"NULL");
        if (cc2->getHostPosition(curFullName)==Coord::ZERO)
            cc2->setNewHostOwner(curFullName,"none");
        cc2->setmyisActive(curFullName,false);
        cc2->setDroneOwnedSize(curFullName,0);


        if(timeOf_Last_ShotDown + 0.1 < simulation.getSimTime())
        {
            timeOf_Last_ShotDown = simulation.getSimTime();
            cc2->setDroneOperateTime(cc2->detachNumFromFullName2(curFullName),false);
        }

        speed = onTheWaySpeed;           // SET SPEED back to onTheWaySpeed

        this->getParentModule()->bubble(" DRONE Fault ");
        targetPosition={0,0,0};
    }
    else if(myRadioRef2.isFault || (simulation.getSimTime() >= 80  && strcmp("drone[1]",myRadioRef2.radioModule->getParentModule()->getParentModule()->getFullName()) == 0)) //MOSHE: THIS SHOULD BE THE FIRST BECAUSE FAULT ONE CAN'T DO NOTHING
    {
        faultInfo.imFault=true;
        cc2->setisFault(curFullName, true);
        EV_INFO << "MMMMMMMMMMMMMMMMMMMMMM FAULT!!!!!!!!!!! MMMMMMMMMMMMMMMMMMM"<<endl;
        this->getParentModule()->setDisplayString("p=35,155;i=misc/node,#FF0000;is=vs"); //set new size and color
        faultInfo.imFault=true;
        cc2->setBestCurrentRmin(curFullName,0);
        cc2->setNameCoordFrameName(curFullName,"NULL");
        if (cc2->getHostPosition(curFullName)==Coord::ZERO)
            cc2->setNewHostOwner(curFullName,"none");
        cc2->setmyisActive(curFullName,false);
        cc2->setDroneOwnedSize(curFullName,0);


        if(timeOf_Last_ShotDown + 0.1 < simulation.getSimTime())
        {
            timeOf_Last_ShotDown = simulation.getSimTime();
            cc2->setDroneOperateTime(cc2->detachNumFromFullName2(curFullName),false);
        }

        speed = onTheWaySpeed;           // SET SPEED back to onTheWaySpeed

        //this->getParentModule()->bubble(" DRONE Fault ");
        targetPosition={0,0,0};
    }
    else if (myRadioRef2.faultHandler){     //Keeps everyone on hold while handling a fault
        if (myRadioRef2.sendDroneToNameCoord){
            targetPosition=detachCoordFromNameCoordFrame(myRadioRef2.nameCoordFrameName);
            speed=onTheWaySpeed;
            if (lastPosition==targetPosition){      //did you arive?
                cc2->setNameCoordFrameName(curFullName,"NULL");
                speed=par("speed");
                cc2->setsendDroneToNameCoord(myRadioRef2.ID,false);
                int initID=cc2->detachNumFromFullName2(myRadioRef2.nameCoordFrameName);
                EV_INFO<<"setisReplaced initID="<< initID<<"from coordname"<<myRadioRef2.nameCoordFrameName<<endl;
                cc2->setisReplaced(initID,true);
                cc2->getAndSetExtraDroneLinker(curFullName,cc2->getnextlinker(curFullName));
            }
        }
        else
            targetPosition=cc2->getHostPosition(curFullName);
    }
    else if(cc2->inFault!=noFault && cc2->inFault!=endFault){
        //leaf dicover case
        EV_INFO<<"infault (leafFall=1,leafDiscover=2,ReplacedByOther)"<<cc2->inFault<<endl;
        if (cc2->inFault==leafDiscover && myRadioRef2.ID==cc2->recognizeID ){
            if (targetPosition!=faultTarget){
                targetPosition=faultTarget;
                speed = onTheWaySpeed;
            }
            else {
                targetPosition=cc2->getHostPosition(curFullName);
                cc2->inFault=endFault;
                speed=par("speed");
                faultStage=endFault;
            }
        }
        /* else if(cc2->inFault==leafFall){
            targetPosition=cc2->getHostPosition(curFullName);
            if(faultInfo.isWaiting){
                cc2->inFault=leafFall;
                faultStage=leafFall;
            }
            else{
                cc2->inFault=endFault;
                faultStage=endFault;
            }
        }*/
        else if(cc2->inFault==ReplacedByOther){
            for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++)       //setting all drones to not move until fault fix
                if(strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0){
                    cc2->setfaultHandler(it->radioModule->getParentModule()->getParentModule()->getFullName(),true);
                    cc2->setmoveID(it->radioModule->getParentModule()->getParentModule()->getFullName(),myRadioRef2.moveID);
                }
            targetPosition=cc2->getHostPosition(curFullName);
        }
        else
            targetPosition=cc2->getHostPosition(curFullName);
    }
    else if(mystage > 1){
        //if to be called from outside and not only from MYIdealRadio::receiveSignal
        myradios2 = cc2->getRadios();
        myRadioRef2 = cc2->lookupRadioByName(curFullName);

        setDroneNeigborsList();
        if(myRadioRef2.is_Drone)                                // only if its a drone
            cc2->update_My_Hops_Counts(myRadioRef2);             // update your number of hops from CC

        size = (int)hostNeigborsList.size();
        cc2->setDroneOwnedSize(curFullName,size);       //update new size to let network know
        //MOSHE: statictics and showing lists
        if(myRadioRef2.myisActive)
        {
            // FOR STATISTICS:
            // Only in the start of the simulation, add for the statistics the fact the 1 drone is in the air
            if(firstDroneOnly_For_Statistics)
            {
                cc2->setDroneOperateTime(myID,true);
                firstDroneOnly_For_Statistics = false;
            }

            //MOSHE: set time of operation
            droneTimeOfOperation_new = simulation.getSimTime();
            if(droneTimeOfOperation_new >= droneTimeOfOperation_old +1)
            {
                cc2->setDroneOperateTime(myID,false);
                droneTimeOfOperation_old = droneTimeOfOperation_new;
            }

            // printing all list
//            for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++){
//                if((strcmp(it->linker,"none")!=0)&&(strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"host")!=0))
//                    EV_INFO << "IIIIIIIII myradios2 ALL IIIIIIIII "<< it->radioModule->getParentModule()->getParentModule()->getFullName()<< " & its myisActive is: " << it->myisActive<< "    ,its linker is: " << it->linker <<" in dist :"<<sqrt(it->pos.sqrdist(cc2->getHostPosition(it->linker)))
//                    << " ,its nameCoordFrameName "<< it->nameCoordFrameName << " ,its bestCurrentRmin "<< it->bestCurrentRmin<< " ,its ownerSize "<< it->ownerSize<<endl;
//                else
//                    EV_INFO << "IIIIIIIII myradios2 ALL IIIIIIIII "<< it->radioModule->getParentModule()->getParentModule()->getFullName()<< " & its myisActive is: " << it->myisActive<< " ,its linker is: " << it->linker << " ,its nameCoordFrameName "<< it->nameCoordFrameName << " ,its bestCurrentRmin "<< it->bestCurrentRmin<<endl;
//            }
//            EV_INFO << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII MY NEIGHBORS LISTS IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << endl;
//
//            for (MYIdealChannelModel::myRadioList::iterator it = hostNeigborsList.begin(); it != hostNeigborsList.end(); it++)
//                EV_INFO << "IIIIIIIII MY hostNeigborsList IIIIIIII : "<< it->radioModule->getParentModule()->getParentModule()->getFullName()<< " & its owner is: "<< it->linker << endl;
//            EV_INFO << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII " << endl;

            for (MYIdealChannelModel::myRadioList::iterator it = droneNeigborsList.begin(); it != droneNeigborsList.end(); it++){
                EV_INFO << "IIIIIIIII MY droneNeigborsList IIIIIIII : "<< it->radioModule->getParentModule()->getParentModule()->getFullName()<< " ,its linker is: "<< it->linker << endl;

                // test when criticalExtraDrone reached its 1st pos so now its active and in the droneNeigborsList, and we initialize the waitingForCriticalSet to regular
                // and if its still on the way then won't get in and nothing would be effected (cause extradrone still on the way and NOT active yet -> won't show in droneNeigborsList)
                if(faultInfo.isWaiting||(waitingForCriticalSet.isWaiting &&strcmp(it->radioModule->getParentModule()->getParentModule()->getFullName(),waitingForCriticalSet.onTheWaylastCalledExtraDrone)==0)){
                    EV_INFO<<""<<endl;
                    EV_INFO << "------------------- REGULAR EXTRA DRONE WHICH THIS WAS WAITING 4 "<<waitingForCriticalSet.onTheWaylastCalledExtraDrone<<" ARRIVED  ------------------- " << endl;
                    EV_INFO<<""<<endl;
                    waitingForCriticalSet.isWaiting = false;
                    if(faultInfo.isWaiting)
                        faultInfo.isWaiting=false;
                    waitingForCriticalSet.onTheWaylastCalledExtraDrone = "NULL";
                    waitingForCriticalSet.lastHelpedHost = "NULL";
                    waitingForCriticalSet.lastHelpedHostPos = {0,0,0};
                }
            }
            EV_INFO << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII " << endl;

            for (MYIdealChannelModel::myRadioList::iterator it = ccNeigborsList.begin(); it != ccNeigborsList.end(); it++)
                EV_INFO << "IIIIIIIII MY ccNeigborsList IIIIIIII : "<< it->radioModule->getParentModule()->getParentModule()->getFullName()<<endl;
            EV_INFO << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII " << endl;

        }

        //for next iteration when criticalContenders is recalculated in calcSECPosition
        criticalContenders.clear();

        // Check if the drone has no hosts-> meaning it's a linker and need to use the function "calculate_Linker_Next_Corrd()" , in the end of this function
        if(size == 0 && myRadioRef2.myisActive)
        {
            isDroneOnlyLinker = true;
            //       getParentModule()->getParentModule()->getParentModule()->getParentModule()->getParentModule()->getSubmodule("MUFinalDroneHost",0)->par("UsedAsLinker").setBoolValue(isDroneOnlyLinker);
            cModule * tempModule = getParentModule();
            tempModule->par("UsedAsLinker").setBoolValue(isDroneOnlyLinker);
            SECcenter = calculate_Linker_Next_Corrd();

        }
        else if (size>0 && myRadioRef2.myisActive){
            isDroneOnlyLinker = false;
            cModule * tempModule = getParentModule();
            tempModule->par("UsedAsLinker").setBoolValue(isDroneOnlyLinker);
        }

        // if(cc2->does_drone_imply_centralAlgoritm(myID)) DO NOT GET INSIDE
        if(cc2->drone_imply_centralAlgo_turn_Duplicat[myID] > 0 || cc2->drone_imply_centralAlgo_turn[myID] > 0)
        {
            radioEntry_radio3 = cc2->find_Radio_from_radios3_by_ID(myID);
            size = radioEntry_radio3.ownerSize;
            goStage1 = true;
        }


        // NOTE THAT CRITICAL STATE CAN BE ONLY IN SIZE > 1 !!
        if(size > 1 || (size == 1 && goStage1) )
        {
            this->getParentModule()->setDisplayString("p=35,155;i=misc/node,#004080;is=s;"); //set new size and color blue
            updateConnectivityTag();

            /*
                if extra drone reached first target and now need to ini vars and then set regular 2 host neighbor
                (edge case when 2 critical calls made from same area, and the same extradrone receives ownership on both)
             */
            if (!(strcmp(myRadioRef2.nameCoordFrameName,"NULL")==0)){
                EV_INFO << ""<<endl;
                EV_INFO << "---------------------------------     Entrance Stage 2 REGULAR EXTRA DRONE case (SIZE>2)      ------------------------------- " << endl;
                EV_INFO << "---------------------------------     REGULAR EXTRA DRONE REACHED FISRT TARGET,RESET VARS!(SIZE>2) ------------------------------- " << endl;

                //EV_INFO << "KEEP KEEP       ONTHEWAY SPEED MAKESURE regular   "<< speed <<"   ------------------------------- " << endl;
                speed = par("speed");           //set back regular speed after when myIsActive is set
                //EV_INFO << "KEEP KEEP       REGULAR SPEED MAKESURE regular   "<< speed <<"  ------------------------------- " << endl;

                cc2->setNameCoordFrameName(curFullName,"NULL");       //turn off signal
                cc2->setmyisActive(curFullName,true);

                this->getParentModule()->bubble(" REACHED 1st REGULAR EXTRA DRONE SIZE>2 ");
                //From now this extra drone who reached its first target would be visible to other drone and act as regular state drone
            }

            //MOSHE: checking of drone has host if so find best position in the middle of all host
            if(!isDroneOnlyLinker)  // don't calculate SEC when drone has not host cover
                SECcenter = calcSECPosition();
            //MOSHE: TODO check if take care of linker cover drone



            //This is the critical contendors with Rmin Pair OR Triplet, and if not in critical its also the next move (so isInCritical should be affected by them and them only)
            EV_INFO << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII MY CRITICAL CONTENNDERS IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << endl;
            for (MYIdealChannelModel::myRadioList::iterator it = criticalContenders.begin(); it != criticalContenders.end(); it++){
                EV_INFO << "IIIIIIIII MYConstSpeedMobilityDrone::setTargetPosition FINAL CRITICALCONTENDORS are: "<< it->radioModule->getParentModule()->getParentModule()->getFullName()<< " & its Owner is: "<< it->linker << endl;
            }
            EV_INFO << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII " << endl;
            EV_INFO << " " << endl;

            ////////**********   Start Central Algorithm Code   ***********//////////

            // Is drone currently participate in central algorithm procedure?
            if(cc2->CENTRAL_ALGORITHM_ON && (myRadioRef2.is_In_CentralAlgorithm_Procedure || cc2->drone_imply_centralAlgo_turn[myID] != -1 ))
            {
                MYIdealChannelModel::RadioEntry r_2 = cc2->find_Radio_from_radios3_by_ID(myID);
                newTargetPosition = r_2.pos;
                if(cc2->does_drone_imply_centralAlgoritm(myID))     //Its the turn of the drone to execute central algorithm relocation
                {
                    currentPosition = cc2->getHostPosition(curFullName);
                    if ( cc2->did_Drone_Arrived_To_Target_Position(myID,currentPosition,newTargetPosition) )  //drone arrived to the position set for it
                    {
                        EV_INFO << "Drone has been arrived !!! " << endl;
                        cc2->update_radios(myID);                   // Imply central algorithm: Drone set its linker, take cover on hosts.
                        keepConnectivityOnly = true;
                        if(cc2->is_Drone_Last_ToRelocate(myID))
                        {
                            cc2->Notify_All_Drones_To_Abort_Central_Procedure();
                            cc2->Set_DroneServers_To_NotUse_SEC();
                            cc2->update_Hosts_Linker();
                            cc2->try_Swap_linker_With_Cover_Linker();
                        }
                        else
                        {
                            //bubble("Drone has been reposted");      // Do nothing
                        }
                    }
                    else
                    {
                        keepConnectivityOnly = true;
                        EV_INFO << "Drone is still moving toward the target position !!! " << endl;            // Do nothing
                    }
                }
                else
                {
                    if( cc2->does_Drone_Needs_To_Wait() )   // Does drone waits for the relocations of the other drones ?
                    {
                        keepConnectivityOnly = true;        // if the drone still waits, then it need to stay in its position
                        newTargetPosition = cc2->getHostPosition(curFullName);
                        EV_INFO << "WAIT FOR OTHER DRONES TO RELOCATED THEMSELVES !!! " << endl;
                    }
                    else
                    {
                        if(isInCritical)    // is the drone in central algorithm procedure in critical stage?
                        {
                            if(myRadioRef2.drone_In_BFS_Mode)           //Is drone waiting for reply messages? (BFS mode)
                            {
                                EV_INFO << " DRONE WAIT FOR MESSAGE REPLAYS FROM ITS NEIGHBORS!!! " << endl;     // Do nothing - just Keep connectivity flow and set target to SEC
                            }
                            else
                            {
                                if(myRadioRef2.central_Algo_Succeeded)  //Did drone finished central algorithm with successes?
                                {
                                    //this->getParentModule()->bubble("STARTING BFS TO MY NEIGHBORS");
                                    emitDroneToDroneSignal(0);
                                }
                                else                                    // End central algorithm procedure, next time the drone will start regular critical sectin wil be  zero time
                                {
                                    cc2->end_Central_Algorithm_Procedure(myID);
                                    nextChange = simTime() + 0.001 ;
                                    return;
                                }
                            }
                        }
                        else
                        {
                            // Nothing to do here... Drone is just going to next SEC point
                        }
                    }
                }
            }
            else{    // drone will goto distributed algorithm critical flow:
                ////////**********   END Central Algorithm Code   ***********//////////
                if(isInCritical)          // Drone is in distributed-critical mode flow
                {
                    //this->getParentModule()->bubble( " IN CRITICAL " );
                    helpNeighborDroneHostList = canMyNeighborDronesHelp();               //can neighbor drone help
                    helpsize = (int)helpNeighborDroneHostList.size();

                    EV_INFO <<" "<< endl;
                    //EV_INFO << "ZZZZZZ setTargetPosition ! helpsize (should be 0 || 2) "<< helpsize << endl;

                    // if droneNeigborsList is empty should call next drone from the parking
                    //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  CAN NEIGHBOR HELP END  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
                    EV_INFO <<" "<< endl;

                    if(helpsize != 0 ){
                        /*
                           if != 0 than help can be giving by drone and host in helpNeighborDroneHostList
                           get which drone can help (new global RadioList var - that is set in the canNeighborDroneHelp())
                           in the same way (and can be using the same RadioList to pass) get which host need its ownership changed
                           changing the ownership of the host to the FullName of the drone
                         */

                        //print the drone host pair that can help and set the host owner/linker
                        for (MYIdealChannelModel::myRadioList::iterator it = helpNeighborDroneHostList.begin(); it != helpNeighborDroneHostList.end(); it++){
                            if (count == 0){
                                droneFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();
                                //EV_INFO << "KEEP KEEP        NEIGHBOR DRONE CAN HELP :"<< droneFullName << endl;
                                count ++;
                            }
                            else{
                                hostFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();
                                //EV_INFO << "KEEP KEEP        HOST CAN HELP IIIIIIIII "<< hostFullName << endl;
                            }
                        }

                        EV_INFO << "----------------------- setTargetPosition ! CHANGING OWNERSHIP OF: "<< hostFullName <<" TO BE: "<< droneFullName <<endl;
                        //this->getParentModule()->bubble("CHANGING OWNERSHIP");
                        EV_INFO <<"" <<endl;

                        cc2->setNewHostOwner(hostFullName,droneFullName);
                        cc2->setDroneOwnedSize(curFullName,size-1); //update size realtime
                    }
                    else{
                        /*
                            no help can be given so tell cc that we in critical, choose one host to change ownership (out of criticalContenders),
                            get its new drone (like in findandsetcclinker) but dont set ismyActive for the drone change the ownership of the host
                            with the to be sent extra drone call cc for help with the changed host name (in order for the cc to send the extra drone to its pos)
                         */

                        const char *transferredHostFullName = "NULL";
                        switch(ccChoosingMode){
                        case 0:
                            transferredHostFullName = findFARTHESTTransferredCriticalHost();
                            EV_INFO <<"ZZZZZZ setTargetPosition ! findFARTHESTTransferredCriticalHost MODE "<<transferredHostFullName<<endl;
                            break;
                        case 1:
                            EV_INFO<<""<<endl;
                            transferredHostFullName = findRANDOMTransferredCriticalHost();
                            EV_INFO <<"ZZZZZZ setTargetPosition ! findRANDOMTransferredCriticalHost MODE "<<transferredHostFullName<<endl;
                            break;
                        default:
                            transferredHostFullName = findFARTHESTTransferredCriticalHost();
                            EV_INFO <<"ZZZZZZ setTargetPosition ! default findFARTHESTTransferredCriticalHost MODE "<<transferredHostFullName<<endl;
                            break;
                        }

                        int hostnum = findTransferredHostId(transferredHostFullName);


                        if(!waitingForCriticalSet.isWaiting)
                        {   // make CALL I for regularextradrone case

                            waitingForCriticalSet.isWaiting = true;
                            const char* calledRegularExtraDrone = cc2->findAndSetCCLinker(transferredHostFullName);   //this should be the chosen host that we change its owner/linker (BUT WITH THE NEW DRONE)

                            waitingForCriticalSet.onTheWaylastCalledExtraDrone = calledRegularExtraDrone;
                            waitingForCriticalSet.lastHelpedHost = transferredHostFullName;
                            waitingForCriticalSet.lastHelpedHostPos = cc2->getHostPosition(transferredHostFullName);

                            EV_INFO << "ZZZZZZZZZ WAITING FROM NOW FOR REGULAREXTRADRONE : CALL I To : "<< calledRegularExtraDrone <<"& ITS CALLED FOR: "<<transferredHostFullName<<endl;
                            EV_INFO <<"" <<endl;

                            emitExtraDroneToCCSignal(hostnum);  //pass signal with the Transferred host num as int
                        }
                        else
                        {   //here when waitingForCriticalSet.isWaiting

                            EV_INFO << "ZZZZZZZZZ YES IS WAITING : CALL II"<< endl;
                            EV_INFO << "ZZZZZZZZZ TESTING VARS: waitingForCriticalSet.onTheWaylastCalledExtraDrone is: "<< waitingForCriticalSet.onTheWaylastCalledExtraDrone<<", & its on the way to:"<<waitingForCriticalSet.lastHelpedHost <<endl;
                            EV_INFO << "ZZZZZZZZZ TESTING VARS: waitingForCriticalSet.lastHelpedHostPos "<< waitingForCriticalSet.lastHelpedHostPos<< endl;

                            if(!isAnyCriticalContendorInSameArea(waitingForCriticalSet.lastHelpedHostPos))
                            {   //when no criticalContendors in the same area new regularExtraDrone is needed (we save only last call ), so we update set var's and call drone as in regular case
                                waitingForCriticalSet.isWaiting = true;
                                const char* calledRegularExtraDrone = cc2->findAndSetCCLinker(transferredHostFullName);   //this should be the chosen host that we change its owner/linker (BUT WITH THE NEW DRONE)

                                waitingForCriticalSet.onTheWaylastCalledExtraDrone = calledRegularExtraDrone;
                                waitingForCriticalSet.lastHelpedHost = transferredHostFullName;
                                waitingForCriticalSet.lastHelpedHostPos = cc2->getHostPosition(transferredHostFullName);

                                EV_INFO << "ZZZZZZZZZ NO CRITICAL CONTENDORS FROM SAME AREA -> RESET SET WITH NEW WAITING EXTRADRONE VARS."<< endl;
                                EV_INFO << "ZZZZZZZZZ THE NEW CALLED WAITING REGULAREXTRADRONE is: "<< calledRegularExtraDrone <<" ITS CALLED TO HELP TO: "<<transferredHostFullName<<endl;
                                EV_INFO <<""<<endl;
                                emitExtraDroneToCCSignal(hostnum);  //pass signal with the Transferred host num as int
                            }
                            else
                            {   //this is the case when isAnyCriticalContendorInSameArea so find ONE from this area and change its ownership to be of the extradrone in the way
                                const char * closestccFullName = getClosestCriticalContendorFromSameArea(waitingForCriticalSet.lastHelpedHost,waitingForCriticalSet.lastHelpedHostPos);

                                if(strcmp(closestccFullName,"NULL")==0){
                                    //this if is just in case
                                    EV_INFO << "ZZZZZZZZZ_1 CHANGING OWNERSHIP for: "<< closestccFullName <<" To be: "<<waitingForCriticalSet.onTheWaylastCalledExtraDrone<<endl;
                                    //this->getParentModule()->bubble("CHANGING OWNERSHIP_1");
                                    cc2->setNewHostOwner(transferredHostFullName,waitingForCriticalSet.onTheWaylastCalledExtraDrone);
                                    cc2->setDroneOwnedSize(curFullName,size-1); //update size realtime
                                }
                                else{
                                    EV_INFO << "ZZZZZZZZZ_2 CHANGING OWNERSHIP for: "<< closestccFullName <<" To be: "<<waitingForCriticalSet.onTheWaylastCalledExtraDrone<<endl;
                                    //this->getParentModule()->bubble("CHANGING OWNERSHIP_2");
                                    cc2->setNewHostOwner(closestccFullName,waitingForCriticalSet.onTheWaylastCalledExtraDrone);
                                    cc2->setDroneOwnedSize(curFullName,size-1); //update size realtime
                                }
                                EV_INFO <<"" <<endl;
                            }
                        }
                    }
                    cc2->reset_Central_Algorithm_Procedure(myID);       // Now reset the Central Algorithm Procedure for the next critical stage.
                }   // end if(isInCritical)
            }
            cc2->multiple_Drones_Cover_closed_Group_Of_Hosts(myID);

        }   // end if(size > 1)
        else if(!goStage1){
            if(size == 1){               //keeps following the only host it got
                this->getParentModule()->setDisplayString("p=35,155;i=misc/node,#004080;is=s"); //set new size and color blue
                updateConnectivityTag();

                //if extra drone reached first target and now need to ini vars and then set regular 1 host neighbor , happends when nameCoordFrameName set in MYIdealRadio.cc
                if (!(strcmp(myRadioRef2.nameCoordFrameName,"NULL")==0)){
                    EV_INFO << ""<<endl;
                    EV_INFO << "--------------------------------- Entrance Stage 2 REGULAR EXTRA DRONE case ------------------------------- " << endl;
                    EV_INFO << "--------------------------------- REGULAR EXTRA DRONE REACHED FISRT TARGET,RESET VARS ! ------------------------------- " << endl;

                    speed = par("speed");           //set back regular speed after when myIsActive is set
                    for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++)       //setting all drones to  move because fault fix
                        if(strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0 )
                            cc2->setfaultHandler(it->radioModule->getParentModule()->getParentModule()->getFullName(),false);

                    //hila & tslil
                     EV_INFO<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Entrance Stage 2 REGULAR EXTRA DRONE case TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"<<endl;


                    cc2->setNameCoordFrameName(curFullName,"NULL");       //turn off signal
                    cc2->setmyisActive(curFullName,true);
                    const char * myNewLinker = cc2->getLinker(myRadioRef2.linker);      //get out the linker (b4 the call) of the drone who called connectivity extra drone

                    //Hila & Tslil
                    char * droneAndLinker=strdup("regular");
                    strcat(droneAndLinker,myNewLinker);
                    print_statistics_time();
                    emitEstablishSubGraphSignal(droneAndLinker);                        //send "msgEstablishSubGraph" HILA & TSLIL
                    change_true_statistics();

                    this->getParentModule()->bubble("REACHED 1st REGULAR EXTRA DRONE ");
                    //this->getParentModule()->bubble("I've changed my linker");

                    //From now this extra drone who reached its first target would be visible to other drone and act as regular state drone
                }

                // WHEN size == 1, we ALWAYS (MAKESURE) want to test if we can pass the 1 host to neighbor drone and be OFF or regular LINKER
                EV_INFO <<""<<endl;
                //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  TESSTING PASSING 1 HOST START  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
                EV_INFO <<""<<endl;
                const char *testedOneHostToPassFullName = hostNeigborsList.begin()->radioModule->getParentModule()->getParentModule()->getFullName();
                const char *canPassResult = canPassHostToNeighborDrone(testedOneHostToPassFullName);
                //EV_INFO << "KEEP KEEP        testedOneHostToPassFullName is: "<< testedOneHostToPassFullName <<" canPassResult is: "<< canPassResult <<"---------------------------------"<<endl;

                //if canPassResult == "NULL" do the regular case else set it to be (the helping drone) new onwer of the 1 host we had and
                //  update drone to be linker (can set this step SECcenter to stay foot and from the next it would be with size == 0 and handled there)
                if (strcmp(canPassResult,"NULL")==0 ){
                    //  no help -> do the regular case been done until now

                    //EV_INFO <<" ZZZZZZ KEEP FOLLOWING 1 HOST -> NO NEIGBOR DRONE TO PASS TOO "<<endl;
                    double distToHost = sqrt(hostNeigborsList.begin()->pos.sqrdist(myRadioRef2.pos));
                    cc2->setBestCurrentRmin(curFullName,distToHost);

                    //regular following 1 hostNeighbor
                    SECcenter = hostNeigborsList.begin()->pos;
                    //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  TESSTING PASSING 1 HOST END  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
                    EV_INFO <<"" <<endl;
                }
                else{     //this is the case we get the name of helping neighbor drone in canPassResult, and want to set it as the new owner of the host we had
                    // FIXME- when in ini numOfHost == 10 problem when passing to drone[5] and then it gives back - around 60 sec +

                    //EV_INFO <<"ZZZZZZ FOUND NEIGBOR DRONE TO PASS TOO THE ONLY HOST THIS DRONE HAVE "<<endl;
                    EV_INFO <<"---------------------------- CHANGING OWNERSHIP OF: "<< testedOneHostToPassFullName <<" TO BE: "<< canPassResult <<endl;
                    //this->getParentModule()->bubble("CHANGING OWNERSHIP : PASSING 1 Host");

                    cc2->setNewHostOwner(testedOneHostToPassFullName,canPassResult);
                    cc2->setBestCurrentRmin(curFullName,0);
                    cc2->setDroneOwnedSize(curFullName,size-1); //update size realtime

                    //EV_INFO <<"ZZZZZZ CHANGING THIS ("<<curFullName <<") TO LINKER MODE - STEP 1 "<<endl;

                    // set this step pos to be the same -> from next step would be LINKER and could decide if leaf linker or not and handle accordingly
                    SECcenter = myRadioRef2.pos;
                    //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  TESSTING PASSING 1 HOST END  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
                    EV_INFO <<"" <<endl;
                }
            }
            else{       //size==0 in this hostNeigborsList

                if (!(strcmp(myRadioRef2.nameCoordFrameName,"NULL")==0)){       //case when nameCoordFrameName just get set in MYIdealRadio.cc

                    SECcenter = detachCoordFromNameCoordFrame(myRadioRef2.nameCoordFrameName);  // just set SECcenter and its ready to go to its new host.
                    EV_INFO << ""<<endl;
                    EV_INFO << "---------------------------- Setting First SECcenter ever: "<< SECcenter << "    --------------------------- " << endl;
                    EV_INFO << ""<<endl;

                    bool isExtraDroneTypeConnectivityVar = isExtraDroneTypeConnectivity(myRadioRef2.nameCoordFrameName);
                    bool isReplace= isReplacedrone(myRadioRef2.nameCoordFrameName);
                    //detach from "myRadioRef2.nameCoordFrameName" the name part and then if isExtraDroneTypeConnectivity == true
                    if(isExtraDroneTypeConnectivityVar)   //this is the connectivityExtraDrone case (and not the regular extradrone case) first time it sets its target
                    {
                        cc2->setNameCoordFrameName(curFullName,"NULL");      //THIS
                        EV_INFO << "---------------------------- Stage 1 CONNECTIVITY EXTRADRONE (reset setNameCoordFrameName) ------------------------------- " << endl;
                    }
                    /*else if(isReplace){

                    }*/
                    else{   //In regular extra drone case no need to reset 'myRadioRef2.nameCoordFrameName' now, it resets when reaching first target ( in the phase where size == 1 )
                        EV_INFO << "---------------------------- Stage 1 REGULAR EXTRADRONE ------------------------------- " << endl;
                    }
                    EV_INFO << ""<<endl;
                    this->getParentModule()->bubble(" SET 1st ANY EXTRA DRONE TARGET ");
                }
                else{
                    // case where this is ConnectivityExtraDrone first target reach so need to final config it b4 turns regular
                    if (!(strcmp(myRadioRef2.linker,"none")==0) && !(myRadioRef2.myisActive))
                    {
                        EV_INFO << "" << endl;
                        EV_INFO << "--------------------------------- Stage 2 CONNECTIVITY EXTRADRONE      ------------------------------- " << endl;
                        EV_INFO << "-----------------------  CONNECTIVITY EXTRADRONE REACHED FISRT TARGET,RESET VARS !  ------------------------------- " << endl;
                        //this->getParentModule()->bubble(" REACHED 1st CONNECTIVITY EXTRA DRONE ");

                        speed = par("speed");           //set back regular speed after when myIsActive is set

                        EV_INFO<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Stage 2 CONNECTIVITY EXTRADRONE TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"<<endl;


                        cc2->setmyisActive(curFullName,true);
                        //set the linkers according to the new situation (connectivity drone reaching it dest and now par of the game)
                        const char * myNewLinker = cc2->getLinker(myRadioRef2.linker);      //get out the linker (b4 the call) of the drone who called connectivity extra drone

                        char * droneAndLinker=strdup(myRadioRef2.linker);
                        strcat(droneAndLinker,myNewLinker);
                        cc2->setNewHostOwner(myRadioRef2.linker,curFullName);                   //set curFullName(this) as the new linker of the caller to connectivity
                        cc2->setNewHostOwner(curFullName,myNewLinker);                      //set myRadioRef2.linker to be the one that the connectivity gap we are closing
                        print_statistics_time();
                        emitEstablishSubGraphSignal(droneAndLinker);                        //send "msgEstablishSubGraph" HILA & TSLIL
                        change_true_statistics();
                    }

                    // FROM NOW ON THE LINKER WOULD CALC ACCORDING TO MIDDLE OF ITS LINKER AND THE DRONE HE IS THE LINKER OF (except when just was in Entrance Stage 2)
                    if(myRadioRef2.myisActive)    //this is do differ between linker with size==0(now as regular drone linker - with myisActive == true) and the case where drone is in OFF mode
                    {
                        SECcenter = calculate_Linker_Next_Corrd();

                    }
                    else
                    {
                        this->getParentModule()->setDisplayString("p=35,155;i=misc/node,#FF0000;is=vs"); //set new size and color -red

                 //       EV_INFO << "---------------------------------          OFF DRONE CASE         --------------------------------- " << endl;

                        SECcenter = myRadioRef2.pos;         // when in OFF mode just need to stay in the same place
                    }
                }
            }
        }
        MYIdealChannelModel::RadioEntry it=cc2->lookupRadioByName(curFullName);
        if(it.stisticsFlag==true){
            print_statistics_hop_count();
            MYIdealChannelModel::RadioEntry *r;
            r =  cc2->lookupRadio(it.radioModule);
            r->stisticsFlag=false;
        }
        if(cc2->noSEC_CountDown[myID] > 0 && isInCritical == false)
        {
            int droneToShotDown;
            cc2->noSEC_CountDown[myID] --;
            newTargetPosition = cc2->getHostPosition(curFullName);

            droneToShotDown = cc2->swap_Linker_Ownership_while_waiting(myID);
            if(droneToShotDown != -1)
            {
                char tempString[12]={0};
                sprintf(tempString,"drone[%d]",droneToShotDown);

                cc2->setBestCurrentRmin(tempString,0);
                cc2->setNameCoordFrameName(tempString,"NULL");
                cc2->setNewHostOwner(tempString,"none");
                cc2->setmyisActive(tempString,false);
                cc2->setDroneOwnedSize(tempString,0);

                cc2->setNotActiveDroneToMoveToCC(droneToShotDown,true);
            }
            cc2->swap_Hosts_Ownership_while_waiting(myID);    // if needed, swap drone's ownership over hosts


            keepConnectivityOnly = true;            // keep the drone is the same spot
            updateConnectivityTag();
        }
        if(myRadioRef2.sendDroneToCC == true)
        {
            newTargetPosition = {0,0,0};
            keepConnectivityOnly = true;    // make sure that the drone will set next target to CC (0,0,0)
            cc2->setNotActiveDroneToMoveToCC(myID,false);
            speed = onTheWaySpeed;           // SET SPEED back to onTheWaySpeed
        }

        // IF this is imply of the Central Algorithm then set the next position to be as it was calculated there, if not set it as the regular flow:
        if(keepConnectivityOnly)
            targetPosition = newTargetPosition;
        else
            targetPosition = SECcenter;       //new target position is calculated when "reached target position"
    }       //mystage > 1
    else//if stage==0||1
        targetPosition = {par("initialX"),par("initialY"),0};  //for fixing the initial position calc bug



    //MOSHE: check anf fix if in NFZ
    //********************ADDED BY TAMIR****************
    int NFZnumber = isAtNFZ(targetPosition.x,targetPosition.y);
    if (NFZnumber != -1){
        //getParentModule()->bubble("---------------- CALCULATING TARGET POSITION AROUND No-Fly-Zone!!! ----------------");
       // EV_INFO << "---------------- CALCULATING TARGET POSITION AROUND No-Fly-Zone!!! ----------------" << endl;
        targetPosition = calculate_TP_Around_NFZ(NFZnumber, targetPosition);     //TP= Target Position
    }
    //**************************************************


    //MOSHE: set time to next change
    Coord positionDelta = targetPosition - lastPosition;
    double distance = positionDelta.length();

    if (distance == 0)  //can't be 0 cuz stackoverflow
        nextChange = simTime() + distance / speed + double(0.1);
    else if(coolDownTime)
    {
        coolDownTime = false;
        nextChange = simTime()+0.2 + distance / speed ;
    }
    else
        nextChange = simTime() + distance / speed ;
}

//MOSHE: return coord for linker new spot (can return to park of no need)
Coord MYConstSpeedMobilityDrone::calculate_Linker_Next_Corrd(){
    Coord myLinkerPos = cc2->getHostPosition(myRadioRef2.linker);
    const char *imLinkedByFullName = "NULL";
    Coord imLinkedByPos,SECcenter;
    bool isLinkerLeafCase = true;  // representing LinkerLeafCase and set as true if it is a leaf linker
    bool dontSetOFF = false,faulted=false;
    //bool isRed=checkIfDroneIsRed();
    this->getParentModule()->setDisplayString("p=35,155;i=misc/node,#FFFF00;is=s"); //set new size and color yellow
    updateConnectivityTag();

    EV_INFO << "" << endl;
    EV_INFO << "ZZZZZZ LINKER DRONE CASE " << endl;

    //find whos linker am i and get it coord inorder to calc
    for (MYIdealChannelModel::myRadioList::iterator it = droneNeigborsList.begin(); it != droneNeigborsList.end(); it++){
        if( (strcmp(it->linker,curFullName) == 0) ){

            imLinkedByFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();
            imLinkedByPos = cc2->getHostPosition(imLinkedByFullName);

            isLinkerLeafCase = false;           //not leaf case if this is even someone's linker
            break;
        }
        // this is the case when myRadioRef2 is NOT linker of anyone(it is a leaf in the connectivity chain) and need to initialize imLinkedByPos
        //  in order to use it in the if test before setting what to do with a linker(size =0) which is a leaf -> can set OFF and back to park
        isLinkerLeafCase = true;        //set as true if it is a leaf linker (if none reached break; only then true)
    }

    EV_INFO << "ZZZZZZ LINKER DRONE DATA : MY linker is: "<< myRadioRef2.linker <<" & imLinkedByFullName: "<< imLinkedByFullName <<" isLinkerLeafCase is: "<< isLinkerLeafCase <<" ,Thier MIDPOINT is: ("<< (myLinkerPos.x + imLinkedByPos.x)/2<<","<<(myLinkerPos.y + imLinkedByPos.y)/2 <<",0)" << endl;
    MYIdealChannelModel::myRadioList::iterator fault;
    for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++){
        if((strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0) && it->isFault && (!(strcmp(it->radioModule->getParentModule()->getParentModule()->getFullName(),curFullName) == 0 ))){
            if (strcmp(it->linker,curFullName) == 0 ){   //it is linked by me
                fault=it;
                bool isLeaf=true;
                MYIdealChannelModel::myRadioList::iterator it2;
                for (it2 = myradios2.begin(); isLeaf && it2 != myradios2.end(); it2++){
                    if(strcmp(it2->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0 && strcmp(it2->linker,it->radioModule->getParentModule()->getParentModule()->getFullName()) == 0 ){
                        //EV_INFO<<"I'M FALLING: "<<it->radioModule->getParentModule()->getParentModule()->getFullName()<<" , I'M the leaf: "<<it2->radioModule->getParentModule()->getParentModule()->getName()<< " , mylinker is: "<<it2->linker <<endl;
                        isLeaf=false;
                    }
                }
                if (isLeaf){
                    faulted=true;
                    faultInfo.isWaiting=true;
                    //EV_INFO<<"I'M FALLING No leaf: "<<it->radioModule->getParentModule()->getParentModule()->getFullName()<<endl;
                    //initiate request while stay in place
                }
            }
        }
    }
    if(faulted){ //case fault in leaf
        int i;
        char firststr[10], secondstr[10];
        char  *replaceHost = (char*)malloc(sizeof(char)*128), *fullname = (char*)malloc(sizeof(char)*128), *cordFrameName=(char*)malloc(sizeof(char)*200);
        Coord tempCoord=fault->pos;
        sprintf(replaceHost,"host[%d]",cc2->findMyHostId(fault->radioModule->getParentModule()->getParentModule()->getFullName()));
        sprintf(fullname,"drone[%d]",myRadioRef2.ID);
        SECcenter=lastPosition;
        sprintf(firststr,"%.3f",tempCoord.x);
        sprintf(secondstr,"%.3f",tempCoord.y);
        char coordFrame[200];
        strcpy (coordFrame,fullname);
        strcat (coordFrame,replaceHost);
        strcat (coordFrame,firststr);
        strcat (coordFrame,"]");
        strcat (coordFrame,secondstr);
        strcat (coordFrame,"]");

        for(i=0;i<128;i++)
            cordFrameName[i]=0;
        i=0;
        while(coordFrame[i]!= 0){
            cordFrameName[i]=coordFrame[i];
            i++;
        }
        cc2->setNameCoordFrameName(fullname,cordFrameName);
        cc2->setisReplaced(myRadioRef2.ID,true);
        for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++)       //setting all drones to not move until fault fix
            if(strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0)
                cc2->setfaultHandler(it->radioModule->getParentModule()->getParentModule()->getFullName(),true);

    }
    else if(waitingForCriticalSet.isWaiting){
        SECcenter=cc2->getHostPosition(curFullName);
    }
    else if(!isLinkerLeafCase){

        //EV_INFO << "---------------------------------           REGULAR LINKER CASE !          --------------------------------- " << endl;
        //////////////////////////////////////////// test redundancy - when not leaf try to swap who imLinkedBy /////////////////////////////////////////////////////////////////////////

        //swap all you can && if at least one drone still need to be linked by this (myRadioRef2 drone), take the 1st and use him as LINKER from now on
        const char * myStillImLinkedByFullName = swapAllToRedundanteLinker();

        dontSetOFF = is_ServerDrone_On_Action();

        //dontSetOFF = true;          //ADDED BY TAMIR!!! DELETE IT AFTER TESTING

        if(strcmp(myStillImLinkedByFullName,"NULL")==0 && !dontSetOFF){
            EV_INFO << "------------------1        JUST TURNED TO LINKER LEAF CASE ! -> Park it (OFF MODE FROM NEXT STEP)        ------------------- " << endl;
            EV_INFO << ""<<endl;
            SECcenter = setDroneOff(curFullName);   //always returns {0,0,0} its parking coord
        }
        else{
            //still LINKER, go to the MidPoint of mylinker and who 1st i'm still linked by(one we received as the result)
            //recalculate here new imStillLinkedByPos (is the 1st imLinkedBy drone out of all drones imLinkedBy)

            Coord imStillLinkedByPos = cc2->getHostPosition(myStillImLinkedByFullName);
            SECcenter = {(myLinkerPos.x + imStillLinkedByPos.x)/2,(myLinkerPos.y + imStillLinkedByPos.y)/2,0};
        }
        //////////////////////////////////////////// test redundancy /////////////////////////////////////////////////////////////////////////
    }
    else if(!dontSetOFF){
        // this is linker(size==0) which is a leaf -> can set OFF and back to park.so need to reset vars. when would arrive would be as OFF with OFF vars
        EV_INFO << "------------------2        LINKER LEAF CASE ! -> Park it (OFF MODE FROM NEXT STEP)        ------------------- " << endl;
        EV_INFO << ""<<endl;
        SECcenter = setDroneOff(curFullName);   //always returns {0,0,0} its parking coord
    }

//    if(isRed==true){
//        emitEstablishSubGraphSignal(myRadioRef2.ID);
//    }
    return SECcenter;
}


bool MYConstSpeedMobilityDrone::is_Drone_Is_Linker_Waiting_For_Relocation()   {
    MYIdealChannelModel::RadioEntry r = cc2->lookupRadioByName(curFullName);
    if(r.ownerSize <= 0)
    {
        MYIdealChannelModel::RadioEntry r2 = cc2->find_Radio_from_radios3_by_ID(r.ID);
        if(r2.ownerSize > 0)
        {
            if( cc2->drone_imply_centralAlgo_turn[r.ID] > 0)
                return true;
        }
    }
    return false;
}

//MOSHE: return true if the drone has at least one host
bool MYConstSpeedMobilityDrone::is_ServerDrone_On_Action()  {
    for (MYIdealChannelModel::myRadioList::iterator it = cc2->radios3.begin(); it != cc2->radios3.end(); ++it)
    {
        if(it->is_Drone && it->ID == myRadioRef2.ID && it->ownerSize > 0)
            return true;
    }
    return false;
}

//MOSHE: check if need to change linker
//testing if return NULL or name of 1st that imLinkedBy but can't be swapped and according to it would calc the SECcenter later (cuz still linker)
const char* MYConstSpeedMobilityDrone::swapAllToRedundanteLinker(){

    bool isAllSwapped = true;
    const char * firstCantSwapFullName = "NULL";

    for (MYIdealChannelModel::myRadioList::iterator it = droneNeigborsList.begin(); it != droneNeigborsList.end(); it++){

        // handle ALL drones who imLinkedBy
        if( (strcmp(it->linker,curFullName) == 0) ){

            const char *currImLinkedByFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();
            Coord currImLinkedByPos = it->pos;

            double currRredundantdist = cc2->getTransmissionRange(currImLinkedByFullName) - criticalConnectivityStrip - strengthStripDelta ;    //get tested R

            Coord myLinkerPos = cc2->getHostPosition(myRadioRef2.linker);
            double sqrdistBetweenSwaped = myLinkerPos.sqrdist(currImLinkedByPos);   //get dist between candidates for the swap in order to test if can

            EV_INFO <<""<<endl;
            //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  LINKER REDUNDANTCY TEST START ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
            EV_INFO <<""<<endl;
            //EV_INFO <<"ZZZZZZ swapAllToRedundanteLinker ! CURR REDUNDANTCY TEST MY linker is: " <<myRadioRef2.linker <<" VS "<<currImLinkedByFullName<<" (currImLinkedByFullName)"<<endl;
            //EV_INFO <<"ZZZZZZ swapAllToRedundanteLinker ! REDUNDANTCY DIST TEST (CONST) is: " <<currRredundantdist <<",& CURR SWAP DIST TESTED (COMPARE 2 CONST) is: "<<sqrt(sqrdistBetweenSwaped)<<endl;

            // test for dist between
            if(sqrdistBetweenSwaped < currRredundantdist*currRredundantdist){
                //EV_INFO <<"ZZZZZZ swapAllToRedundanteLinker ! CAN SWAP ! Swapping linker of: "<<currImLinkedByFullName <<" too be "<< myRadioRef2.linker<<" instead of this drone"<<endl;
                //totalSwappedCount++;
                cc2->setNewHostOwner(currImLinkedByFullName,myRadioRef2.linker);    //SWAP - set my linker to be 'imLinkedByFullName' new linker (and it sould use it for connectivity)
            }
            else{
                //EV_INFO <<"ZZZZZZ swapAllToRedundanteLinker ! CAN'T SWAP ! "<<endl;
                // save 1st drone who does not match and keep testing all the rest regularly, and when done -> return that 1st one
                if(isAllSwapped){
                    isAllSwapped = false;
                    firstCantSwapFullName = currImLinkedByFullName;
                    //EV_INFO <<"ZZZZZZ swapAllToRedundanteLinker ! CAN'T SWAP -> Not redundant, linked by : "<<firstCantSwapFullName<<endl;
                }
            }
        }
        //not linked by zone
    }//end for
//    EV_INFO <<""<<endl;
//    EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  LINKER REDUNDANTCY TEST END  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
//    EV_INFO <<""<<endl;
    return firstCantSwapFullName;

}


/* reset all needed vars to set the drone to parking and turn to off mode there */
Coord MYConstSpeedMobilityDrone::setDroneOff(const char * curFullName){



    cc2->setBestCurrentRmin(curFullName,0);
    cc2->setNameCoordFrameName(curFullName,"NULL");
    cc2->setNewHostOwner(curFullName,"none");
    cc2->setmyisActive(curFullName,false);
    cc2->setDroneOwnedSize(curFullName,0);


    if(timeOf_Last_ShotDown + 0.1 < simulation.getSimTime())
    {
        timeOf_Last_ShotDown = simulation.getSimTime();
        cc2->setDroneOperateTime(cc2->detachNumFromFullName2(curFullName),false);
    }

    speed = onTheWaySpeed;           // SET SPEED back to onTheWaySpeed

    //this->getParentModule()->bubble(" DRONE OFF ");
    return {0,0,0};
}

bool MYConstSpeedMobilityDrone::isAnyCriticalContendorInSameArea(Coord preCallHostPos){
    /*
         here we just want to know iF there is at least one (in this call 2) criticalcontendor who is with in the Rarea
         means we need to have the criticalContendorsLIST and to compare if(distsqtr between current iteratorPos and preCallHostPos >?< Rarea^2)
         if Rarea bigger then we return true on the spot(still in the for loop), else we finish testing all and if none we return false
     */
    EV_INFO <<"" <<endl;
    for (MYIdealChannelModel::myRadioList::iterator it = criticalContenders.begin(); it != criticalContenders.end(); it++){
        if(preCallHostPos.sqrdist(it->pos)<RwhenExDroneOnTheWay*RwhenExDroneOnTheWay){
            EV_INFO << "ZZZZZZ isAnyCriticalContendorInSameArea ! FOUND CRITICALCONTENDOR "<<it->radioModule->getParentModule()->getParentModule()->getFullName()<<" IN SAME AREA "<< endl;
            return true;
        }
    }
    EV_INFO << "ZZZZZZ isAnyCriticalContendorInSameArea ! NO CRITICALCONTENDORS IN SAME AREA AT ALL "<< endl;
    return false;
}


const char *MYConstSpeedMobilityDrone::getClosestCriticalContendorFromSameArea(const char *preCallHostFullName, Coord preCallHostPos){
    /*
        when there is from same area (NOW), we want to find the closest from the same area in this CALL II
        so we need to find the closest criticalcontendor from this area (most of the times it would be only 1 cc, and in edge cases can be even 2 cc's)
        so we need to know the previous called for host pos and find from criticalContendorsLIST if in the Rarea and then for each in Rarea save closest
        and if more then 1 compare(compare the in Rarea cc's dist to the previously called host pos->save closest) and return closest FullName
     */
    double bestClosestDistsqrt=0,currDistsqrtToCCInArea;
    const char * bestClosestHostInAreaName = "NULL";

    for (MYIdealChannelModel::myRadioList::iterator it = criticalContenders.begin(); it != criticalContenders.end(); it++)
    {
        currDistsqrtToCCInArea = preCallHostPos.sqrdist(it->pos);

        if(currDistsqrtToCCInArea < RwhenExDroneOnTheWay*RwhenExDroneOnTheWay)
        {
            if (bestClosestDistsqrt == 0){          //this would be the first criticalcontendor found within RwhenExDroneOnTheWay so we save it anyway
                bestClosestDistsqrt = currDistsqrtToCCInArea;
                bestClosestHostInAreaName = it->radioModule->getParentModule()->getParentModule()->getFullName();
            }
            else
            {   //this is when we have more that 1 criticalcontendor within RwhenExDroneOnTheWay (means this iteraton is the 2nd (or 3rd?) that within RwhenExDroneOnTheWay)
                // so now we need to compare inorder to find the best to choose means the closest whitin the area

                if (bestClosestDistsqrt>currDistsqrtToCCInArea){
                    bestClosestDistsqrt = currDistsqrtToCCInArea;
                    bestClosestHostInAreaName = it->radioModule->getParentModule()->getParentModule()->getFullName();
                }
            }
        }
    }
    return bestClosestHostInAreaName;
}

//MOSHE: get number from host name
int MYConstSpeedMobilityDrone::findTransferredHostId(const char *hostFullName){
    char fullNamestr[80];
    const char * charVal;
    int intVal;
    char * pch;

    strcpy(fullNamestr,hostFullName);
    pch = strtok (fullNamestr,"[");
    pch = strtok (NULL, "]");
    charVal = pch;

    intVal = atoi(charVal);

    return intVal;

}

const char *MYConstSpeedMobilityDrone::findFARTHESTTransferredCriticalHost(){
    const char * curTransferredMaxFullName;
    double cursqrdist, maxsqrdist = 0;

    Coord linkerCoord = cc2->getHostPosition(myRadioRef2.linker);

    for (MYIdealChannelModel::myRadioList::iterator it = criticalContenders.begin(); it != criticalContenders.end(); it++){

        cursqrdist = linkerCoord.sqrdist(it->pos);

        if(maxsqrdist < cursqrdist){
            maxsqrdist = cursqrdist;
            curTransferredMaxFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();
        }
    }
    return curTransferredMaxFullName;
}


const char *MYConstSpeedMobilityDrone::findRANDOMTransferredCriticalHost(){

    int size = (int)criticalContenders.size();
    int randomContender;
    const char * curTransferredRANDFullName=NULL;
    int i = 1;

    randomContender = rand() % size + 1;  //from 1 - size (originally returns size - 1)
    EV_INFO<<"ZZZZZZ (CONST) randomContender is: "<<randomContender<<endl;

    for (MYIdealChannelModel::myRadioList::iterator it = criticalContenders.begin(); it != criticalContenders.end(); it++){
        if(randomContender==i)
            return it->radioModule->getParentModule()->getParentModule()->getFullName();
        else
            i++;
    }
    return curTransferredRANDFullName;
}

// this actually just the name (drone/host) and not fullname (drone[?]/host[?])
bool MYConstSpeedMobilityDrone::isExtraDroneTypeConnectivity(const char *nameCoordFrameName)
{
    char nameCoordFrameNamestr[80];
    char * pch;

    strcpy(nameCoordFrameNamestr,nameCoordFrameName);
    pch = strtok (nameCoordFrameNamestr,"[");

    if (strcmp(pch,"drone")==0)
        return true;

    return false;

}

bool MYConstSpeedMobilityDrone::isReplacedrone(const char *nameCoordFrameName)
{
    char nameCoordFrameNamestr[80];
    char * pch;

    strcpy(nameCoordFrameNamestr,nameCoordFrameName);
    pch = strtok (nameCoordFrameNamestr,"[");

    if (strcmp(pch,"host")==0)
        return true;

    return false;

}

//MOSHE: get coord from coord string
Coord MYConstSpeedMobilityDrone::detachCoordFromNameCoordFrame(const char *nameCoordFrameName)
{
    Coord coord;
    char nameCoordFrameNamestr[80];
    const char * charCoordx;
    const char * charCoordy;
    char * pch;
    int count1 = 0;

    strcpy(nameCoordFrameNamestr,nameCoordFrameName);
    pch = strtok (nameCoordFrameNamestr,"]");

    while (pch != NULL)
    {
        if (count1==1)
            charCoordx = pch;
        if (count1==2)
            charCoordy = pch;

        pch = strtok (NULL, "]");
        count1++;
    }

    coord.x = atof(charCoordx);
    coord.y = atof(charCoordy);

    return coord;
}

/*
    need to return:if help is given : a list of two radioEntry elements in the helpDroneHostList 1'st drone and 2'nd the host (helper drone and passed host),
    if no help is given should not touch helpDroneHostList and it would be return empty
 */
MYIdealChannelModel::myRadioList MYConstSpeedMobilityDrone::canMyNeighborDronesHelp(){

    MYIdealChannelModel::myRadioList helpDroneHostList;
    DroneHostRminSet currBestHelpingNeighborSet , bestHelpingNeighborSet;

    //initialize to find current closest host tested for specific drone
    currBestHelpingNeighborSet.droneHelping ="NULL";
    currBestHelpingNeighborSet.hostHelped ="NULL";
    currBestHelpingNeighborSet.newTogetherRminsqrt=0;

    //initialize to find Best drone that can help, to initiating drone (which is this and in the critical state)
    bestHelpingNeighborSet.droneHelping ="NULL";
    bestHelpingNeighborSet.hostHelped ="NULL";
    bestHelpingNeighborSet.newTogetherRminsqrt=0;

    EV_INFO << "############ INCRITICAL canMyNeighborDroneHelp ########## "<< endl;
    EV_INFO << ""<<endl;

    for (MYIdealChannelModel::myRadioList::iterator it = droneNeigborsList.begin(); it != droneNeigborsList.end(); it++)
    {
        // initialize var's pear drone
        const char *currDroneNeighbor = it->radioModule->getParentModule()->getParentModule()->getFullName();
        double minCurrsqrdist = 7777777;
        double currsqrdist;

        //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  CAN NEIGHBOR HELP START  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
        //find the best helping host current drone (checked from droneNeigborsList), HERE WE NEED TO FIND THE CLOSEST CRITICALCONTENDOR
        for (MYIdealChannelModel::myRadioList::iterator it1 = criticalContenders.begin(); it1 != criticalContenders.end(); it1++)
        {
            const char *currHostCriticalContendor = it1->radioModule->getParentModule()->getParentModule()->getFullName();

            currsqrdist = it->pos.sqrdist(it1->pos);
            //EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! CURRENT TESTED DRONE-HOST PAIR: "<< currDroneNeighbor<< "-"<< currHostCriticalContendor<< " ,where dist between them is: "<< sqrt(currsqrdist)<< endl;

            //must enter at least in the first pair and in the 2nd or 3rd time (if 3rd exist, in case of TRIPLET) its depends the sqrtdist
            if(currsqrdist < minCurrsqrdist){

                minCurrsqrdist = currsqrdist;
                currBestHelpingNeighborSet.droneHelping = currDroneNeighbor;
                currBestHelpingNeighborSet.hostHelped = currHostCriticalContendor;
                currBestHelpingNeighborSet.newTogetherRminsqrt = minCurrsqrdist;
            }

        }//end criticalContendors loop

        //after finding best helping pair from current droneNeigborsList we CHECK IF CURRENT DRONE CAN HELP AS LEAST TO HIM
        bool canDroneHelp = isDroneCanHelpWithHostCover(currBestHelpingNeighborSet);

        if(canDroneHelp)
        {
            //when we are here it means drone can help to its paired host according to strip check so now also
            //we want to check if 'currBestHelpingNeighborSet.newTogetherRminsqrt' is better then pairs we saw so far and update 'bestHelpingNeighborSet' accordingly.

//            EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! FINDING BEST DRONE HELPER !! "<< endl;
//            EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! currBestHelpingNeighborSet.newTogetherRminsqrt "<<currBestHelpingNeighborSet.newTogetherRminsqrt<<" , & Real dist is:"<<sqrt(currBestHelpingNeighborSet.newTogetherRminsqrt)<< endl;
//            EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! bestHelpingNeighborSet.newTogetherRminsqrt "<<bestHelpingNeighborSet.newTogetherRminsqrt<<" , & Real dist is:"<<sqrt(bestHelpingNeighborSet.newTogetherRminsqrt)<< endl;

            // if its the first drone which can help, we push 1st helper anyway
            if(bestHelpingNeighborSet.newTogetherRminsqrt == 0)
            {
                //EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! FIRST "<< endl;
                bestHelpingNeighborSet.droneHelping = currBestHelpingNeighborSet.droneHelping;
                bestHelpingNeighborSet.hostHelped = currBestHelpingNeighborSet.hostHelped;
                bestHelpingNeighborSet.newTogetherRminsqrt = currBestHelpingNeighborSet.newTogetherRminsqrt;
            }
            else    //when not first we need to compare to find the best
            {
                //EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! NOT FIRST -> COMPARE "<< endl;
                if(bestHelpingNeighborSet.newTogetherRminsqrt > currBestHelpingNeighborSet.newTogetherRminsqrt)
                {
                    //EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! FOUND BETER with dist: "<<sqrt(currBestHelpingNeighborSet.newTogetherRminsqrt)<< endl;
                    bestHelpingNeighborSet.droneHelping = currBestHelpingNeighborSet.droneHelping;
                    bestHelpingNeighborSet.hostHelped = currBestHelpingNeighborSet.hostHelped;
                    bestHelpingNeighborSet.newTogetherRminsqrt = currBestHelpingNeighborSet.newTogetherRminsqrt;
                }
            }
        }
        else{
            //EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! DRONE CANT HELP!! "<< endl;
        }
    }//end droneNeigborsList loop
    EV_INFO << ""<<endl;
    //EV_INFO << "ZZZZZZ canMyNeighborDroneHelp ! BEST DRONE HELPER IS: "<< bestHelpingNeighborSet.droneHelping<< endl;

    //this if is to know if there was a best helping neighbor, and if so we want to push to helpDroneHostList best pair
    // but this if would be skipped if no drones neighbor at all, or if none of the droneNeigborsList drones could help with the cover
    if(!(strcmp(bestHelpingNeighborSet.droneHelping,"NULL")==0))
    {
        EV_INFO << "----------------------  canMyNeighborDroneHelp ! HELP FOUND !! DRONE : "<< bestHelpingNeighborSet.droneHelping<<" WILL OWN HOST "
                <<bestHelpingNeighborSet.hostHelped << " with new dist between them: "<<sqrt(bestHelpingNeighborSet.newTogetherRminsqrt)<< endl;

        //so after testing all neighbor drones we want to push the best to radio list(think how to get the relevantPair pointers)
        //maybe can iterate agian and compare there full names and PUSH then
        for (MYIdealChannelModel::myRadioList::iterator it = droneNeigborsList.begin(); it != droneNeigborsList.end(); it++){
            if(strcmp(it->radioModule->getParentModule()->getParentModule()->getFullName(),bestHelpingNeighborSet.droneHelping)==0){
                helpDroneHostList.push_back(*it);
            }
        }

        for (MYIdealChannelModel::myRadioList::iterator it1 = criticalContenders.begin(); it1 != criticalContenders.end(); it1++){
            if(strcmp(it1->radioModule->getParentModule()->getParentModule()->getFullName(),bestHelpingNeighborSet.hostHelped)==0){
                helpDroneHostList.push_back(*it1);
            }
        }
    }
    else{
        EV_INFO << "---------------------- canMyNeighborDroneHelp ! ALL NEIGBOR DRONES CANT HELP"<<  endl;
    }

    // if droneNeigborsList is empty should call next drone from the parking
    //EV_INFO <<"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  CAN NEIGHBOR HELP END  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"<<endl;
    //EV_INFO <<" "<< endl;
    return helpDroneHostList;
}

bool MYConstSpeedMobilityDrone::isDroneCanHelpWithHostCover(DroneHostRminSet currDronesBestNeighborSet)
{
    //receive the closest criticalcontender to this drone (from droneNeigborsList)
    /*
       if with owning also best criticalContendor host, helping neighbor drone new Rmin is:
            1. Or smaller then its current Rmin and then it does not increase helping neighbor drone Rmin.
               if 1. is NOT the case then we can also test
            2. drone RneighborHelpStrip and if its smaller then still can help
               else we say this drone can't risk in helping to this host(which is also the closest criticalContendor)
     */

    EV_INFO << "ZZZZZZ isDroneCanHelpWithHostCover ! CLOSEST DRONE-HOST is: "<< currDronesBestNeighborSet.droneHelping<< "-"<<currDronesBestNeighborSet.hostHelped<<
            " with dist: "<<sqrt(currDronesBestNeighborSet.newTogetherRminsqrt)<<endl;

    double realDroneHelpingRmin = cc2->getBestCurrentRmin(currDronesBestNeighborSet.droneHelping);
    double RneighborHelpStrip = cc2->getTransmissionRange(currDronesBestNeighborSet.droneHelping) - criticalStrip - RneighborhelpstripDelta ;
    double testedNewRminToBesqrt = currDronesBestNeighborSet.newTogetherRminsqrt;

    EV_INFO << "FIXFIX realDroneHelpingRmin is: "<<realDroneHelpingRmin<<endl;
    EV_INFO << "FIXFIX RneighborHelpStrip*RneighborHelpStrip >? testedNewRminToBesqrt "<<RneighborHelpStrip*RneighborHelpStrip<<" >? "<<testedNewRminToBesqrt<<" IF YES -> return TRUE"<<endl;
    //EV_INFO << "FIXFIX testedNewRminToBe is: "<<sqrt(testedNewRminToBesqrt)<<endl;

    //FIXME - 2
    //if((realDroneHelpingRmin*realDroneHelpingRmin > testedNewRminToBesqrt) && (RneighborHelpStrip*RneighborHelpStrip > testedNewRminToBesqrt))
    //if(realDroneHelpingRmin*realDroneHelpingRmin > testedNewRminToBesqrt)
    //    return true;

    //always want to test this sepertly also
    if(RneighborHelpStrip*RneighborHelpStrip > testedNewRminToBesqrt)
        return true;

    return false;
}

//MOSHE: can drone with one host can pass it to other drone (return drone to pass name)
const char *MYConstSpeedMobilityDrone::canPassHostToNeighborDrone(const char * testedOneHostToPassFullName){
    /*
       we here when this is regular drone with 1 host & to test if 'testedOneHostToPassFullName' can be passed to anyone out of the drone neighborsList without effecting
       badly on this helping drone with critical state, ONLY critical state ! (connectivity chain is tested automatically and handled when size == 0)
       so because we don't need to thing of connectivity chain, so we will test if conditions to pass host, meet with drones contender Rmin not to be critically effected
          DEFINE AND INITIALIZE NEW LIST TO HOLD THE droneNeigborsList CONTENDORS THAT CAN COVER TO 'canHelpWithHostCoverDronesList'
          AFTER PUSHING THEM IN, ITARATE OVER 'canHelpWithHostCoverDronesList' and breakdown to lists :size>1, size=1, size==0; this is helping drone types that can help
          if size > 1 not empty, find the closest drone with best can help and return its fullname and so on
     */
    DroneHostRminSet currDroneHostRminSet;
    double currsqrdist = 0 ;
    MYIdealChannelModel::myRadioList canHelpNeighborDroneContendorList;
    MYIdealChannelModel::myRadioList canHelpSizeMoreOneNeighborDroneContendorList;
    MYIdealChannelModel::myRadioList canHelpSizeEquelOneNeighborDroneContendorList;

    Coord hostPos = cc2->getHostPosition(testedOneHostToPassFullName);
    currDroneHostRminSet.hostHelped = testedOneHostToPassFullName;

    //go over all the droneNeigborsList to find who isDroneCanHelpWithHostCover and push him to canHelpNeighborDroneContendorList for next checks
    for (MYIdealChannelModel::myRadioList::iterator it = droneNeigborsList.begin(); it != droneNeigborsList.end(); it++){

        const char *currDroneNeighborFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();
        currsqrdist = hostPos.sqrdist(it->pos);

        currDroneHostRminSet.droneHelping = currDroneNeighborFullName;
        currDroneHostRminSet.newTogetherRminsqrt = currsqrdist;

        if(isDroneCanHelpWithHostCover(currDroneHostRminSet)){
            EV_INFO << "ZZZZZZ canPassHostToNeighborDrone ! CAN HELP -> push to 'canHelpNeighborDroneContendorList' "<<  endl;
            canHelpNeighborDroneContendorList.push_back(*it);   //get all neighbor drones that can help first

            //TODO - make sure we want to fix in this way
            //also want to make sure that even after we know a drone can help, also his dist is not to close to the criticalStrip range.
            //the closenes to the criticalStrip range is smt not updated (when on the way to 1st target after change) so we have intraste to make sure
            //that we don't pass the 1 host we have to contendor that is in risk to be too far, smt that would

        }
        else{
            EV_INFO << "ZZZZZZ canPassHostToNeighborDrone ! CANT HELP -> TEST NEXT Neighbor DRONE"<<  endl;
        }
        EV_INFO <<""<<endl;
    }

    int helpsize = (int)canHelpNeighborDroneContendorList.size();
    EV_INFO << "ZZZZZZ canPassHostToNeighborDrone ! canHelpNeighborDroneContendorList.size() is: "<< helpsize << endl;

    if(helpsize!=0){
        //go over canHelpNeighborDroneContendorList and break to 2 lists according to owned num of hosts for each drone contender
        for (MYIdealChannelModel::myRadioList::iterator it1 = canHelpNeighborDroneContendorList.begin(); it1 != canHelpNeighborDroneContendorList.end(); it1++)
        {
            /*
                get hosts num of drones in order to push again to relevant list (according to size>1 or size ==1 && not this(initiator), when size==0 we don't move FROM 1 owning to LINKER)
                 then test size>1 list to know how many are there if only 1 in list return its FULLNAME if more then 1 find the closest drone to the host(testing dist)
                 and return its FULLNAME, if none in the list test the list where size == 1 and do the same test as before and return according the same way
                 if list with size == 1 is also empty return NULL
             */

            const char *currDroneNeighborFullName = it1->radioModule->getParentModule()->getParentModule()->getFullName();
            int hostOwnedSize = cc2->getDroneOwnedSize(currDroneNeighborFullName);

            if(hostOwnedSize > 1)
                canHelpSizeMoreOneNeighborDroneContendorList.push_back(*it1);
            if(hostOwnedSize == 1)
                canHelpSizeEquelOneNeighborDroneContendorList.push_back(*it1);
            // when hostOwnedSize == 0 move to test next canHelpNeighborDroneContendorList
        }


        int helpsize1 = (int)canHelpSizeMoreOneNeighborDroneContendorList.size();
        EV_INFO << "ZZZZZZ canPassHostToNeighborDrone ! canHelpSizeMoreOneNeighborDroneContendorList.size() is: "<< helpsize1 << endl;
        EV_INFO << ""<< endl;

        // finding the best helper 1st by size owned & then by dist. Routing rule is - first try passing to drone with size > 1 and only if none try passing to size == 1
        if(helpsize1!=0)
        {
            if(helpsize1 > 1){
                //find the best(with smallest effect to it) contender
                const char * closestCanHelpNeighborDrone = getClosestNeighborDroneToHost(canHelpSizeMoreOneNeighborDroneContendorList,testedOneHostToPassFullName);
                return closestCanHelpNeighborDrone;
            }
            else{
                //must be helpsize1 = 1 so return its FullName
                return canHelpSizeMoreOneNeighborDroneContendorList.begin()->radioModule->getParentModule()->getParentModule()->getFullName();
            }
        }
        else
        {       //here when helpsize1 == 0 means non with hostOwnedSize > 1 && can help -> test those with hostOwnedSize == 1

            int helpsize2 = (int)canHelpSizeEquelOneNeighborDroneContendorList.size();
            EV_INFO << "ZZZZZZ canPassHostToNeighborDrone ! canHelpSizeEquelOneNeighborDroneContendorList.size() is: "<< helpsize2 << endl;
            EV_INFO << ""<< endl;

            if(helpsize2!=0)
            {
                if(helpsize2 > 1){
                    //find the best(with smallest effect to it) contender
                    const char * closestCanHelpNeighborDrone = getClosestNeighborDroneToHost(canHelpSizeEquelOneNeighborDroneContendorList,testedOneHostToPassFullName);
                    return closestCanHelpNeighborDrone;
                }
                else{
                    //must be helpsize2 = 1 so return its FullName
                    return canHelpSizeEquelOneNeighborDroneContendorList.begin()->radioModule->getParentModule()->getParentModule()->getFullName();
                }
            }//end test list size == 1
        }//end test list size > 1
    }//end test list of can help drones

    EV_INFO << ""<< endl;
    // when none of those test gave help we say no help can be given from droneNeigborsList at all (due to 1 of reasons 1.no Neighbor drones at all 2.can cover 3.size == 0)
    return "NULL";
}

const char *MYConstSpeedMobilityDrone::getClosestNeighborDroneToHost(MYIdealChannelModel::myRadioList canHelpSizeMoreOneNeighborDroneContendorList, const char*testedOneHostToPassFullName){

    DroneHostRminSet currBestHelpingNeighborSet , bestHelpingNeighborSet;

    //initialize to find current closest host tested for specific drone
    currBestHelpingNeighborSet.droneHelping ="NULL";
    currBestHelpingNeighborSet.hostHelped ="NULL";
    currBestHelpingNeighborSet.newTogetherRminsqrt=0;

    //initialize to find Best drone that can help, to initiating drone (which is this and in the critical state)
    bestHelpingNeighborSet.droneHelping ="NULL";
    bestHelpingNeighborSet.hostHelped ="NULL";
    bestHelpingNeighborSet.newTogetherRminsqrt=0;

    Coord hostPos = cc2->getHostPosition(testedOneHostToPassFullName);

    for (MYIdealChannelModel::myRadioList::iterator it = canHelpSizeMoreOneNeighborDroneContendorList.begin(); it != canHelpSizeMoreOneNeighborDroneContendorList.end(); it++){

        double currsqrdist = hostPos.sqrdist(it->pos);
        const char * currDroneFullName = it->radioModule->getParentModule()->getParentModule()->getFullName();

        // if its the first drone which can help, we push 1st helper anyway
        if(bestHelpingNeighborSet.newTogetherRminsqrt == 0)
        {
            EV_INFO << "ZZZZZZ getClosestNeighborDroneToHost ! FIRST "<< endl;
            bestHelpingNeighborSet.droneHelping = currDroneFullName;
            bestHelpingNeighborSet.hostHelped = testedOneHostToPassFullName;        //not realy nedded - here just to keep good format
            bestHelpingNeighborSet.newTogetherRminsqrt = currsqrdist;
        }
        else    //when not first we need to compare to find the best
        {
            EV_INFO << "ZZZZZZ getClosestNeighborDroneToHost ! NOT FIRST -> COMPARE "<< endl;
            if(bestHelpingNeighborSet.newTogetherRminsqrt > currsqrdist)
            {
                bestHelpingNeighborSet.droneHelping = currDroneFullName;
                bestHelpingNeighborSet.hostHelped = testedOneHostToPassFullName;    //not realy nedded - here just to keep good format
                bestHelpingNeighborSet.newTogetherRminsqrt = currsqrdist;
            }
        }
    }

    return bestHelpingNeighborSet.droneHelping;
}

//MOSHE: emit Extra Drone Signal
void MYConstSpeedMobilityDrone::emitExtraDroneToCCSignal(long hostNum)
{
    // e.g - 7 is the host[7] representer
    emit(extraDroneToCCSignal, hostNum);
}

//MOSHE: emit Extra Drone Signal
void MYConstSpeedMobilityDrone::emitFaultSignal(long hostNum)
{
    emit(faultSignal, hostNum);
}

//MOSHE: emit Drone to Drone signal
void MYConstSpeedMobilityDrone::emitDroneToDroneSignal(long hostNum)
{
    emit(droneToDrone, hostNum);
}


//hila tslil: emit establishSubGraph
void MYConstSpeedMobilityDrone::emitEstablishSubGraphSignal(const char * droneAndLinker)
{
    emit(establishSubGraph, droneAndLinker);
}

//MOSHE: check best drone position by checking pairs, triple and host
Coord MYConstSpeedMobilityDrone::calcSECPosition()
{
    double RminPair = 77777, RminTriplet = 77777, TripletSECX, TripletSECY,ax=0,ay=0,bx=0,by=0;
    double criticalRange = 0, Xnew=0, Ynew=0;
    MYIdealChannelModel::myRadioList pairSEC;
    MYIdealChannelModel::myRadioList tripletsSEC;
    int size = (int)hostNeigborsList.size(),count=0;
    Coord p, PairSECCoord, TripletSECCoord,currPos;

    //calc the position according to the hostNeigborsList, droneNeigborsList and ccRadioEntry (need to check size cuz not always set)
    //MOSHE: find best position for more then 2 hosts set pairSECCord as the middle of the two best pairs that cover all neighbour hosts or to zero
    if(size >= 2){
        //get BEST PAIR can be only if isCoveringAll
        pairSEC = checkPairs();

        for (MYIdealChannelModel::myRadioList::iterator it0 = pairSEC.begin(); it0!=pairSEC.end() ; it0++){
            switch(count){
            case 0:{
                RminPair = it0->pos.z;
                it0->pos.z = 0;
                ax = it0->pos.x;
                ay = it0->pos.y;
                break;
            }
            case 1:{
                bx = it0->pos.x;
                by = it0->pos.y;
                break;
            }
            }
            count++;
        }
        PairSECCoord = {(ax + bx)/2, (ay + by)/2, 0};
    }
    //Hila & Tslil
    if((int)pairSEC.size() == 0){       // when (int)PairSEC.size() == 0 so no PairSECCoord to set
               char lastSenderName[9];
        AtachNumToFullName(myRadioRef2.ID,lastSenderName);
        const char* lastSenderName1 = (const char*)lastSenderName;
        currPos= cc2->getHostPosition(lastSenderName1);
        PairSECCoord.x=currPos.x;
        PairSECCoord.y=currPos.y;
        PairSECCoord.z=currPos.z;
    }

    EV_INFO << " "<< endl;
    // IF BESTPair exist == if Pair exist that covers all
    /*  KEEP FOR PRINT
    for (MYIdealChannelModel::myRadioList::iterator it0 = pairSEC.begin(); it0!=pairSEC.end() ; it0++){
        EV_INFO << "KEEP KEEP        BEST PAIR in pairSEC FFFF1 "<< *it0 << endl;
    }
     */
    EV_INFO << " "<< endl;
    //MOSHE: find best position for more then 3 hosts set tripleSECCord as the middle of the two best triple that cover all neighbour hosts or to zero
    if(size >= 3){
        count = 0 ;     //for this iteration
        //get BEST TRIPLET only if it CoveringAllTriplets
        tripletsSEC = checkTriplets();


        for (MYIdealChannelModel::myRadioList::iterator it0 = tripletsSEC.begin(); it0!=tripletsSEC.end() ; it0++){
            //get out Rmin secX and sec Y from triplets z values and set back to 0
            switch(count){
            case 0:{
                RminTriplet = it0->pos.z;
                it0->pos.z = 0;
                break;
            }
            case 1:{
                TripletSECX = it0->pos.z;
                it0->pos.z = 0;
                break;
            }
            case 2:{
                TripletSECY = it0->pos.z;
                it0->pos.z = 0;
                break;
            }
            }
            Xnew+=it0->pos.x;
            Ynew+=it0->pos.y;
            count++;
        }
        Xnew=Xnew/3;
        Ynew=Ynew/3;
        TripletSECCoord = {TripletSECX,TripletSECY,0};
    }

    //hostNeigborsList.clear(); //reset for next iteration
    hostNeigborsList.clear();
    isInCritical = false;

    criticalRange = myRadioRef2.transmissionRange - criticalStrip;

    // if Best Pair that cover all exist and > criticalRange (isInCritical => true), if Pair dont exist means Pairs cant cover all -> check Best Triplet if >= criticalRange
    if(((RminPair >= criticalRange)&&(RminPair != 77777)) || ((RminTriplet >= criticalRange)&&(RminTriplet != 77777)&&(RminPair > RminTriplet)))
        isInCritical = true;
    else
        isInCritical = false;


    // BUG FIXED shay
    if(RminPair <= RminTriplet || (TripletSECCoord.x == 0 && TripletSECCoord.y == 0) ){   // check the case when (int)hostNeigborsList.size() >= 2
        criticalContenders = pairSEC;
        cc2->setBestCurrentRmin(curFullName,RminPair);

        if((PairSECCoord.x == 0 && PairSECCoord.y == 0))
        {
            if( all_Hosts_In_One_Spot_Under_One_Drone() )       // CHECK if all the hosts under the same drone covering zone and if the are all close to the same point on map
            {
                TripletSECCoord = find_CenterPoint();
                return TripletSECCoord;
            }
        }
        return PairSECCoord;
    }
    criticalContenders = tripletsSEC;
    cc2->setBestCurrentRmin(curFullName,RminTriplet);

    return TripletSECCoord;
}

//MOSHE: return list of best 2 host that if we put drone in the middle it will cover all hosts
MYIdealChannelModel::myRadioList MYConstSpeedMobilityDrone::checkPairs(){

    std::list<MYIdealChannelModel::RadioEntry>::iterator it2;
    MYIdealChannelModel::myRadioList bestPair;
    double Rmin=77777, currR=0, failedRmax=0;     //any big number
    Coord currCenter;
    int count = 0;
    MYIdealChannelModel::RadioEntry *r1,*r2;

    for (MYIdealChannelModel::myRadioList::iterator it = hostNeigborsList.begin(); count!=(int)hostNeigborsList.size()-1 ; it++){
        count++;
        for ( it2 = it ; it2 != hostNeigborsList.end(); it2++){
            if (it2 == it)
                it2++;
            r1 = &*it;
            r2 = &*it2;
            //EV_INFO << "KEEP KEEP        current checked pair AAAA1 "<< *it << endl;
            //EV_INFO << "KEEP KEEP        current checked pair AAAA2 "<< *it2 << endl;

            //currR = it->pos.distance(it2->pos)/2;
            //this is the currR cuz it must be the diameter, means its the distance between 2 point / 2 (cuz diameter)
            currR = sqrt(pow((it->pos.x-it2->pos.x),2.0)+pow((it->pos.y-it2->pos.y),2.0))/2;

            currCenter = {(it->pos.x + it2->pos.x)/2,(it->pos.y + it2->pos.y)/2,0};

            //EV_INFO << "KEEP KEEP        currR        AAAA3 "<< currR << endl;
            //EV_INFO << "KEEP KEEP        currMidPoint AAAA4 "<< currMidPoint << endl;
            //EV_INFO << "KEEP KEEP        failedRmax   AAAA5 "<< failedRmax << endl;
            //EV_INFO << "KEEP KEEP        Rmin         AAAA6 "<< Rmin << endl;

            if(failedRmax <= currR){
                if(isThisPairCoveringAllPairs(currCenter, currR, it->radioModule, it2->radioModule))  //if covering check if minimal else to next pair
                {
                    if(Rmin > currR){
                        EV_INFO << "KEEP KEEP        current checked pair is: "<< it->radioModule->getParentModule()->getParentModule()->getFullName() <<"-"<< it2->radioModule->getParentModule()->getParentModule()->getFullName() <<endl;
                        EV_INFO << "KEEP KEEP        found CURR BEST Pair (TEST TO SEE CORRECTNES OF Pair CHOOSING + who finally choosen) "<< currR << endl;
                        Rmin = currR;
                        bestPair.clear();                   //save ONLY one best pair in the list
                        it->pos.z = Rmin;
                        bestPair.push_back(*it);
                        bestPair.push_back(*it2);
                    }
                }
                else
                    failedRmax=currR;
            }
        }
    } //end for
    return bestPair;
}

//MOSHE: return list of best 3 host that if we put drone in the middle it will cover all hosts
MYIdealChannelModel::myRadioList MYConstSpeedMobilityDrone::checkTriplets(){
    std::list<MYIdealChannelModel::RadioEntry>::iterator it2,it3;
    MYIdealChannelModel::myRadioList bestTriplet;
    int count = 0, count2 = 0 ,scenario = 0 ;
    double firstSlope=0, secondSlope=0, m_it_it2=0, m_it_it3=0, Ymid=0, Xmid=0, Rmin=77777, currR=0, failedRmax=0, distance_From_Circle_Middle;
    bool toBreak, notValid = false;
    Coord it_it2_MidPoint,it_it3_MidPoint,currCenter, current_Drone_Position;

    for (MYIdealChannelModel::myRadioList::iterator it = hostNeigborsList.begin(); count!=(int)hostNeigborsList.size()-2 ; it++){
        MYIdealChannelModel::RadioEntry *r = &*it;
        count++;
        count2=count;
        for ( it2 = it ; count2!=(int)hostNeigborsList.size()-1; it2++){
            MYIdealChannelModel::RadioEntry *r2 = &*it2;
            count2 ++;
            if (it2 == it)
                it2++;

            for ( it3 = it2 ; it3 != hostNeigborsList.end() ; it3++){
                MYIdealChannelModel::RadioEntry *r3 = &*it3;
                if (it3 == it2)
                    it3++;

                //EV_INFO << "KEEP KEEP        current checked triplet BBBB1 "<< *it <<"-"<< *it2 <<"-"<<*it3 <<endl;

                scenario = checkScenario(it->pos.x, it->pos.y,  it2->pos.x, it2->pos.y,  it3->pos.x,it3->pos.y);
                //EV_INFO << "KEEP KEEP        scenario BBBB "<< scenario << endl;

                toBreak = false; //ini for each triplet, and all the cases that need to be checked

                it_it2_MidPoint = {(it->pos.x + it2->pos.x)/2,(it->pos.y + it2->pos.y)/2,0};
                it_it3_MidPoint = {(it->pos.x + it3->pos.x)/2,(it->pos.y + it3->pos.y)/2,0};

                switch(scenario){
                case 1:{
                    toBreak = true;
                    break;
                }
                case 2:{
                    toBreak = true;
                    break;
                }
                case 3:{
                    currCenter = {it_it2_MidPoint.x,it_it3_MidPoint.y,0};
                    break;
                }
                case 4:{
                    currCenter = {it_it3_MidPoint.x,it_it2_MidPoint.y,0};
                    break;
                }
                case 5:{
                    secondSlope = (it3->pos.y-it->pos.y)/(it3->pos.x-it->pos.x);
                    m_it_it3 = -1/secondSlope;
                    currCenter = {(it_it2_MidPoint.y+(m_it_it3*it_it3_MidPoint.x)-it_it3_MidPoint.y)/m_it_it3 , it_it2_MidPoint.y , 0};
                    break;
                }
                case 6:{
                    firstSlope = (it2->pos.y-it->pos.y)/(it2->pos.x-it->pos.x);
                    m_it_it2 = -1/firstSlope;
                    currCenter = {(it_it3_MidPoint.y+(m_it_it2*it_it2_MidPoint.x)-it_it2_MidPoint.y)/m_it_it2 , it_it3_MidPoint.y , 0};
                    break;
                }
                case 7:{
                    firstSlope = (it2->pos.y-it->pos.y)/(it2->pos.x-it->pos.x);
                    secondSlope = (it3->pos.y-it->pos.y)/(it3->pos.x-it->pos.x);
                    m_it_it2 = -1/firstSlope;
                    m_it_it3 = -1/secondSlope;

                    Xmid = 0;
                    Ymid = 0;

                    Xmid = ((m_it_it2*it_it2_MidPoint.x) - (m_it_it3*it_it3_MidPoint.x) - it_it2_MidPoint.y + it_it3_MidPoint.y)/(m_it_it2-m_it_it3);
                    Ymid = m_it_it3*(Xmid-it_it3_MidPoint.x) + it_it3_MidPoint.y ;

                    current_Drone_Position = cc2->getHostPosition(curFullName);
                    distance_From_Circle_Middle = (current_Drone_Position.x - Xmid)*(current_Drone_Position.x - Xmid)+(current_Drone_Position.y - Ymid)*(current_Drone_Position.y - Ymid);

                    if(Xmid < 0 || Ymid < 0 || distance_From_Circle_Middle > myRadioRef2.transmissionRange * myRadioRef2.transmissionRange)
                        notValid = true;
                    else
                        currCenter = {Xmid,Ymid,0};

                    break;
                }
                }
                //EV_INFO << "KEEP KEEP        currCenter BBBB "<< currCenter << endl;
                //no need to check CURR triplet, this case is covered in the Pairs check
                if (toBreak){
                    //EV_INFO << "KEEP KEEP        no need to check CURR triplet - so no need to do any thing BBBB " << endl;
                }
                else{
                    //EV_INFO << "KEEP KEEP        currCenter BBBB "<< currCenter << endl;
                    //EV_INFO << "KEEP KEEP        currCenter.x-it->pos.x BBBB "<< currCenter.x-it->pos.x << endl;
                    //EV_INFO << "KEEP KEEP        urrCenter.y-it->pos.y BBBB "<< currCenter.y-it->pos.y << endl;
                    if(notValid)
                    {
                        notValid = false;
                        currR = 0;
                    }
                    else
                        currR = sqrt(pow((currCenter.x-it->pos.x),2.0)+pow((currCenter.y-it->pos.y),2.0));  //triplets so, it means its the distance between 2 point (here no need in / 2)

                    //EV_INFO << "KEEP KEEP        currR BBBB "<< currR << endl;
                    //EV_INFO << "KEEP KEEP        failedRmax BBBB "<< failedRmax << endl;

                    if(failedRmax <= currR)
                    {
                        if(isThisTripletCoveringAllTriplets(currCenter, currR, it->radioModule, it2->radioModule,it3->radioModule))  //if covering check if minimal else to next pair
                        {
                            //EV_INFO << "KEEP KEEP        isThisTripletCoveringAllTriplets == true BBBB "<< failedRmax << endl;
                            if(Rmin > currR)                          //check if current R is the optimal R
                            {
                                EV_INFO << "KEEP KEEP        current checked triplet is: "<< it->radioModule->getParentModule()->getParentModule()->getFullName() <<"-"<< it2->radioModule->getParentModule()->getParentModule()->getFullName() <<"-"<<it3->radioModule->getParentModule()->getParentModule()->getFullName() <<endl;
                                EV_INFO << "KEEP KEEP        found CURR BEST Triplet (TEST TO SEE CORRECTNES OF TRIPLET CHOOSING + who finally choosen) "<< currR << endl;
                                Rmin = currR;
                                bestTriplet.clear();                   //save ONLY one best triplet in the list
                                it->pos.z = Rmin;
                                it2->pos.z = currCenter.x;
                                it3->pos.z = currCenter.y;
                                bestTriplet.push_back(*it);
                                bestTriplet.push_back(*it2);
                                bestTriplet.push_back(*it3);
                            }
                        }
                        else
                        {
                            failedRmax=currR;
                        }
                    }
                }
            }
        }
    }
    return bestTriplet;
}

//MOSHE: check if all hosts in neighbor list covered if we put drone in currCenter except it it2 it3
bool MYConstSpeedMobilityDrone::isThisTripletCoveringAllTriplets(Coord currCenter, double currR, cModule *itradioModule, cModule *it2radioModule, cModule *it3radioModule){

    for (MYIdealChannelModel::myRadioList::iterator it4 = hostNeigborsList.begin(); it4!=hostNeigborsList.end(); it4++){

        //check distance only to other points and not to those that made the currMidpoint and currR, don't need to check curr points, cuz they on the border, so must cover
        const char *runnerFullName = it4->radioModule->getParentModule()->getParentModule()->getFullName();
        if(!(strcmp(itradioModule->getParentModule()->getParentModule()->getFullName(),runnerFullName)==0) &&
                !(strcmp(it2radioModule->getParentModule()->getParentModule()->getFullName(),runnerFullName)==0) &&
                !(strcmp(it3radioModule->getParentModule()->getParentModule()->getFullName(),runnerFullName)==0))
        {

            if(currCenter.sqrdist(it4->pos) > currR*currR)
                return false;
        }
    }

    return true;
}

//MOSHE: check if all hosts in neighbor list covered if we put drone in currCenter except it it2
bool MYConstSpeedMobilityDrone::isThisPairCoveringAllPairs(Coord currCenter, double currR, cModule *itradioModule, cModule *it2radioModule){

    for (MYIdealChannelModel::myRadioList::iterator it3 = hostNeigborsList.begin(); it3!=hostNeigborsList.end(); it3++){
        //check distance only to other points and not to those that made the currMidpoint and currR
        //don't need to check curr points, cuz they on the border, so must cover
        MYIdealChannelModel::RadioEntry *r = &*it3;
        const char *runnerFullName = it3->radioModule->getParentModule()->getParentModule()->getFullName();
        if(!(strcmp(itradioModule->getParentModule()->getParentModule()->getFullName(),runnerFullName)==0) &&
                !(strcmp(it2radioModule->getParentModule()->getParentModule()->getFullName(),runnerFullName)==0))
        {
            if(currCenter.sqrdist(it3->pos) > currR*currR)
                return false;
        }
    }
    return true;
}

//MOSHE: return: 1 - all Y equal, 2 - all X equal, 3 - Y=Y2 && X=X3, 4 - X=X2 && Y=Y3, 5 - X=X2 && Y!=Y3, 6 - Y=Y2 && X!=X3 , 7 - otherwise
int MYConstSpeedMobilityDrone::checkScenario(double itX,double itY,  double it2X,double it2Y,  double it3X,double it3Y){

    if (itY == it2Y && itY == it3Y)
        return 1;

    if (itX == it2X && itX == it3X)
        return 2;

    if (itY == it2Y){
        if(itX == it3X)
            return 3;
        else
            return 6;
    }

    if (itX == it2X){
        if(itY == it3Y)
            return 4;
        else
            return 5;
    }

    //means we must have two good slopes
    return 7;
}

//MOSHE: update all drone neighbor list
void MYConstSpeedMobilityDrone::setDroneNeigborsList()
{
    hostNeigborsList.clear();//reset for next iteration
    int sum=0;
    droneNeigborsList.clear();
    ccNeigborsList.clear();

    myRadioRef2.pos = lastPosition;     //set lastPosition updated

    for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); it++){

        //if self no need to check
        if(!(strcmp(it->radioModule->getParentModule()->getParentModule()->getFullName(),curFullName) == 0 )){

            bool inRange = myRadioRef2.pos.sqrdist(it->pos) < myRadioRef2.transmissionRange*myRadioRef2.transmissionRange;
            if (inRange)   //include only if inRange and not self
            {
                if ((strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"host") == 0 ) && (strcmp(it->linker,curFullName) == 0) ){   //only drone "owned" hosts
                    hostNeigborsList.push_back(*it);
                    sum+=(int)it->radioModule->getParentModule()->getParentModule()->par("priority");
                }
                else if((strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0) && (it->myisActive )) //also check if myisActive set in drones
                    droneNeigborsList.push_back(*it);
                else if((strcmp(it->radioModule->getParentModule()->getParentModule()->getName(),"cc") == 0))
                    ccNeigborsList.push_back(*it);
            }
        }
    }
    cc2->setMyCoverSum(curFullName,sum);
    EV_INFO <<endl<< " MMM I'm covering: " << cc2->getMyCoverSum(curFullName) <<endl;
}

MYIdealChannelModel::myRadioList::iterator MYConstSpeedMobilityDrone::getDroneNeighborsList(){
    return droneNeigborsList.begin();
}


//MOSHE: something in display
void MYConstSpeedMobilityDrone::updateConnectivityTag(){
    if (ev.isGUI()){
        char buf[80];
        Coord myLinkerPos = cc2->getHostPosition(myRadioRef2.linker);
        distToLinkerTag = sqrt(myLinkerPos.sqrdist(myRadioRef2.pos));

        double Rmindist = cc2->getBestCurrentRmin(curFullName);
        if(Rmindist<5)
            sprintf(buf, " Conn. : %4.2f",distToLinkerTag);
        else
            sprintf(buf, " Conn. : %4.2f, & Crit. : %4.2f",distToLinkerTag,Rmindist);

        getParentModule()->getDisplayString().setTagArg("t",0,buf);
    }
}

//MOSHE: return true if all host under the same drone
bool MYConstSpeedMobilityDrone::all_Hosts_In_One_Spot_Under_One_Drone() {
    double squreDistance;
    long transmit_Radios;
    int hostsCounter = 0;
    int number_Of_Hosts = this->getParentModule()->getParentModule()->par("numHosts");

    if(myRadioRef2.ownerSize == number_Of_Hosts)        // Check Only if you know all the hosts are in the cover zone of this drone
    {
        transmit_Radios = myRadioRef2.transmissionRange*myRadioRef2.transmissionRange ;
        for (MYIdealChannelModel::myRadioList::iterator it = cc2->radios2.begin(); it != cc2->radios2.end(); it++)
        {
            if(it->is_Host)
            {
                squreDistance = (it->pos.x - myRadioRef2.pos.x)*(it->pos.x - myRadioRef2.pos.x) + (it->pos.y - myRadioRef2.pos.y)*(it->pos.y - myRadioRef2.pos.y);
                if(strcmp(it->linker,myRadioRef2.radioModule->getParentModule()->getParentModule()->getFullName()) == 0 && squreDistance < transmit_Radios)
                {
                    hostsCounter ++;
                    if(hostsCounter == number_Of_Hosts)
                        return true;
                }
            }
        }
    }
    return false;
}

//MOSHE: getting coord centeer of edge host per parameter x,y,z
Coord MYConstSpeedMobilityDrone::find_CenterPoint() {
    double minLeftPoint =111111111, maxRightPoint = -1, minDownPoint = 111111111, maxUpperPoint=-1;
    Coord result;
    for (MYIdealChannelModel::myRadioList::iterator it = cc2->radios2.begin(); it != cc2->radios2.end(); it++)
    {
        if(it->is_Host)
        {
            if(it->pos.x < minLeftPoint)
                minLeftPoint = it->pos.x;
            if(it->pos.x > maxRightPoint)
                maxRightPoint = it->pos.x;
            if(it->pos.y < minDownPoint)
                minDownPoint = it->pos.y;
            if(it->pos.y > maxUpperPoint)
                maxUpperPoint = it->pos.y;
        }
    }
    result.z = 0;
    result.x = (minLeftPoint + maxRightPoint)/2;
    result.y = (minDownPoint + maxUpperPoint)/2;
    return result;
}

//MOSHE: return sqrt((x1-x2)^2+(y1-y2)^2)
double MYConstSpeedMobilityDrone:: euclidean_distance(int x1, int y1, int x2, int y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}


bool MYConstSpeedMobilityDrone::checkIfDroneIsBlue(){
    const char* str=this->getParentModule()->getDisplayString().str();
    int i=0;
    std :: string str1=str;
    std :: string newstr="";
    for(i=0;i<str1.length();i++){
        if(str[i]=='#'){
            newstr=str1.substr(i,7);
            break;
        }
    }
    str=newstr.c_str();
    if(strcmp(str,"#004080")==0){
        return true;
    }
    return false;
}

void MYConstSpeedMobilityDrone::AtachNumToFullName(int senderID, char * senderName){

    sprintf(senderName,"drone[%d]",senderID);
    if(senderID == -1){
        sprintf(senderName,"cc");
    }
    else if (senderID == -2){
        sprintf(senderName,"regular");
    }
    return ;
}

void MYConstSpeedMobilityDrone::print_statistics_time(){
    std::ofstream point;
        point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/res_time.txt",std::ios::app);
        simtime_t time = simTime();
        double d_time=time.dbl();
        point<<"start time: "<<time<<std::endl;
        point.close();
}

void MYConstSpeedMobilityDrone::print_statistics_hop_count() {

    std::ofstream point;
    point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/res.txt",std::ios::app);
    MYIdealChannelModel::RadioEntry re;
    bool check = checkIfDroneIsBlue();
    if (check == true) {
        int count = 0;
        char name[9];
        AtachNumToFullName(myRadioRef2.ID, name);
        const char * start = (const char*) name;

        const char* next = myRadioRef2.linker;


        while (strcmp(next, start) != 0) {
            if (strcmp(next, "cc") == 0)
                break;
            re = cc2->lookupRadioByName(next);
            next = re.linker;
            count++;
        }
        int count2=0;
        for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); ++it){
               // MYIdealChannelModel::myRadioList::iterator it1=it;
                //d= &*it;
                if(it->is_Drone == true){
                    if(it->myisActive==true)
                        count2++;
                }
            }

        double time = simTime().dbl();
        point<<"name: drone["<<myRadioRef2.ID<<"], owner of:"<< myRadioRef2.ownerSize<<", hop count: "<<count<<", active drones: "<<count2<<", time:"<<time<<std::endl;

    }
    //point<<"hello"<<std::endl;
    point.close();
}


void MYConstSpeedMobilityDrone :: change_true_statistics(){
    MYIdealChannelModel::RadioEntry *r,*d;
    MYIdealChannelModel::RadioEntry re;
    for (MYIdealChannelModel::myRadioList::iterator it = myradios2.begin(); it != myradios2.end(); ++it){
        MYIdealChannelModel::myRadioList::iterator it1=it;
        d= &*it;
        if(it->is_Drone == true){
            char full[9];
            const char* fullname;
            int id=it->ID;
            AtachNumToFullName(id,full);
            fullname=(const char *) full;
            re=cc2->lookupRadioByName(fullname);
            r =  cc2->lookupRadio(re.radioModule);
            r->stisticsFlag=true;
        }
        //it=it1;
    }

}


