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

#include <string.h>
#include "MYIdealChannelModel.h"
#include "MYIdealRadio.h"

Define_Module(MYIdealChannelModel);


std::ostream& operator<<(std::ostream& os, const MYIdealChannelModel::RadioEntry& radio)
{
    os << radio.radioModule->getFullPath() << " (x=" << radio.pos.x << ",y=" << radio.pos.y << ")";
    return os;
}

MYIdealChannelModel::MYIdealChannelModel()
{
}

MYIdealChannelModel::~MYIdealChannelModel()
{
}

void MYIdealChannelModel::initialize()
{
    int i;
    EV << "initializing MYIdealChannelModel" << endl;

    maxTransmissionRange = 0;

    simulationTimeOfOperation_new = 0;
    simulationTimeOfOperation_old = 0;


    //FOR STATISTICS:
    Total_Number_Of_Drones22 = new cDoubleHistogram("Total Number Of Drones");
    Total_Number_Of_Drones22->setRangeAuto(100,1.);
    Total_Number_Of_Drones22_Vec = new cOutVector("Total Number Of Drones");


    for(i=0; i< INF_DRONE; i++)
    {
        drone_imply_centralAlgo_turn[i]= -1;
        drone_imply_centralAlgo_turn_Duplicat[i] = -1;
        noSEC_CountDown[i] = 0;
    }
    WATCH_LIST(radios);

    CENTRAL_ALGORITHM_ON = this->getParentModule()->par("Central_Algorithm_On");

}

MYIdealChannelModel::RadioEntry *MYIdealChannelModel::registerRadio(cModule *radio, cGate *radioInGate)
{

    Enter_Method_Silent();

    RadioEntry *radioRef = lookupRadio(radio);

    if (radioRef)
        throw cRuntimeError("Radio %s already registered", radio->getFullPath().c_str());

    MYIdealRadio *idealRadio = check_and_cast<MYIdealRadio *>(radio);

    if (maxTransmissionRange < 0.0)    // invalid value
        recalculateMaxTransmissionRange();

    if (maxTransmissionRange < idealRadio->getTransmissionRange())
        maxTransmissionRange = idealRadio->getTransmissionRange();

    if (!radioInGate)
        radioInGate = radio->gate("radioIn");

    RadioEntry re;
    re.radioModule = radio;
    re.isReplaced=false;
    re.faultHandler=false;
    re.sendDroneToNameCoord=false;
    re.radioInGate = radioInGate->getPathStartGate();
    re.isActive = true;
    re.nameCoordFrameName = "NULL";
    re.bestCurrentRmin = 0;
    re.ownerSize = 0;
    re.is_Drone = isDrone(radio->getParentModule()->getParentModule()->getFullName());
    re.is_Host = isHost(radio->getParentModule()->getParentModule()->getFullName());
    re.ID = detachNumFromFullName2(radio->getParentModule()->getParentModule()->getFullName());
    re.central_Algo_Succeeded = true;
    re.is_In_CentralAlgorithm_Procedure = true;
    re.drone_In_BFS_Mode = false;
    re.num_Of_Hops = 0;
    re.linkerID = -1;
    re.seconds_Of_Operation = 0;
    re.sendDroneToCC = false;
    re.isFault=false; //MOSHE: every drone is ok at start
    re.sendDroneToNameCoord=false;
    re.isFirstEstablish=false;
    re.currCost=-10;
    re.lastSender=-1;
    re.stisticsFlag=false;
    //only happens in ini stages. set only first drone to be active
    if (strcmp(radio->getParentModule()->getParentModule()->getFullName(),"drone[0]")==0){   //set myisActive to be the on/off mode for drones (waiting or not)
        re.myisActive = true;
        re.linker = "cc";
    }
    else
        re.myisActive = false;

    //only happens in ini stages. set all hosts owner to be drone[0]
    if (strcmp(radio->getParentModule()->getParentModule()->getName(),"host")==0)   //set owner drone[0] as linker for hosts, and linker cc for drone[0]
        re.linker = "drone[0]";
    else if ((strcmp(radio->getParentModule()->getParentModule()->getName(),"cc")==0) || (strcmp(radio->getParentModule()->getParentModule()->getFullName(),"drone[0]") != 0) )
        re.linker = "none";     //set linker to none to cc and to all extra drones(all but drone[0] in this ini stage)


    re.transmissionRange = maxTransmissionRange;
    radios.push_back(re);
    return &radios.back(); // last element
}

void MYIdealChannelModel::recalculateMaxTransmissionRange()
{
    double newRange = 0.0;

    for (RadioList::iterator it = radios.begin(); it != radios.end(); ++it)
    {
        MYIdealRadio *idealRadio = check_and_cast<MYIdealRadio *>(it->radioModule);
        if (newRange < idealRadio->getTransmissionRange())
            newRange = idealRadio->getTransmissionRange();
    }
    maxTransmissionRange = newRange;
}

void MYIdealChannelModel::unregisterRadio(RadioEntry *r)
{
    Enter_Method_Silent();
    for (RadioList::iterator it = radios.begin(); it != radios.end(); ++it)
    {
        if (it->radioModule == r->radioModule)
        {
            // erase radio from registered radios
            radios.erase(it);
            maxTransmissionRange = -1.0;    // invalidate the value
            return;
        }
    }

    error("unregisterRadio failed: no such radio");
}
//MOSHE: searching by radio in radios3
MYIdealChannelModel::RadioEntry *MYIdealChannelModel::lookupRadio(cModule *radio)
{
    for (RadioList::iterator it = radios.begin(); it != radios.end(); it++)
        if (it->radioModule == radio)
            return &(*it);
    return NULL;
}

void MYIdealChannelModel::setRadioPosition(RadioEntry *r, const Coord& pos)
{
    r->pos = pos;
}

//just a TMP for the real function to set a new owner(linker) of the hosts
void MYIdealChannelModel::setmyisActive(const char *targetName, const bool& myisActive)
{
    RadioEntry re = lookupRadioByName(targetName);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->myisActive = myisActive;
}
//MOSHE: set nextlinker
void MYIdealChannelModel::setnextlinker(const char *targetName,const char *nextLinker){
    RadioEntry re = lookupRadioByName(targetName);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->nextlinker = nextLinker;
}

const char* MYIdealChannelModel::getnextlinker(const char *targetName){
    RadioEntry re = lookupRadioByName(targetName);
    RadioEntry *r = lookupRadio(re.radioModule);
    return r->nextlinker;
}

//MOSHE: set isFault
void MYIdealChannelModel::setisFault(const char *targetName, const bool& fault)
{
    RadioEntry re = lookupRadioByName(targetName);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->isFault = fault;
}

//MOSHE: set isFault
void MYIdealChannelModel::setmoveID(const char *targetName, int id)
{
    RadioEntry re = lookupRadioByName(targetName);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->moveID = id;
}

//MOSHE: set isFault
void MYIdealChannelModel::setfaultHandler(const char *targetName, const bool& fault)
{
    RadioEntry re = lookupRadioByName(targetName);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->faultHandler = fault;
}

void MYIdealChannelModel::setNameCoordFrameName(const char *currfullname,const char *nameCoordFrameName){
    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->nameCoordFrameName = nameCoordFrameName;
}

//this used in 2 places from MYConstSpeedMobilityDrone to change host owner/linker and from MYIdealRadio to change linker to the drone or cc
void MYIdealChannelModel::setNewHostOwner(const char *hostFullName,const char *droneFullName)
{
    RadioEntry re = lookupRadioByName(hostFullName);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->linker = droneFullName;
}

void MYIdealChannelModel::setDroneOwnedSize(const char *currfullname,int size){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->ownerSize = size;
}

int MYIdealChannelModel::getDroneOwnedSize(const char * currfullname){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    return r->ownerSize;
}

void MYIdealChannelModel::setMyCoverSum(const char *currfullname,int size){
    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->myCoverSum = size;
}

int MYIdealChannelModel::getMyCoverSum(const char * currfullname){
    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    return r->myCoverSum;
}

void MYIdealChannelModel::update_My_Hops_Counts(RadioEntry re1) {
    int nameSize;
    if(re1.linker != NULL)
    {
        RadioEntry *drone1 = lookupRadio(re1.radioModule);
        nameSize = strlen(drone1->linker);
        if(nameSize == 2 || (nameSize >6 && nameSize < 11 ))
        {
            RadioEntry re2 = lookupRadioByName(drone1->linker);
            RadioEntry *drone2 = lookupRadio(re2.radioModule);
            drone1->num_Of_Hops = drone2->num_Of_Hops + 1 ;
        }
    }
    nameSize= 2;
}

//MOSHE: searching by ID in radios3
MYIdealChannelModel::RadioEntry MYIdealChannelModel::find_Radio_from_radios3_by_ID(int drone_ID)  {
    for (RadioList::iterator it = radios3.begin(); it != radios3.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->ID == drone_ID)
            return *r;
    }
}

//MOSHE: get from radios and set linker
void MYIdealChannelModel::getAndSetExtraDroneLinker(const char *currfullname,const char * newLinkerFullName)
{
    for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it){
        RadioEntry *r = &*it;

        const char *fullName = r->radioModule->getParentModule()->getParentModule()->getFullName();

        if(!(strcmp(fullName,newLinkerFullName)==0))
            continue;
        setNewHostOwner(currfullname, fullName);
    }
}

//MOSHE: look for free drone in list
const char* MYIdealChannelModel::findAndSetCCLinker(const char *currfullname)
{
    bool flag = false;              // is inorder to stop looking when reach 1st relevant choise of extradrone
    const char *extradronefullname;

    for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it){
        RadioEntry *r = &*it;

        const char *name = r->radioModule->getParentModule()->getParentModule()->getName();

        /*
                get just the first drone which is myisActive == false
                and that nameCoordFrameName == "NULL" (so CC wont take as next linker a drone that was already set to go as regular extradrone)
                and that linker == "none" so that also in the case of connectivity extradrone the drone wont choosen as a new any extradrone
         */
        if((strcmp(name,"drone")==0) && (r->myisActive == false) && (flag == false) && (r->isFault== false) &&(strcmp(r->nameCoordFrameName,"NULL")==0) && (strcmp(r->linker,"none")==0))
        {
            extradronefullname = r->radioModule->getParentModule()->getParentModule()->getFullName();
            flag = true;
        }
    }

    // END SIMULATION TESTING HERE - test if flag == false when out of the loop (here) then END SIMULATION
    // if flag is false - means no extra drone found as cc so can we want to end simulation here
    if(!flag){

        EV_INFO <<" ************************** "<<endl;
        EV_INFO <<" CAN'T FIND AN EXTRA DRONES "<<endl;
        EV_INFO <<" ************************** "<<endl;
        for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it){
            RadioEntry *r = &*it;

            const char *name = r->radioModule->getParentModule()->getParentModule()->getName();

            /*
                        get  the  drone which has minimum coverage

             */
            if((strcmp(name,"drone")==0) && (!r->isFault) && (flag == false) && (strcmp(r->nameCoordFrameName,"NULL")==0) && (strcmp(r->linker,"none")==0))
            {
                extradronefullname = r->radioModule->getParentModule()->getParentModule()->getFullName();
                flag = true;
            }
        }
        /*
        this->endSimulation();
        //in order to continue the simulation while there are no extra drones, we need to cover more important areas(with maximum sum of priorities)
        // ***************** Adi & Tamir**********************************************
        //the drone which in critical situation makes and sends a priority message
        //  ********************SEND A PRIORITY MESSAGE *************************************
        MYIdealAirFrame frame ;
        frame.priority_message_type=PRIORITY_REQUEST;
        frame.src_sent_drone_id=detachNumFromFullName2(getParentModule()->getSubmodule("MYFinalDroneHost")->getFullName());
        int num_of_covered_hosts=getParentModule()->getSubmodule("MYFinalDroneHost")->par("num_Of_Hosts_Cover");
        int sum=0;
        //sums up the sum of all hosts which are covered by him
        for(int i=0;i<num_of_covered_hosts;i++)
            sum+=(int)getParentModule()->getSubmodule("MYAdhocHost",i)->par("priority");
        frame.priority_sum=sum;
        //update the location which the chosen drone will need to change its position
        frame.xLocation=getParentModule()->getSubmodule("MYFinalDroneHost")->par("xPosition");
        frame.yLocation=getParentModule()->getSubmodule("MYFinalDroneHost")->par("yPosition");
        RadioEntry radioo;
        //send the first priority message ( type of priority request)
        //  send_Find_Message(&radioo,frame,frame.src_sent_drone_id);

         */

    }
    EV_INFO << "000 me: " << getFullName() << endl <<"001 my parent: "  <<getParentModule()->getFullName()<<endl;

    //sets the next extra drone as linker to cc

    setNewHostOwner(currfullname, extradronefullname);

    return extradronefullname;
}

Coord& MYIdealChannelModel::getHostPosition(const char * currfullname){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    if(currfullname == NULL || strcmp(currfullname,"NULL") == 0|| strcmp(currfullname,"none")==0)
        error("Cannot return Coord for none exists host/drone/cc.  Check: MYIdealChannelModel::getHostPosition");
    return r->pos;
}

const char * MYIdealChannelModel::getLinker(const char * currfullname){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    return r->linker;
}

double MYIdealChannelModel::getTransmissionRange(const char * currfullname){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    return r->transmissionRange;
}

void MYIdealChannelModel::setBestCurrentRmin(const char * currfullname, double currBestRmin){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    r->bestCurrentRmin = currBestRmin;
}


double MYIdealChannelModel::getBestCurrentRmin(const char * currfullname){

    RadioEntry re = lookupRadioByName(currfullname);
    RadioEntry *r = lookupRadio(re.radioModule);

    return r->bestCurrentRmin;
}

//MOSHE: send message to all drones in range with propagation delay
//should make same function but for drone communication only
void MYIdealChannelModel::sendToChannel(RadioEntry *srcRadio, MYIdealAirFrame *airFrame)
{
    // NOTE: no Enter_Method()! We pretend this method is part of ChannelAccess

    if (maxTransmissionRange < 0.0)    // invalid value
        recalculateMaxTransmissionRange();

    double sqrTransmissionRange = airFrame->getTransmissionRange()*airFrame->getTransmissionRange();

    // loop through all radios
    for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it)
    {
        RadioEntry *r = &*it;

        const char *name = r->radioModule->getParentModule()->getParentModule()->getName();
        const char *fullname = r->radioModule->getParentModule()->getParentModule()->getFullName();
        const char *srcfullname = srcRadio->radioModule->getParentModule()->getParentModule()->getFullName();

        if (r == srcRadio)
            continue;   // skip sender radio

        if (!r->isActive)
            continue;   // skip disabled radio interfaces

        if (isMYRadioOwnerHost(name))
            continue;   // skip drone or cc (just testing host, cuz he have ping app)


        double sqrdist = srcRadio->pos.sqrdist(r->pos);
        if (sqrdist <= sqrTransmissionRange)
        {
            // account for propagation delay, based on distance in meters
            // Over 300m, dt=1us=10 bit times @ 10Mbps
            simtime_t delay = sqrt(sqrdist) / SPEED_OF_LIGHT;
            airFrame->setName("MY MSG");
            EV <<""<<endl;
            EV << " ----------------------  SEND DIRECT from " <<srcfullname  <<" to "<< fullname <<" THE MSG: "<< airFrame->getName() <<" ---------------------- MYIdealChannelModel"<<endl;
            EV <<""<<endl;
            check_and_cast<cSimpleModule*>(srcRadio->radioModule)->sendDirect(airFrame->dup(), delay, airFrame->getDuration(), r->radioInGate);
        }
    }
    delete airFrame;
}

//MOSHE: send message to linker only
//from MYIdealChannelModelAccess::sendToMYChannel, here we send using direct and delete the msg
int MYIdealChannelModel::sendToMYLinker(RadioEntry *srcRadio, MYIdealAirFrame *airFrame)
{
    int messageSentCounter = 0;
    //ONLY DRONE TO ASKS FOR EXTRA DRONE USES sendToMYLinkerTEST to SEND the msg
    if (maxTransmissionRange < 0.0)    // invalid value
        recalculateMaxTransmissionRange();

    double sqrTransmissionRange = srcRadio->transmissionRange*srcRadio->transmissionRange;

    // loop through all radios to find get the linker
    for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it)
    {
        RadioEntry *r = &*it;

        const char *fullname = r->radioModule->getParentModule()->getParentModule()->getFullName();
        const char *srcfullname = srcRadio->radioModule->getParentModule()->getParentModule()->getFullName();

        if (r == srcRadio)
            continue;   // skip sender radio

        if (!(strcmp(fullname,srcRadio->linker)==0))
            continue;   // skip all who isn't linker

        double sqrdist = srcRadio->pos.sqrdist(r->pos);
        if (sqrdist <= sqrTransmissionRange)    //make sure that linker must be in transmissionRange
        {
            // account for propagation delay, based on distance in meters
            // Over 300m, dt=1us=10 bit times @ 10Mbps
            simtime_t delay = sqrt(sqrdist) / SPEED_OF_LIGHT;
            EV <<""<<endl;
            EV << " ---------------------- SEND DIRECT from: "<< srcfullname << " to: "<< fullname <<" THE MSG: "<< airFrame->getName()<<" frame->getKind(): "<< airFrame->getEncapsulatedPacket()->getKind()<< " frame->getName(): "<< airFrame->getEncapsulatedPacket()->getName() <<" ----------------------MYIdealChannelModel" <<endl;
            EV <<""<<endl;
            messageSentCounter ++;
            check_and_cast<cSimpleModule*>(srcRadio->radioModule)->sendDirect(airFrame->dup(), delay, airFrame->getDuration(), r->radioInGate);
        }
    }
    delete airFrame;
    return messageSentCounter;
}

//MOSHE: send message to all drone, but if signalID==faultsignal send to only to who linked by me
////   SHAY 2///////
int MYIdealChannelModel::send_Find_Message(RadioEntry *srcRadio, MYIdealAirFrame *airFrame,int lastSender)    {
//    EV_INFO<<"send_Find_Message airFrame->setIsFindFrame(1);"<<airFrame->getIsFindFrame()<<endl;
//    EV_INFO<<"send_Find_Message airFrame name:"<<airFrame->getName()<<endl;
//    EV_INFO<<"send_Find_Message airFrame currk:"<<airFrame->getCurrK()<<endl;
    double sqrTransmissionRange = srcRadio->transmissionRange*srcRadio->transmissionRange;
    int messageSent = 0;
    const char *fullname;
    const char *srcfullname;
    bool noNeihgbors = true;
    RadioEntry *r, ccRadio;
    int i=0;
    srcfullname = srcRadio->radioModule->getParentModule()->getParentModule()->getFullName();
    // loop through all radios to find get the linker
    for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it){
        r= &*it;
        if((!strcmp(airFrame->getName(),"FindMsgForFaultDrones") || !strcmp(airFrame->getName(),"ChosenDroneFind"))&& strcmp(srcfullname,r->linker))
            continue;
        i=strcmp(airFrame->getName(),"FindMsgForEstablishSubGraph");
        if(i != 0){


            fullname = r->radioModule->getParentModule()->getParentModule()->getFullName();
            if( !(strcmp(fullname,srcfullname) == 0 ) && lastSender != detachNumFromFullName2(fullname))  // don't consider yourself and don't send the message back to its sender
            {
                bool inRange = srcRadio->pos.sqrdist(r->pos) <= sqrTransmissionRange -50;//////////////////////////hila & tslil
                if (inRange)   //include only if inRange and not self
                {
                    if(((strcmp(r->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0) && (r->myisActive ) )/* || (strcmp(r->radioModule->getParentModule()->getParentModule()->getName(),"cc") == 0)*/) //also check if myisActive set in drones
                    {
                        ccRadio = lookupRadioByName("cc");
                        if(ccRadio.pos.sqrdist(r->pos) > sqrTransmissionRange/3)       // patch for avoiding to send messages to drones that still are in CC but their status is myisActive =true;
                        {
                            // account for propagation delay, based on distance in meters
                            // Over 300m, dt=1us=10 bit times @ 10Mbps
                            simtime_t delay = sqrt(srcRadio->pos.sqrdist(r->pos)) / SPEED_OF_LIGHT;
                            EV <<""<<endl;
                            EV << " ---------------------- SEND DIRECT from: "<< srcfullname << " to: "<< fullname <<endl;
                            EV <<""<<endl;
                            messageSent++;
                            noNeihgbors = false;
                            check_and_cast<cSimpleModule*>(srcRadio->radioModule)->sendDirect(airFrame->dup(), delay, airFrame->getDuration(), r->radioInGate);
                        }
                    }
                }
            }
        }
        else{ //Hila & Tslil- if establish message send to all! neighbors including last sender
            fullname = r->radioModule->getParentModule()->getParentModule()->getFullName();
            if( !(strcmp(fullname,srcfullname) == 0 ))  // don't consider yourself
            {
                bool inRange = srcRadio->pos.sqrdist(r->pos) <= sqrTransmissionRange -50;//////////////////////////hila & tslil
                if (inRange)   //include only if inRange and not self
                {
                    if(((strcmp(r->radioModule->getParentModule()->getParentModule()->getName(),"drone") == 0) && (r->myisActive ) )) //also check if myisActive set in drones
                    {
                        ccRadio = lookupRadioByName("cc");
                        if(ccRadio.pos.sqrdist(r->pos) > sqrTransmissionRange/3)       // patch for avoiding to send messages to drones that still are in CC but their status is myisActive =true;
                        {
                            // account for propagation delay, based on distance in meters
                            // Over 300m, dt=1us=10 bit times @ 10Mbps
                            simtime_t delay = sqrt(srcRadio->pos.sqrdist(r->pos)) / SPEED_OF_LIGHT;
                            EV <<""<<endl;
                            EV << " ---------------------- SEND DIRECT from: "<< srcfullname << " to: "<< fullname <<endl;
                            EV <<""<<endl;
                            messageSent++;
                            noNeihgbors = false;
                            check_and_cast<cSimpleModule*>(srcRadio->radioModule)->sendDirect(airFrame->dup(), delay, airFrame->getDuration(), r->radioInGate);
                        }
                    }
                }
            }
        }
    }
    if(!noNeihgbors)
        delete airFrame;
    return messageSent;
}


void MYIdealChannelModel::send_Reply_Message(RadioEntry *srcRadio, MYIdealAirFrame *airFrame,int lastSender) {
    RadioEntry rRadio;
    char droneName[11];
    const char *srcfullname= srcRadio->radioModule->getParentModule()->getParentModule()->getFullName();
    double sqrTransmissionRange = srcRadio->transmissionRange*srcRadio->transmissionRange;
    double tempRange;

    sprintf (droneName, "drone[%d]",lastSender);
    rRadio = lookupRadioByName(droneName);
    if(!strcmp(airFrame->getName(),"ReplyMsgForFaultDrones"))
        rRadio=lookupRadioByName(srcRadio->linker);

    tempRange = srcRadio->pos.sqrdist(rRadio.pos);
    bool inRange = tempRange <= sqrTransmissionRange ;
    if (inRange && rRadio.isActive)   //include only if inRange and not self
    {
        // account for propagation delay, based on distance in meters
        // Over 300m, dt=1us=10 bit times @ 10Mbps
        simtime_t delay = sqrt(srcRadio->pos.sqrdist(rRadio.pos)) / SPEED_OF_LIGHT;
        EV <<""<<endl;
        EV << " ---------------------- SEND DIRECT from: "<< srcfullname << " to: "<< droneName <<endl;
        EV <<""<<endl;
        check_and_cast<cSimpleModule*>(srcRadio->radioModule)->sendDirect(airFrame->dup(), delay, airFrame->getDuration(), rRadio.radioInGate);
    }
    else
        error("The FindFrame sender drone is now out of range");
    delete airFrame;
}

//returns the name cc, node or drone
bool MYIdealChannelModel::isMYRadioOwnerHost(const char *name)
{
    if (strcmp(name,"host")==0)
        return true;
    return false;
}

int MYIdealChannelModel::getFromICM()
{
    pass = 1;
    return pass;
}

//MOSHE: refresh radios
MYIdealChannelModel::myRadioList MYIdealChannelModel::getRadios()
{
    radios2 = radios;
    return radios2;
}

//MOSHE: Find radio by name
MYIdealChannelModel::RadioEntry MYIdealChannelModel::lookupRadioByName(const char *name)
{
    Enter_Method_Silent();
    for (RadioList::iterator it = radios.begin(); it != radios.end(); it++)
        if (strstr(it->radioModule->getParentModule()->getParentModule()->getFullName(),name)!=NULL)
            return *it;
}

const char * MYIdealChannelModel::lookupNameByName(const char *name)
{
    Enter_Method_Silent();
    for (RadioList::iterator it = radios.begin(); it != radios.end(); it++)
        if (strstr(it->radioModule->getParentModule()->getParentModule()->getFullName(),name)!=NULL)

            return it->radioModule->getParentModule()->getParentModule()->getFullName();
}

//MOSHE: get number from name
int MYIdealChannelModel::detachNumFromFullName2(const char *senderName)
{
    char fullNamestr[80];
    const char * charVal;
    int intVal;
    char * pch;

    strcpy(fullNamestr,senderName);
    pch = strtok (fullNamestr,"[");
    pch = strtok (NULL, "]");
    charVal = pch;

    intVal = atoi(charVal);

    return intVal;
}

//MOSHE: check if it drone
bool MYIdealChannelModel::isDrone(const char *senderName)    {
    if( strcmp(senderName,"cc") == 0 )
        return false;
    if(senderName[4] == '[')
        return false;
    return true;
}

//MOSHE: check if it host
bool MYIdealChannelModel::isHost(const char *senderName)    {
    if( strcmp(senderName,"cc") == 0 )
        return false;
    if(senderName[5] == '[')
        return false;
    return true;
}


bool MYIdealChannelModel::does_drone_imply_centralAlgoritm(int drone_ID)    {
    int i,max = drone_imply_centralAlgo_turn[drone_ID];
    if (max == -1)
        return false;
    for(i=0; i< INF_DRONE; i++)
    {
        if(drone_imply_centralAlgo_turn[i] > max)
            return false;
    }
    return true;
}

//MOSHE: set drone linker and change host to cover by this drone
void MYIdealChannelModel::update_radios(int droneServer_ID) {
    int i;
    RadioEntry *rr1,linkerRadio,hostRadio,rNew = find_Radio_from_radios3_by_ID(droneServer_ID);
    const char *linkerName, *hostName;
    char tempName[12]={0};

    // Set the drone values in radios list:
    for (RadioList::iterator it=radios.begin(); it !=radios.end(); ++it)
    {
        rr1 = &*it;
        if(rr1->ID == rNew.ID && rr1->is_Drone)                    //find matching drone
        {
            rr1->linkerID = rNew.linkerID;
            rr1->num_Of_Hops = rNew.num_Of_Hops;
            rr1->ownerSize = rNew.ownerSize;
            rr1->myCoverSum = rNew.myCoverSum;
            rr1->pos = rNew.pos;
            break;
        }
    }
    for(i=0;i<12;i++) tempName[i]=0;

    // Set the host values in radios list:
    for (RadioList::iterator it3=radios3.begin(); it3 !=radios3.end(); ++it3)           // search for hosts that been covered by this drone - as part of the central algorithm
    {
        if(it3->is_Host)     // equal to:  " strcmp(hostName, "host") == 0 "
        {
            if(it3->linkerID == rNew.ID)    // find the linker of this host, according to central algorithm
            {
                sprintf(tempName,"drone[%d]",rNew.ID);
                linkerRadio = lookupRadioByName(tempName);        // find drone RadioEntry on radios list

                linkerName = linkerRadio.radioModule->getParentModule()->getParentModule()->getFullName();      // get the string of the drone name

                for (RadioList::iterator it2=radios.begin(); it2 !=radios.end(); ++it2)                         // find the host in the radios list
                {
                    if( it2->is_Host && it2->ID == it3->ID)
                    {
                        it2->linker = linkerName;
                        it2->linkerID = rNew.ID;
                        break;
                    }
                }
            }
        }
    }

    drone_imply_centralAlgo_turn_Duplicat[droneServer_ID] = drone_imply_centralAlgo_turn[droneServer_ID];

    // Now choose and notify the next drone to imply the central algorithm output and reset flags:
    drone_imply_centralAlgo_turn[droneServer_ID] = -1;        // another drone priority will now be the highest
}

bool MYIdealChannelModel::check_Linker_In_Range(int droneID) {
    double distance;
    RadioEntry *r1, *rLinker, *r2;
    for (myRadioList::iterator it = radios3.begin(); it != radios3.end(); it++)
    {
        r1 = &*it;
        if(r1->ID == droneID && r1->is_Drone)        // find the drone with the given ID
            break;
    }
    for (myRadioList::iterator it2 = radios3.begin(); it2 != radios3.end(); it2++)
    {
        rLinker = &*it2;
        if(rLinker->ID == r1->linkerID && rLinker->is_Drone)        // find the linker of the given drone, according to radios3 list
            break;
    }

    for (myRadioList::iterator it3 = radios.begin(); it3 != radios.end(); it3++)
    {
        r2 = &*it3;
        if(r2->ID == droneID && r2->is_Drone)        // find the drone with the given ID in the radios list
            break;
    }

    distance = (r1->pos.x - rLinker->pos.x)*(r1->pos.x - rLinker->pos.x) + (r1->pos.y - rLinker->pos.y)*(r1->pos.y - rLinker->pos.y);

    if(r1->linkerID != r2->linkerID)        // relevant just if the drone's linkers are different in real time to the linker assigned by the central algorithm
    {
        if(distance + 100 > r2->transmissionRange * r2->transmissionRange)    // if the drone is now out of range from the linker assigned to it by the algorithm
        {
            return false;
        }
        else return true;
    }
    return false;
}

void MYIdealChannelModel::Notify_Algorithm_isOver(bool isValid,int drone_ID) {
    // Set the transmission range of the drone.
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if( r->ID == drone_ID && r->is_Drone )
        {
            r->drone_In_BFS_Mode = false;
            r->central_Algo_Succeeded = isValid;
            if(isValid == false)
                r->is_In_CentralAlgorithm_Procedure = false;
            break;
        }
    }
}

//MOSHE: return true if myID is the last drone to imply central algo
bool MYIdealChannelModel::is_Drone_Last_ToRelocate(int myID)  {
    int i;
    for(i=0; i<INF_DRONE; i++)
    {
        if( myID != i && drone_imply_centralAlgo_turn[i] != -1 )
            return false;
    }
    return true;
}

bool MYIdealChannelModel::does_Drone_Needs_To_Wait()  {
    int i;
    for(i=0; i<INF_DRONE; i++)
    {
        if(drone_imply_centralAlgo_turn[i] != -1)
            return true;
    }
    return false;;
}


void MYIdealChannelModel::end_Central_Algorithm_Procedure(int droneID)    {
    int i;
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Drone && r->ID == droneID)
        {
            r->central_Algo_Succeeded = true;
            r->is_In_CentralAlgorithm_Procedure = false;
            break;
        }
    }
}

//MOSHE: set droneID.central_Algo_Succeeded and .is_In_CentralAlgorithm_Procedure to true
void MYIdealChannelModel::reset_Central_Algorithm_Procedure(int droneID)    {
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Drone && r->ID == droneID)
        {
            r->central_Algo_Succeeded = true;
            r->is_In_CentralAlgorithm_Procedure = true;
        }
    }
}

bool MYIdealChannelModel::did_Drone_Arrived_To_Target_Position(int droneID,Coord droneCurrPostion,Coord new_TargetPosition)  {
    double result;
    result = abs(droneCurrPostion.x - new_TargetPosition.x) + abs(droneCurrPostion.y - new_TargetPosition.y);
    if( result == 0)
        return true;
    return false;
}

//MOSHE: set is_In_CentralAlgorithm_Procedure = false in all drones
void MYIdealChannelModel::Notify_All_Drones_To_Abort_Central_Procedure()  {
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Drone && drone_imply_centralAlgo_turn_Duplicat[r->ID] > 0)
        {
            r->is_In_CentralAlgorithm_Procedure = false;
        }
    }
}

int MYIdealChannelModel::findMyHostId(const char *me)  {
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++){
        RadioEntry *r = &*it;
        if(r->is_Host && strcmp(r->linker,me)==0)
            return r->ID;
    }

}

//MOSHE: set timer for not using SEC
void MYIdealChannelModel::Set_DroneServers_To_NotUse_SEC()    {
    int i;
    for(i=0; i<INF_DRONE; i++)
    {
        if(drone_imply_centralAlgo_turn_Duplicat[i] > 0)
            noSEC_CountDown[i] = 100;
    }
}

//MOSHE: inside
void MYIdealChannelModel::update_Hosts_Linker()   {
    const char *myLinker;
    int ownerSize,i;
    RadioEntry rDrone, *rDrone2 , rDrone3;
    //MOSHE: reset all drones
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Drone)
        {
            r->ownerSize = 0;    // reset owner size. recalculate the current owner size in next step
            r->linkerID = detachNumFromFullName2(r->linker);
        }
    }
    //MOSHE: update all drones from their host
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Host)
        {
            myLinker = r->linker;
            rDrone = lookupRadioByName(myLinker);
            rDrone2 = lookupRadio(rDrone.radioModule);

            rDrone3 = find_Radio_from_radios3_by_ID(rDrone2->ID);
            ownerSize = rDrone3.ownerSize;
            if(ownerSize < 0)           // fix the -1, if there is one
                ownerSize = 0;
            rDrone2->ownerSize = ownerSize;
        }
    }
    //MOSHE: no centeal algorithm for all
    for(i=0; i<INF_DRONE; i++)
        drone_imply_centralAlgo_turn_Duplicat[i] = -1;
}

//MOSHE: if my linker isn't cover drone look for other linkers in my sight
void MYIdealChannelModel::try_Swap_linker_With_Cover_Linker()  {
    RadioEntry linkerRadio;
    double squr_Distance, squr_Radius;
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r1 = &*it;
        if(r1->is_Drone && r1->ownerSize >0)
        {
            linkerRadio = lookupRadioByName(r1->linker);
            if(linkerRadio.ownerSize < 1)                   // check if the linker is not also cover drone, only then we can look for a swap of linkers.
            {
                for (myRadioList::iterator it2 = radios.begin(); it2 != radios.end(); it2++)        // now loop again and try to find a cover drone that can be served this drone as its linker and swap its not covering linker
                {
                    RadioEntry *r2 = &*it2;
                    if(r2->is_Drone && r2->ID != r1->ID && r2->ownerSize > 0)
                    {
                        if(r2->num_Of_Hops <= linkerRadio.num_Of_Hops)      // check that number of hops is the same, indication for a right replacment drone
                        {
                            squr_Distance = (r1->pos.x - r2->pos.x)*(r1->pos.x - r2->pos.x) + (r1->pos.y - r2->pos.y)*(r1->pos.y - r2->pos.y);
                            squr_Radius = (r1->transmissionRange - Drone_criticalStrip)*(r1->transmissionRange - Drone_criticalStrip);
                            if( squr_Radius >=  squr_Distance)  // if the linker is in the drone's radius
                            {
                                r1->linker = r2->radioModule->getParentModule()->getParentModule()->getFullName();
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}

//MOSHE: if (take_Only_numbe_of_drones =true) setting info about how many drones are active, else add one to drone seconds_Of_Operation
void MYIdealChannelModel::setDroneOperateTime(int myID, bool take_Only_numbe_of_drones)   {
    int number_of_drones;
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Drone && r->ID == myID)
        {
            if(take_Only_numbe_of_drones)
            {
                number_of_drones = number_Of_Active_Drones();
                Total_Number_Of_Drones22->collect(number_of_drones);
                Total_Number_Of_Drones22_Vec->record(number_of_drones);
            }
            else
                r->seconds_Of_Operation ++;
            break;
        }
    }
}

//MOSHE: return number of active drones
int MYIdealChannelModel::number_Of_Active_Drones()    {
    int droneCounter = 0;
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        if(it->is_Drone && it->myisActive)
        {
            droneCounter++;
        }
    }
    return droneCounter;
}

void MYIdealChannelModel::finish()    {
    FILE *file;
    double aveDroneTime, TotalDroneTime=0, aveNumber_Of_Drones_PerSecond;
    int numOfDrones = getParentModule()->par("numDrones");

    Total_Number_Of_Drones22->recordAs("number of drones");

    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *r = &*it;
        if(r->is_Drone)
            TotalDroneTime += (double)(r->seconds_Of_Operation);
    }
    aveDroneTime = (double)(TotalDroneTime)/numOfDrones;

    aveNumber_Of_Drones_PerSecond = Total_Number_Of_Drones22->getMean();

    file = fopen("Stats.txt","w+");
    fprintf(file,"Total Average Time of a drone on air is: %f\n",aveDroneTime);
    fprintf(file,"Average number of active drones per second: %f\n",aveNumber_Of_Drones_PerSecond);
    fclose(file);
}

void MYIdealChannelModel::swap_Hosts_Ownership_while_waiting(int DroneID)  {
    double transmit_Radios, squreDistance, minSqureDist = 11111111,tempSqureDist;
    int count=0;
    RadioEntry *coverHost_Drone,tempRadio, *ChoosenRadio;
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *hostRadio = &*it;
        if(hostRadio->is_Host)
        {
            tempRadio = lookupRadioByName(hostRadio->linker);          // get the linker of that host
            coverHost_Drone = lookupRadio(tempRadio.radioModule);
            if(coverHost_Drone->ID == DroneID)                         // on applied to host that their linker is the calling drone
            {
                squreDistance = (hostRadio->pos.x - coverHost_Drone->pos.x)*(hostRadio->pos.x - coverHost_Drone->pos.x) + (hostRadio->pos.y - coverHost_Drone->pos.y)*(hostRadio->pos.y - coverHost_Drone->pos.y);

                for (myRadioList::iterator it2 = radios.begin(); it2 != radios.end(); it2++)
                {
                    RadioEntry *r2 = &*it2;
                    if(r2->is_Drone && r2->ownerSize > 0 && coverHost_Drone->ID != r2->ID)           // drone is cover one and not the linker of the host
                    {
                        count ++;
                        tempSqureDist = (hostRadio->pos.x - r2->pos.x)*(hostRadio->pos.x - r2->pos.x) + (hostRadio->pos.y - r2->pos.y)*(hostRadio->pos.y - r2->pos.y);
                        if(minSqureDist > tempSqureDist)
                        {
                            minSqureDist = tempSqureDist;
                            ChoosenRadio = r2;
                        }
                    }
                }
            }
        }
        if(count >0)
        {
            if(squreDistance > tempSqureDist + transmit_Radios/15)       // if you find a cover host, more close to the host then the current one- then swap ownership
            {
                hostRadio->linker = ChoosenRadio->radioModule->getParentModule()->getParentModule()->getFullName();
                ChoosenRadio->ownerSize ++;
                coverHost_Drone->ownerSize --;
            }
        }
        count = 0;
        minSqureDist = 11111111;
    }
}

// swap ownership of a linker that might arrive during the waiting time even if there is no need for it
int MYIdealChannelModel::swap_Linker_Ownership_while_waiting(int DroneID)  {
    double transmit_Radios, squreDistance, minSqureDist = 11111111;
    RadioEntry tempRadio, *ChoosenRadio;
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        RadioEntry *droneRadio = &*it;
        transmit_Radios = droneRadio->transmissionRange;
        if(droneRadio->is_Drone && droneRadio->ID == DroneID)  // find the drone
        {
            tempRadio = lookupRadioByName(droneRadio->linker);          // find the drone's current linker
            ChoosenRadio = lookupRadio(tempRadio.radioModule);
            if(ChoosenRadio->ownerSize > 0)                            // if the drone's linker is not a covering drone. then end the function.
                return -1;

            for (myRadioList::iterator it2 = radios.begin(); it2 != radios.end(); it2++)  // Your linker is not covering hosts (yellow) and might be un-needed
            {
                RadioEntry *linkerRadio = &*it2;
                if(linkerRadio->is_Drone && linkerRadio->ID != droneRadio->ID && linkerRadio->ID != ChoosenRadio->ID && linkerRadio->ownerSize > 0)   // choose only drones that are cover/not yourself/ not your current drone
                {
                    squreDistance = (droneRadio->pos.x - linkerRadio->pos.x)*(droneRadio->pos.x - linkerRadio->pos.x) + (droneRadio->pos.y - linkerRadio->pos.y)*(droneRadio->pos.y - linkerRadio->pos.y);
                    if(squreDistance <= ( (transmit_Radios - Drone_criticalStrip)*(transmit_Radios - Drone_criticalStrip) ) )    // if the linker candidant is in the drone's radius
                        // change linker
                    {
                        if(linkerRadio->num_Of_Hops < ChoosenRadio->num_Of_Hops)    // take the drone as a link only if the current linker doesn't connect you to the CC
                        {
                            droneRadio->linkerID = linkerRadio->ID;
                            droneRadio->linker = linkerRadio->radioModule->getParentModule()->getParentModule()->getFullName();
                            return ChoosenRadio->ID;
                        }
                    }
                }
            }
            break;
        }
    }
    return -1;
}

//MOSHE: set sroneID.sendDroneToCC = flag
void MYIdealChannelModel::setNotActiveDroneToMoveToCC(int droneID, bool flag)    {
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        if(it->is_Drone && it->ID == droneID)
        {
            it->sendDroneToCC = flag;
        }
    }
}

void MYIdealChannelModel::setsendDroneToNameCoord(int droneID, bool flag)    {
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        if(it->is_Drone && it->ID == droneID)
        {
            it->sendDroneToNameCoord = flag;
        }
    }
}

void MYIdealChannelModel::setisReplaced(int droneID, bool flag)    {
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        if(it->is_Drone && it->ID == droneID)
        {
            it->isReplaced = flag;
        }
    }
}

bool MYIdealChannelModel::multiple_Drones_Cover_closed_Group_Of_Hosts(int droneID)   {
    double transmit_Radios, squreDistance;
    RadioEntry *Drone_Radio;
    int numberOfHosts = this->getParentModule()->par("numHosts");
    for (myRadioList::iterator it = radios.begin(); it != radios.end(); it++)
    {
        if(it->is_Drone && it->ID == droneID)
        {
            Drone_Radio = &*it;
            transmit_Radios = (Drone_Radio->transmissionRange * 5)/100;        // take only 20% of the drone's radius
            transmit_Radios = transmit_Radios*transmit_Radios;
            break;
        }
    }
    for (myRadioList::iterator it2 = radios.begin(); it2 != radios.end(); it2++)
    {
        if(it2->is_Host)
        {
            squreDistance = (it2->pos.x - Drone_Radio->pos.x)*(it2->pos.x - Drone_Radio->pos.x) + (it2->pos.y - Drone_Radio->pos.y)*(it2->pos.y - Drone_Radio->pos.y);
            if(strcmp(it2->linker,Drone_Radio->radioModule->getParentModule()->getParentModule()->getFullName()) != 0 && squreDistance < transmit_Radios)
            {
                it2->linker = Drone_Radio->radioModule->getParentModule()->getParentModule()->getFullName();
                Drone_Radio->ownerSize ++;
                if(Drone_Radio->ownerSize == numberOfHosts)
                    return true;
            }
        }
    }
    return false;
}

//MOSHE: check if drone is leaf (no drone is linked to it)
bool MYIdealChannelModel::isDroneLeaf(int droneID){
    bool isLeaf=true;
    myRadioList::iterator it2;
    for (it2 = radios.begin(); isLeaf && it2 != radios.end(); it2++){ //check if no other drone use it as linker
        if(it2->is_Drone && detachNumFromFullName2(it2->linker) == droneID)
            isLeaf=false;
    }
    return isLeaf;
}

//MOSHE: return the priority of all host under drone
int MYIdealChannelModel::dronePriority(int droneID){
    int  sum=0;
    char tempName[12]={0};
    sprintf(tempName,"drone[%d]",droneID);
    RadioEntry myRadioRef=find_Radio_from_radios3_by_ID(droneID);
    EV_INFO <<" cover: "<< getMyCoverSum(tempName)<<endl;
    for ( myRadioList::iterator it = radios3.begin(); it != radios3.end(); it++){
        EV_INFO <<" it->linker: "<<it->linker <<" droneID "<< droneID <<" cover: "<<myRadioRef.myCoverSum<<endl;
        if(detachNumFromFullName2(it->linker) == droneID){
            bool inRange = myRadioRef.pos.sqrdist(it->pos) < myRadioRef.transmissionRange*myRadioRef.transmissionRange;
            if (inRange)   //include only if inRange and not self
                EV_INFO <<" host? "<<it->radioModule->getParentModule()->getParentModule()->getName() <<" droneID "<< droneID <<endl;
            if (it->is_Host && (detachNumFromFullName2(it->linker)==droneID) )   //only drone "owned" hosts
                sum=sum+(int)it->radioModule->getParentModule()->getParentModule()->par("priority");
        }
    }
    EV_INFO<<"Priority sum:"<< sum <<endl;
    setMyCoverSum(tempName,sum);
    return sum;
}



