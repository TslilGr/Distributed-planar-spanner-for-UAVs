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


#include "MYIdealRadio.h"

#include "ModuleAccess.h"
#include "NodeOperations.h"
#include "NodeStatus.h"
#include <fstream>
//
#include "MYIdealChannelModel.h"

#define MK_TRANSMISSION_OVER  1
#define MK_RECEPTION_COMPLETE 2

simsignal_t MYIdealRadio::radioStateSignal = registerSignal("radioState");

Define_Module(MYIdealRadio);

MYIdealRadio::MYIdealRadio()
{
    rs = RadioState::IDLE;
    inTransmit = false;
    concurrentReceives = 0;
}

void MYIdealRadio::initialize(int stage)
{
    MYIdealChannelModelAccess::initialize(stage);

    EV << "Initializing MYIdealRadio, stage=" << stage << endl;

    if (stage == 0)
    {
        inTransmit = false;
        concurrentReceives = 0;

        upperLayerInGateId = gate("upperLayerIn")->getId();
        upperLayerOutGateId = gate("upperLayerOut")->getId();
        radioInGateId = gate("radioIn")->getId();

        gate(radioInGateId)->setDeliverOnReceptionStart(true);

        // read parameters
        transmissionRange = par("transmissionRange").doubleValue();
        bitrate = par("bitrate").doubleValue();
        drawCoverage = par("drawCoverage");
        criticalConnectivityStrip = par("criticalConnectivityStrip");

        rs = RadioState::IDLE;
        WATCH(rs);

        // signals
    }
    else if (stage == 1)
    {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        rs = isOperational ? RadioState::IDLE : RadioState::OFF;
    }
    else if (stage == 2)
    {
        emit(radioStateSignal, rs);

        // draw the interference distance
        if (ev.isGUI() && drawCoverage)
            updateDisplayString();
    }
}

void MYIdealRadio::finish()
{
}

MYIdealRadio::~MYIdealRadio()
{
    // Clear the recvBuff
    for (RecvBuff::iterator it = recvBuff.begin(); it!=recvBuff.end(); ++it)
    {
        cMessage *endRxTimer = *it;
        cancelAndDelete(endRxTimer);
    }
    recvBuff.clear();

    //handle the un-disposed msg and airframes that was sent during the run
    cOwnedObject *Del=NULL;
    int OwnedSize = this->defaultListSize();        //from MYIdealRadio take num of cObjects it owns (cPacket)

    for(int i=0;i<OwnedSize;i++){                   //run on all objects, get always the first in the q drop it (from being owned) and then delete it (cuz its now owned anymore)
        Del = this->defaultListGet(0);
        this->drop(Del);
        delete Del;
    }

}

void MYIdealRadio::handleMessage(cMessage *msg)
{
    //HILA & TSLIL

    simtime_t time=simTime();
    std::ofstream point;
    point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/total.txt",std::ios::app);
    point<<"1,simtime is:"<<time<<std::endl;
    point.close();

    if (msg->isSelfMessage())
    {   //step 1 : to received msg from SendDirect
        handleSelfMsg(msg);
    }
    else if (msg->getArrivalGateId() == upperLayerInGateId)
    {
        if (!msg->isPacket()){
            handleCommand(msg);
            return;
        }

        if (isEnabled()){
            handleUpperMsg(msg);
        }
        else{
            EV << "MYIdealRadio disabled. ignoring frame" << endl;
            delete msg;
        }
    }
    else if (msg->getArrivalGateId() == radioInGateId)
    {
        // must be an MYIdealAirFrame
        MYIdealAirFrame *airframe = check_and_cast<MYIdealAirFrame*>(msg);
//        EV_INFO<<"handleMessage  airframe->setIsFindFrame(1);"<<airframe->getIsFindFrame()<<endl;
//        EV_INFO<<"handleMessage  airframe->get name:"<<airframe->getName()<<endl;
        if (isEnabled()){
            //step 11 : to received msg from SendDirect
            handleLowerMsgStart(airframe);
        }
        else{
            EV << "MYIdealRadio disabled. ignoring airframe" << endl;
            delete msg;
        }
    }
    else
    {
        throw cRuntimeError("Model error: unknown arrival gate '%s'", msg->getArrivalGate()->getFullName());
        delete msg;
    }
}

//drone forwards msg to its linker on its way to the cc
void MYIdealRadio::forwardMYUpperMsg(MYIdealAirFrame *airframe)
{

    if (rs == RadioState::TRANSMIT)
        error("Trying to send a message while already transmitting -- MAC should "
                "take care this does not happen");

    //sendDown
    inTransmit = true;
    updateRadioState();

    simtime_t endOfTransmission = simTime() + airframe->getDuration();

    sendToMYChannel(airframe,0);      //MYIdealChannelModelAccess::sendToMYChannel

    cMessage *timer = new cMessage("endTx", MK_TRANSMISSION_OVER);
    scheduleAt(endOfTransmission, timer);

}

void MYIdealRadio::sendUp(MYIdealAirFrame *airframe)
{

    const char * currname=NULL;
    const char * currfullname=NULL;
    if(airframe->getIsFindFrame() == 1)
    {
        currfullname = this->getParentModule()->getParentModule()->getFullName();
        receivedFindMessage(airframe);
    }
    else if(airframe->getIsReplyFrame() == 1)
    {
        currfullname = this->getParentModule()->getParentModule()->getFullName();
        receivedReplyMessage(airframe);
    }
    else{
        cPacket *frame = airframe->decapsulate();

        const char * currname = this->getParentModule()->getParentModule()->getName();
        const char * currfullname = this->getParentModule()->getParentModule()->getFullName();

        if (strcmp(airframe->getName(),"MSG_TO_CC_FROM_DRONE")==0){

            if (strcmp(currname,"drone") == 0){
                //this must be active drone so need to forward msg to its linker (for it to get to the cc)

                MYIdealAirFrame *airframe3 = buildDroneToLinkerMsg(frame);
                forwardMYUpperMsg(airframe3);

            }else if (strcmp(currname,"cc") == 0){

                setCCLinker(currfullname);      // setCCLinker deals also the case where extradrone and connectivityextradrone are both used closely.

                MYIdealAirFrame *airframe3 = buildCCToExtraDroneMsg(frame);
                forwardMYUpperMsg(airframe3);
            }

        }
        else{
            /*
                Basically can be received only in the extra drone itself (should make sure anyways)
                the msg should have the coord of the target and the host[?]
                the airframe and frame Kind is actually the initiator drone id, needed in order to set the linker of the extra drone
                and in connectivityextradrone case we need just set 'setNameCoordFrameName' in what was passed in the msg by the CC (he knows whats good for us)
             */
            if(strcmp(airframe->getName(),"MSG_FROM_CC_TO_DRONE")==0){

                EV_INFO << ""<<endl;
                EV_INFO << "---------------------------------  MSG_FROM_CC_TO_DRONE RECEIVED AT EXTRADORNE  -------------------------------MYIdealRadio " << endl;
                EV_INFO << ""<<endl;
                drawCoverage = true;
                if (ev.isGUI() && drawCoverage)
                    updateDisplayString();
                if (myRadioRef->faultHandler){
                    cc->setfaultHandler(myRadioRef->radioModule->getParentModule()->getParentModule()->getFullName(),false);
                    int hostid = cc->detachNumFromFullName2(frame->getName());
                    char host[12]={0};
                    sprintf(host,"host[%d]",hostid);
                    setExtraDroneLinker(host,currfullname);
                }
                char LinkerFormat[80];
                long id;
                id = (long)frame->getKind();                            //get id in both extra drone cases
                sprintf(LinkerFormat, "drone[%ld]",id);                 //after getting the drone id in msg, need to build its fullname inorder to set as linker

                setExtraDroneLinker(currfullname,LinkerFormat);         //setting this extra drones linker according to info in the msg

                //THE EMIT SIGNAL : send the Name in the format host[?]coord.x]coord.y] or "connectivity FRAMENAME" to let drone mobility know (there it would be initialized back to NULL)
                setNameCoordFrameName(currfullname,frame->getName());       // set what was passed in the msg, to be in this extradrone 'nameCoordFrameName' so mobility to use.
                //EV_INFO << " frame->getName(): "<<frame->getName() << endl;
            }
            else{
                //step 5 : to received msg from SendDirect, the regular msgs type passed i.e. the PingAPP msgs
                EV << "sending up frame " << frame->getName() << endl;
                send(frame, upperLayerOutGateId);
            }
        }
        delete airframe;
    }
}

//MOSHE: create forwad message
MYIdealAirFrame *MYIdealRadio::buildDroneToLinkerMsg(cPacket * frame){

    // rebuilding the msg from the start
    MYIdealAirFrame *airframe2 = createAirFrame();
    //here no need to pass coord cuz just forwarding to next linker
    airframe2->setName("MSG_TO_CC_FROM_DRONE");
    airframe2->setKind(frame->getKind());
    airframe2->setTransmissionRange(transmissionRange);
    airframe2->encapsulate(frame);
    airframe2->setDuration(airframe2->getBitLength() / bitrate);
    airframe2->setTransmissionStartPosition(getRadioPosition());

    return airframe2;
}

//MOSHE: build cc message to extra drone
MYIdealAirFrame *MYIdealRadio::buildCCToExtraDroneMsg(cPacket * frame){

    //rebuild the airframe to pass the coord of the host we want extra drone to cover, and also pass the rest as should
    const char *nameCoordFrameName="NULL";
    char *tempName = (char*)malloc(sizeof(char)*128);
    int i;
    Coord extraDroneTargetPosition,extraDroneHalfTargetPosition;
    MYIdealAirFrame *airframe2 = createAirFrame();

    airframe2->setName("MSG_FROM_CC_TO_DRONE");

    char initiatorNameFormat[80];
    long id;
    id = (long)frame->getKind();
    sprintf(initiatorNameFormat, "drone[%ld]",id);                 //after getting the drone id in msg, need to build its fullname inorder to set as linker
    Coord initiatorPos = getHostPosition(initiatorNameFormat);
    if (strcmp(frame->getName(),"connectivity FRAMENAME")==0)
    {
        /*
            this is cc, have only id of initiator in frame->kind so need to rebuild hole drone[id]
            calc the coord of the settarget position that need to send the connectivityextradrone too
            attach the coord to the name after rebuild and set as 'nameCoordFrameName' cuz it would be used later on by the connectivityextradrone
            calc extraDroneTargetPosition to be the final target using the mid of the initiator and its linker
         */

        const char * initiatorLinkerName = getLinker(initiatorNameFormat);
        Coord initiatorLinkersPos = getHostPosition(initiatorLinkerName);

        extraDroneTargetPosition = {(initiatorPos.x + initiatorLinkersPos.x)/2,(initiatorPos.y + initiatorLinkersPos.y)/2,0};
        nameCoordFrameName = attachCoordToName(initiatorNameFormat,extraDroneTargetPosition);
    }
    else{
        char nameCoordFrameNameSTR[80];
        char * pch;
        strcpy(nameCoordFrameNameSTR,frame->getName());
        pch=strtok(nameCoordFrameNameSTR,"[");
        if(strcmp(pch,"drone")==0){
            pch=strtok(NULL,"]");
            int linker=atoi(pch);
            frame->setKind(linker);
            pch=strtok(NULL,"\0");
            nameCoordFrameName=pch;
        }
        else{
            //we rebuild frame->setName() - so when extradrone receives FROM CC its "host[?]COORD",
            //so we need to rebuild it and just in this case, when the extradrone is the receiver its to get it out
            extraDroneTargetPosition = getHostPosition(frame->getName());
            extraDroneHalfTargetPosition = {(initiatorPos.x + extraDroneTargetPosition.x)/2,(initiatorPos.y + extraDroneTargetPosition.y)/2,0};

            //EV_INFO << "KEEP KEEP        ----------------------- Initiator at :("<< initiatorPos.x <<","<<initiatorPos.y <<",0) extraDroneTargetPosition(host Real) at : "<< extraDroneTargetPosition<<"extraDroneHalfTargetPosition(middle) at : "<< extraDroneHalfTargetPosition<<" ------------------------------- " << endl;

            nameCoordFrameName = attachCoordToName(frame->getName(),extraDroneHalfTargetPosition);
        }
    }

    for(i=0;i<128;i++)
        tempName[i]=0;
    i=0;
    while(nameCoordFrameName[i]!= 0)
    {
        tempName[i]=nameCoordFrameName[i];
        i++;
    }
    frame->setName(tempName);
    airframe2->setKind(frame->getKind());                           //passes the kind like received in both airframe and frame
    airframe2->setTransmissionRange(transmissionRange);
    airframe2->encapsulate(frame);                                  // PASS THE COORD IN THE FRAME NAME, EXTRA DRONE NEEDS TO KNOW WHERE TO GO
    airframe2->setDuration(airframe2->getBitLength() / bitrate);
    airframe2->setTransmissionStartPosition(getRadioPosition());

    return airframe2;
}

const char * MYIdealRadio::attachIdToDroneLinker(int droneId)
{
    char LinkerFormatstr[80];

    std::ostringstream s;
    s << droneId;
    std::string output = s.str();
    const char* Idstr = output.c_str();

    strcpy (LinkerFormatstr,"drone[");
    strcat (LinkerFormatstr,Idstr);
    strcat (LinkerFormatstr,"]");

    return LinkerFormatstr;
}

const char *MYIdealRadio::attachCoordToName(const char* currfullname, Coord coord ){

    double first = coord.x;
    double second = coord.y;

    std::ostringstream s;
    s << first;
    std::string output = s.str();
    const char* firststr = output.c_str();

    std::ostringstream s1;
    s1 << second;
    std::string output1 = s1.str();
    const char* secondstr = output1.c_str();

    char nameCoordFrameNamestr[80];
    strcpy (nameCoordFrameNamestr,currfullname);
    strcat (nameCoordFrameNamestr,firststr);
    strcat (nameCoordFrameNamestr,"]");
    strcat (nameCoordFrameNamestr,secondstr);
    strcat (nameCoordFrameNamestr,"]");

    return nameCoordFrameNamestr;
}

const char *MYIdealRadio::attachLinkerToCoordName(const char* linker, const char* coordFramename){
    char nameCoordFrameNamestr[80];
    strcpy (nameCoordFrameNamestr,linker);
    strcat (nameCoordFrameNamestr,coordFramename);
    return nameCoordFrameNamestr;
}

void MYIdealRadio::sendDown(MYIdealAirFrame *airframe)
{
    // change radio status
    EV << "sending, changing RadioState to TRANSMIT\n";
    inTransmit = true;
    updateRadioState();

    simtime_t endOfTransmission = simTime() + airframe->getDuration();
    sendToChannel(airframe);
    cMessage *timer = new cMessage("endTx", MK_TRANSMISSION_OVER);
    scheduleAt(endOfTransmission, timer);
}

//MOSHE: send extra drone msg in order to regain cover from replacing drone
void MYIdealRadio::initiateExtraDroneReplaceyMsg(){

    if(isEnabled())
    {
        long connectivityVal = -10 ;
        handleMYUpperDroneMsg(connectivityVal,0);
    }
}

/**
 * If a message is already being transmitted, an error is raised.
 *
 * Otherwise the RadioState is set to TRANSMIT and a timer is
 * started. When this timer expires the RadioState will be set back to RECV
 * (or IDLE respectively) again.
 */
void MYIdealRadio::handleUpperMsg(cMessage *msg)
{
    MYIdealAirFrame *airframe = encapsulatePacket(PK(msg));
    EV_INFO<<"handleUpperMsg  airframe->setIsFindFrame(1);"<<airframe->getIsFindFrame()<<endl;
    EV_INFO<<"handleUpperMsg  airframe->get name:"<<airframe->getName()<<endl;

    if (rs == RadioState::TRANSMIT)
        error("Trying to send a message while already transmitting -- MAC should "
                "take care this does not happen");

    sendDown(airframe);
}

//MOSHE: send extra drone message
void MYIdealRadio::initiateExtraDroneConnectivityMsg(){

    if(isEnabled())
    {
        long connectivityVal = -7 ;
        handleMYUpperDroneMsg(connectivityVal,0);
    }
}

//MOSHE: set location and in nameCoordFrame for mobility
void MYIdealRadio::initiateFaultMsg(MYIdealAirFrame *airframe){
    int i,id;
    const char *coordFrame="NULL";
    char *initDrone = (char*)malloc(sizeof(char)*128), *fullname = (char*)malloc(sizeof(char)*128), *cordFrameName = (char*)malloc(sizeof(char)*128);
    Coord tempCoord;
    cc->setDroneOwnedSize(myRadioRef->linker,1);
    tempCoord.x=airframe->getMinCoverLocation().x;
    tempCoord.y=airframe->getMinCoverLocation().y;
    tempCoord.z=airframe->getMinCoverLocation().z;
    //sprintf(coordFrame,"drone[%d]%f]%f]",airframe->getInitiatorID(),airframe->getMinCoverLocation().x,airframe->getMinCoverLocation().y );
    sprintf(fullname,"drone[%d]",airframe->getMinCoverID());
    id=cc->findMyHostId(fullname);
    sprintf(initDrone,"drone[%d]",airframe->getInitiatorID());
    coordFrame=attachCoordToName(initDrone,tempCoord);
    EV_INFO<<"coordFrame:"<<coordFrame<<" fullname:"<<fullname<<"initDrone"<<initDrone<<endl;
    for(i=0;i<128;i++)
        cordFrameName[i]=0;
    i=0;
    while(coordFrame[i]!= 0)
    {
        cordFrameName[i]=coordFrame[i];
        i++;
    }
    airframe->setMinCoverID(detachNumFromFullName3(myRadioRef->linker));
    setNameCoordFrameName(fullname,cordFrameName);
    setExtraDroneLinker(initDrone,fullname);            //set raeplace as recognize drone linker
    cc->setnextlinker(fullname,airframe->getreplaceLinker());       //set nextlinker on replacing drone in order to set it as linker when arrived
    setsendToNameCoordFrameName(fullname,true);                 //set ro relocate
}

//MOSHE: set the location for asking cc for replace drone
void MYIdealRadio::setReplaceCoordName(MYIdealAirFrame *airframe){
    int i;
    const char *coordFrame="NULL";
    char tempNameLinker[12]={0};
    char *replaceHost = (char*)malloc(sizeof(char)*128), *fullname = (char*)malloc(sizeof(char)*128), *cordFrameName = (char*)malloc(sizeof(char)*128);
    Coord tempCoord;
    tempCoord.x=airframe->getMinCoverLocation().x;
    tempCoord.y=airframe->getMinCoverLocation().y;
    tempCoord.z=airframe->getMinCoverLocation().z;
    //sprintf(coordFrame,"drone[%d]%f]%f]",airframe->getInitiatorID(),airframe->getMinCoverLocation().x,airframe->getMinCoverLocation().y );
    sprintf(replaceHost,"host[%d]",airframe->getMinCoverID());
    sprintf(fullname,"drone[%d]",airframe->getInitiatorID());
    coordFrame=attachCoordToName(replaceHost,tempCoord);
    EV_INFO<<"coordFrame:"<<coordFrame<<" fullname:"<<fullname<<" replaceHost: "<<replaceHost<<endl;
    int linkerID=airframe->getMinCoverID();         //adding linker to coordname
    sprintf(tempNameLinker,"drone[%d]",linkerID);
    coordFrame=attachLinkerToCoordName(tempNameLinker,coordFrame);
    for(i=0;i<128;i++)
        cordFrameName[i]=0;
    i=0;
    while(coordFrame[i]!= 0)
    {
        cordFrameName[i]=coordFrame[i];
        i++;
    }

    setNameCoordFrameName(fullname,cordFrameName);
    cc->setmoveID(fullname,linkerID);
}

void MYIdealRadio::handleMYUpperDroneMsg(long hostNum,int signalID)
{
    int hostID, linkerID;
    Coord place;
    Enter_Method_Silent();
    EV_INFO<<"HHHHHHHHHHH handleMYUpperDroneMsg TTTTTTTTTTTTTTTTT"<<endl;
    // build the initiating msg from drone (comes from signal)
    char name[32],targetHost[32];
    if(signalID == (int)faultSignal){
        sprintf(targetHost, "FINDING");
    }
    else if(signalID == (int)droneToDrone)
    {
        sprintf(targetHost, "FIND_FRAME");
    }
    else if(hostNum == -7) // extradrone connectivity case framename
    {
        EV_INFO << " ---------------------  HANDLING CONNECTIVITY ALERT CASE ---------------------------- MYIdealRadio "<< endl;
        sprintf(targetHost, "connectivity FRAMENAME");
    }
    else if(hostNum == -10){//asking for drone to replace the one we replace with fault
        EV_INFO << " ---------------------  HANDLING REPLACE ALERT CASE ---------------------------- MYIdealRadio " << endl;
        sprintf(targetHost,myRadioRef->nameCoordFrameName);
    }
    else{       // extradrone regular case
        EV_INFO << " ---------------------  HANDLING CRITICAL ALERT CASE ---------------------------- MYIdealRadio " << endl;
        sprintf(targetHost, "host[%ld]", hostNum);
    }


    cPacket *frame = new cPacket();
    frame->setName(targetHost);
    int initiatorDroneId = detachNumFromFullName();
    frame->setKind(initiatorDroneId);           //SHOULD BE THE CURR DRONE INITIATOR - BUT !! only if not used in any other place

    cMessage *msg2 = frame;
    frame = check_and_cast<cPacket *>(msg2);

    delete frame->removeControlInfo();

    MYIdealAirFrame *airframe = createAirFrame();
    if(signalID == (int)faultSignal){
        int k,i;
        sprintf(name, "FindMsgForFaultDrones");
        airframe->setCurrDepth_lvl(0);
        k = this->getParentModule()->getParentModule()->getParentModule()->par("K_parameter");
        airframe->setK_parameter(k);
        airframe->setIsFindFrame(1);
        airframe->setIsRelocationFrame(0);
        airframe->setIsReplyFrame(0);
        airframe->setInitiatorID(detachNumFromFullName());
        airframe->setLastSender(detachNumFromFullName());
        airframe->setMinCover(INT_MAX);
        airframe->setreplaceLinker(getLinker(myRadioRef->linker));
        airframe->setMinCoverID(-1);
        for(i=0 ;i< 2048; i++)
            airframe->setDroneDatabase(i,0);
    }
    else if(signalID == (int)droneToDrone)
    {
        int k,i;
        sprintf(name, "FindMsgForCoverDrones");
        airframe->setCurrDepth_lvl(0);
        k = this->getParentModule()->getParentModule()->getParentModule()->par("K_parameter");
        airframe->setK_parameter(k);
        airframe->setIsFindFrame(1);
        airframe->setIsRelocationFrame(0);
        airframe->setIsReplyFrame(0);
        airframe->setInitiatorID(detachNumFromFullName());
        airframe->setLastSender(detachNumFromFullName());
        airframe->setFrame_ID(frameID_counter++);
        for(i=0 ;i< 2048; i++)
            airframe->setDroneDatabase(i,0);
    }
//   // Tslil & Hila
//    else if(signalID == (int)establishSubGraph){
//        EV_INFO<<"HHHHHHHHHHHHHHHHHHHHH receive signal send msg TTTTTTTTTTTTTTTTTTTTTT"<<endl;
//        int k,i;
//        double R_u=culc_beta();
//        sprintf(name, "FindMsgForEstablishSubGraph");
//        airframe->setCurrDepth_lvl(0);
//        k = this->getParentModule()->getParentModule()->getParentModule()->par("K_parameter");
//        airframe->setK_parameter(k);
//        airframe->setCurrK(k);
//        airframe->setR_u(R_u);
//        airframe->setIsFindFrame(1);
//        EV_INFO<<"  airframe->setIsFindFrame(1);"<<airframe->getIsFindFrame()<<endl;
//        airframe->setIsRelocationFrame(0);
//        airframe->setIsReplyFrame(0);
//        airframe->setInitiatorID(detachNumFromFullName());
//        airframe->setLastSender(detachNumFromFullName());
//        airframe->setFrame_ID(frameID_counter++);
//        for(i=0 ;i< 2048; i++)
//            airframe->setDroneDatabase(i,0);
//    }
    else
        sprintf(name, "MSG_TO_CC_FROM_DRONE");

    airframe->setName(name);
    airframe->setKind(frame->getKind());
    airframe->setTransmissionRange(transmissionRange);
    airframe->encapsulate(frame);
    airframe->setDuration(airframe->getBitLength() / bitrate);
    airframe->setTransmissionStartPosition(getRadioPosition());


    //    if (rs == RadioState::TRANSMIT)
    //        error("Trying to send a message while already transmitting -- MAC should "
    //              "take care this does not happen");

    //sendDown
    inTransmit = true;
    updateRadioState();

    simtime_t endOfTransmission = simTime() + airframe->getDuration();
    EV_INFO<<"signal id: "<<signalID<<(int)establishSubGraph<<endl;
    if(signalID == (int)droneToDrone || signalID == (int)faultSignal)// || signalID == (int)establishSubGraph)
    {
        sendToMYChannel(airframe,signalID);
    }
    else
        sendToMYChannel(airframe,0);      //MYIdealChannelModelAccess::sendToMYChannel

    cMessage *timer = new cMessage("endTx", MK_TRANSMISSION_OVER);
    scheduleAt(endOfTransmission, timer);
}

// Tslil & Hila
void MYIdealRadio::handleMYUpperDroneMsg(const char * droneAndLinker,int signalID)
{
    int hostID, linkerID;
    Coord place;
    Enter_Method_Silent();
    EV_INFO<<"HHHHHHHHHHH handleMYUpperDroneMsg TTTTTTTTTTTTTTTTT"<<endl;
    // build the initiating msg from drone (comes from signal)
    char name[32],targetHost[32];

        EV_INFO << " ---------------------  HANDLING CRITICAL ALERT CASE ---------------------------- MYIdealRadio " << endl;
        sprintf(targetHost, "establishFrame");

    cPacket *frame = new cPacket();
    frame->setName(targetHost);
    int initiatorDroneId = detachNumFromFullName();
    frame->setKind(initiatorDroneId);           //SHOULD BE THE CURR DRONE INITIATOR - BUT !! only if not used in any other place

    cMessage *msg2 = frame;
    frame = check_and_cast<cPacket *>(msg2);

    delete frame->removeControlInfo();

    MYIdealAirFrame *airframe = createAirFrame();

    if(signalID == (int)establishSubGraph){
        EV_INFO<<"HHHHHHHHHHHHHHHHHHHHH receive signal send msg TTTTTTTTTTTTTTTTTTTTTT"<<endl;
        int k,i;
        std::string initator1= std:: string(droneAndLinker,0,8);
        std::string linker1 = std::string(droneAndLinker,8,8);
        const char * initiator=initator1.c_str();
        const char * linker=linker1.c_str();
        double R_u=culc_R_u();
        sprintf(name, "FindMsgForEstablishSubGraph");
        airframe->setCurrDepth_lvl(0);
        k = this->getParentModule()->getParentModule()->getParentModule()->par("K_parameter");
        airframe->setK_parameter(k);
        airframe->setCurrK(k);
        airframe->setR_u(R_u);
        airframe->setIsFindFrame(1);
        airframe->setIsRelocationFrame(0);
        airframe->setIsReplyFrame(0);
        airframe->setInitiatorID(detachNumFromFullName(initiator));
        airframe->setLastSender(detachNumFromFullName());
        airframe->setFrame_ID(frameID_counter++);
        airframe->setInitLinker(detachNumFromFullName(linker));
        airframe->setSenderLinker(detachNumFromFullName(getLinker(myRadioRef->linker)));
        for(i=0 ;i< 2048; i++)
            airframe->setDroneDatabase(i,0);
    }
    else
        sprintf(name, "MSG_TO_CC_FROM_DRONE");

    airframe->setName(name);
    airframe->setKind(frame->getKind());
    airframe->setTransmissionRange(transmissionRange);
    airframe->encapsulate(frame);
    airframe->setDuration(airframe->getBitLength() / bitrate);
    airframe->setTransmissionStartPosition(getRadioPosition());


    //sendDown
    inTransmit = true;
    updateRadioState();

    simtime_t endOfTransmission = simTime() + airframe->getDuration();
    EV_INFO<<"signal id: "<<signalID<<(int)establishSubGraph<<endl;

    sendToMYChannel(airframe,0);      //MYIdealChannelModelAccess::sendToMYChannel

    cMessage *timer = new cMessage("endTx", MK_TRANSMISSION_OVER);
    scheduleAt(endOfTransmission, timer);
}

//ONLY DRONEs RECEIVE THIS SIGNAL which tells it to send MSG_TO_CC_FROM_DRONE to its linker (and this linker would need to know what to do with it, linker can be cc or drone)
void MYIdealRadio::receiveSignal(cComponent *source, simsignal_t signalID, long hostNum)
{
    //HILA & TSLIL
    simtime_t time=simTime();
    std::ofstream point;
    point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/total.txt",std::ios::app);
    point<<"1,simtime is:"<<time<<std::endl;
    point.close();

    //EV_INFO << "KEEP KEEP         MYIdealRadio::receiveSignal, signalID = " << signalID << " CCCCCCCCCCCCCCCCCCBB" <<endl;

    ///Construct here the msg as should be with relevant data as needed on the other side
    //cMessage *msg is the what received in handlemsg and should be set here the same and just passed to handleMYUpperMsg as msg
    if(signalID == droneToDrone || signalID == faultSignal || signalID == establishSubGraph)
        handleMYUpperDroneMsg(hostNum,signalID);
    else if(isEnabled())
        handleMYUpperDroneMsg(hostNum,0);
}
void MYIdealRadio::receiveSignal(cComponent *source, simsignal_t signalID, const char * droneAndLinker)
{
    //HILA & TSLIL
    simtime_t time=simTime();
    std::ofstream point;
    point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/total.txt",std::ios::app);
    point<<"1,simtime is:"<<time<<std::endl;
    point.close();

    //EV_INFO << "KEEP KEEP         MYIdealRadio::receiveSignal, signalID = " << signalID << " CCCCCCCCCCCCCCCCCCBB" <<endl;

    ///Construct here the msg as should be with relevant data as needed on the other side
    //cMessage *msg is the what received in handlemsg and should be set here the same and just passed to handleMYUpperMsg as msg
    if(signalID == establishSubGraph)
        handleMYUpperDroneMsg(droneAndLinker ,signalID);
    else if(isEnabled())
        handleMYUpperDroneMsg(droneAndLinker,0);
}
void MYIdealRadio::handleCommand(cMessage *msg)
{
    throw cRuntimeError("Command '%s' not accepted.", msg->getName());
}

void MYIdealRadio::handleSelfMsg(cMessage *msg)
{

    EV << "MYIdealRadio::handleSelfMsg" << msg->getKind() << endl;
    if (msg->getKind() == MK_RECEPTION_COMPLETE)
    {   //step 2 : to received msg from SendDirect
        EV << "frame is completely received now\n";

        // unbuffer the message
        MYIdealAirFrame *airframe = check_and_cast<MYIdealAirFrame *>(msg->removeControlInfo());
        EV_INFO<<"handleSelfMsg  airframe:"<<airframe->getName()<<endl;
        EV_INFO<<" handleSelfMsg airframe->setIsFindFrame(1);"<<airframe->getIsFindFrame()<<endl;
        bool found = false;
        for (RecvBuff::iterator it = recvBuff.begin(); it != recvBuff.end(); ++it)
        {
            if (*it == msg)
            {
                recvBuff.erase(it);
                found = true;
                break;
            }
        }
        if (!found)
            throw cRuntimeError("Model error: self message not found in recvBuff buffer");
        delete msg;
        //step 3 : to received msg from SendDirect
        handleLowerMsgEnd(airframe);
    }
    else if (msg->getKind() == MK_TRANSMISSION_OVER)
    {
        inTransmit = false;
        // Transmission has completed. The RadioState has to be changed
        // to IDLE or RECV, based on the noise level on the channel.
        // If the noise level is bigger than the sensitivity switch to receive mode,
        // otherwise to idle mode.

        // delete the timer
        delete msg;

        updateRadioState(); // now the radio changes the state and sends the signal
        EV << "transmission over, switch to state: "<< RadioState::stateName(rs) << endl;
    }
    else
    {
        error("Internal error: unknown self-message `%s'", msg->getName());
    }
    //step 6 : to received msg from SendDirect msg sent in step 5
    EV << "MYIdealRadio::handleSelfMsg END" << endl;
}

//MOSHE: set schedule at end of transmission.
/**
 * This function is called right after a packet arrived, i.e. right
 * before it is buffered for 'transmission time'.
 *
 * The message is not treated as noise if the transmissionRange of the
 * received signal is higher than the distance.
 *
 * Update radioState.
 */
void MYIdealRadio::handleLowerMsgStart(MYIdealAirFrame *airframe)
{
    EV << "receiving frame " << airframe->getName() << endl;
    EV_INFO<<"handleLowerMsgStart  airframe->setIsFindFrame(1);"<<airframe->getIsFindFrame()<<endl;
    EV_INFO<<"handleLowerMsgStart  airframe->get name:"<<airframe->getName()<<endl;
    cMessage *endRxTimer = new cMessage("endRx", MK_RECEPTION_COMPLETE);
    endRxTimer->setControlInfo(airframe);
    recvBuff.push_back(endRxTimer);

    // NOTE: use arrivalTime instead of simTime, because we might be calling this
    // function during a channel change, when we're picking up ongoing transmissions
    // on the channel -- and then the message's arrival time is in the past!
    scheduleAt(airframe->getArrivalTime() + airframe->getDuration(), endRxTimer);

    concurrentReceives++;
    // check the RadioState and update if necessary
    updateRadioState();
}

/**
 * This function is called right after the transmission is over.
 * Additionally the RadioState has to be updated.
 */
void MYIdealRadio::handleLowerMsgEnd(MYIdealAirFrame *airframe)
{
    EV_INFO<<"handleLowerMsgEnd  airframe->setIsFindFrame(1);"<<airframe->getIsFindFrame()<<endl;
    EV_INFO<<"handleLowerMsgEnd  airframe->get name:"<<airframe->getName()<<endl;
    //step 4 : to received msg from SendDirect
    concurrentReceives--;
    sendUp(airframe);
    updateRadioState();
}

bool MYIdealRadio::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();
    if (dynamic_cast<NodeStartOperation *>(operation)) {
        if (stage == NodeStartOperation::STAGE_PHYSICAL_LAYER)
            setRadioState(RadioState::IDLE);            //FIXME only if the interface is up, too
    }
    else if (dynamic_cast<NodeShutdownOperation *>(operation)) {
        if (stage == NodeStartOperation::STAGE_PHYSICAL_LAYER)
            setRadioState(RadioState::OFF);
    }
    else if (dynamic_cast<NodeCrashOperation *>(operation)) {
        if (stage == NodeStartOperation::STAGE_LOCAL)  // crash is immediate
            setRadioState(RadioState::OFF);
    }
    return true;

}

void MYIdealRadio::setRadioState(RadioState::State newState)
{
    if (rs != newState)
    {
        if (newState == RadioState::SLEEP || newState == RadioState::OFF)
            cc->disableReception(myRadioRef);
        else
            cc->enableReception(myRadioRef);

        rs = newState;
        if (rs == RadioState::SLEEP || newState == RadioState::OFF)
        {
            // Clear the recvBuff
            for (RecvBuff::iterator it = recvBuff.begin(); it!=recvBuff.end(); ++it)
            {
                cMessage *endRxTimer = *it;
                cancelAndDelete(endRxTimer);
            }
            recvBuff.clear();
        }
        emit(radioStateSignal, newState);
    }
}

void MYIdealRadio::updateRadioState()
{
    RadioState::State newState = isEnabled()
                                                                            ? (inTransmit ? RadioState::TRANSMIT : ((concurrentReceives>0) ? RadioState::RECV : RadioState::IDLE))
                                                                                    : RadioState::SLEEP;
    setRadioState(newState);
}

void MYIdealRadio::updateDisplayString()
{
    if (myRadioRef) {
        cDisplayString& d = hostModule->getDisplayString();

        // communication area
        // FIXME this overrides the ranges if more than one radio is present is a host
        d.removeTag("r1");
        d.insertTag("r1");
        d.setTagArg("r1", 0, (long) transmissionRange);
        d.setTagArg("r1", 2, "gray");
        d.removeTag("r2");
    }
}


MYIdealAirFrame *MYIdealRadio::encapsulatePacket(cPacket *frame)
{
    delete frame->removeControlInfo();

    // Note: we don't set length() of the MYIdealAirFrame, because duration will be used everywhere instead
    MYIdealAirFrame *airframe = createAirFrame();
    airframe->setName(frame->getName());
    airframe->setTransmissionRange(transmissionRange);
    airframe->encapsulate(frame);
    airframe->setDuration(airframe->getBitLength() / bitrate);
    airframe->setTransmissionStartPosition(getRadioPosition());

    EV << "Frame (" << frame->getClassName() << ")" << frame->getName()
                                                                       << " will be transmitted at " << (bitrate/1e6) << "Mbps" << endl;
    return airframe;
}


//on every movement from mobility, the signal would be received here, b4 going to MYIdealChannelModelAccess
void MYIdealRadio::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj){
    //EV_INFO << "KEEP KEEP         MYIdealRadio::receiveSignal, signalID = " << signalID << " CCCCCCCCCCCCCCCCCCAA" <<endl;

    //receive the "mobilityStateChangedSignal" signal and pass it to MYIdealChannelModelAccess::receiveSignal
    MYIdealChannelModelAccess::receiveSignal(source, signalID, obj);
}


//TSLIL & HILA: calculate beta value for exponential probability
double MYIdealRadio::culc_beta(){
    double c_par=this->getParentModule()->getParentModule()->getParentModule()->par("C_parameter");
    int number_Of_Hosts = sizeof(this->final_List)/sizeof(DronesList);
    int k_par=this->getParentModule()->getParentModule()->getParentModule()->par("K_parameter");

    return log(c_par*number_Of_Hosts)/k_par;
}

double MYIdealRadio::culc_R_u(){
    double r_u= exponential(1/culc_beta());
    return r_u;
}
