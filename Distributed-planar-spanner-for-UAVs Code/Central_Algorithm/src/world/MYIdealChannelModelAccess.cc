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

#include "MYIdealChannelModelAccess.h"
#include <fstream>
#include "IMobility.h"
#include "MYIdealRadio.h"
//zmani
#include "MYLineSegmentsMobilityBase.h"
#include "MYIdealChannelModel.h"
#include "MYConstSpeedMobilityDrone.h"
#define coreEV (ev.isDisabled()||!coreDebug) ? EV : EV << logName() << "::MYIdealChannelModelAccess: "

static int Counter = 0;

simsignal_t MYIdealChannelModelAccess::mobilityStateChangedSignal =
        registerSignal("mobilityStateChanged");
simsignal_t MYIdealChannelModelAccess::extraDroneToCCSignal = registerSignal(
        "extraDroneToCC");
simsignal_t MYIdealChannelModelAccess::droneToDrone = registerSignal(
        "msgToDrone");
simsignal_t MYIdealChannelModelAccess::faultSignal = registerSignal(
        "msgFaultDrone");
simsignal_t MYIdealChannelModelAccess::establishSubGraph = registerSignal(
        "msgEstablishSubGraph"); // Tslil & Hila

// the distructor unregister the radio module
MYIdealChannelModelAccess::~MYIdealChannelModelAccess() {
    if (cc && myRadioRef) {
        // check if channel control exist
        MYIdealChannelModel *cc =
                dynamic_cast<MYIdealChannelModel *>(simulation.getModuleByPath("channelControl"));
                if (cc)
                cc->unregisterRadio(myRadioRef);
                myRadioRef = NULL;
            }
        }

        /**
         * Upon initialization MYIdealChannelModelAccess registers the nic parent module
         * to have all its connections handled by ChannelControl
         */
void MYIdealChannelModelAccess::initialize(int stage) {
    BasicModule::initialize(stage);

    if (stage == 0) {
        cc =
                dynamic_cast<MYIdealChannelModel *>(simulation.getModuleByPath("channelControl"));
                if (!cc)
                throw cRuntimeError("Could not find MYIdealChannelModel module with name 'channelControl' in the toplevel network.");

                hostModule = findHost();

                positionUpdateArrived = false;
                isWaitingForConnectivity = false;

                // register to get a notification when position changes
                hostModule->subscribe(mobilityStateChangedSignal, this);
                hostModule->subscribe(extraDroneToCCSignal, this);
                hostModule->subscribe(droneToDrone, this);
                hostModule->subscribe(faultSignal, this);
                hostModule->subscribe(establishSubGraph, this);

                numOfDrones = getParentModule()->getParentModule()->getParentModule()->par("numDrones");
                numOfHosts = getParentModule()->getParentModule()->getParentModule()->par("numHosts");

                min_currDepthLvl_Receveid = 10000000;//   reset to an infinite value

                int i,j;
                for(i=0; i<INF_DRONE; i++)
                droneArray[i] = -1;
                for(i=0; i<INF_MessageID_Numner; i++)
                {
                    for(j=0; j<DRONE_BUFFER; j++)
                    databaseArray[i].databaseArray[j]=0;
                    databaseArray[i].messageCounter = 0;
                    databaseArray[i].messageID = -1;
                }

            }
            else if (stage == 2)
            {
                if (!positionUpdateArrived)
                throw cRuntimeError("The coordinates of '%s' host are invalid. Please configure Mobility for this host.", hostModule->getFullPath().c_str());

                myRadioRef = cc->registerRadio(this);
                cc->setRadioPosition(myRadioRef, radioPos);
                update_soldier_array();
            }

        }

        /**
         * This function has to be called whenever a packet is supposed to be
         * sent to the channel.
         *
         * This function really sends the message away, so if you still want
         * to work with it you should send a duplicate!
         */
void MYIdealChannelModelAccess::receivedFindMessage(MYIdealAirFrame *airframe) {
    int currDepthLvl = airframe->getCurrDepth_lvl();
    int newSender = detachNumFromFullName();
    int lastSender = airframe->getLastSender();
    int kParameter = airframe->getK_parameter();
    int msgCounter, index;
    int frameID = airframe->getFrame_ID();
    bool ChangeToReply = false;
    MYIdealChannelModel::RadioEntry myRadioRef3;
    EV_INFO<<"airframe name:"<<airframe->getName()<<endl;
    ///////////Adi & Tamir ////////////////////////////////////////////////////
    EV_INFO<< "MYIdealChannelModelAccess::receivedFindMessage ME: " << getFullName() << endl << "my parent: " << getParentModule()->getFullName() << endl << "MY GREND PARENT: " << getParentModule()->getParentModule()->getFullName() << endl;
    int priority_type = airframe->priority_message_type;
    //check if this is a priority type message. if yes, which type? , if no, continue
    if (priority_type) {
        //get my id number
        int myid =
                detachNumFromFullName3(
                        getParentModule()->getParentModule()->getParentModule()->getFullName());
        int sum = 0; // sum of cur priorities
        EV_INFO<<"MMMMMM priority (1-request , 2-reply , 3-move): "<< priority_type<<"MMMMMMMMMMMMMMMM"<<endl;
        if (priority_type == PRIORITY_REQUEST) { //check if im a leaf . if yes - sum all covered prio hosts by me
            if (IsLeafDrone()) {
                int num_of_covered_hosts = par("num_Of_Hosts_Cover");
                for (int i = 0; i < num_of_covered_hosts; i++) {
                    sum +=
                            (int) getParentModule()->getParentModule()->getSubmodule(
                                    "MYNodeBase", i)->par("priority");
                }
                if (airframe->priority_sum > sum) {
                    airframe->priority_message_type = PRIORITY_REPLY;
                    airframe->priority_sum = sum;
                    airframe->dst_sent_drone_id = airframe->src_sent_drone_id;
                    airframe->src_sent_drone_id = myid;
                    //send priority reply message
                }
                //  airframe->priority_sum=sum;

            }

            // if cur drone is not a leaf, forward the message
        }

        ////*************************** REPLY MESSAGE ************************************
        else if (priority_type == PRIORITY_REPLY
                && airframe->dst_sent_drone_id == myid) { // update DB with min sum and and source drone
            priority_sums[airframe->src_sent_drone_id] = airframe->priority_sum;
            //wait until all reply messages arrived
            if ((simTime()
                    - getParentModule()->getParentModule()->getSubmodule(
                            "MYIdealChannelModel")->par(
                            "REQUEST_message_time_sent")) > 2) { // wait until all drones had enough time to answer
                /* send_MOVE_message to the drone with the min sum  */
                int min_sum = 9999, min_idx;
                for (int i = 0; i < 20; i++) {
                    if (min_sum > priority_sums[i] && priority_sums[i]) {
                        min_sum = priority_sums[i];
                        min_idx = i;
                    }
                }

                //send Move message with drone id to move
                airframe->priority_message_type = PRIORITY_MOVE;
                airframe->dst_sent_drone_id = min_idx;
                airframe->src_sent_drone_id = myid;
                //sendmove
            }
        }

        //************************MOVE MESSAGE***************************************
        else if (priority_type == PRIORITY_MOVE) { // change ownership of overlapping hosts and move to new location
            // change ownership and maybe move to new location
            // cc2=getParentModule()->getParentModule()->getSubmodule("MYMobilityBase")->getSubmodule("MYMovingMobilityBase")->getSubmodule("MYLineSegmentMobilityBase")->getSubmodule("MYConstSpeedMobilityDrone")->par();
            //if you are the drone who need to move, change ownership and move
            if (myid == airframe->dst_sent_drone_id) {
                //needs to be current drone (a leaf which is the one who need to move
                //Drone_Info *d;
                //for  change ownership
                char* newdroneFullName = (char*) malloc(sizeof(char) * 8);
                char* hostFullName = (char*) malloc(sizeof(char) * 8);
                // sprintf(newdroneFullName,"host[%d]",d->drone_ID);
                for (HostList::iterator it = hostList.begin();
                        it != hostList.end(); ++it) {
                    Host_Info *host = &*it;
                    it->coverDrone_ID;
                    //    sprintf(newdroneFullName,"host[%d]",);
                    sprintf(hostFullName, "host[%d]", it->host_ID);
                    cc2->setNewHostOwner((const char*) newdroneFullName,
                            (const char*) hostFullName);
                }
                //MYIdealRadio *r;
                MYLineSegmentsMobilityBase * drone;
                //this->priority_move_case=true;
                //update that we are in priority case
                getParentModule()->getParentModule()->getSubmodule(
                        "MYMobilityBase")->getSubmodule("MYMovingMobilityBase")->getSubmodule(
                        "MYLineSegmentMobilityBase")->par("priority_case").setBoolValue(
                        true);
                //update the position which the chosen drone needs to be located
                getParentModule()->getParentModule()->getParentModule()->par(
                        "xPosition").setDoubleValue(airframe->xLocation);
                getParentModule()->getParentModule()->getParentModule()->par(
                        "yPosition").setDoubleValue(airframe->yLocation);
                //move the drone to the new chosen location
                drone->move();
                //update the priority case is done . now we are not in priority case
                getParentModule()->getParentModule()->getParentModule()->par(
                        "priority_case").setBoolValue(false);
                // this->priority_move_case=false;
            }
        }

    }

    //***************************************************************************************************/
    if (isFrmaeExist(frameID)
            && strcmp(airframe->getName(), "FindMsgForEstablishSubGraph") != 0) // check if this message has been received before - if so: send it back as a Junk frame - Mark it as a Leaf-linker and the receiver will just ignore it
                    {
        EV_INFO<<"deleteEstablish"<<endl;
        airframe->setLastSender(newSender);
        airframe->setIsFindFrame(0);
        airframe->setIsReplyFrame(1);
        cc->send_Reply_Message(myRadioRef, airframe,lastSender);
    }
    else if (!strcmp(airframe->getName(),"ChosenDroneFind")) { //MOSHE: drone has been drone message with fault drone location
        char tempName[12]= {0};
        sprintf(tempName,"drone[%d]",myRadioRef->ID);
        Coord position2send=cc->getHostPosition(tempName);
        if (airframe->getMinCoverID() == myRadioRef->ID) {
            check_and_cast<MYIdealRadio *>(myRadioRef->radioModule)->initiateFaultMsg(airframe);
            airframe->setCurrDepth_lvl(0);
            airframe->setLastSender(newSender);
            airframe->setIsFindFrame(0);
            airframe->setIsReplyFrame(1);
            airframe->setMinCoverLocation(position2send); //MOSHE: sending my location
            airframe->setMinCoverID(detachNumFromFullName3(myRadioRef->linker));//sending my linker ID
            airframe->setName("ReplyMsgFromChosenDrone");
            cc->send_Reply_Message(myRadioRef, airframe,lastSender);
            cc->setDroneOwnedSize(tempName,0);
        }
        else {
            if (airframe->getMinCoverID()==cc->detachNumFromFullName2(myRadioRef->linker) && cc->getDroneOwnedSize(myRadioRef->linker)==0)
            cc->setDroneOwnedSize(myRadioRef->linker,1);
            airframe->setCurrDepth_lvl(currDepthLvl+1);
            airframe->setLastSender(newSender);
            msgCounter = cc->send_Find_Message(myRadioRef, airframe,lastSender);
        }
    }
    else if(!strcmp(airframe->getName(),"FindMsgForFaultDrones")) { //MOSHE: fault drone message for finding minimum in my linked
        char tempName[12]= {0};
        sprintf(tempName,"drone[%d]",myRadioRef->ID);
        int cover=cc->getMyCoverSum(tempName);
        EV_INFO << "before airframe mincover:"<< airframe->getMinCover()<<" its ID:"<< airframe->getMinCoverID() <<" cover:"<<cover<<"MyID:"<< myRadioRef->ID<<endl;
        if (cc->isDroneLeaf(myRadioRef->ID) && myRadioRef->ownerSize!=0 && (airframe->getMinCover() >= cover)) { //MOSHE: update airframe min cover
            airframe->setMinCover(cover);
            airframe->setMinCoverID(myRadioRef->ID);
            airframe->setMinCoverLocation(myRadioRef->pos);

        }
        EV_INFO << "after airframe mincover:"<< airframe->getMinCover()<<" its ID:"<< airframe->getMinCoverID() <<endl;
        airframe->setCurrDepth_lvl(currDepthLvl+1);
        airframe->setLastSender(newSender);
        msgCounter = cc->send_Find_Message(myRadioRef, airframe,lastSender);
        if(msgCounter == 0) // MOSHE: if you are a leaf drone in the network then send back a reply message.
        ChangeToReply = true;
        if( ChangeToReply == true)// MOSHE: received Find Message and you the last that suppose to get it -> covert to Reply message
        {
            const char *myFullName;
            myFullName = this->getParentModule()->getParentModule()->getFullName();
            myRadioRef3 = cc->lookupRadioByName(myFullName);

            airframe->setCurrDepth_lvl(0);
            airframe->setLastSender(newSender);
            airframe->setIsFindFrame(0);
            airframe->setIsReplyFrame(1);
            airframe->setName("ReplyMsgForFaultDrones");
            cc->send_Reply_Message(myRadioRef, airframe,lastSender);
        }
    }
    //Tslil & Hila- forward message to all neighbors
    else if(!strcmp(airframe->getName(),"FindMsgForEstablishSubGraph")) { //handle recived message
        simtime_t time=simTime();
        std::ofstream point;
        point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/est_msg.txt",std::ios::app);
        point<<"1,simtime is:"<<time<<std::endl;
        point.close();

        EV_INFO<<"start time"<<time<<endl;
        int k=airframe->getK_parameter();
        int currk=airframe->getCurrK();
        double R_u=airframe->getR_u();
        double newCost= R_u-(k-currk);
        int initLinker_int=airframe->getInitLinker();
        int myid=myRadioRef->ID;

        int initiatorID = airframe->getInitiatorID();
        char initLinker[9];
        AtachNumToFullName(initLinker_int,initLinker);
        const char * newInitLinker = (const char*)initLinker;

        char lastSenderName[9];
        AtachNumToFullName(lastSender,lastSenderName);
        const char* lastSenderName1 = (const char*)lastSenderName;

        //************************** Handle received message*************************
        //checking if there is a need to update the linker
        if(newCost>myRadioRef->currCost) {
            const char * linker=myRadioRef->linker;
            if(strcmp(linker,"cc")!=0) {
                if(!(myRadioRef->ID == initiatorID && strcmp(newInitLinker,lastSenderName1)==0)) { //dont connect to the original linker before the extradrone was added
                    int currLinker=detachNumFromFullName(linker);
                    const char * curFullName = this->getParentModule()->getParentModule()->getFullName();
                    const char * newLinker = cc->lookupNameByName(lastSenderName1);
                    cc->setNewHostOwner(curFullName,newLinker);
                    if(check_cycles(curFullName) == true) { // ok to update the linker
                        this->getParentModule()->getParentModule()->bubble(" I have changed my linker ");
                        myRadioRef->currCost = newCost;
                    }
                    else {            //change back to org linker
                        char orgLinker[9];
                        AtachNumToFullName(currLinker,orgLinker);
                        const char * newOrgLinker = (const char*)orgLinker;
                        const char * newCurrLinker = cc->lookupNameByName(newOrgLinker);
                        cc->setNewHostOwner(curFullName,newCurrLinker);//set myRadioRef2.linker to be the one that the connectivity gap we are closing
                    }
                }
            }
        }            //end Handle received message

                     //*******broadcast the message
        if (myRadioRef->isFirstEstablish == false && currk == k) { //if its your first sent message generate your random number
            double R_u = culc_R_u();
            airframe->setR_u(R_u);
            myRadioRef->isFirstEstablish = true;
        }
        if (currk > 1) {            //forward the message
            airframe->setSenderLinker(detachNumFromFullName(myRadioRef->linker));
            airframe->setIsFindFrame(1);
            currk = currk - 1;
            airframe->setCurrK(currk);
            airframe->setIsReplyFrame(0);
            airframe->setLastSender(myRadioRef->ID);
            //airframe->setInitLinker(myRadioRef->linker);
            cc->send_Find_Message(myRadioRef, airframe, lastSender);// Broadcast the message to all neighbors
        } else {
            myRadioRef->isFirstEstablish = false;
            //myRadioRef->stisticsFlag=true;
            airframe->setIsFindFrame(0);
            time = simTime();
            EV_INFO<<"finish time"<<time<<endl;
            print_statistics_time();
        }

    }
    else {
        droneArray[lastSender] = frameID;
        if(currDepthLvl < airframe->getK_parameter()) // received Find Message, pass it on
        {
            airframe->setCurrDepth_lvl(currDepthLvl+1);
            airframe->setLastSender(newSender);
            msgCounter = cc->send_Find_Message(myRadioRef, airframe,lastSender);

            if(msgCounter == 0) // if you are a leaf drone in the network then send back a reply message.
            ChangeToReply = true;
            else
            {
                index = findOpenCell();
                if(index == -1)
                error("Drone's databaseArray is full");

                // set this values, so when the reply messages will received - we can count the right amount of messages to get.

                databaseArray[index].messageCounter = msgCounter;
                databaseArray[index].messageID = frameID;
            }
        }
        if(currDepthLvl == kParameter || ChangeToReply == true) // received Find Message and you the last that suppose to get it -> covert to Reply message
        {
            const char *myFullName;
            myFullName = this->getParentModule()->getParentModule()->getFullName();
            myRadioRef3 = cc->lookupRadioByName(myFullName);

            airframe->setCurrDepth_lvl(0);
            airframe->setLastSender(newSender);
            airframe->setIsFindFrame(0);
            airframe->setIsReplyFrame(1);
            airframe->setLeaf_linker(1);           // may change after few lines

            if(myRadioRef3.ownerSize > 0)// Only if you are a covering drone sent this Replay message
            {
                airframe->setLeaf_linker(false);
                set_Drone_Database(airframe,false);
            }
            else
            airframe->setLeaf_linker(true);     // for the ReplayMessage

            cc->send_Reply_Message(myRadioRef, airframe,lastSender);
            reset_All_Structures(frameID);
        }
    }
}

void MYIdealChannelModelAccess::receivedReplyMessage(
        MYIdealAirFrame *airframe) {
    int currDepthLvl = airframe->getCurrDepth_lvl();
    int newSender = detachNumFromFullName();
    int index, i, Mydest, frameID = airframe->getFrame_ID();
    index = findCellIndex(frameID);
    databaseArray[index].messageCounter--;
    MYIdealChannelModel::RadioEntry myLinker;
    if (!strcmp(airframe->getName(), "ReplyMsgFromChosenDrone")) { //MOSHE: reply massage received from the chosen drone
        myLinker = cc->lookupRadioByName(myRadioRef->linker);
        if (airframe->getInitiatorID() == myRadioRef->ID) { //only the one that initiate the message (recognize one) should answer it
            check_and_cast<MYIdealRadio *>(myRadioRef->radioModule)->setReplaceCoordName(
                    airframe);
            char tempName[12] = { 0 };
            sprintf(tempName, "drone[%d]", myRadioRef->ID);
            EV_INFO<<" received ReplyMsgFromChosenDrone !!!!!!!!!!!!!!!!!!!!! "<<endl;
            EV_INFO<<" Replace Location: "<<airframe->getMinCoverLocation()<<endl;
        }
        else {   //MOSHE: others should send it
            EV_INFO<<" sending ReplyMsgFromChosenDrone !!!!!!!!!!!!!!!!!!!! "<<endl;
            cc->send_Reply_Message(myRadioRef, airframe,myLinker.ID);
        }
    }
    else if(!strcmp(airframe->getName(),"ReplyMsgForFaultDrones")) { //MOSHE: reply massage to find thr
        myLinker=cc->lookupRadioByName(myRadioRef->linker);
        EV_INFO<<" my linker: "<<myLinker.ID<<" is fault"<< myLinker.isFault <<endl;
        if(airframe->getInitiatorID() == myRadioRef->ID) { //MOSHE: reply massage received from the chosen drone
            EV_INFO<<" recieved ReplyMsgForFaultDrones!!!!!!!!!!!!!!!!!!!!! "<<endl;
            if (airframe->getMinCoverID()!=-1) { //MOSHE: there is min coer drone
                char tempName[12]= {0},tempNameLinker[12]= {0};
                sprintf(tempName,"drone[%d]",myRadioRef->ID);
                sprintf(tempNameLinker,"drone[%d]",airframe->getMinCoverID());
                Coord replaced =airframe->getMinCoverLocation(); //the replaced drone location
                //there is a replace drone
                airframe->setName("RelocationMsgForFaultDrones");
                airframe->setMinCoverLocation(cc->lookupRadioByName(myRadioRef->linker).pos);
                EV_INFO<<" cc->lookupRadioByName(myRadioRef->linker).pos: "<<cc->lookupRadioByName(myRadioRef->linker).pos<<endl;
                airframe->setName("ChosenDroneFind");
                airframe->setLastSender(newSender);
                airframe->setIsFindFrame(1);
                airframe->setIsReplyFrame(0);
                setExtraDroneLinker(tempName,tempNameLinker);
                cc->send_Find_Message(myRadioRef, airframe,myLinker.ID);
            }
            else
            EV_INFO<<"NO ONE CAN HELP"<<endl;
        }
        else {   //MOSHE: others should send it
            EV_INFO<<" sending ReplyMsgForFaultDrones!!!!!!!!!!!!!!!!!!!! "<<endl;
            cc->send_Reply_Message(myRadioRef, airframe,myLinker.ID);
        }
    }
    else if(databaseArray[index].messageCounter > 0) // meaning you still wait for the other drones to reply
    {
        if(!airframe->getLeaf_linker()) // if this is'nt a leaf-linker drone
        Update_Drone_Database(airframe,index,false,false,false);// I don't want to put my/my hosts/my linker's location now.  this is not a leaf - so put the messages's info inside my database

        delete airframe;
    }
    else // That means all of the Drones that you send to them a FindFrame has return to you a ReplayFrame.
    {
        // take the most shortest path that the received Reply Message has reached to you. use this value to send it to the next hop drone.
        if(currDepthLvl < min_currDepthLvl_Receveid)
        min_currDepthLvl_Receveid = currDepthLvl;

        if(min_currDepthLvl_Receveid < airframe->getK_parameter() && airframe->getInitiatorID() != newSender)// Pass the Reply message, with all the data you collect to the drone that sent you a find message
        {
            airframe->setCurrDepth_lvl(min_currDepthLvl_Receveid+1);
            airframe->setLastSender(newSender);
            if(!airframe->getLeaf_linker()) // if this is'nt a leaf-linker drone
            Update_Drone_Database(airframe,index,true,false,false);
            else
            Update_Drone_Database(airframe,index,true,true,false);// this is a leaf - so don't put its info inside the drone's database.

            airframe->setLeaf_linker(false);
            for (i = 0; i < DRONE_BUFFER; i++)// copy to the message.
            {
                if(databaseArray[index].databaseArray[i] == 0)
                break;
                airframe->setDroneDatabase(i,databaseArray[index].databaseArray[i]);
            }

            Mydest = findMessageSender(frameID);
            cc->send_Reply_Message(myRadioRef, airframe,Mydest);
            reset_All_Structures(frameID);
        }
        else // This is the initiator of the algorithm and now he has all the information of his Network to continue.
        {
            if(!airframe->getLeaf_linker()) // if this is'nt a leaf-linker drone
            Update_Drone_Database(airframe,index,true,false,true);
            else
            Update_Drone_Database(airframe,index,true,true,true);// this is a leaf - so don't put its info inside the drone's database.
            delete airframe;

            // ALGORITHM!!!!
            centralAlgorithm(frameID);

        }
        for (i = 0; i < DRONE_BUFFER; i++)
        databaseArray[index].databaseArray[i] = 0;
        databaseArray[index].messageCounter = 0;
        databaseArray[index].messageID = -1;
    }
}

void MYIdealChannelModelAccess::centralAlgorithm(int msgID) {
    int replaced_DroneID, hostGroup_size;
    int RecHorizontal_UpperLine, RecHorizontal_DownLine;
    int RecVertical_LeftLine, RecVertical_RightLine;
    int newServerDrone, mostFarHost;
    int droneServer_Counter = 0;
    bool isValid = true;       // is the algorithm decision are valid so far.
    Coord midRec;

    buildDataStructure(msgID);

    Rectangular_Cover_Hosts(&RecHorizontal_UpperLine, &RecHorizontal_DownLine,
            &RecVertical_LeftLine, &RecVertical_RightLine);

    // find the rectangular middle point to place a drove server:
    midRec.x = (RecVertical_LeftLine + RecVertical_RightLine) / 2;
    midRec.y = (RecHorizontal_UpperLine + RecHorizontal_DownLine) / 2;

    replaced_DroneID = find_Best_Drone_Replacment(midRec);

    if (replaced_DroneID == -1) // Check if there is a cover drone to be found for the job, if not - abort algorithm
        isValid = false;
    else {
        remove_Hosts_From_Drone_Cover(replaced_DroneID, droneServer_Counter++); // Mark the host that in the mid-drone radios and update the mid-drone with its new hosts to cover

        hostGroup_size = find_HostGroup_size();
    }

    if (hostGroup_size == 0) // if just 1 drone will cover all host then let the distributed algorithm use the SCE point instead
        isValid = false;

    while (hostGroup_size > 0 && isValid) // Second Phase of the algorithm: keep going while hosts group still not-empty or changes are valid.
    {
        Find_Far_Distance_Pair(&newServerDrone, &mostFarHost); // Find the host server drone that have the most longest distance.

        replaced_DroneID = add_New_ServerDrone(newServerDrone, mostFarHost);

        if (replaced_DroneID == -1) {
            isValid = false;
            break;
        }

        remove_Hosts_From_Drone_Cover(replaced_DroneID, droneServer_Counter++); // Mark the host that in the drone's radios and update the drone with its new hosts to cover

        hostGroup_size = find_HostGroup_size();

    }

    if (isValid)
        isValid = is_New_Replacments_Are_Better();

    //////////  ***    Central Algorithm has finished !!   ///////// **

    if (isValid) {
        imply_CentralAlgorithm_Changes();
    }
    cc->Notify_Algorithm_isOver(isValid, detachNumFromFullName());
    reset_All_Structures(msgID);
}

void MYIdealChannelModelAccess::imply_CentralAlgorithm_Changes() {
    double transmit_Radios, squreDist, minDist = 100000000000;
    bool hasConnectivity = false;
    int counter_of_priority = numOfHosts + 1;

    // Set for every Drone on the "droneServerList" their new LINKER:
    for (DronesList::iterator it = droneServerList.begin();
            it != droneServerList.end(); ++it) {
        Drone_Info *dServer = &*it;
        Drone_Info *dLinker, *min_Distance_Drone;
        if (dServer->drone_ID > numOfDrones - 1)
            error(
                    "error: value 'dServer->drone_ID' has a number bigger then the number of hosts ");

        // for each drone, in the order we push them - set a priority to imply the changes on it.
        cc->drone_imply_centralAlgo_turn[dServer->drone_ID] =
                counter_of_priority--;

        for (DronesList::iterator it2 = droneList.begin();
                it2 != droneList.end(); ++it2) {
            dLinker = &*it2;
            transmit_Radios = dLinker->droneRadios - 10;
            squreDist = (dLinker->drone_Cord.x - dServer->drone_Cord.x)
                    * (dLinker->drone_Cord.x - dServer->drone_Cord.x)
                    + (dLinker->drone_Cord.y - dServer->drone_Cord.y)
                            * (dLinker->drone_Cord.y - dServer->drone_Cord.y);
            transmit_Radios = transmit_Radios * transmit_Radios;

            // drone server in linker's radios and its more closest to CC and the linker only may be, in case of being drone-server, a drone that was relocate this drone will.
            if (squreDist <= transmit_Radios
                    && (dServer->number_Of_Hops > dLinker->number_Of_Hops)
                    && dServer->droneIndex > dLinker->droneIndex) {
                hasConnectivity = true; // this drone can be linked to the swarm.
                if (squreDist < minDist) {
                    minDist = squreDist;
                    min_Distance_Drone = dLinker;
                }
            }
        }
        if (hasConnectivity)         // this is the drone's linker -> update it
        {
            dServer->droneLinker_ID = min_Distance_Drone->drone_ID;
            dServer->droneLinker_Cord = min_Distance_Drone->drone_Cord;
            dServer->number_Of_Hops = min_Distance_Drone->number_Of_Hops + 1;
        } else {
            hasConnectivity = false;
            dServer->droneLinker_ID = -2; // -2 means that we cauldn't found a linker for this drone-server. but the CC will allocated an extra drone for it
        }
    }

    set_Final_droneList();     // Create a new Drone list that will hold all the
    cc->radios3.clear();

    for (DronesList::iterator it = final_List.begin(); it != final_List.end();
            ++it) {
        Drone_Info *drone = &*it;
        MYIdealChannelModel::RadioEntry newRadio;

        newRadio.ID = drone->drone_ID;
        newRadio.is_Drone = true;
        newRadio.is_Host = false;
        newRadio.linkerID = drone->droneLinker_ID;
        newRadio.num_Of_Hops = drone->number_Of_Hops;
        newRadio.ownerSize = drone->num_Of_Hosts_Cover;
        newRadio.pos = drone->drone_Cord;
        newRadio.transmissionRange = drone->droneRadios;
        cc->radios3.push_back(newRadio);
    }

    for (HostList::iterator it2 = hostList.begin(); it2 != hostList.end();
            ++it2)        // Find the drone that much
            {
        Host_Info *host = &*it2;
        MYIdealChannelModel::RadioEntry newRadio;

        newRadio.ID = host->host_ID;
        newRadio.is_Drone = false;
        newRadio.is_Host = true;
        newRadio.linkerID = host->coverDrone_ID;
        newRadio.num_Of_Hops = -1;
        newRadio.ownerSize = -1;
        newRadio.pos = host->host_Pos;
        cc->radios3.push_back(newRadio);
    }
}

bool MYIdealChannelModelAccess::is_New_Replacments_Are_Better() {
    int original_Cover_Drones_Num = 0;
    bool linkerIsCoverDrone = false, droneI_Link_isCover_Drone = false;

    for (MYIdealChannelModel::myRadioList::iterator it1 = cc->radios2.begin();
            it1 != cc->radios2.end(); ++it1) {
        MYIdealChannelModel::RadioEntry *r1 = &*it1;
        if (r1->is_Drone && r1->ownerSize > 0) // check if this drone is a cover one.
                {
            original_Cover_Drones_Num++;
        } else if (r1->is_Drone && r1->ownerSize <= 0) // find if this drone is only linker and found between 2 cover drones:
                {
            for (MYIdealChannelModel::myRadioList::iterator it2 =
                    cc->radios2.begin(); it2 != cc->radios2.end(); ++it2) // find the drone's linker and check if it's a cover one
                    {
                MYIdealChannelModel::RadioEntry *r2 = &*it2;
                if (strcmp(r1->linker,
                        r2->radioModule->getParentModule()->getParentModule()->getFullName())
                        == 0)        // find the linker of the drone.
                        {
                    if (r2->ownerSize > 0) // Check that the linker is also a cover drone
                        linkerIsCoverDrone = true;
                    break;
                }
            }

            if (linkerIsCoverDrone) // go on only if the linker is a cover drone
            {
                for (MYIdealChannelModel::myRadioList::iterator it3 =
                        cc->radios2.begin(); it3 != cc->radios2.end(); ++it3) {
                    MYIdealChannelModel::RadioEntry *r3 = &*it3;
                    if (strcmp(
                            r1->radioModule->getParentModule()->getParentModule()->getFullName(),
                            r3->linker) == 0) // find the drone that this drone is its linker and find out if it's a cover drone
                            {
                        if (r3->ownerSize > 0) // Check that the drone I link is also a cover drone
                            droneI_Link_isCover_Drone = true;
                        break;
                    }
                }
            }
            if (droneI_Link_isCover_Drone) // this linker drone is in the middle of 2 covering drones - so make it also a potentially server drone
            {
                original_Cover_Drones_Num++;
            }
            linkerIsCoverDrone = false;
            droneI_Link_isCover_Drone = false;
        }
    }
    if (original_Cover_Drones_Num <= (int) droneServerList.size())
        return false;
    return true;
}

void MYIdealChannelModelAccess::set_Final_droneList() {
    Drone_Info *d;
    Drone_Info *dServer;
    bool isFound = false;
    final_List.clear();
    for (DronesList::iterator it1 = droneList.begin(); it1 != droneList.end();
            ++it1) {
        d = &*it1;
        for (DronesList::iterator it2 = droneServerList.begin();
                it2 != droneServerList.end(); ++it2) {
            dServer = &*it2;
            if (dServer->drone_ID == d->drone_ID) {
                isFound = true;
                break;
            }
        }
        if (isFound) // if the drone is already in "droneServerList", then take it has it is and push it into the new list
            final_List.push_back(*dServer);
        else // if the drone is not found in "droneServerList", then change the values of the drone and push it into the new list
        {
            it1->num_Of_Hosts_Cover = 0;
            final_List.push_back(*d);
        }
        isFound = false;
    }
}

void MYIdealChannelModelAccess::set_Drone_Database(MYIdealAirFrame *airframe,
        bool lastPazel) {
    int i, start;
    const char *myFullName;
    MYIdealChannelModel::myRadioList myradios3;
    MYIdealChannelModel::RadioEntry myRadioRef3;
    MYIdealChannelModel::RadioEntry myLinkerRadioRef3;
    MYIdealChannelModel::RadioEntry ccRadioRef3 = cc->lookupRadioByName("cc");

    myFullName = this->getParentModule()->getParentModule()->getFullName();
    myRadioRef3 = cc->lookupRadioByName(myFullName);
    myradios3 = cc->getRadios();
    myLinkerRadioRef3 = cc->lookupRadioByName(myRadioRef3.linker);

    for (i = 0; i < DRONE_BUFFER; i++) // find your point of start in the array:
            {
        if (airframe->getDroneDatabase(i) == 0) {
            start = i;
            break;
        }
    }
    if (strcmp(myFullName, "cc") == 0)      //MOSHE: first letter C-cc D-drone
            {
        airframe->setDroneDatabase(start, 'C');
        start = fill_Drone_Linker_data(airframe, "cc", start + 1, "cc",
                ccRadioRef3.pos);
    } else {
        airframe->setDroneDatabase(start, 'D');
        start = fill_Drone_Linker_data(airframe, "drone", start + 1, myFullName,
                myRadioRef3.pos);
    }

    if (strcmp(myRadioRef3.linker, "cc") == 0) {
        airframe->setDroneDatabase(start, 'L');
        airframe->setDroneDatabase(start + 1, 'C');
        start = fill_Drone_Linker_data(airframe, "cc", start + 2, "cc",
                ccRadioRef3.pos);
    } else {
        airframe->setDroneDatabase(start, 'L');
        airframe->setDroneDatabase(start + 1, 'D');
        start = fill_Drone_Linker_data(airframe, "drone", start + 2,
                myRadioRef3.linker, myLinkerRadioRef3.pos);
    }

    for (MYIdealChannelModel::myRadioList::iterator it = myradios3.begin();
            it != myradios3.end(); it++) {
        MYIdealChannelModel::RadioEntry *r = &*it;
        if (!(strcmp(
                r->radioModule->getParentModule()->getParentModule()->getFullName(),
                myFullName) == 0))     //if self no need to check
        {
            // check the module is drone or cc
            if ((strcmp(
                    r->radioModule->getParentModule()->getParentModule()->getName(),
                    "host") == 0) && (strcmp(r->linker, myFullName) == 0)) //only drone "owned" hosts
                    {
                bool inRange = myRadioRef3.pos.sqrdist(r->pos)
                        < myRadioRef3.transmissionRange
                                * myRadioRef3.transmissionRange;
                const char *hostFullName =
                        r->radioModule->getParentModule()->getParentModule()->getFullName();
                if (inRange)   //include only if inRange and not self
                {
                    airframe->setDroneDatabase(start, 'H');
                    start = fill_Drone_Linker_data(airframe, "host", start + 1,
                            hostFullName, r->pos);
                }
            }
        }
    }
    airframe->setDroneDatabase(start, '|');        //close this drone database
    if (lastPazel)
        airframe->setDroneDatabase(start, '#'); //End of the database - for initiator
}

void MYIdealChannelModelAccess::Update_Drone_Database(MYIdealAirFrame *airframe,
        int index, bool useMyDatabase, bool isLeaf, bool is_Sender) {
    int i, j, start;
    char c;
    MYIdealAirFrame *tempFrame = new MYIdealAirFrame();
    for (i = 0; i < DRONE_BUFFER; i++)
        tempFrame->setDroneDatabase(i, 0);
    for (i = 0; i < DRONE_BUFFER; i++)
        if (databaseArray[index].databaseArray[i] == 0) {
            start = i;
            break;
        }
    if (useMyDatabase == false && isLeaf == false) // when the drone will add to it's database the received reply message database and its own.
            {
        j = 0;
        for (i = start; i < DRONE_BUFFER; i++) {
            c = airframe->getDroneDatabase(j);
            j++;
            if (c != 0)
                databaseArray[index].databaseArray[i] = c;
            else
                break;
        }
    } // end false,false

    else if (useMyDatabase == true && isLeaf == false) {
        j = 0;
        for (i = start; i < DRONE_BUFFER; i++) {
            c = airframe->getDroneDatabase(j);
            j++;
            if (c != 0)
                databaseArray[index].databaseArray[i] = c;
            else {
                start = i;
                break;
            }
        }
        set_Drone_Database(tempFrame, is_Sender);

        j = 0;
        for (i = start; i < DRONE_BUFFER; i++) {
            c = tempFrame->getDroneDatabase(j);
            j++;
            if (c != 0)
                databaseArray[index].databaseArray[i] = c;
            else {
                start = i;
                break;
            }
        }
    }   // end true,false
    else if (useMyDatabase == true && isLeaf == true)  // true & true
            {
        set_Drone_Database(tempFrame, is_Sender);

        j = 0;
        for (i = start; i < DRONE_BUFFER; i++) {
            c = tempFrame->getDroneDatabase(j);
            j++;
            if (c != 0)
                databaseArray[index].databaseArray[i] = c;
            else {
                start = i;
                break;
            }
        }
    }   // end true,true
    delete tempFrame;
}

int MYIdealChannelModelAccess::add_New_ServerDrone(int serverDrone,
        int mostFarHost) {
    Coord V, newPosition;
    double V_Magnitude, R, criticalStrip = cc->Drone_criticalStrip;
    int newAddedDrone;
    for (DronesList::iterator it = droneServerList.begin();
            it != droneServerList.end(); ++it) {
        Drone_Info *d = &*it;
        if (d->drone_ID == serverDrone)          // Find the right server drone.
                {
            // change from 30 to "criticalStrip"
            R = d->droneRadios - criticalStrip - 10;
            for (HostList::iterator it = hostList.begin(); it != hostList.end();
                    ++it) {
                Host_Info *host = &*it;
                if (host->host_ID == mostFarHost)         // Find the right host
                        {
                    V.x = host->host_Pos.x - d->drone_Cord.x;
                    V.y = host->host_Pos.y - d->drone_Cord.y;
                    V_Magnitude = sqrt(V.x * V.x + V.y * V.y);
                    newPosition.x = d->drone_Cord.x + (V.x / V_Magnitude) * R;
                    newPosition.y = d->drone_Cord.y + (V.y / V_Magnitude) * R;
                    break;
                }
            }
        }
    }
    newAddedDrone = find_Best_Drone_Replacment(newPosition);
    return newAddedDrone;
}

void MYIdealChannelModelAccess::reset_All_Structures(int msgID) {
    int i, j;
    for (i = 0; i < INF_DRONE; i++)
        if (droneArray[i] == msgID) {
            droneArray[i] = -1;
            break;
        }
    for (i = 0; i < INF_MessageID_Numner; i++) {
        if (databaseArray[i].messageID == msgID) {
            databaseArray[i].messageCounter = 0;
            databaseArray[i].messageID = -1;
            for (j = 0; j < INF_MessageID_Numner; j++)
                databaseArray[i].databaseArray[j] = 0;
            break;
        }
    }
    droneList.clear();
    droneServerList.clear();
    hostList.clear();
}

void MYIdealChannelModelAccess::Find_Far_Distance_Pair(int *droneServer_ID,
        int *mostFarHost) {
    double currDist, minDist = 9999999999;
    Host_Drone_Dist list1, list2;
    list2.droneID = -1;
    list2.hostID = -1;
    list2.host_drone_Distance = -1;
    for (HostList::iterator it = hostList.begin(); it != hostList.end(); ++it) {
        Host_Info *host = &*it;
        if (host->isDealeted == false) // treat only hosts that are not been cover yet
                {
            // Find the smallest distance from the host to any server drone:
            for (DronesList::iterator it = droneServerList.begin();
                    it != droneServerList.end(); ++it) {
                Drone_Info *d = &*it;
                currDist = d->drone_Cord.distance(host->host_Pos);
                if (currDist < minDist) {
                    minDist = currDist;
                    list1.droneID = d->drone_ID;
                    list1.hostID = host->host_ID;
                    list1.host_drone_Distance = minDist;
                }
            }
            minDist = 999999999;
            // find the biggest distance of host-server_drone:
            if (list1.host_drone_Distance > list2.host_drone_Distance) {
                list2.droneID = list1.droneID;
                list2.hostID = list1.hostID;
                list2.host_drone_Distance = list1.host_drone_Distance;
            }

        }
    }
    *droneServer_ID = list2.droneID;
    *mostFarHost = list2.hostID;
}

int MYIdealChannelModelAccess::find_Best_Drone_Replacment(Coord midRec) {
    double minDist = 1000000000, result;
    int returnID = -1, i;
    bool linkerIsCoverDrone = false, droneI_Link_isCover_Drone = false;
    Drone_Info newDrone;
    MYIdealChannelModel::RadioEntry r1, r2, r3;
    char droneName[12] = { 0 };

    for (DronesList::iterator it = droneList.begin(); it != droneList.end();
            ++it) {
        Drone_Info *d = &*it;
        sprintf(droneName, "drone[%d]", d->drone_ID);
        r1 = cc->lookupRadioByName(droneName);
        for (i = 0; i < 12; i++)
            droneName[i] = 0;
        if (d->isCover && !d->useAsServer && r1.ownerSize > 0) // only relevant for covering drones and drones that not picked already to be used in the central algorithm as servers
                {
            result = (midRec.x - d->drone_Cord.x) * (midRec.x - d->drone_Cord.x)
                    + (midRec.y - d->drone_Cord.y)
                            * (midRec.y - d->drone_Cord.y);
            if (result < minDist) {
                minDist = result;
                returnID = d->drone_ID;
            }
        } else if (!d->useAsServer) // drone is only a linker, but may be very fit to cover - only if it's in between 2 covering drones
        {
            for (DronesList::iterator it2 = droneList.begin();
                    it2 != droneList.end(); ++it2) {
                Drone_Info *dLink = &*it2;
                if (d->droneLinker_ID == dLink->drone_ID) // find the linker of the drone.
                        {
                    sprintf(droneName, "drone[%d]", dLink->drone_ID);
                    r2 = cc->lookupRadioByName(droneName);
                    for (i = 0; i < 12; i++)
                        droneName[i] = 0;
                    if (dLink->isCover && r2.ownerSize > 0) // Check that the linker is also a cover drone
                        linkerIsCoverDrone = true;
                    break;
                }
            }
            if (linkerIsCoverDrone) {
                for (DronesList::iterator it3 = droneList.begin();
                        it3 != droneList.end(); ++it3) {
                    Drone_Info *droneToLink = &*it3;
                    if (droneToLink->droneLinker_ID == d->drone_ID) // find drones that this drone is their linker
                            {
                        sprintf(droneName, "drone[%d]", droneToLink->drone_ID);
                        r3 = cc->lookupRadioByName(droneName);
                        if (droneToLink->isCover && r3.ownerSize > 0) // Check that the drone I link is also a cover drone
                            droneI_Link_isCover_Drone = true;
                        break;
                    }
                }
            }
            if (droneI_Link_isCover_Drone) // this linker drone is in the middle of 2 covering drones - so make it also a potentially server drone
            {
                result = (midRec.x - d->drone_Cord.x)
                        * (midRec.x - d->drone_Cord.x)
                        + (midRec.y - d->drone_Cord.y)
                                * (midRec.y - d->drone_Cord.y);
                if (result < minDist) {
                    minDist = result;
                    returnID = d->drone_ID;
                }
            }
            linkerIsCoverDrone = false;
            droneI_Link_isCover_Drone = false;
        }
    }
    if (returnID == -1)
        return -1;
    for (DronesList::iterator it = droneList.begin(); it != droneList.end();
            ++it) {
        Drone_Info *d = &*it;
        if (d->drone_ID == returnID) {
            d->drone_Cord.x = midRec.x;
            d->drone_Cord.y = midRec.y;
            d->isCover = true;              // for the linker case
            break;
        }
    }
    return returnID;
}

void MYIdealChannelModelAccess::remove_Hosts_From_Drone_Cover(int droneID,
        int droneServer_Counter) {
    Coord midDrone_Coord;
    int i, host_Counter = 0;
    double transmit_Radios, squreDist, criticalStrip = cc->Drone_criticalStrip;
    Drone_Info *d, newDrone;

    for (DronesList::iterator it = droneList.begin(); it != droneList.end();
            ++it) {
        d = &*it;
        if (d->drone_ID == droneID) {
            midDrone_Coord.x = d->drone_Cord.x;
            midDrone_Coord.y = d->drone_Cord.y;
            break;
        }
    }
    //changed from 30 to "Drone_criticalStrip"
    transmit_Radios = (d->droneRadios - criticalStrip)
            * (d->droneRadios - criticalStrip);

    /*
     Remove the hosts that are in the radios of the mid drone- by marking them with the value "isDealeted", update the drone info, in the "droneList"
     * in the array: "coverHost" that indicates the drone's new host to cover.
     */
    for (HostList::iterator it = hostList.begin(); it != hostList.end(); it++) {
        Host_Info *host = &*it;
        // Circle formula: (x-a)^2 + (y-b)^2 = R
        squreDist = (host->host_Pos.x - midDrone_Coord.x)
                * (host->host_Pos.x - midDrone_Coord.x)
                + (host->host_Pos.y - midDrone_Coord.y)
                        * (host->host_Pos.y - midDrone_Coord.y);

        if (squreDist < transmit_Radios && host->isDealeted == false) // Host's in mid drone's radios
                {
            for (i = 0; i < INF_HOSTS; i++) {
                if (d->coverHost[i] == -1) {
                    d->coverHost[i] = host->host_ID;
                    host_Counter++;
                    break;
                }
            }
            host->coverDrone_ID = d->drone_ID;
            host->isDealeted = true;
        }
    }
    d->useAsServer = true;
    d->num_Of_Hosts_Cover = host_Counter;
    d->droneIndex = droneServer_Counter++;
    droneServerList.push_back(*d);
}

void MYIdealChannelModelAccess::Rectangular_Cover_Hosts(
        int* RecHorizontal_UpperLine, int* RecHorizontal_DownLine,
        int* RecVertical_LeftLine, int* RecVertical_RightLine) {
    int maxHortiztal_Up = 10000000, maxHortiztal_Down = -1, maxVert_Right = -1,
            maxVert_Left = 10000000;
    for (HostList::iterator it = hostList.begin(); it != hostList.end(); ++it) {
        Host_Info *host = &*it;
        if (host->host_Pos.y < maxHortiztal_Up)
            maxHortiztal_Up = host->host_Pos.y;
        if (host->host_Pos.y > maxHortiztal_Down)
            maxHortiztal_Down = host->host_Pos.y;
        if (host->host_Pos.x > maxVert_Right)
            maxVert_Right = host->host_Pos.x;
        if (host->host_Pos.x < maxVert_Left)
            maxVert_Left = host->host_Pos.x;
    }
    *RecHorizontal_UpperLine = maxHortiztal_Up;
    *RecHorizontal_DownLine = maxHortiztal_Down + 1;
    *RecVertical_LeftLine = maxVert_Left;
    *RecVertical_RightLine = maxVert_Right + 1;
}

int MYIdealChannelModelAccess::find_HostGroup_size() {
    int counter = numOfHosts;
    for (HostList::iterator it = hostList.begin(); it != hostList.end(); ++it) {
        Host_Info *host = &*it;
        if (host->isDealeted)
            counter--;
    }
    return counter;
}

void MYIdealChannelModelAccess::buildDataStructure(int msgID) {
    int i, whileCounter = 0, index;
    char temp[DRONE_BUFFER];
    char tempSet[DRONE_BUFFER];
    char * pFullSet, *p;
    Host_Info h;
    Drone_Info d;
    Coord hostCoord, droneCoord;

    droneList.clear();
    hostList.clear();
    droneServerList.clear();

    for (i = 0; i < DRONE_BUFFER; i++)
        tempSet[i] = 0;

    index = findCellIndex(msgID);
    strcpy(temp, databaseArray[index].databaseArray);

    pFullSet = strtok(temp, "|");
    while (pFullSet != NULL) {
        for (i = 0; i < INF_HOSTS; i++) // this array will served us later, but must be initialize now
                {
            d.coverHost[i] = -1;
        }

        strcpy(tempSet, pFullSet);

        // take the Drone ID and set it:
        p = strtok(tempSet, "D(.),LH");
        d.drone_ID = atoi(p);

        // take the Drone Cords and set them:
        p = strtok(NULL, "D(),LH");
        droneCoord.x = strtod(p, NULL);

        p = strtok(NULL, "D(),LH");
        droneCoord.y = strtod(p, NULL);

        d.drone_Cord = droneCoord;

        // take the Drone Linker name and Cords and set them:
        p = strtok(NULL, "D(),LH");
        d.droneLinker_ID = atoi(p);

        p = strtok(NULL, "D(),LH");
        d.droneLinker_Cord.x = strtod(p, NULL);

        d.currCost = 1000;  //TSLIL& HILA

        p = strtok(NULL, "D(),LH");
        d.droneLinker_Cord.y = strtod(p, NULL);
        d.isCover = false;      // correct lower if it is.

        whileCounter++;

        // take the Hosts info and set them up:
        while (p != NULL) {
            p = strtok(NULL, "D(),LH|");
            if (p == NULL || strcmp(p, "#") == 0)
                break;
            h.host_ID = atoi(p);
            p = strtok(NULL, "D(),LH|");
            h.host_Pos.x = strtod(p, NULL);
            p = strtok(NULL, "D(),LH|");
            h.host_Pos.y = strtod(p, NULL);
            h.coverDrone_ID = d.drone_ID;
            h.isDealeted = false;
            d.isCover = true;
            d.useAsServer = false;
            d.droneRadios = -1;
            hostList.push_back(h);
        }

        // Set the transmission range of the drone.
        for (MYIdealChannelModel::myRadioList::iterator it =
                cc->radios2.begin(); it != cc->radios2.end(); it++) {
            MYIdealChannelModel::RadioEntry *r = &*it;
            if (r->ID == d.drone_ID && r->is_Drone) {
                d.droneRadios = r->transmissionRange;
                d.number_Of_Hops = r->num_Of_Hops;
            }
        }
        d.num_Of_Hosts_Cover = 0;
        d.droneIndex = -1;
        droneList.push_back(d);

        for (i = 0; i < DRONE_BUFFER; i++)
            tempSet[i] = 0;
        strcpy(temp, databaseArray[index].databaseArray);

        pFullSet = strtok(temp, "|");
        for (i = whileCounter; i > 0; i--)
            pFullSet = strtok(NULL, "|");
        d.droneLinker_Cord.x = -1;
        d.droneLinker_Cord.y = -1;
        d.droneLinker_ID = -1;
        d.drone_ID = -1;
        h.coverDrone_ID = -1;
        h.host_ID = -1;
        h.host_Pos.x = -1;
        h.host_Pos.y = -1;
    }
}

int MYIdealChannelModelAccess::findMessageSender(int frameID) {
    int i;
    for (i = 0; i < INF_DRONE; i++)
        if (droneArray[i] == frameID)
            return i;
    return -1;
}

int MYIdealChannelModelAccess::findCellIndex(int farmeID) {
    int i;
    for (i = 0; i < INF_MessageID_Numner; i++)
        if (databaseArray[i].messageID == farmeID)
            return i;
    return -1;
}

int MYIdealChannelModelAccess::findOpenCell() {
    int i;
    for (i = 0; i < INF_MessageID_Numner; i++)
        if (databaseArray[i].messageID == -1)
            return i;
    return -1;
}

int MYIdealChannelModelAccess::changeCoordTOString(MYIdealAirFrame *airframe,
        int startIdx, Coord pos) {
    char c[40];
    int i, end;
    for (i = 0; i < 40; i++)
        c[i] = 0;
    sprintf(c, "%0.14f", pos.x);
    sprintf(c + 18, "%c", ',');
    sprintf(c + 19, "%0.14f", pos.y);

    airframe->setDroneDatabase(startIdx, '(');

    for (i = 0; i < 40; i++) {
        if ((c[i] != 0 && c[i] >= 48 && c[i] < 58) || c[i] == ','
                || c[i] == '.')
            airframe->setDroneDatabase(i + startIdx + 1, c[i]);
        else {
            end = i;
            break;
        }
    }
    airframe->setDroneDatabase(startIdx + end + 1, ')');
    return startIdx + end + 2;
}

int MYIdealChannelModelAccess::fill_Drone_Linker_data(MYIdealAirFrame *airframe,
        const char* moduleType, int index, const char* fullName, Coord pos) {
    int numOfDigist, start = index;
    int typeM = (strcmp("drone", moduleType) == 0) ? 6 : 5;
    numOfDigist = howManyDigits(fullName, typeM + 1);
    if (numOfDigist == 1) {
        airframe->setDroneDatabase(start, fullName[typeM]);
        start += 1;
    } else if (numOfDigist == 2)   // in case of dozens of drones
            {
        airframe->setDroneDatabase(start, fullName[typeM]);
        airframe->setDroneDatabase(start + 1, fullName[typeM + 1]);
        start += 2;
    } else        // in case of Hundreds of drones
    {
        airframe->setDroneDatabase(start, fullName[typeM]);
        airframe->setDroneDatabase(start + 1, fullName[typeM + 1]);
        airframe->setDroneDatabase(start + 2, fullName[typeM + 2]);
        start += 3;
    }
    start = changeCoordTOString(airframe, start, pos);
    return start;
}

int MYIdealChannelModelAccess::howManyDigits(const char *fullName, int index) {
    if (fullName[index] == ']')
        return 1;
    else if (fullName[index + 1] == ']')
        return 2;
    else
        return 3;
}

bool MYIdealChannelModelAccess::isFrmaeExist(int frameID) {
    int i;
    for (i = 0; i < numOfDrones; i++)
        if (droneArray[i] == frameID)
            return true;
    return false;
}

int MYIdealChannelModelAccess::detachNumFromFullName() {
    const char * currfullname =
            this->getParentModule()->getParentModule()->getFullName();

    char fullNamestr[80];
    const char * charVal;
    int intVal;
    char * pch;
    if (strcmp(currfullname, "cc") == 0)        // cc id will be the last number
        return numOfDrones;
    strcpy(fullNamestr, currfullname);
    pch = strtok(fullNamestr, "[");
    pch = strtok(NULL, "]");
    charVal = pch;

    intVal = atoi(charVal);

    return intVal;
}

int MYIdealChannelModelAccess::detachNumFromFullName(
        const char * currfullname) {        //HILA & TSLIL

    char fullNamestr[80];
    const char * charVal;
    int intVal;
    char * pch;
    if (strcmp(currfullname, "cc") == 0)        // cc id will be the last number
        return -1;
    else if (strcmp(currfullname, "regular") == 0) // regular drone id, in case of extra drone establish msg
        return -2;

    strcpy(fullNamestr, currfullname);
    pch = strtok(fullNamestr, "[");
    pch = strtok(NULL, "]");
    charVal = pch;

    intVal = atoi(charVal);

    return intVal;
}

//MOSHE: uses sendToChannel with 2 inputs from MYIdealChannelModel
void MYIdealChannelModelAccess::sendToChannel(MYIdealAirFrame *msg) {
    coreEV << "sendToChannel: sending to gates\n";
    cc->sendToChannel(myRadioRef, msg);         // delegate it to ChannelControl
}

//from MYIdealRadio::handleMYUpperMsg
void MYIdealChannelModelAccess::sendToMYChannel(MYIdealAirFrame *msg,
        int SignalID) {
    EV_INFO<<"sendToMYChannel msg->setIsFindFrame(1);"<<msg->getIsFindFrame()<<endl;
    EV_INFO<<"sendToMYChannel msg->getname;"<<msg->getName()<<"my signal id is : "<<SignalID<<endl;
    int senderID = detachNumFromFullName();
    int msgCounter, index;
    int frameID = msg->getFrame_ID();
    int lastSender = msg->getLastSender();
    const char * name=msg->getName();
    //strcpy(name,msg->getName());
    if(SignalID == (int)droneToDrone || SignalID == (int)faultSignal || (strcmp(name,"FindMsgForEstablishSubGraph")==0))
    {
        msgCounter = cc->send_Find_Message(myRadioRef, msg,senderID);
        index = findOpenCell();
        if(index == -1)
        error("Drone's databaseArray is full");
        if(senderID ==0 && frameID == 3)
        frameID =3;
        droneArray[lastSender] = frameID;
        databaseArray[index].messageCounter = msgCounter;
        databaseArray[index].messageID = frameID;
    }

    else
    cc->sendToMYLinker(myRadioRef, msg);        // delegate it to ChannelControl
}

const char * MYIdealChannelModelAccess::getLinker(const char * currfullname) {

    const char * droneLinker = cc->getLinker(currfullname);
    return droneLinker;
}

Coord& MYIdealChannelModelAccess::getHostPosition(const char * currfullname) {
    return cc->getHostPosition(currfullname);
}

void MYIdealChannelModelAccess::setmyisActive(const char *currfullname,
        const bool& myisActive) {
    cc->setmyisActive(currfullname, true);
}

void MYIdealChannelModelAccess::setExtraDroneLinker(const char *currfullname,
        const char * newLinkerFullName) {
    cc->getAndSetExtraDroneLinker(currfullname, newLinkerFullName);
}

void MYIdealChannelModelAccess::setCCLinker(const char *currfullname) {
    cc->findAndSetCCLinker(currfullname);
}

void MYIdealChannelModelAccess::setNameCoordFrameName(const char *currfullname,
        const char *nameCoordFrameName) {
    cc->setNameCoordFrameName(currfullname, nameCoordFrameName);
}
void MYIdealChannelModelAccess::setsendToNameCoordFrameName(
        const char *currfullname, bool send) {
    cc->setsendDroneToNameCoord(detachNumFromFullName3(currfullname), send);
}
void MYIdealChannelModelAccess::receiveSignal(cComponent *source,
        simsignal_t signalID, cObject *obj) {
    int CriticalConnectivityStrip;
    if (signalID == mobilityStateChangedSignal) {
        IMobility *mobility = check_and_cast<IMobility*>(obj);
        radioPos = mobility->getCurrentPosition();

        positionUpdateArrived = true;
        //if there is diff -> we want to try and set the older pos again and
        if (myRadioRef) {

            cc->setRadioPosition(myRadioRef, radioPos);
            update_soldier_array();
            update_farest_soldier();

            //   we want to check here if connectivity to linker is kept, by checking dist to linker is < R , if so good (can continue as usual)
            //  if connectivity NOT kept we call extra drone to be this new linker and would set it to be the older linker of the initiator (of the connectivity request)
            //   should generally be the same procedure but diff msg.

            //MOSHE: only drone who have myisActive = true should be tested for connectivity (linker drone get out of the frame)
            if (cc->inFault < 4
                    && strcmp(
                            myRadioRef->radioModule->getParentModule()->getParentModule()->getName(),
                            "drone") == 0 && myRadioRef->myisActive) { //MOSHE: all cases that are no noFault or endFault
                EV_INFO<<"MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMSSSSSSSSSSSSSSSSSSSSSSXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
                if(cc->inFault==1) {
                    MYIdealRadio *idealRadio = check_and_cast<MYIdealRadio *>(myRadioRef->radioModule);

                    //update the connectivity strip to hold and not be overwriten over by color updates of drone (implemented in MYConstSpeedMobilityDrone)
                    idealRadio->updateDisplayString();

                    if(!isWaitingForConnectivity)
                    {

                        /*                     this drone need to initiate a msg to CC (via MYIdealRadio to build the message and then to send direct)
                         only then we could deal with how should the CC pass the msg to the extradronelinker and how to set nameCoordFrameName (so mobility would know too)
                         and then in mobility to set first target as new extradronelinker and handle what to do when first target reached by self (and make sure it).
                         affects the other situations happening on the field.*/

                        isWaitingForConnectivity = true;

                        strcpy(linkerWhenCalledConnectivity,myRadioRef->linker);

                        //this->getParentModule()->getParentModule()->bubble(" CALL CONNECTIVITY ");
                        EV_INFO<<""<<endl;
                        EV_INFO << " --------------------- INITIATE CONNECTIVITY MSG TO CC ----------------------------MYIdealChannelModelAccess " <<endl;

                        Counter ++;
                        cc->setDroneOperateTime(myRadioRef->ID, true);
                        idealRadio->initiateExtraDroneConnectivityMsg();//MOSHE: create and send extradrone msg
                    }
                    else {

                        /*
                         always when no connectivity problem (just in case) and also for when the called linker arrived and change this linker as
                         self and as a result this would go to here and reset flag for next times when connectivity problem appear .
                         when new connectivity linker arrived (and will change caller linker to be self), and not in connectivity problem
                         also need to test to make sure that no ZIG ZAG case that calls connectivity extradrone again b4 we set to is waiting for connectivity
                         want to check if name of this linker is chanching and only then to set 'isWaitingForConnectivity = false'
                         */

                        if(!(strcmp(linkerWhenCalledConnectivity,myRadioRef->linker)==0)) {
                            //EV_INFO << "KEEP KEEP         MYIdealChannelModelAccess::receiveSignal, IN CONNECTIVITY ZIGZAG TEST NEW LINKER, STOP WAIT"<<endl;
                            isWaitingForConnectivity = false;
                        }
                    }
                }
            }

            else if ((strcmp(myRadioRef->radioModule->getParentModule()->getParentModule()->getName(),"drone")==0)&&(myRadioRef->myisActive)) {

                double cursqrdist;
                double finalConnectivityRange = 0;
                MYIdealRadio *idealRadio = check_and_cast<MYIdealRadio *>(myRadioRef->radioModule);

                //update the connectivity strip to hold and not be overwriten over by color updates of drone (implemented in MYVonstSpeedMobilityDrone)
                idealRadio->updateDisplayString();

                CriticalConnectivityStrip = idealRadio->getCriticalConnectivityStrip();

                finalConnectivityRange = myRadioRef->transmissionRange - CriticalConnectivityStrip;

                double sqrfinalConnectivityRange = finalConnectivityRange*finalConnectivityRange;
                if(strcmp(myRadioRef->linker , "none") != 0)
                {
                    Coord currLinkerCoord = cc->getHostPosition(myRadioRef->linker);
                    cursqrdist = currLinkerCoord.sqrdist(myRadioRef->pos);
                }

                if((cursqrdist > sqrfinalConnectivityRange)&&(!isWaitingForConnectivity))
                {

                    /*                     this drone need to initiate a msg to CC (via MYIdealRadio to build the message and then to send direct)
                     only then we could deal with how should the CC pass the msg to the extradronelinker and how to set nameCoordFrameName (so mobility would know too)
                     and then in mobility to set first target as new extradronelinker and handle what to do when first target reached by self (and make sure it).
                     affects the other situations happening on the field.*/

                    isWaitingForConnectivity = true;

                    strcpy(linkerWhenCalledConnectivity,myRadioRef->linker);

                    //this->getParentModule()->getParentModule()->bubble(" CALL CONNECTIVITY ");
                    EV_INFO<<""<<endl;
                    EV_INFO << " --------------------- INITIATE CONNECTIVITY MSG TO CC ----------------------------MYIdealChannelModelAccess " <<endl;

                    Counter ++;
                    cc->setDroneOperateTime(myRadioRef->ID, true);
                    idealRadio->initiateExtraDroneConnectivityMsg();//MOSHE: create and send extradrone msg
                }
                else {

                    /*
                     always when no connectivity problem (just in case) and also for when the called linker arrived and change this linker as
                     self and as a result this would go to here and reset flag for next times when connectivity problem appear .
                     when new connectivity linker arrived (and will change caller linker to be self), and not in connectivity problem
                     also need to test to make sure that no ZIG ZAG case that calls connectivity extradrone again b4 we set to is waiting for connectivity
                     want to check if name of this linker is chanching and only then to set 'isWaitingForConnectivity = false'
                     */

                    if(!(strcmp(linkerWhenCalledConnectivity,myRadioRef->linker)==0)) {
                        //EV_INFO << "KEEP KEEP         MYIdealChannelModelAccess::receiveSignal, IN CONNECTIVITY ZIGZAG TEST NEW LINKER, STOP WAIT"<<endl;
                        isWaitingForConnectivity = false;
//                        //Tslil & Hila
//                        EV_INFO<<"Tslil &HilaTslil &HilaTslil &HilaTslil &HilaTslil &HilaTslil &HilaTslil &Hila"<<endl;
//                        emit(establishSubGraph,NULL);
                    }
                }
            }
        }
    }
    if (signalID == faultSignal) {

    }
}

bool MYIdealChannelModelAccess::IsLeafDrone() {
    //  if(hostNeigborsList.size()==1)
    cModule * tempModule =
            getParentModule()->getParentModule()->getParentModule();

    bool islinker = (bool) tempModule->par("UsedAsLinker");
    if (!islinker)
        return true;
    return false;
}

int MYIdealChannelModelAccess::detachNumFromFullName3(const char *senderName) {
    char fullNamestr[80];
    const char * charVal;
    int intVal;
    char * pch;

    strcpy(fullNamestr, senderName);
    pch = strtok(fullNamestr, "[");
    pch = strtok(NULL, "]");
    charVal = pch;

    intVal = atoi(charVal);

    return intVal;
}

void MYIdealChannelModelAccess::AtachNumToFullName(int senderID,
        char * senderName) {

    sprintf(senderName, "drone[%d]", senderID);
    if (senderID == -1) {
        sprintf(senderName, "cc");
    } else if (senderID == -2) {
        sprintf(senderName, "regular");
    }
    return;
}

//TSLIL & HILA: calculate beta value for exponential probability
double MYIdealChannelModelAccess::culc_beta() {
    double c_par =
            this->getParentModule()->getParentModule()->getParentModule()->par(
                    "C_parameter");
    int number_Of_Hosts = sizeof(this->final_List) / sizeof(DronesList);
    int k_par =
            this->getParentModule()->getParentModule()->getParentModule()->par(
                    "K_parameter");

    return log(c_par * number_Of_Hosts) / k_par;
}

double MYIdealChannelModelAccess::culc_R_u() {
    double r_u = exponential(1 / culc_beta());
    return r_u;
}

//HILA & TSLIL return true- if no cycle found, false- there is a cycle
bool MYIdealChannelModelAccess::check_cycles(const char * start) {
    const char* next = getLinker(start);
    while (strcmp(next, start) != 0) {
        if (strcmp(next, "cc") == 0)
            return true;
        next = getLinker(next);
    }
    return false;
}

void MYIdealChannelModelAccess::print_statistics_time(){
    std::ofstream point;
        point.open("C:/Users/Hila urevich/Documents/FINAL_PROJECT/res_time.txt",std::ios::app);
        simtime_t time = simTime();
        double d_time=time.dbl();
        point<<"finish time: "<<time<<std::endl;
        point.close();
}

void MYIdealChannelModelAccess::update_soldier_array(){
    if(myRadioRef->is_Host==true){
        int i=0;
        for(i=0;i<SOLDEIR_NUMBER;i++){
            //srand( time ( NULL)+i);
            double rand_x =dblrand()*DISTANCE_FROM_COMMANDER-DISTANCE_FROM_COMMANDER/2;
            //srand( time ( NULL)+i);
            double rand_y =dblrand()*DISTANCE_FROM_COMMANDER-DISTANCE_FROM_COMMANDER/2;
            myRadioRef->soldierArray[i].x = myRadioRef->pos.x + rand_x;
            myRadioRef->soldierArray[i].y = myRadioRef->pos.y + rand_y;
        }
    }
}

void MYIdealChannelModelAccess:: update_farest_soldier(){
    Coord farest,linkerPos;
    if(myRadioRef->is_Host==true){
        linkerPos=getHostPosition(myRadioRef->linker);
        double dist=0, currDist=0;
        int i=0;
        dist=linkerPos.sqrdist(myRadioRef->pos);
        farest=myRadioRef->pos;
        for(i=0;i<SOLDEIR_NUMBER;i++){
            currDist=linkerPos.sqrdist(myRadioRef->soldierArray[i]);
            if(dist<currDist && currDist <cc->getTransmissionRange(myRadioRef->linker)){
                dist = linkerPos.sqrdist(myRadioRef->soldierArray[i]) ;
                farest = myRadioRef->soldierArray[i];
            }
        }
        myRadioRef->pos=farest;
    }
}
