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


#ifndef __INET_IDEALRADIO_H
#define __INET_IDEALRADIO_H


#include "INETDefs.h"

#include "MYIdealAirFrame_m.h"
#include "MYIdealChannelModelAccess.h"
#include "ILifecycle.h"
#include "RadioState.h"

//
#include "MYIdealChannelModel.h"

static int frameID_counter = 0;

/**
 * This module implements a full-duplex, collision-free and interference-free radio.
 * It should be used together with IdealWirelessMac.
 *
 * See the NED file for details.
 */
class MYIdealRadio : public MYIdealChannelModelAccess, public ILifecycle
{
  public:
    MYIdealRadio();
    virtual ~MYIdealRadio();

    /** Returns the current transmission range */
    virtual int getTransmissionRange() const { return transmissionRange; }

    virtual double getCriticalConnectivityStrip() const { return criticalConnectivityStrip; }

    //virtual void testIdealRadio() { return setmyisActive("drone[2]", true); }

    virtual void initiateExtraDroneReplaceyMsg();

    virtual void initiateExtraDroneConnectivityMsg();

    virtual void initiateFaultMsg(MYIdealAirFrame *airframe);

    virtual void setReplaceCoordName(MYIdealAirFrame *airframe);

    virtual void updateDisplayString();

    bool isEnabled() const { return rs != RadioState::OFF && rs != RadioState::SLEEP; }

    //TSLIL & HILA
    virtual double culc_beta();
    virtual double culc_R_u();

  protected:
    virtual void initialize(int stage);
    virtual void finish();

    virtual void handleMessage(cMessage *msg);

    virtual void handleUpperMsg(cMessage *msg);

    virtual void handleMYUpperDroneMsg(long hostNum,int signalID);

    virtual void handleMYUpperDroneMsg(const char * droneAndLinker,int signalID);

    virtual void handleSelfMsg(cMessage *msg);

    virtual void handleCommand(cMessage *msg);

    virtual void handleLowerMsgStart(MYIdealAirFrame *airframe);

    virtual void handleLowerMsgEnd(MYIdealAirFrame *airframe);

    //forwards msg to its linker on its way to the cc
    virtual void forwardMYUpperMsg(MYIdealAirFrame *airframe);

    /** Sends a message to the upper layer */
    virtual void sendUp(MYIdealAirFrame *airframe);

    virtual const char *attachIdToDroneLinker(int droneId);

    virtual MYIdealAirFrame *buildDroneToLinkerMsg(cPacket * frame);

    virtual MYIdealAirFrame *buildCCToExtraDroneMsg(cPacket * frame);

    virtual const char * attachCoordToName(const char* currfullname,Coord coord );


    virtual const char *attachLinkerToCoordName(const char* linker, const char* coordFramename);

    /** Sends a message to the channel */
    virtual void sendDown(MYIdealAirFrame *airframe);

    /** Encapsulates a MAC frame into an Air Frame */
    virtual MYIdealAirFrame *encapsulatePacket(cPacket *msg);

    /** Updates the radio state, and also sends a radioState signal */
    virtual void updateRadioState();

    /** Create a new MYIdealAirFrame */
    virtual MYIdealAirFrame *createAirFrame() { return new MYIdealAirFrame(); }

    virtual void setRadioState(RadioState::State newState);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, long hostNum);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char * droneAndLinker);

    // ILifecycle:
    virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback);


  protected:
    enum Kind
    {
      TOCC,
      FROMCC,
    };


    typedef std::list<cMessage *> RecvBuff;
    RecvBuff recvBuff;

    /** Parameters */
    //@{
    double transmissionRange;           // [meter]
    double bitrate;                     // [bps]
    bool drawCoverage;                  // if true, draw coverage circles

    double criticalConnectivityStrip;

    int mypassed;
    MYIdealChannelModel::myRadioList myradios;
    //@}

    /** @name Gate Ids */
    //@{
    int upperLayerOutGateId;
    int upperLayerInGateId;
    int radioInGateId;
    //@}

    /** Radio State and helper variables */
    //@{
    int concurrentReceives;    // number of current receives
    bool inTransmit;           // is in transmit mode
    RadioState::State rs;
    //@}

    // signals:
    static simsignal_t radioStateSignal; // signaling RadioState::State enum when state changed
};

#endif

