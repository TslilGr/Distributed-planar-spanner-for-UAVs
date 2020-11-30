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


cplusplus {{
#include "INETDefs.h"

#include "Coord.h"
}}


class noncobject Coord;

//
// MYAirFrame used with ~MYIdealRadio and ~MYIdealChannelModel
//
packet MYIdealAirFrame
{
    simtime_t transmissionDuration;     // Time it takes to transmit the packet, in seconds
    Coord transmissionStartPosition;    // position of sender when transmission starts
    double transmissionRange;           // max. distance from sender where reception is possible
    char DroneDatabase[4096];
    int K_parameter;
    int currK;							//count down k parameter, for distibute R_u msg //Tslil & Hila
    double R_u;							//Tslil & Hila
    int currDepth_lvl;
    int isFindFrame;					// boolean- is this is a find message?
    int isReplyFrame;					// boolean- is this is a reply message?
 	int isRelocationFrame;	            // boolean- is this is a relocation message?
 	int initiatorID;					// ID of the first sending drone
 	int frame_ID;						
 	int lastSender;    					// ID of the last drone that pass the message
 	bool leaf_linker;					// if this drone is a leaf (till K_Parameter length) and its a linker then true, meening that the drone that sent him the FindFrame will still receive an answer but want invetegate it.
	int initLinker;						// linker from whom i received the org message HILA & TSLIL
	int senderLinker;					//linker from whom i received the message HILA & TSLIL=
}

