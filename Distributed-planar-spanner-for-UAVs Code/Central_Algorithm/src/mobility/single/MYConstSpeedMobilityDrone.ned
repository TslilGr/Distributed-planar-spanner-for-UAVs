//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package _25_8.Central_Algorithm.src.mobility.single;


import _25_8.Central_Algorithm.src.mobility.common.MYMovingMobilityBase;

//
// Controls all movement related things of a host
//
// ~ConstSpeedMobility does not use one of the standard mobility approaches.
// The user can define a velocity for each Host and an update interval. If
// the velocity is greater than zero (i.e. the Host is not stationary) the
// ~ConstSpeedMobility module calculates a random target position for the Host.
// Every update interval ~ConstSpeedMobility calculates the new position on its
// way to the target position and updates the display. Once the target position
// is reached ~ConstSpeedMobility calculates a new target position.
//
// This component has been taken over from Mobility Framework 1.0a5.
//
// @author Steffen Sroka, Marc Loebbers, Daniel Willkomm
//
simple MYConstSpeedMobilityDrone extends MYMovingMobilityBase
{
    parameters:
        int ccChoosingMode = default(0);
        
        double criticalStrip @unit(m);
        double RneighborhelpstripDelta @unit(m);
        double strengthStripDelta @unit(m);
        double criticalConnectivityStrip @unit(m);
        double RwhenExDroneOnTheWay @unit(m);
        double speed @unit(mps); // speed of the host
        //double onTheWayspeed @unit(mps); // speed of the host when on its way or OFF
        
        double maxOnTheWayDistX @unit(m);
    	double maxOnTheWayDistY @unit(m);
    	double transmissionRange @unit(m);
        
        double initialX @unit(m) = default(uniform(constraintAreaMinX, constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(constraintAreaMinY, constraintAreaMaxY));
        double initialZ @unit(m) = default(uniform(constraintAreaMinZ, constraintAreaMaxZ));
        bool initFromDisplayString = default(true);
        
        
        @class(MYConstSpeedMobilityDrone);
        @signal[extraDroneToCC](type=long);
}
