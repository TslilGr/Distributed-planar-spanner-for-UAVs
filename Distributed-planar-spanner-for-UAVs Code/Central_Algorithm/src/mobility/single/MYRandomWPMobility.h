//
// Copyright (C) 2005 Georg Lutz, Institut fuer Telematik, University of Karlsruhe
// Copyright (C) 2005 Andras Varga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//


#ifndef RANDOM_WP_MOBILITY_H
#define RANDOM_WP_MOBILITY_H

#include "INETDefs.h"

#include "MYLineSegmentsMobilityBase.h"


/**
 * Random Waypoint mobility model. See NED file for more info.
 *
 * @author Georg Lutz (georglutz AT gmx DOT de), Institut fuer Telematik,
 *  Universitaet Karlsruhe, http://www.tm.uka.de, 2004-2005
 * @author Andras Varga (generalized, ported to MYLineSegmentsMobilityBase)
 */
class MYRandomWPMobility : public MYLineSegmentsMobilityBase
{
  protected:
    bool nextMoveIsWait;
    bool goToSinglePoint;       // shay's flag. if true, then all hosts will go to a specific point on map after a given time.

  protected:
    virtual int numInitStages() const { return 3; }

    /** @brief Initializes mobility model parameters.*/
    virtual void initialize(int stage);

    /** @brief Overridden from MYLineSegmentsMobilityBase.*/
    virtual void setTargetPosition();

    /** @brief Overridden from MYLineSegmentsMobilityBase.*/
    virtual void move();

  public:
    MYRandomWPMobility();
};

#endif
