//
// Generated file, do not edit! Created by opp_msgc 4.3 from Central_Algorithm/src/linklayer/MYIdealAirFrame.msg.
//

#ifndef _MYIDEALAIRFRAME_M_H_
#define _MYIDEALAIRFRAME_M_H_

#include <omnetpp.h>

// opp_msgc version check
#define MSGC_VERSION 0x0403
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of opp_msgc: 'make clean' should help.
#endif

// cplusplus {{
#include "INETDefs.h"

#include "Coord.h"
// }}



/**
 * Class generated from <tt>Central_Algorithm/src/linklayer/MYIdealAirFrame.msg</tt> by opp_msgc.
 * <pre>
 * packet MYIdealAirFrame
 * {
 *     simtime_t transmissionDuration;     
 *     Coord transmissionStartPosition;    
 *     double transmissionRange;           
 *     char DroneDatabase[2048];
 *     int K_parameter;
 *         int currK_var;
 *      double R_u_var;
 *     int currDepth_lvl;
 *     int isFindFrame;
 *     int isReplyFrame;
 *  	int isRelocationFrame;
 *  	int initiatorID;
 *  	int frame_ID;
 *  	int lastSender;
 *  	bool leaf_linker;
 * }
 * </pre>
 */
class MYIdealAirFrame : public ::cPacket
{
  protected:
    simtime_t transmissionDuration_var;
    Coord transmissionStartPosition_var;
    double transmissionRange_var;
    char DroneDatabase_var[2048];
    int K_parameter_var;
    int currK_var;
    double R_u_var;
    int currDepth_lvl_var;
    int isFindFrame_var;
    int isReplyFrame_var;
    int isRelocationFrame_var;
    int initiatorID_var;
    int frame_ID_var;
    int lastSender_var;
    int minCover;
    int minCoverID;
    Coord minCoverLocation;
    bool leaf_linker_var;
    const char *replaceLinker;
    int initLinker;
    int senderLinker;

  private:
    void copy(const MYIdealAirFrame& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const MYIdealAirFrame&);

  public:
    //MOSHE:FIX

    int priority_message_type;
    int src_sent_drone_id;
    int priority_sum;
    double xLocation;
    double yLocation;
    int dst_sent_drone_id;

    MYIdealAirFrame(const char *name=NULL, int kind=0);
    MYIdealAirFrame(const MYIdealAirFrame& other);
    virtual ~MYIdealAirFrame();
    MYIdealAirFrame& operator=(const MYIdealAirFrame& other);
    virtual MYIdealAirFrame *dup() const {return new MYIdealAirFrame(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual simtime_t getTransmissionDuration() const;
    virtual void setTransmissionDuration(simtime_t transmissionDuration);
    virtual Coord& getTransmissionStartPosition();
    virtual const Coord& getTransmissionStartPosition() const {return const_cast<MYIdealAirFrame*>(this)->getTransmissionStartPosition();}
    virtual void setTransmissionStartPosition(const Coord& transmissionStartPosition);
    virtual double getTransmissionRange() const;
    virtual void setTransmissionRange(double transmissionRange);
    virtual unsigned int getDroneDatabaseArraySize() const;
    virtual char getDroneDatabase(unsigned int k) const;
    virtual void setDroneDatabase(unsigned int k, char DroneDatabase);
    virtual int getK_parameter() const;
    virtual void setK_parameter(int K_parameter);
    virtual int getCurrK() const;
    virtual void setCurrK(int currK);
    virtual double getR_u() const;
    virtual void setR_u(double R_u);
    virtual int getCurrDepth_lvl() const;
    virtual void setCurrDepth_lvl(int currDepth_lvl);
    virtual int getIsFindFrame() const;
    virtual void setIsFindFrame(int isFindFrame);
    virtual int getIsReplyFrame() const;
    virtual void setIsReplyFrame(int isReplyFrame);
    virtual int getIsRelocationFrame() const;
    virtual void setIsRelocationFrame(int isRelocationFrame);
    virtual int getInitiatorID() const;
    virtual void setInitiatorID(int initiatorID);
    virtual int getFrame_ID() const;
    virtual void setFrame_ID(int frame_ID);
    virtual int getLastSender() const;
    virtual void setLastSender(int lastSender);
    virtual bool getLeaf_linker() const;
    virtual void setLeaf_linker(bool leaf_linker);
    virtual int getMinCover() const;
    virtual void setMinCover(int cover);
    virtual int getMinCoverID() const;
    virtual void setMinCoverID(int ID);
    virtual Coord getMinCoverLocation() const;
    virtual void setMinCoverLocation(Coord pos);
    virtual const char * getreplaceLinker() const;
    virtual void setreplaceLinker(const char * ID);
    virtual int getInitLinker() const;
    virtual void setInitLinker(int initLinker);
    virtual int getSenderLinker() const;
    virtual void setSenderLinker(int senderLinker);
};

inline void doPacking(cCommBuffer *b, MYIdealAirFrame& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, MYIdealAirFrame& obj) {obj.parsimUnpack(b);}


#endif // _MYIDEALAIRFRAME_M_H_