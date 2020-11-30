//
// Generated file, do not edit! Created by opp_msgc 4.3 from Central_Algorithm/src/linklayer/MYIdealAirFrame.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "MYIdealAirFrame_m.h"

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}




Register_Class(MYIdealAirFrame);

MYIdealAirFrame::MYIdealAirFrame(const char *name, int kind) : cPacket(name,kind)
{
    this->transmissionDuration_var = 0;
    this->transmissionRange_var = 0;
    for (unsigned int i=0; i<2048; i++)
        this->DroneDatabase_var[i] = 0;
    this->K_parameter_var = 0;
    this->currK_var = 0;
    this->R_u_var = 0;
    this->currDepth_lvl_var = 0;
    this->isFindFrame_var = 0;
    this->isReplyFrame_var = 0;
    this->isRelocationFrame_var = 0;
    this->initiatorID_var = 0;
    this->frame_ID_var = 0;
    this->lastSender_var = 0;
    this->minCover = INT_MAX;
    this->minCoverID = -1;
    this->minCoverLocation ={-1,-1,0};
    this->replaceLinker="";
    this->leaf_linker_var = 0;
    this->initLinker = 0;
    this->senderLinker=0;
}

MYIdealAirFrame::MYIdealAirFrame(const MYIdealAirFrame& other) : cPacket(other)
{
    copy(other);
}

MYIdealAirFrame::~MYIdealAirFrame()
{
}

MYIdealAirFrame& MYIdealAirFrame::operator=(const MYIdealAirFrame& other)
{
    if (this==&other) return *this;
    cPacket::operator=(other);
    copy(other);
    return *this;
}

void MYIdealAirFrame::copy(const MYIdealAirFrame& other)
{
    this->transmissionDuration_var = other.transmissionDuration_var;
    this->transmissionStartPosition_var = other.transmissionStartPosition_var;
    this->transmissionRange_var = other.transmissionRange_var;
    for (unsigned int i=0; i<2048; i++)
        this->DroneDatabase_var[i] = other.DroneDatabase_var[i];
    this->K_parameter_var = other.K_parameter_var;
    this->currK_var = other.currK_var;
    this->R_u_var = other.R_u_var;
    this->currDepth_lvl_var = other.currDepth_lvl_var;
    this->isFindFrame_var = other.isFindFrame_var;
    this->isReplyFrame_var = other.isReplyFrame_var;
    this->isRelocationFrame_var = other.isRelocationFrame_var;
    this->initiatorID_var = other.initiatorID_var;
    this->frame_ID_var = other.frame_ID_var;
    this->lastSender_var = other.lastSender_var;
    this->leaf_linker_var = other.leaf_linker_var;
    this->initLinker = other.initLinker;
    this->senderLinker = other.senderLinker;

}

void MYIdealAirFrame::parsimPack(cCommBuffer *b)
{
    cPacket::parsimPack(b);
    doPacking(b,this->transmissionDuration_var);
    doPacking(b,this->transmissionStartPosition_var);
    doPacking(b,this->transmissionRange_var);
    doPacking(b,this->DroneDatabase_var,2048);
    doPacking(b,this->K_parameter_var);
    doPacking(b,this->currK_var);
    doPacking(b,this->R_u_var);
    doPacking(b,this->currDepth_lvl_var);
    doPacking(b,this->isFindFrame_var);
    doPacking(b,this->isReplyFrame_var);
    doPacking(b,this->isRelocationFrame_var);
    doPacking(b,this->initiatorID_var);
    doPacking(b,this->frame_ID_var);
    doPacking(b,this->lastSender_var);
    doPacking(b,this->leaf_linker_var);
    doPacking(b,this->initLinker);
    doPacking(b,this->senderLinker);

}

void MYIdealAirFrame::parsimUnpack(cCommBuffer *b)
{
    cPacket::parsimUnpack(b);
    doUnpacking(b,this->transmissionDuration_var);
    doUnpacking(b,this->transmissionStartPosition_var);
    doUnpacking(b,this->transmissionRange_var);
    doUnpacking(b,this->DroneDatabase_var,2048);
    doUnpacking(b,this->K_parameter_var);
    doUnpacking(b,this->currK_var);
    doUnpacking(b,this->R_u_var);
    doUnpacking(b,this->currDepth_lvl_var);
    doUnpacking(b,this->isFindFrame_var);
    doUnpacking(b,this->isReplyFrame_var);
    doUnpacking(b,this->isRelocationFrame_var);
    doUnpacking(b,this->initiatorID_var);
    doUnpacking(b,this->frame_ID_var);
    doUnpacking(b,this->lastSender_var);
    doUnpacking(b,this->leaf_linker_var);
    doUnpacking(b,this->initLinker);
    doUnpacking(b,this->senderLinker);

}

simtime_t MYIdealAirFrame::getTransmissionDuration() const
{
    return transmissionDuration_var;
}

void MYIdealAirFrame::setTransmissionDuration(simtime_t transmissionDuration)
{
    this->transmissionDuration_var = transmissionDuration;
}

Coord& MYIdealAirFrame::getTransmissionStartPosition()
{
    return transmissionStartPosition_var;
}

void MYIdealAirFrame::setTransmissionStartPosition(const Coord& transmissionStartPosition)
{
    this->transmissionStartPosition_var = transmissionStartPosition;
}

double MYIdealAirFrame::getTransmissionRange() const
{
    return transmissionRange_var;
}

void MYIdealAirFrame::setTransmissionRange(double transmissionRange)
{
    this->transmissionRange_var = transmissionRange;
}

unsigned int MYIdealAirFrame::getDroneDatabaseArraySize() const
{
    return 2048;
}

char MYIdealAirFrame::getDroneDatabase(unsigned int k) const
{
    if (k>=2048) throw cRuntimeError("Array of size 2048 indexed by %lu", (unsigned long)k);
    return DroneDatabase_var[k];
}

void MYIdealAirFrame::setDroneDatabase(unsigned int k, char DroneDatabase)
{
    if (k>=2048) throw cRuntimeError("Array of size 2048 indexed by %lu", (unsigned long)k);
    this->DroneDatabase_var[k] = DroneDatabase;
}

int MYIdealAirFrame::getK_parameter() const
{
    return K_parameter_var;
}

void MYIdealAirFrame::setK_parameter(int K_parameter)
{
    this->K_parameter_var = K_parameter;
}

int MYIdealAirFrame::getCurrK() const
{
    return currK_var;
}

void MYIdealAirFrame::setCurrK(int currK)
{
    this->currK_var = currK;
}

double MYIdealAirFrame::getR_u() const
{
    return R_u_var;
}

void MYIdealAirFrame::setR_u(double R_u)
{
    this->R_u_var = R_u;
}

int MYIdealAirFrame::getCurrDepth_lvl() const
{
    return currDepth_lvl_var;
}

void MYIdealAirFrame::setCurrDepth_lvl(int currDepth_lvl)
{
    this->currDepth_lvl_var = currDepth_lvl;
}

int MYIdealAirFrame::getIsFindFrame() const
{
    return isFindFrame_var;
}

void MYIdealAirFrame::setIsFindFrame(int isFindFrame)
{
    this->isFindFrame_var = isFindFrame;
}

int MYIdealAirFrame::getIsReplyFrame() const
{
    return isReplyFrame_var;
}

void MYIdealAirFrame::setIsReplyFrame(int isReplyFrame)
{
    this->isReplyFrame_var = isReplyFrame;
}

int MYIdealAirFrame::getIsRelocationFrame() const
{
    return isRelocationFrame_var;
}

void MYIdealAirFrame::setIsRelocationFrame(int isRelocationFrame)
{
    this->isRelocationFrame_var = isRelocationFrame;
}

int MYIdealAirFrame::getInitiatorID() const
{
    return initiatorID_var;
}

void MYIdealAirFrame::setInitiatorID(int initiatorID)
{
    this->initiatorID_var = initiatorID;
}

int MYIdealAirFrame::getFrame_ID() const
{
    return frame_ID_var;
}

void MYIdealAirFrame::setFrame_ID(int frame_ID)
{
    this->frame_ID_var = frame_ID;
}

int MYIdealAirFrame::getLastSender() const
{
    return lastSender_var;
}

void MYIdealAirFrame::setLastSender(int lastSender)
{
    this->lastSender_var = lastSender;
}


bool MYIdealAirFrame::getLeaf_linker() const
{
    return leaf_linker_var;
}

void MYIdealAirFrame::setLeaf_linker(bool leaf_linker)
{
    this->leaf_linker_var = leaf_linker;
}
int MYIdealAirFrame::getMinCover() const
{
    return minCover;
}

void MYIdealAirFrame::setMinCover(int cover)
{
    this->minCover = cover;
}

int MYIdealAirFrame::getMinCoverID() const
{
    return minCoverID;
}

void MYIdealAirFrame::setMinCoverID(int ID)
{
    this->minCoverID = ID;
}

Coord MYIdealAirFrame::getMinCoverLocation() const
{
    return minCoverLocation;
}

void MYIdealAirFrame::setMinCoverLocation(Coord pos)
{
    this->minCoverLocation = pos;
}
const char * MYIdealAirFrame::getreplaceLinker() const
{
    return replaceLinker;
}

void MYIdealAirFrame::setreplaceLinker(const char * ID)
{
    this->replaceLinker = ID;
}

int MYIdealAirFrame::getInitLinker() const
{
    return initLinker;
}

void MYIdealAirFrame::setInitLinker(int initLinker)
{
    this->initLinker = initLinker;
}

int MYIdealAirFrame::getSenderLinker() const
{
    return senderLinker;
}

void MYIdealAirFrame::setSenderLinker(int senderLinker)
{
    this->senderLinker = senderLinker;
}
class MYIdealAirFrameDescriptor : public cClassDescriptor
{
  public:
    MYIdealAirFrameDescriptor();
    virtual ~MYIdealAirFrameDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(MYIdealAirFrameDescriptor);

MYIdealAirFrameDescriptor::MYIdealAirFrameDescriptor() : cClassDescriptor("MYIdealAirFrame", "cPacket")
{
}

MYIdealAirFrameDescriptor::~MYIdealAirFrameDescriptor()
{
}

bool MYIdealAirFrameDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<MYIdealAirFrame *>(obj)!=NULL;
}

const char *MYIdealAirFrameDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int MYIdealAirFrameDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 17+basedesc->getFieldCount(object) : 17;
}

unsigned int MYIdealAirFrameDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISARRAY | FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<17) ? fieldTypeFlags[field] : 0;
}

const char *MYIdealAirFrameDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "transmissionDuration",
        "transmissionStartPosition",
        "transmissionRange",
        "DroneDatabase",
        "K_parameter",
        "currK",
        "R_u",
        "currDepth_lvl",
        "isFindFrame",
        "isReplyFrame",
        "isRelocationFrame",
        "initiatorID",
        "frame_ID",
        "lastSender",
        "leaf_linker",
        "initLinker",
        "senderLinker",
    };
    return (field>=0 && field<17) ? fieldNames[field] : NULL;
}

int MYIdealAirFrameDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='t' && strcmp(fieldName, "transmissionDuration")==0) return base+0;
    if (fieldName[0]=='t' && strcmp(fieldName, "transmissionStartPosition")==0) return base+1;
    if (fieldName[0]=='t' && strcmp(fieldName, "transmissionRange")==0) return base+2;
    if (fieldName[0]=='D' && strcmp(fieldName, "DroneDatabase")==0) return base+3;
    if (fieldName[0]=='K' && strcmp(fieldName, "K_parameter")==0) return base+4;
    if (fieldName[0]=='c' && strcmp(fieldName, "currK")==0) return base+5;
    if (fieldName[0]=='R' && strcmp(fieldName, "R_u")==0) return base+6;
    if (fieldName[0]=='c' && strcmp(fieldName, "currDepth_lvl")==0) return base+7;
    if (fieldName[0]=='i' && strcmp(fieldName, "isFindFrame")==0) return base+8;
    if (fieldName[0]=='i' && strcmp(fieldName, "isReplyFrame")==0) return base+9;
    if (fieldName[0]=='i' && strcmp(fieldName, "isRelocationFrame")==0) return base+10;
    if (fieldName[0]=='i' && strcmp(fieldName, "initiatorID")==0) return base+11;
    if (fieldName[0]=='f' && strcmp(fieldName, "frame_ID")==0) return base+12;
    if (fieldName[0]=='l' && strcmp(fieldName, "lastSender")==0) return base+13;
    if (fieldName[0]=='l' && strcmp(fieldName, "leaf_linker")==0) return base+14;
    if (fieldName[0]=='i' && strcmp(fieldName, "initLinker")==0) return base+15;
    if (fieldName[0]=='s' && strcmp(fieldName, "senderLinker")==0) return base+16;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *MYIdealAirFrameDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "simtime_t",
        "Coord",
        "double",
        "char",
        "int",
        "int",
        "double",
        "int",
        "int",
        "int",
        "int",
        "int",
        "int",
        "int",
        "bool",
        "int",
        "int",

    };
    return (field>=0 && field<17) ? fieldTypeStrings[field] : NULL;
}

const char *MYIdealAirFrameDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    }
}

int MYIdealAirFrameDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    MYIdealAirFrame *pp = (MYIdealAirFrame *)object; (void)pp;
    switch (field) {
        case 3: return 2048;
        default: return 0;
    }
}

std::string MYIdealAirFrameDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    MYIdealAirFrame *pp = (MYIdealAirFrame *)object; (void)pp;
    switch (field) {
        case 0: return double2string(pp->getTransmissionDuration());
        case 1: {std::stringstream out; out << pp->getTransmissionStartPosition(); return out.str();}
        case 2: return double2string(pp->getTransmissionRange());
        case 3: return long2string(pp->getDroneDatabase(i));
        case 4: return long2string(pp->getK_parameter());
        case 5: return long2string(pp->getCurrK());
        case 6: return double2string(pp->getR_u());
        case 7: return long2string(pp->getCurrDepth_lvl());
        case 8: return long2string(pp->getIsFindFrame());
        case 9: return long2string(pp->getIsReplyFrame());
        case 10: return long2string(pp->getIsRelocationFrame());
        case 11: return long2string(pp->getInitiatorID());
        case 12: return long2string(pp->getFrame_ID());
        case 13: return long2string(pp->getLastSender());
        case 14: return bool2string(pp->getLeaf_linker());
        case 15: return long2string(pp->getInitLinker());
        case 16: return long2string(pp->getSenderLinker());

        default: return "";
    }
}

bool MYIdealAirFrameDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    MYIdealAirFrame *pp = (MYIdealAirFrame *)object; (void)pp;
    switch (field) {
        case 0: pp->setTransmissionDuration(string2double(value)); return true;
        case 2: pp->setTransmissionRange(string2double(value)); return true;
        case 3: pp->setDroneDatabase(i,string2long(value)); return true;
        case 4: pp->setK_parameter(string2long(value)); return true;
        case 5: pp->setCurrK(string2long(value)); return true;
        case 6: pp->setR_u(string2double(value)); return true;
        case 7: pp->setCurrDepth_lvl(string2long(value)); return true;
        case 8: pp->setIsFindFrame(string2long(value)); return true;
        case 9: pp->setIsReplyFrame(string2long(value)); return true;
        case 10: pp->setIsRelocationFrame(string2long(value)); return true;
        case 11: pp->setInitiatorID(string2long(value)); return true;
        case 12: pp->setFrame_ID(string2long(value)); return true;
        case 13: pp->setLastSender(string2long(value)); return true;
        case 14: pp->setLeaf_linker(string2bool(value)); return true;
        case 15: pp->setInitLinker(string2long(value)); return true;
        case 16: pp->setSenderLinker(string2long(value)); return true;


        default: return false;
    }
}
const char *MYIdealAirFrameDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldStructNames[] = {
        NULL,
        "Coord",
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
     };
    return (field>=0 && field<17) ? fieldStructNames[field] : NULL;
}

void *MYIdealAirFrameDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    MYIdealAirFrame *pp = (MYIdealAirFrame *)object; (void)pp;
    switch (field) {
        case 1: return (void *)(&pp->getTransmissionStartPosition()); break;
        default: return NULL;
    }
}


