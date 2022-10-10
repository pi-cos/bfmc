// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for utils/vehicles
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "utils/vehicles.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class UTILS_EXPORT utils_msg_vehicles_common : public MATLABROSMsgInterface<utils::vehicles> {
  public:
    virtual ~utils_msg_vehicles_common(){}
    virtual void copy_from_struct(utils::vehicles* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const utils::vehicles* msg, MultiLibLoader loader, size_t size = 1);
};
  void utils_msg_vehicles_common::copy_from_struct(utils::vehicles* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //ID
        const matlab::data::TypedArray<uint8_t> ID_arr = arr["ID"];
        msg->ID = ID_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ID' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ID' is wrong type; expected a uint8.");
    }
    try {
        //timestamp
        const matlab::data::TypedArray<float> timestamp_arr = arr["Timestamp"];
        msg->timestamp = timestamp_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Timestamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Timestamp' is wrong type; expected a single.");
    }
    try {
        //posA
        const matlab::data::TypedArray<float> posA_arr = arr["PosA"];
        msg->posA = posA_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PosA' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PosA' is wrong type; expected a single.");
    }
    try {
        //posB
        const matlab::data::TypedArray<float> posB_arr = arr["PosB"];
        msg->posB = posB_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PosB' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PosB' is wrong type; expected a single.");
    }
    try {
        //rotA
        const matlab::data::TypedArray<float> rotA_arr = arr["RotA"];
        msg->rotA = rotA_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RotA' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RotA' is wrong type; expected a single.");
    }
    try {
        //rotB
        const matlab::data::TypedArray<float> rotB_arr = arr["RotB"];
        msg->rotB = rotB_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RotB' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RotB' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T utils_msg_vehicles_common::get_arr(MDFactory_T& factory, const utils::vehicles* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ID","Timestamp","PosA","PosB","RotA","RotB"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("utils/vehicles");
    // ID
    auto currentElement_ID = (msg + ctr)->ID;
    outArray[ctr]["ID"] = factory.createScalar(currentElement_ID);
    // timestamp
    auto currentElement_timestamp = (msg + ctr)->timestamp;
    outArray[ctr]["Timestamp"] = factory.createScalar(currentElement_timestamp);
    // posA
    auto currentElement_posA = (msg + ctr)->posA;
    outArray[ctr]["PosA"] = factory.createScalar(currentElement_posA);
    // posB
    auto currentElement_posB = (msg + ctr)->posB;
    outArray[ctr]["PosB"] = factory.createScalar(currentElement_posB);
    // rotA
    auto currentElement_rotA = (msg + ctr)->rotA;
    outArray[ctr]["RotA"] = factory.createScalar(currentElement_rotA);
    // rotB
    auto currentElement_rotB = (msg + ctr)->rotB;
    outArray[ctr]["RotB"] = factory.createScalar(currentElement_rotB);
    }
    return std::move(outArray);
  } 
class UTILS_EXPORT utils_vehicles_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~utils_vehicles_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          utils_vehicles_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<utils::vehicles,utils_msg_vehicles_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         utils_vehicles_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<utils::vehicles,utils::vehicles::ConstPtr,utils_msg_vehicles_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         utils_vehicles_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<utils::vehicles,utils_msg_vehicles_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(utils_msg_vehicles_common, MATLABROSMsgInterface<utils::vehicles>)
CLASS_LOADER_REGISTER_CLASS(utils_vehicles_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1