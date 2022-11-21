// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for utils/environmental
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
#include "utils/environmental.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class UTILS_EXPORT utils_msg_environmental_common : public MATLABROSMsgInterface<utils::environmental> {
  public:
    virtual ~utils_msg_environmental_common(){}
    virtual void copy_from_struct(utils::environmental* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const utils::environmental* msg, MultiLibLoader loader, size_t size = 1);
};
  void utils_msg_environmental_common::copy_from_struct(utils::environmental* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //obstacle_id
        const matlab::data::TypedArray<uint8_t> obstacle_id_arr = arr["ObstacleId"];
        msg->obstacle_id = obstacle_id_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ObstacleId' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ObstacleId' is wrong type; expected a uint8.");
    }
    try {
        //x
        const matlab::data::TypedArray<float> x_arr = arr["X"];
        msg->x = x_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'X' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'X' is wrong type; expected a single.");
    }
    try {
        //y
        const matlab::data::TypedArray<float> y_arr = arr["Y"];
        msg->y = y_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Y' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Y' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T utils_msg_environmental_common::get_arr(MDFactory_T& factory, const utils::environmental* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ObstacleId","X","Y"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("utils/environmental");
    // obstacle_id
    auto currentElement_obstacle_id = (msg + ctr)->obstacle_id;
    outArray[ctr]["ObstacleId"] = factory.createScalar(currentElement_obstacle_id);
    // x
    auto currentElement_x = (msg + ctr)->x;
    outArray[ctr]["X"] = factory.createScalar(currentElement_x);
    // y
    auto currentElement_y = (msg + ctr)->y;
    outArray[ctr]["Y"] = factory.createScalar(currentElement_y);
    }
    return std::move(outArray);
  } 
class UTILS_EXPORT utils_environmental_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~utils_environmental_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          utils_environmental_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<utils::environmental,utils_msg_environmental_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         utils_environmental_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<utils::environmental,utils::environmental::ConstPtr,utils_msg_environmental_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         utils_environmental_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<utils::environmental,utils_msg_environmental_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(utils_msg_environmental_common, MATLABROSMsgInterface<utils::environmental>)
CLASS_LOADER_REGISTER_CLASS(utils_environmental_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1