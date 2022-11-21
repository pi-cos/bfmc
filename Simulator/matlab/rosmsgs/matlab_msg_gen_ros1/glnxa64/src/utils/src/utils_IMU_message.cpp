// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for utils/IMU
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
#include "utils/IMU.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class UTILS_EXPORT utils_msg_IMU_common : public MATLABROSMsgInterface<utils::IMU> {
  public:
    virtual ~utils_msg_IMU_common(){}
    virtual void copy_from_struct(utils::IMU* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const utils::IMU* msg, MultiLibLoader loader, size_t size = 1);
};
  void utils_msg_IMU_common::copy_from_struct(utils::IMU* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //roll
        const matlab::data::TypedArray<float> roll_arr = arr["Roll"];
        msg->roll = roll_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Roll' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Roll' is wrong type; expected a single.");
    }
    try {
        //pitch
        const matlab::data::TypedArray<float> pitch_arr = arr["Pitch"];
        msg->pitch = pitch_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Pitch' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Pitch' is wrong type; expected a single.");
    }
    try {
        //yaw
        const matlab::data::TypedArray<float> yaw_arr = arr["Yaw"];
        msg->yaw = yaw_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Yaw' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Yaw' is wrong type; expected a single.");
    }
    try {
        //accelx
        const matlab::data::TypedArray<float> accelx_arr = arr["Accelx"];
        msg->accelx = accelx_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Accelx' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Accelx' is wrong type; expected a single.");
    }
    try {
        //accely
        const matlab::data::TypedArray<float> accely_arr = arr["Accely"];
        msg->accely = accely_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Accely' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Accely' is wrong type; expected a single.");
    }
    try {
        //accelz
        const matlab::data::TypedArray<float> accelz_arr = arr["Accelz"];
        msg->accelz = accelz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Accelz' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Accelz' is wrong type; expected a single.");
    }
    try {
        //posx
        const matlab::data::TypedArray<float> posx_arr = arr["Posx"];
        msg->posx = posx_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Posx' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Posx' is wrong type; expected a single.");
    }
    try {
        //posy
        const matlab::data::TypedArray<float> posy_arr = arr["Posy"];
        msg->posy = posy_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Posy' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Posy' is wrong type; expected a single.");
    }
    try {
        //timestamp
        const matlab::data::TypedArray<double> timestamp_arr = arr["Timestamp"];
        msg->timestamp = timestamp_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Timestamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Timestamp' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T utils_msg_IMU_common::get_arr(MDFactory_T& factory, const utils::IMU* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Roll","Pitch","Yaw","Accelx","Accely","Accelz","Posx","Posy","Timestamp"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("utils/IMU");
    // roll
    auto currentElement_roll = (msg + ctr)->roll;
    outArray[ctr]["Roll"] = factory.createScalar(currentElement_roll);
    // pitch
    auto currentElement_pitch = (msg + ctr)->pitch;
    outArray[ctr]["Pitch"] = factory.createScalar(currentElement_pitch);
    // yaw
    auto currentElement_yaw = (msg + ctr)->yaw;
    outArray[ctr]["Yaw"] = factory.createScalar(currentElement_yaw);
    // accelx
    auto currentElement_accelx = (msg + ctr)->accelx;
    outArray[ctr]["Accelx"] = factory.createScalar(currentElement_accelx);
    // accely
    auto currentElement_accely = (msg + ctr)->accely;
    outArray[ctr]["Accely"] = factory.createScalar(currentElement_accely);
    // accelz
    auto currentElement_accelz = (msg + ctr)->accelz;
    outArray[ctr]["Accelz"] = factory.createScalar(currentElement_accelz);
    // posx
    auto currentElement_posx = (msg + ctr)->posx;
    outArray[ctr]["Posx"] = factory.createScalar(currentElement_posx);
    // posy
    auto currentElement_posy = (msg + ctr)->posy;
    outArray[ctr]["Posy"] = factory.createScalar(currentElement_posy);
    // timestamp
    auto currentElement_timestamp = (msg + ctr)->timestamp;
    outArray[ctr]["Timestamp"] = factory.createScalar(currentElement_timestamp);
    }
    return std::move(outArray);
  } 
class UTILS_EXPORT utils_IMU_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~utils_IMU_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          utils_IMU_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<utils::IMU,utils_msg_IMU_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         utils_IMU_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<utils::IMU,utils::IMU::ConstPtr,utils_msg_IMU_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         utils_IMU_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<utils::IMU,utils_msg_IMU_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(utils_msg_IMU_common, MATLABROSMsgInterface<utils::IMU>)
CLASS_LOADER_REGISTER_CLASS(utils_IMU_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1