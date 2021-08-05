// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for custom_msgs/Landmark
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
#include "custom_msgs/Landmark.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class CUSTOM_MSGS_EXPORT custom_msgs_msg_Landmark_common : public MATLABROSMsgInterface<custom_msgs::Landmark> {
  public:
    virtual ~custom_msgs_msg_Landmark_common(){}
    virtual void copy_from_struct(custom_msgs::Landmark* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const custom_msgs::Landmark* msg, MultiLibLoader loader, size_t size = 1);
};
  void custom_msgs_msg_Landmark_common::copy_from_struct(custom_msgs::Landmark* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //distance
        const matlab::data::TypedArray<float> distance_arr = arr["Distance"];
        msg->distance = distance_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Distance' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Distance' is wrong type; expected a single.");
    }
    try {
        //angle
        const matlab::data::TypedArray<float> angle_arr = arr["Angle"];
        msg->angle = angle_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Angle' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Angle' is wrong type; expected a single.");
    }
    try {
        //id
        const matlab::data::TypedArray<uint32_t> id_arr = arr["Id"];
        msg->id = id_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Id' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Id' is wrong type; expected a uint32.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T custom_msgs_msg_Landmark_common::get_arr(MDFactory_T& factory, const custom_msgs::Landmark* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Distance","Angle","Id"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("custom_msgs/Landmark");
    // distance
    auto currentElement_distance = (msg + ctr)->distance;
    outArray[ctr]["Distance"] = factory.createScalar(currentElement_distance);
    // angle
    auto currentElement_angle = (msg + ctr)->angle;
    outArray[ctr]["Angle"] = factory.createScalar(currentElement_angle);
    // id
    auto currentElement_id = (msg + ctr)->id;
    outArray[ctr]["Id"] = factory.createScalar(currentElement_id);
    }
    return std::move(outArray);
  } 
class CUSTOM_MSGS_EXPORT custom_msgs_Landmark_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~custom_msgs_Landmark_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          custom_msgs_Landmark_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<custom_msgs::Landmark,custom_msgs_msg_Landmark_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         custom_msgs_Landmark_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<custom_msgs::Landmark,custom_msgs::Landmark::ConstPtr,custom_msgs_msg_Landmark_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(custom_msgs_msg_Landmark_common, MATLABROSMsgInterface<custom_msgs::Landmark>)
CLASS_LOADER_REGISTER_CLASS(custom_msgs_Landmark_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1