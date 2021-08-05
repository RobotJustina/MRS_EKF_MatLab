// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for custom_msgs/Landmarks
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
#include "custom_msgs/Landmarks.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
COMMON_FW_DECLARE(custom_msgs_msg_Landmark_common, custom_msgs::Landmark)
class CUSTOM_MSGS_EXPORT custom_msgs_msg_Landmarks_common : public MATLABROSMsgInterface<custom_msgs::Landmarks> {
  public:
    virtual ~custom_msgs_msg_Landmarks_common(){}
    virtual void copy_from_struct(custom_msgs::Landmarks* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const custom_msgs::Landmarks* msg, MultiLibLoader loader, size_t size = 1);
};
  void custom_msgs_msg_Landmarks_common::copy_from_struct(custom_msgs::Landmarks* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //landmarks
        const matlab::data::StructArray landmarks_arr = arr["Landmarks_"];
        for (auto _landmarksarr : landmarks_arr) {
        	custom_msgs::Landmark _val;
        static custom_msgs_msg_Landmark_common obj_landmarks;
        	obj_landmarks.copy_from_struct(&_val,_landmarksarr,loader);
        	msg->landmarks.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Landmarks_' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Landmarks_' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T custom_msgs_msg_Landmarks_common::get_arr(MDFactory_T& factory, const custom_msgs::Landmarks* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Landmarks_"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("custom_msgs/Landmarks");
    // landmarks
    auto currentElement_landmarks = (msg + ctr)->landmarks;
    static custom_msgs_msg_Landmark_common obj_landmarks;
    	outArray[ctr]["Landmarks_"] = obj_landmarks.get_arr(factory,&currentElement_landmarks[0],loader,currentElement_landmarks.size());
    }
    return std::move(outArray);
  } 
class CUSTOM_MSGS_EXPORT custom_msgs_Landmarks_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~custom_msgs_Landmarks_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          custom_msgs_Landmarks_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<custom_msgs::Landmarks,custom_msgs_msg_Landmarks_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         custom_msgs_Landmarks_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<custom_msgs::Landmarks,custom_msgs::Landmarks::ConstPtr,custom_msgs_msg_Landmarks_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(custom_msgs_msg_Landmarks_common, MATLABROSMsgInterface<custom_msgs::Landmarks>)
CLASS_LOADER_REGISTER_CLASS(custom_msgs_Landmarks_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1