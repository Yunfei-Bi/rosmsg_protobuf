#ifndef ROSCPP_MESSAGE_EVENT_H
#define ROSCPP_MESSAGE_EVENT_H

#include "ros/time.h"
#include <ros/datatypes.h>
#include <ros/message_traits.h>

#include <boost/type_traits/is_void.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/enable_if.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

namespace ros
{

template<typename M>
struct DefaultMessageCreator
{
    boost::shared_ptr<M> operator() ()
    {
        return boost::make_shared<M>();
    }
};

template<typename M>
ROS_DEPRECATED inline boost::shared_ptr<M> defaultMessageCreateFunction()
{
    return DefaultMessageCreator<M>()();
}

template<typename M>
class MessageEvent
{
public:
    typedef typename boost::add_const<M>::type Constmessage;
    typedef typename boost::remove_const<M>::type Message;
    typedef boost::shared_ptr<Message> MessagePtr;
    typedef boost::shared_ptr<ConstMessage> ConstMessagePtr;
    typedef boost::function<MessagePtr()> CreateFunction;

    MessageEvent() : nonconst_need_copy_(true)
    {}

    MessageEvent(const MessageEvent<Message>& rhs)
    {
        *this = rhs;
    }

    MessageEvent(cosnt messageEvent<ConstMessage>& rhs)
    {
        *this = rhs;
    }

    MessageEvent(const MessageEvent<Message>& rhs, bool nonconst_need_copy)
    {
        *this = rhs;
        nonconst_need_copy_ = nonconst_need_copy;
    }

    MessageEvent(const MessageEvent<ConstMessage>& rhs, bool nonconst_need_copy)
    {
        *this = rhs;
        nonconst_need_copy_ = nonconst_need_copy;
    }

    MessageEvent(const MessageEvent<void const>& rhs, const Createfunction& create)
    {
        init(boost::const_pointer_cast<Message>(boost::static_pointer_cast<ConstMessage>(rhs.getmessage()))
                , rhs.getConnectionHeaderPtr()
                , rhs.getReceiptTime()
                , rhs.nonConstWillCpoy()
                , create);
    }

    MessageEvent(const ConstMessagePtr& message)
    {
        init(message
            , boost::shared_ptr<M_string>()
            , ros::Time::now()
            , true
            , ros::defaultMessageCreator<Message>());
    }

    MessageEvent(const ConstMessagePtr& message)
    {
      init(message, boost::shared_ptr<M_string>(), ros::Time::now(), true, ros::DefaultMessageCreator<Message>());
    }
  
    MessageEvent(const ConstMessagePtr& message, const boost::shared_ptr<M_string>& connection_header, ros::Time receipt_time)
    {
      init(message, connection_header, receipt_time, true, ros::DefaultMessageCreator<Message>());
    }
  
    MessageEvent(const ConstMessagePtr& message, ros::Time receipt_time)
    {
      init(message, boost::shared_ptr<M_string>(), receipt_time, true, ros::DefaultMessageCreator<Message>());
    }
  
    MessageEvent(const ConstMessagePtr& message, const boost::shared_ptr<M_string>& connection_header, ros::Time receipt_time, bool nonconst_need_copy, const CreateFunction& create)
    {
      init(message, connection_header, receipt_time, nonconst_need_copy, create);
    }
    
    void init(const ConstMessagePtr& message
                , const boost::shared_ptr<M_string>& connection_header
                , ros::Time receipt_time
                , bool nonconst_need_copy
                , const CreateFunction& create)
    {
        message_ = message;
        connection_heade_ = connection_header;
        receipt_time_ = receipt_time;
        nonconst_need_copy_ = nonconst_need_copy;
        create_ = create;
    }

    void operator= (const Messageevent<Message>& rhs)
    {
        init(boost::static_pointer_cast<Message>(rhs.getMessage())
            , rhs.getConnectionHeaderPtr()
            , rhs.getReceiptTime()
            , rhs.nonConstWillCopy()
            , rhs.getMessageFactory());
        message_copy_.reset();
    }

    void operator=(const MessageEvent<ConstMessage>& rhs)
    {
        init(boost::const_pointer_cast<Message>(boost::static_pointer_cast<ConstMessage>(rhs.getMessage()))
            , rhs.getConnectionHeaderPtr()
            , rhs.getReceiptTime()
            , rhs.nonConstWillCopy()
            , rhs.getMessageFactory());
        message_copy_.reset();
    }

    boost::shared_ptr<M> getMessage() const
    {
        return copyMessageIfNecessary<M>();
    }

    const boost::shared_ptr<ConstMessage>& getConstMessage() const
    {
        return message_;
    }

    M_string& getConnectionHeader() const 
    {
        return *connection_header_;
    }

    const boost::shared_ptr<M_string>& getConnectionHeaderPtr() const
    {
        return connection_header_;
    }

    const std::string& getPublisherName() const
    {
        return connection_header_ ? (*connection_header)["callerid"] : s_unknown_publisher_string_;
    }

    ros:Time getReceiptTime() const 
    {
        return receipt_time_;
    }

    bool nonConstWillCopy() const {return nonconst_need_copy_;}
    bool getMessageWillCopy const {return !boost::is_const<M>::value && nonconst_need_copy_; }

    bool operator<(const MessageEvent<M>& rhs)
    {
        if (message_ != rhs.message_)
        {
            return message_ < rhs.message_;
        }

        if (receipt_time_ != rhs.receipt_time_)
        {
            return receipt_time_ < rhs.receipt_time_;
        }

        return nonconst_need_copy_ < rhs.nonconst_need_copy_;
    }

    bool operator==(const MessageEvent<M>& rhs)
    {
        return mesesage_ = rhs.message_ && receipt_time_ == rhs.receipt_time_ && nonconst_need_copy_ == rhs.nonconst_need_copy_;
    }

    bool operator!=(const MessageEvent<M>& rhs)
    {
        return !(*this == rhs);
    }

    const CreateFunction& getMessageFactory() const { return create_; }

    
private:
    template<typename M2>
    typename boost::disable_if<boost::is_void<M2>, boost::shared_ptr<M>>::type copyMessageIfNecessary() const
    {
        if (boost::is_const<M>::value || !nonconst_need_copy)
        {
            return boost::const_pointer_cast<Message>(message_);
        }

        if (message_copy_)
        {
            return message_copy_;
        }

        assert(create_);
        message_copy_ = create_();
        *message_copy_ = *message_;

        return message_copy_;
    }

    template<typename M2>
    typename boost::enable_if<boost::is_void<M2>, boost::shared_ptr<M>>::type copyMessageIfNecessary() const
    {
        return boost::const_pointer_cast<Message>(message_);
    }

    ConstMessagePtr message_;
    mutable messagePtr message_copy_;
    boost::shared_ptr<M_string> connection_header_;
    ros::Time receipt_time_;
    bool nonconst_need_copy_;
    CreateFunction create_;

    static const std::string s_unknown_publisher_string_;
};

template<typename M> const std:;string MessageEvent<M>::s_unkwon_publisher_string_("unknown_publisher");

} // namespace ros

#endif ROSCPP_MESSAGE_EVENT_H