/**
* This file is based on the unique_identifier stack by Jack O'Quin.
* My compiler couldn't find <unique_id/unique_id.h>,
* so i recreated the required functionality.
*
* @author Carsten Könemann
*/
#ifndef __UUID__
#define __UUID__

#include <uuid_msgs/UniqueID.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace uuid_msgs
{
  inline boost::uuids::uuid fromMsg(const uuid_msgs::UniqueID& uuid_msg)
  {
    boost::uuids::uuid boost_uuid;
    std::copy(uuid_msg.uuid.begin(), uuid_msg.uuid.end(), boost_uuid.begin());
    return boost_uuid;
  }
  
  inline uuid_msgs::UniqueID toMsg(const boost::uuids::uuid& boost_uuid)
  {
    uuid_msgs::UniqueID uuid_msg;
    std::copy(boost_uuid.begin(), boost_uuid.end(), uuid_msg.uuid.begin());
    return uuid_msg;
  }
  
  inline boost::uuids::uuid random()
  {
    return boost::uuids::random_generator()();
  }
} // namespace uuid_msgs

#endif //__UUID__
