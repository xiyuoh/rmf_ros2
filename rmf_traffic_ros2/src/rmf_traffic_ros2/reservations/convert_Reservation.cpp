#include <rmf_traffic_msgs/msg/reservation.hpp>
#include <rmf_traffic/reservations/Reservation.hpp>

#include <rmf_traffic_ros2/reservations/Reservation.hpp>
#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_traffic_ros2 {

///=============================================================================
rmf_traffic::reservations::Reservation convert(
  rmf_traffic_msgs::msg::Reservation& reservation)
{
  return rmf_traffic::reservations::Reservation::make_reservation(
    convert(reservation.start_time),
    reservation.resource,
    (reservation.has_duration) ?
      std::optional{convert(reservation.duration)} : std::nullopt,
    (reservation.has_finish_time) ?
      std::optional{convert(reservation.finish_time)} : std::nullopt
  );
}

///=============================================================================
rmf_traffic_msgs::msg::Reservation convert(
  rmf_traffic::reservations::Reservation& reservation)
{
  rmf_traffic_msgs::msg::Reservation msg;
  msg.start_time = convert(reservation.start_time());
  msg.resource = reservation.resource_name();

  msg.has_duration = reservation.duration().has_value();
  if(msg.has_duration)
    msg.duration = convert(reservation.duration().value());

  msg.has_finish_time = reservation.duration().has_value();
  if(msg.has_finish_time)
    msg.finish_time = convert(reservation.finish_time().value());

  return msg;
}


}