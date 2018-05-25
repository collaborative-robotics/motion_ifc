#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <ros/rate.h>

class WatchDog{
public:
    WatchDog(double time_out = 0.5){
        m_expire_duration.fromSec(time_out);
    }

    void set_wd_time_out(double time_out){
        m_expire_duration.fromSec(time_out);
    }

    inline void acknowledge_wd(){
        m_next_cmd_expected_time= ros::Time::now() + m_expire_duration;
    }
    inline bool is_wd_expired(){
        bool expired = ros::Time::now() > m_next_cmd_expected_time ? true : false;
        return expired;
    }

private:
    ros::Time m_next_cmd_expected_time;
    ros::Duration m_expire_duration;
    bool m_initialized;
};
#endif // WATCHDOG_H
