#include "acc_gyro_attitude_observer.hpp"

AccGyroAttitudeObserver::AccGyroAttitudeObserver(BodyAccProvider* t_acc, BodyRateProvider* t_rate) : AttitudeProvider(t_acc, t_rate)
{
    m_acc = t_acc;
    m_rate = t_rate;
}

void AccGyroAttitudeObserver::setFilterType(DataFilter* t_pitch_filter, DataFilter* t_roll_filter)
{
    m_pitch_filter = t_pitch_filter;
    m_roll_filter = t_roll_filter;
}

void AccGyroAttitudeObserver::updateSettings(FilterSettings* t_settings, float t_val_threshold)
{
    m_val_threshold = t_val_threshold;
    m_pitch_filter->setFilterSettings(t_settings);
    m_roll_filter->setFilterSettings(t_settings);
}
AttitudeMsg AccGyroAttitudeObserver::getAttitude()
{
    AttitudeMsg acc_attitude;
    acc_data = m_acc->getBodyAcceleration();
    gyro_data = m_rate->getBodyRate();
    acc_attitude = getAccAttitude();
    if(fabs(sqrt((acc_data.x*acc_data.x) + (acc_data.y*acc_data.y) + (acc_data.z*acc_data.z)) - grav) > m_val_threshold)
    {
        filtered_attitude.pitch = m_pitch_filter->getFilteredData(gyro_data.y);
        filtered_attitude.roll = m_roll_filter->getFilteredData(gyro_data.x);
    }
    else
    {
        filtered_attitude.pitch = m_pitch_filter->getFilteredData(acc_attitude.pitch, gyro_data.y);
        filtered_attitude.roll = m_roll_filter->getFilteredData(acc_attitude.roll, gyro_data.x);
    }
    return filtered_attitude;    
}

AttitudeMsg AccGyroAttitudeObserver::getAccAttitude()
{
    AttitudeMsg acc_att;
    acc_att.pitch =  atan2(-acc_data.x , sqrt((acc_data.y*acc_data.y) + (acc_data.z*acc_data.z)));
    acc_att.roll = atan2( acc_data.y , sqrt((acc_data.x*acc_data.x) + (acc_data.z*acc_data.z)));
    return acc_att;
}