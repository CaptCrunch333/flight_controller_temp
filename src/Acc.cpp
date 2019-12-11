#include "Acc.hpp"
#include <iostream>
//TODO remove

Acc::Acc()
{
	calibration_offset = 0;
}

sens_type Acc::getSensType(void)
{
	return ACCELEROMETER;
}

void Acc::resetCalibration(void)
{
	calibration_offset = 0;
	calibration_counter = 0;
}

void Acc::startCalibration(void)
{
	resetCalibration();
	calibration_flag = true;
}

void Acc::stopCalibration(void)
{
	calibration_flag = false;
	updateCalibrationTerms();
}

void Acc::updateCalibrationTerms(void)
{
	calibration_offset = vec_3d<int32_t>(calibration_temp) * conv_coef;
	calibration_offset.z = calibration_offset.z - (conv_coef*g_in_working_range);
	std::cout << "Calib_X " <<  calibration_offset.x << std::endl;
	std::cout << "Calib_Y " <<  calibration_offset.y << std::endl;
	std::cout << "Calib_Z " <<  calibration_offset.z << std::endl;
	std::cout << "Calib_Sens " <<  sens << std::endl;
}

sen_status Acc::getSensorStatus(void)
{
	if(calibration_flag)
	{
		return CALIBRATING;
	}
	else
	{
		return CALIBRATED;
	}
}

void Acc::doCalibrationCycle(void)
{
	float avg_factor = 0;

	if((calibration_counter + 1) > 0)
	{
		calibration_counter++;

		avg_factor = (float) (calibration_counter - 1);
		avg_factor = (float) avg_factor/calibration_counter;
		calibration_temp = vec_3d<int>(vec_3d<float>(calibration_temp) * avg_factor + vec_3d<float>(raw) / (float)calibration_counter);
	}
}

void Acc::setSens(float tmp)
{
	sens = tmp;
}

void Acc::setCalibCoeff(float tmp)
{
	conv_coef = tmp;
}

void Acc::setgWorkingRange(float tmp)
{
	g_in_working_range = tmp;
}

vec_3d<float> Acc::getCalibratedData(void)
{
	raw = getRawData();
	if(calibration_flag) //TODO: raw is int, but the func returns float
	{
		// std::cout << "Raw_X " <<  raw.x << std::endl;
		// std::cout << "Raw_Y " <<  raw.y << std::endl;
		// std::cout << "Raw_Z " <<  raw.z << std::endl;
		doCalibrationCycle();
		return raw;
	}
	calibrated = vec_3d<float>((raw - calibration_offset)) / sens;

	// std::cout << "Calibrated_X " <<  calibrated.x << std::endl;
	// std::cout << "Calibrated_Y " <<  calibrated.y << std::endl;
	// std::cout << "Calibrated_Z " <<  calibrated.z << std::endl;

	return calibrated;
}

vec_3d<float> Acc::getBodyAcceleration(void)
{
	return getCalibratedData();
}