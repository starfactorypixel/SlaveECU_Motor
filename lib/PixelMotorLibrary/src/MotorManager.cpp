#include <MotorManager.h>

void MotorManager::SetModel(uint8_t idx, MotorDeviceInterface &device)
{
	if(idx >= _max_dev) return;
	if(device.IsInitiated() == true) return;
	
	device.PrepareInit(this, idx);
	device.Init();
	_device[idx] = &device;
	
	return;
}

void MotorManager::Tick(uint32_t &time)
{
	MotorDeviceInterface::error_code_t error_code;
	MotorDeviceInterface *device = nullptr;
	for(uint8_t idx = 0; idx < _max_dev; ++idx)
	{
		device = _device[idx];
		if(device == nullptr) continue;
		
		device->Tick(time);
		
		if(device->GetNewError(error_code) == true)
		{
			_callback_error(idx, error_code);
		}
	}
	
	return;
}

void MotorManager::RawRx(uint8_t idx, const uint8_t *raw, const uint8_t length)
{
	if(idx >= _max_dev) return;
	if(_device[idx] == nullptr) return;

	_device[idx]->RawRx(raw, length);
	
	return;
}

void MotorManager::RawTx(uint8_t idx, const uint8_t *raw, const uint8_t length)
{
	_callback_tx(idx, raw, length);
	
	return;
}

void MotorManager::ResetCommonData(uint8_t idx)
{
	if(idx >= _max_dev) return;
	
	memset(&common_data[idx], 0x00, sizeof(MotorManagerData::common_data_t));
	common_data_ready[idx] = false;

	return;
}
