#include "CRC8.h"
#include <Arduino.h>
#include "goBILDA_Pinpoint.h"

void goBILDA::Pinpoint::begin(TwoWire &wire)
{
    i2c = wire;
    i2c.begin();

    if(getDeviceID() == PINPOINT_DEVICE_ID)
    {
        uint32_t version = getDeviceVersion();
        if(version >= 3)
            hasUpdatedFirmware = true;
    }
}

goBILDA::PinpointError goBILDA::Pinpoint::getLastError() const
{
    return _lastError;
}

uint32_t goBILDA::Pinpoint::getDeviceID(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::DEVICE_ID, sizeof(uint32_t));
        _lastDeviceId = convertVectorToUint(vec);
    }
    return _lastDeviceId;
}

uint32_t goBILDA::Pinpoint::getDeviceVersion(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::DEVICE_VERSION, sizeof(uint32_t));
        _lastDeviceVersion = convertVectorToUint(vec);
    }
    return _lastDeviceVersion;
}

float goBILDA::Pinpoint::getYawScalar(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::YAW_SCALAR, sizeof(float));
        _lastYawScalar = convertVectorToFloat(vec);
    }
    return _lastYawScalar;
}

goBILDA::PinpointStatus goBILDA::Pinpoint::getDeviceStatus(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::DEVICE_STATUS, sizeof(uint32_t));
        _lastDeviceStatus = convertVectorToUint(vec);
    }
    return PinpointStatus::GetStatus(_lastDeviceStatus);
}

uint32_t goBILDA::Pinpoint::getLoopTime(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::LOOP_TIME, sizeof(uint32_t));
        _lastLoopTime = convertVectorToUint(vec);
    }

    return _lastLoopTime;
}

float goBILDA::Pinpoint::getFrequency(bool useBulkReadValue)
{
    uint32_t loopTime = getLoopTime(useBulkReadValue);
    if(loopTime == 0)
        return 0;
    return 1000000.0 / loopTime;
}

int32_t goBILDA::Pinpoint::getEncoderX(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::X_ENCODER_VALUE, sizeof(int32_t));
        _lastEncoderX = convertVectorToint(vec);
    }
    return _lastEncoderX;
}

int32_t goBILDA::Pinpoint::getEncoderY(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::Y_ENCODER_VALUE, sizeof(int32_t));
        _lastEncoderY = convertVectorToint(vec);
    }
    return _lastEncoderY;
}

float goBILDA::Pinpoint::getPositionXInMM(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::X_POSITION, sizeof(float));
        _lastPositionX = convertVectorToFloat(vec);
    }
    return _lastPositionX;
}

float goBILDA::Pinpoint::getPositionYInMM(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::Y_POSITION, sizeof(float));
        _lastPositionY = convertVectorToFloat(vec);
    }
    return _lastPositionY;
}

float goBILDA::Pinpoint::getNormalizedHeading(bool useBulkReadValue) // AngleUnit
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::H_ORIENTATION, sizeof(float));
        _lastHeading = convertVectorToFloat(vec);
    }

    // Wrap to [-180, 180) degrees, keeping the sign of the heading.
    float heading = fmod(_lastHeading * RAD_TO_DEG + 180.0, 360.0);
    if (heading < 0)
        heading += 360.0f;
    return heading - 180.0f;
}

float goBILDA::Pinpoint::getUnNormalizedHeading(bool useBulkReadValue)     // UnnormalizedAngleUnit unnormalizedAngleUnit
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::H_ORIENTATION, sizeof(float));
        _lastHeading = convertVectorToFloat(vec);
    }
    
    return _lastHeading * RAD_TO_DEG;
}

float goBILDA::Pinpoint::getVelocityX(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::X_VELOCITY, sizeof(float));
        _lastVelocityX = convertVectorToFloat(vec);
    }
    return _lastVelocityX;
}

float goBILDA::Pinpoint::getVelocityY(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::Y_VELOCITY, sizeof(float));
        _lastVelocityY = convertVectorToFloat(vec);
    }
    return _lastVelocityY;
}

float goBILDA::Pinpoint::getVelocityHeading(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::H_VELOCITY, sizeof(float));
        _lastVelocityH = convertVectorToFloat(vec);
    }
    return _lastVelocityH;
}

float goBILDA::Pinpoint::getMmPerTick(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::MM_PER_TICK, sizeof(float));
        _lastMmPerTick = convertVectorToFloat(vec);
    }
    return _lastMmPerTick;
}

float goBILDA::Pinpoint::getOffsetX(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::X_POD_OFFSET, sizeof(float));
        _lastOffsetX = convertVectorToFloat(vec);
    }
    return _lastOffsetX;
}

float goBILDA::Pinpoint::getOffsetY(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::Y_POD_OFFSET, sizeof(float));
        _lastOffsetY = convertVectorToFloat(vec);
    }
    return _lastOffsetY;
}

float goBILDA::Pinpoint::getPitch(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::PITCH))
        return 0.0;

    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::PITCH, sizeof(float));
        _lastPitch = convertVectorToFloat(vec);
    }
    return _lastPitch;
}

float goBILDA::Pinpoint::getRoll(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::ROLL))
        return 0.0;

    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::ROLL, sizeof(float));
        _lastRoll = convertVectorToFloat(vec);
    }
    return _lastRoll;
}

float goBILDA::Pinpoint::getQuaternionW(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_W))
        return 0.0;

    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::QUATERNION_W, sizeof(float));
        _lastQuaternionW = convertVectorToFloat(vec);
    }
    return _lastQuaternionW;
}

float goBILDA::Pinpoint::getQuaternionX(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_X))
        return 0.0;

    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::QUATERNION_X, sizeof(float));
        _lastQuaternionX = convertVectorToFloat(vec);
    }
    return _lastQuaternionX;
}

float goBILDA::Pinpoint::getQuaternionY(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_Y))
        return 0.0;

    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::QUATERNION_Y, sizeof(float));
        _lastQuaternionY = convertVectorToFloat(vec);
    }
    return _lastQuaternionY;
}

float goBILDA::Pinpoint::getQuaternionZ(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_Z))
        return 0.0;

    if(!useBulkReadValue){
        std::vector<uint8_t> vec = getData(Register::QUATERNION_Z, sizeof(float));
        _lastQuaternionZ = convertVectorToFloat(vec);
    }
    return _lastQuaternionZ;
}

goBILDA::Pose2D goBILDA::Pinpoint::getPosition(bool useBulkReadValue)     // Pose2D
{
    float posX = getPositionXInMM(useBulkReadValue);
    float posY = getPositionYInMM(useBulkReadValue);
    float head = getUnNormalizedHeading(useBulkReadValue);

    return goBILDA::Pose2D{posX, posY, head};
}

goBILDA::Quaternion goBILDA::Pinpoint::getQuaternion(bool useBulkReadValue)   // Quaternion
{
    float w = getQuaternionW(useBulkReadValue);
    float x = getQuaternionX(useBulkReadValue);
    float y = getQuaternionY(useBulkReadValue);
    float z = getQuaternionZ(useBulkReadValue);

    return goBILDA::Quaternion{w, x, y, z};
}

goBILDA::BulkReadData goBILDA::Pinpoint::bulkRead(void)
{
    std::vector<uint8_t> data = getData(Register::BULK_READ, sizeof(uint32_t) * bulkReadScope.size());
    BulkReadData brData;
    brData.Error = getLastError();

    if(brData.Error != PinpointError::None)
        return brData;

    for(int i = 0; i < bulkReadScope.size(); i++){
        std::vector<uint8_t>::const_iterator bytes_start = data.begin() + (i * 4);
        std::vector<uint8_t> bytes(bytes_start, bytes_start + 4);
        saveData(bulkReadScope[i], bytes, brData);
    }

    return brData;
}

void goBILDA::Pinpoint::resetBulkRead(void)
{
    std::vector<uint8_t> newBulkReadScope = {static_cast<uint8_t>(Register::BULK_READ)};
    writeData(Register::SET_BULK_READ, newBulkReadScope);
    bulkReadScope = {
        PinpointRegisters::DeviceStatus,
        PinpointRegisters::LoopTime,
        PinpointRegisters::EncoderValueX,
        PinpointRegisters::EncoderValueY,
        PinpointRegisters::PositionX,
        PinpointRegisters::PositionY,
        PinpointRegisters::Heading,
        PinpointRegisters::VelocityX,
        PinpointRegisters::VelocityY,
        PinpointRegisters::VelocityH
    };
}

void goBILDA::Pinpoint::setBulkReadScope(std::vector<PinpointRegisters> registers)
{
    if(!hasUpdatedFirmware)
        return;
    
    uint32_t reg_bm = 0;    // Remove all duplicates
    bulkReadScope.clear();
    for(int i = 0; i < registers.size(); i++){
        uint32_t curr_reg = 1 << static_cast<uint32_t>(registers[i]);
        if(!(reg_bm & curr_reg)){
            reg_bm |= curr_reg;
            bulkReadScope.push_back(registers[i]);
        }
    }

    std::vector<uint8_t> vec;
    for(auto reg : bulkReadScope)
        vec.push_back(static_cast<uint8_t>(reg));
    writeData(Register::SET_BULK_READ, vec);
}

void goBILDA::Pinpoint::recalibrateIMU(void) const
{
    uint32_t bit_mask = 1 << 0;
    std::vector<uint8_t> vec;
    loadVectorWithUint(vec, bit_mask);
    writeData(Register::DEVICE_CONTROL, vec);
}

void goBILDA::Pinpoint::resetPositionAndIMU(void) const
{
    uint32_t bit_mask = 1 << 1;
    std::vector<uint8_t> vec;
    loadVectorWithUint(vec, bit_mask);
    writeData(Register::DEVICE_CONTROL, vec);
}

void goBILDA::Pinpoint::setOffsets(float x, float y)
{
    std::vector<uint8_t> vec;

    loadVectorWithFloat(vec, x);
    writeData(Register::X_POD_OFFSET, vec);
    loadVectorWithFloat(vec, y);
    writeData(Register::Y_POD_OFFSET, vec);
}

void goBILDA::Pinpoint::setEncoderDirections(EncoderDirection x, EncoderDirection y)
{
    constexpr uint32_t X_FORWARD_BM  = 1 << 5;
    constexpr uint32_t X_BACKWARD_BM = 1 << 4;
    constexpr uint32_t Y_FORWARD_BM  = 1 << 3;
    constexpr uint32_t Y_BACKWARD_BM = 1 << 2;

    uint32_t control_reg_val = 0;
    control_reg_val |= (x == EncoderDirection::Forward ? X_FORWARD_BM : X_BACKWARD_BM);
    control_reg_val |= (y == EncoderDirection::Forward ? Y_FORWARD_BM : Y_BACKWARD_BM);

    std::vector<uint8_t> vec;
    loadVectorWithUint(vec, control_reg_val);
    writeData(Register::DEVICE_CONTROL, vec);
}

void goBILDA::Pinpoint::setEncoderResolution(EncoderResolution resolution)
{
    std::vector<uint8_t> vec;
    if(resolution == EncoderResolution::goBILDA_4_BAR_POD)
        loadVectorWithFloat(vec, 19.89436789f);
    else if(resolution == EncoderResolution::goBILDA_SWINGARM_POD)
        loadVectorWithFloat(vec, 13.26291192f);
    
    writeData(Register::MM_PER_TICK, vec);
}

void goBILDA::Pinpoint::setEncoderResolution(float ticks_per_mm)
{
    std::vector<uint8_t> vec;
    loadVectorWithFloat(vec, ticks_per_mm);
    writeData(Register::MM_PER_TICK, vec);
}

void goBILDA::Pinpoint::setYawScalar(float yawScalar)
{
    std::vector<uint8_t> vec;
    loadVectorWithFloat(vec, yawScalar);
    writeData(Register::YAW_SCALAR, vec);
}

void goBILDA::Pinpoint::setPosition(Pose2D pos)
{
    setPosX(pos.x);
    setPosY(pos.y);
    setHeading(pos.heading);
}

void goBILDA::Pinpoint::setPosX(float positionInMM)
{
    std::vector<uint8_t> vec;
    loadVectorWithFloat(vec, positionInMM);
    writeData(Register::X_POSITION, vec);
}

void goBILDA::Pinpoint::setPosY(float positionInMM)
{
    std::vector<uint8_t> vec;
    loadVectorWithFloat(vec, positionInMM);
    writeData(Register::Y_POSITION, vec);
}

void goBILDA::Pinpoint::setHeading(float angleInRadians)
{
    std::vector<uint8_t> vec;
    loadVectorWithFloat(vec, angleInRadians);
    writeData(Register::H_ORIENTATION, vec);
}

// #region Private Member Functions
std::vector<uint8_t> goBILDA::Pinpoint::getData(Register reg, uint8_t count)
{
    _lastError = PinpointError::None;

    i2c.beginTransmission(i2c_address);
    i2c.write(static_cast<uint8_t>(reg));
    i2c.endTransmission(true);

    std::vector<uint8_t> vec;
    uint8_t bytes_to_request = count + (hasUpdatedFirmware ? 1 : 0);
    uint8_t bytes_to_read = i2c.requestFrom(i2c_address, bytes_to_request, static_cast<uint8_t>(true));
    if(bytes_to_read == 0){
        _lastError = PinpointError::I2C_Error;
        for(int i = 0; i < count; i++)
            vec.push_back(0);
        return vec;
    }

    for(uint8_t i = 0; i < std::min(count, bytes_to_read); i++)
        vec.push_back(i2c.read());
    
    if(hasUpdatedFirmware && i2c.available()){
        uint8_t crc = i2c.read();
        if(CRC8_ComputeFast(vec.data(), count) != crc){
            _lastError = PinpointError::CRC_Failed;
            for(auto &byte : vec)
                byte = 0;
        }
    }
    
    return vec;
}

void goBILDA::Pinpoint::writeData(Register reg, std::vector<uint8_t> &data) const
{
    i2c.beginTransmission(i2c_address);
    i2c.write(static_cast<uint8_t>(reg));
    i2c.write(data.data(), data.size());
    i2c.endTransmission(true);
}

void goBILDA::Pinpoint::saveData(PinpointRegisters reg, std::vector<uint8_t> &data, BulkReadData &bulk_data)
{
    switch(reg){
    case PinpointRegisters::EncoderValueX:
        _lastEncoderX = bulk_data.EncoderX = convertVectorToint(data);
    break;
    case PinpointRegisters::EncoderValueY:
        _lastEncoderY = bulk_data.EncoderY = convertVectorToint(data);
    break;
    case PinpointRegisters::DeviceVersion:
        _lastDeviceVersion = bulk_data.DeviceVersion = convertVectorToUint(data);
    break;
    case PinpointRegisters::DeviceStatus:
        _lastDeviceStatus = convertVectorToUint(data);
        bulk_data.DeviceStatus = PinpointStatus::GetStatus(_lastDeviceStatus);
    break;
    case PinpointRegisters::Heading:
        _lastHeading = bulk_data.Position.heading = convertVectorToFloat(data);
    break;
    case PinpointRegisters::QuaternionW:
        _lastQuaternionW = bulk_data.quaternion.w = convertVectorToFloat(data);
    break;
    case PinpointRegisters::QuaternionX:
        _lastQuaternionX = bulk_data.quaternion.x = convertVectorToFloat(data);
    break;
    case PinpointRegisters::QuaternionY:
        _lastQuaternionY = bulk_data.quaternion.y = convertVectorToFloat(data);
    break;
    case PinpointRegisters::QuaternionZ:
        _lastQuaternionZ = bulk_data.quaternion.z = convertVectorToFloat(data);
    break;
    case PinpointRegisters::PodOffsetX:
        _lastOffsetX = bulk_data.OffsetX = convertVectorToFloat(data);
    break;
    case PinpointRegisters::PodOffsetY:
        _lastOffsetY = bulk_data.OffsetY = convertVectorToFloat(data);
    break;
    case PinpointRegisters::MmPerTick:
        _lastMmPerTick = bulk_data.MmPerTick = convertVectorToFloat(data);
    break;
    case PinpointRegisters::YawScalar:
        _lastYawScalar = bulk_data.YawScalar = convertVectorToFloat(data);
    break;
    case PinpointRegisters::PositionX:
        _lastPositionX = bulk_data.Position.x = convertVectorToFloat(data);
    break;
    case PinpointRegisters::PositionY:
        _lastPositionY = bulk_data.Position.y = convertVectorToFloat(data);
    break;
    case PinpointRegisters::VelocityX:
        _lastVelocityX = bulk_data.VelocityX = convertVectorToFloat(data);
    break;
    case PinpointRegisters::VelocityY:
        _lastVelocityY = bulk_data.VelocityY = convertVectorToFloat(data);
    break;
    case PinpointRegisters::VelocityH:
        _lastVelocityH = bulk_data.VelocityH = convertVectorToFloat(data);
    break;
    case PinpointRegisters::DeviceID:
        _lastDeviceId = bulk_data.DeviceId = convertVectorToUint(data);
    break;
    case PinpointRegisters::LoopTime:
        _lastLoopTime = bulk_data.LoopTime = convertVectorToUint(data);
    break;
    case PinpointRegisters::Pitch:
        _lastPitch = bulk_data.Pitch = convertVectorToFloat(data);
    break;
    case PinpointRegisters::Roll:
        _lastRoll = bulk_data.Roll = convertVectorToFloat(data);
    break;
    }
}

void goBILDA::Pinpoint::loadVectorWithFloat(std::vector<uint8_t> &vec, float value) const
{
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = value;
    vec.clear();
    vec.push_back(u.b[0]);
    vec.push_back(u.b[1]);
    vec.push_back(u.b[2]);
    vec.push_back(u.b[3]);
}

void goBILDA::Pinpoint::loadVectorWithUint(std::vector<uint8_t> &vec, uint32_t value) const
{
    vec.push_back((value >>  0) & 0xFF);
    vec.push_back((value >>  8) & 0xFF);
    vec.push_back((value >> 16) & 0xFF);
    vec.push_back((value >> 24) & 0xFF);
}

int32_t goBILDA::Pinpoint::convertVectorToint(std::vector<uint8_t> &vec) const
{
    int32_t returnVal = 0;
    returnVal |= vec[3] << 24;
    returnVal |= vec[2] << 16;
    returnVal |= vec[1] <<  8;
    returnVal |= vec[0] <<  0;
    return returnVal;
}

uint32_t goBILDA::Pinpoint::convertVectorToUint(std::vector<uint8_t> &vec) const
{
    uint32_t returnVal = 0;
    returnVal |= vec[3] << 24;
    returnVal |= vec[2] << 16;
    returnVal |= vec[1] <<  8;
    returnVal |= vec[0] <<  0;
    return returnVal;
}

float goBILDA::Pinpoint::convertVectorToFloat(std::vector<uint8_t> &vec) const
{
    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[3] = vec[3];
    u.b[2] = vec[2];
    u.b[1] = vec[1];
    u.b[0] = vec[0];
    return u.f;
}

bool goBILDA::Pinpoint::firmwareIsAbleToRead(Register reg) const
{
    switch(reg){
    case Register::DEVICE_ID:
    case Register::DEVICE_VERSION:
    case Register::DEVICE_STATUS:
    case Register::LOOP_TIME:
    case Register::X_ENCODER_VALUE:
    case Register::Y_ENCODER_VALUE:
    case Register::X_POSITION:
    case Register::Y_POSITION:
    case Register::H_ORIENTATION:
    case Register::X_VELOCITY:
    case Register::Y_VELOCITY:
    case Register::H_VELOCITY:
    case Register::MM_PER_TICK:
    case Register::X_POD_OFFSET:
    case Register::Y_POD_OFFSET:
    case Register::YAW_SCALAR:
        return true;

    default:
    case Register::QUATERNION_W:
    case Register::QUATERNION_X:
    case Register::QUATERNION_Y:
    case Register::QUATERNION_Z:
    case Register::PITCH:
    case Register::ROLL:
        return hasUpdatedFirmware;
    }
}
// #endregion Private Member Functions