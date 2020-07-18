--[[
 MPU9250 Lua module
 Written by Lukas Voborsky, @voborsky
]]

local mpu9250 = {}

local string_char = string.char
local i2c_start, i2c_stop, i2c_address, i2c_read, i2c_write, i2c_TRANSMITTER, i2c_RECEIVER =
  i2c.start, i2c.stop, i2c.address, i2c.read, i2c.write, i2c.TRANSMITTER, i2c.RECEIVER
-- local tmr_create, tmr_ALARM_SINGLE, tmr_ALARM_AUTO, tmr_now, delay =
  -- tmr.create, tmr.ALARM_SINGLE, tmr.ALARM_AUTO, tmr.now, tmr.delay
local tmr_create, tmr_ALARM_SINGLE, tmr_ALARM_AUTO, tmr_now =
  tmr.create, tmr.ALARM_SINGLE, tmr.ALARM_AUTO, tmr.now
local pi, asin, atan, sqrt = math.pi, math.asin, math.atan, math.sqrt

local MPU9250_ADDRESS = 0x68 -- MPU9250 address when ADO = 1
local AK8963_ADDRESS = 0x0C -- Address of AK8963 (MPU9250) magnetometer

-- local  MPU9250Gscale = 0 --GFS_250DPS
-- enum MPU9250Gscale {
  -- GFS_250DPS = 0,
  -- GFS_500DPS,
  -- GFS_1000DPS,
  -- GFS_2000DPS
-- };
-- local MPU9250Ascale = 0 --AFS_2G
-- enum MPU9250Ascale {
  -- AFS_2G = 0,
  -- AFS_4G,
  -- AFS_8G,
  -- AFS_16G
-- };
-- local MPU9250Mscale = 0 -- MFS_14BITS
-- enum MPU9250Mscale {
  -- MFS_14BITS = 0, // 0.6 mG per LSB
  -- MFS_16BITS      // 0.15 mG per LSB
-- };



-- See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00,
-- Rev. 1.4, 9/9/2013 for registers not listed in

-- above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
--
-- Magnetometer Registers
local WHO_AM_I_AK8963 = 0x00 -- should return 0x48
-- local INFO = 0x01
local AK8963_ST1 = 0x02  -- data ready status bit 0
local AK8963_XOUT_L = 0x03  -- data
-- local AK8963_XOUT_H = 0x04
-- local AK8963_YOUT_L = 0x05
-- local AK8963_YOUT_H = 0x06
-- local AK8963_ZOUT_L = 0x07
-- local AK8963_ZOUT_H = 0x08
-- local AK8963_ST2 = 0x09  -- Data overflow bit 3 and data read error status bit 2
    -- Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
local AK8963_CNTL = 0x0A
-- local AK8963_ASTC = 0x0C  -- Self test control
-- local AK8963_I2CDIS = 0x0F  -- I2C disable
local AK8963_ASAX = 0x10  -- Fuse ROM x-axis sensitivity adjustment value
-- local AK8963_ASAY = 0x11  -- Fuse ROM y-axis sensitivity adjustment value
-- local AK8963_ASAZ = 0x12  -- Fuse ROM z-axis sensitivity adjustment value$

local MPU9250_SELF_TEST_X_GYRO = 0x00
-- local MPU9250_SELF_TEST_Y_GYRO = 0x01
-- local MPU9250_SELF_TEST_Z_GYRO = 0x02

local MPU9250_SELF_TEST_X_ACCEL = 0x0D
-- local MPU9250_SELF_TEST_Y_ACCEL = 0x0E
-- local MPU9250_SELF_TEST_Z_ACCEL = 0x0F

-- local MPU9250_SELF_TEST_A = 0x10

local MPU9250_XG_OFFSET_H = 0x13  -- User-defined trim values for gyroscope
-- local MPU9250_XG_OFFSET_L = 0x14
-- local MPU9250_YG_OFFSET_H = 0x15
-- local MPU9250_YG_OFFSET_L = 0x16
-- local MPU9250_ZG_OFFSET_H = 0x17
-- local MPU9250_ZG_OFFSET_L = 0x18
local MPU9250_SMPLRT_DIV = 0x19
local MPU9250_CONFIG = 0x1A
local MPU9250_GYRO_CONFIG = 0x1B
local MPU9250_ACCEL_CONFIG = 0x1C
local MPU9250_ACCEL_CONFIG2 = 0x1D
-- local MPU9250_LP_ACCEL_ODR = 0x1E
-- local MPU9250_WOM_THR = 0x1F

-- local MPU9250_MOT_DUR = 0x20  -- Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
-- local MPU9250_ZMOT_THR = 0x21  -- Zero-motion detection threshold bits [7:0]
    -- Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
-- local MPU9250_ZRMOT_DUR = 0x22


local MPU9250_FIFO_EN = 0x23
local MPU9250_I2C_MST_CTRL = 0x24
-- local MPU9250_I2C_SLV0_ADDR = 0x25
-- local MPU9250_I2C_SLV0_REG = 0x26
-- local MPU9250_I2C_SLV0_CTRL = 0x27
-- local MPU9250_I2C_SLV1_ADDR = 0x28
-- local MPU9250_I2C_SLV1_REG = 0x29
-- local MPU9250_I2C_SLV1_CTRL = 0x2A
-- local MPU9250_I2C_SLV2_ADDR = 0x2B
-- local MPU9250_I2C_SLV2_REG = 0x2C
-- local MPU9250_I2C_SLV2_CTRL = 0x2D
-- local MPU9250_I2C_SLV3_ADDR = 0x2E
-- local MPU9250_I2C_SLV3_REG = 0x2F
-- local MPU9250_I2C_SLV3_CTRL = 0x30
-- local MPU9250_I2C_SLV4_ADDR = 0x31
-- local MPU9250_I2C_SLV4_REG = 0x32
-- local MPU9250_I2C_SLV4_DO = 0x33
-- local MPU9250_I2C_SLV4_CTRL = 0x34
-- local MPU9250_I2C_SLV4_DI = 0x35
-- local MPU9250_I2C_MST_STATUS = 0x36
local MPU9250_INT_PIN_CFG = 0x37
local MPU9250_INT_ENABLE = 0x38
-- local MPU9250_DMP_INT_STATUS = 0x39  -- Check DMP interrupt
-- local MPU9250_INT_STATUS = 0x3A
local MPU9250_ACCEL_XOUT_H = 0x3B
-- local MPU9250_ACCEL_XOUT_L = 0x3C
-- local MPU9250_ACCEL_YOUT_H = 0x3D
-- local MPU9250_ACCEL_YOUT_L = 0x3E
-- local MPU9250_ACCEL_ZOUT_H = 0x3F
-- local MPU9250_ACCEL_ZOUT_L = 0x40
-- local MPU9250_TEMP_OUT_H = 0x41
-- local MPU9250_TEMP_OUT_L = 0x42
local MPU9250_GYRO_XOUT_H = 0x43
-- local MPU9250_GYRO_XOUT_L = 0x44
-- local MPU9250_GYRO_YOUT_H = 0x45
-- local MPU9250_GYRO_YOUT_L = 0x46
-- local MPU9250_GYRO_ZOUT_H = 0x47
-- local MPU9250_GYRO_ZOUT_L = 0x48
-- local MPU9250_EXT_SENS_DATA_00 = 0x49
-- local MPU9250_EXT_SENS_DATA_01 = 0x4A
-- local MPU9250_EXT_SENS_DATA_02 = 0x4B
-- local MPU9250_EXT_SENS_DATA_03 = 0x4C
-- local MPU9250_EXT_SENS_DATA_04 = 0x4D
-- local MPU9250_EXT_SENS_DATA_05 = 0x4E
-- local MPU9250_EXT_SENS_DATA_06 = 0x4F
-- local MPU9250_EXT_SENS_DATA_07 = 0x50
-- local MPU9250_EXT_SENS_DATA_08 = 0x51
-- local MPU9250_EXT_SENS_DATA_09 = 0x52
-- local MPU9250_EXT_SENS_DATA_10 = 0x53
-- local MPU9250_EXT_SENS_DATA_11 = 0x54
-- local MPU9250_EXT_SENS_DATA_12 = 0x55
-- local MPU9250_EXT_SENS_DATA_13 = 0x56
-- local MPU9250_EXT_SENS_DATA_14 = 0x57
-- local MPU9250_EXT_SENS_DATA_15 = 0x58
-- local MPU9250_EXT_SENS_DATA_16 = 0x59
-- local MPU9250_EXT_SENS_DATA_17 = 0x5A
-- local MPU9250_EXT_SENS_DATA_18 = 0x5B
-- local MPU9250_EXT_SENS_DATA_19 = 0x5C
-- local MPU9250_EXT_SENS_DATA_20 = 0x5D
-- local MPU9250_EXT_SENS_DATA_21 = 0x5E
-- local MPU9250_EXT_SENS_DATA_22 = 0x5F
-- local MPU9250_EXT_SENS_DATA_23 = 0x60
-- local MPU9250_MOT_DETECT_STATUS = 0x61
-- local MPU9250_I2C_SLV0_DO = 0x63
-- local MPU9250_I2C_SLV1_DO = 0x64
-- local MPU9250_I2C_SLV2_DO = 0x65
-- local MPU9250_I2C_SLV3_DO = 0x66
-- local MPU9250_I2C_MST_DELAY_CTRL = 0x67
-- local MPU9250_SIGNAL_PATH_RESET = 0x68
-- local MPU9250_MOT_DETECT_CTRL = 0x69
local MPU9250_USER_CTRL = 0x6A  -- Bit 7 enable DMP, bit 3 reset DMP
local MPU9250_PWR_MGMT_1 = 0x6B -- Device defaults to the SLEEP mode
local MPU9250_PWR_MGMT_2 = 0x6C
-- local MPU9250_DMP_BANK = 0x6D  -- Activates a specific bank in the DMP
-- local MPU9250_DMP_RW_PNT = 0x6E  -- Set read/write pointer to a specific start address in specified DMP bank
-- local MPU9250_DMP_REG = 0x6F  -- Register in DMP from which to read or to which to write
-- local MPU9250_DMP_REG_1 = 0x70
-- local MPU9250_DMP_REG_2 = 0x71
local MPU9250_FIFO_COUNTH = 0x72
-- local MPU9250_FIFO_COUNTL = 0x73
local MPU9250_FIFO_R_W = 0x74
local MPU9250_WHO_AM_I = 0x75 -- Should return 0x71
local MPU9250_XA_OFFSET_H = 0x77
-- local MPU9250_XA_OFFSET_L = 0x78
-- local MPU9250_YA_OFFSET_H = 0x7A
-- local MPU9250_YA_OFFSET_L = 0x7B
-- local MPU9250_ZA_OFFSET_H = 0x7D
-- local MPU9250_ZA_OFFSET_L = 0x7E

-- Local functions
local mpu9250_setup
local mpu9250_read
local mpu9250_readMagData
local mpu9250_selftest
local mpu9250_accelgyrocal
local mpu9250_magcal
local MadgwickQuaternionUpdate
local read_reg
local write_reg
local S16BE, S16LE

-- -- Note that the space between debug and the arglist is there for a reason
-- -- so that a simple global edit "   -- debug(" -> "-- debug(" or v.v. to
-- -- toggle debug compiled into the module.
local print, node_heap = print, node.heap
local function debug (fmt, ...) -- upval: cnt (, print, node_heap)
  -- if not mpu9250.debug then return end
  if (...) then fmt = fmt:format(...) end
  print("[mpu9250]", node_heap(), fmt)
end

local delay = function(w)
  debug("delay: %dus", w)
  tmr.delay(w)
end



--------------------------- Set up the mpu9250 object ----------------------------
--       mpu9250 has method setup to create the sensor object and setup the sensor
--       object created by mpu9250.setup() has methods: read, qfe2qnh, altitude, dewpoint
---------------------------------------------------------------------------------

function mpu9250.setup(id, callback, MPU9250Ascale, MPU9250Gscale, MPU9250Mscale)
  return mpu9250_setup(nil, id, callback, MPU9250Ascale, MPU9250Gscale, MPU9250Mscale)
end

------------------------------------------------------------------------------
local sum, sumCount = 0, 0

local q_init = {1, 0, 0, 0}

function mpu9250_read(self, q)
  q = q or q_init
  local id = self.id
  local data

  data = read_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 14)
  if not data then return end

  -- debug("registers: %s", ("%02x "):rep(#data):format(data:byte(1,#data)))
  local MPU9250aRes, MPU9250gRes = self.MPU9250aRes, self.MPU9250gRes
  local ax = S16BE(data:byte(1, 2))*MPU9250aRes - self.MPU9250accelBias[1]
  local ay = S16BE(data:byte(3, 4))*MPU9250aRes - self.MPU9250accelBias[2]
  local az = S16BE(data:byte(5, 6))*MPU9250aRes - self.MPU9250accelBias[3]
  local gx = S16BE(data:byte(9, 10))*MPU9250gRes
  local gy = S16BE(data:byte(11, 12))*MPU9250gRes
  local gz = S16BE(data:byte(13, 14))*MPU9250gRes
  local readout = {
    ax = ax, ay = ay, az = az,
    gx = gx, gy = gy, gz = gz,
    temp = S16BE(data:byte(7, 8))/340+36.53,
  }

  -- MPU9250readMagData
  local mx, my, mz = mpu9250_readMagData(self)
  readout.mx, readout.my, readout.mz = mx, my, mz

  local now = tmr_now()
  local deltat = (now - self.lastUpdate)/1000000.0
  self.lastUpdate = now

  sum = sum + deltat -- sum for averaging filter update rate
  sumCount = sumCount + 1

  -- Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer
  -- the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  -- We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  -- For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East.
  -- So if we want te quaternions properly aligned we need to feed into the Madgwick function
  -- Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention positive down,
  -- we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
  -- function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
  -- This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  -- Pass gyro rate as rad/s

  q = MadgwickQuaternionUpdate(q, -ax, ay, az, gx*pi/180.0, -gy*pi/180.0, -gz*pi/180.0,  my,  -mx, mz, deltat) or q_init
  -- local q = MahonyQuaternionUpdate(-ax, ay, az, gx*pi/180.0, -gy*pi/180.0, -gz*pi/180.0,  my,  -mx, mz)
  readout.q = q

  -- Define output variables from updated quaternion -
  -- these are Tait-Bryan angles, commonly used in aircraft orientation.
  -- In this coordinate system, the positive z-axis is down toward Earth.
  -- Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination,
  -- looking down on the sensor positive yaw is counterclockwise. Pitch is angle between sensor x-axis and Earth
  -- ground plane, toward the Earth is positive, up toward the sky is negative.
  -- Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  -- These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  -- Tait-Bryan angles as well as Euler angles are non-commutative that is, the get the correct orientation
  -- the rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.
  -- For more see http:--en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has
  -- additional links.
  -- Software AHRS:
   --   yaw   = atan2f(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
   --   pitch = -asinf(2.0 * (q[1] * q[3] - q[0] * q[2]))
   --   roll  = atan2f(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
   --   pitch *= 180.0 / PI
   --   yaw   *= 180.0 / PI
   --   yaw   += 13.8f -- Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
   --   if(yaw < 0) yaw   += 360.0 -- Ensure yaw stays between 0 and 360
   --   roll  *= 180.0 / PI
  local q1, q2, q3, q4 = q[1], q[2], q[3], q[4] --unpack(q)
  local a12 =   2.0 * (q2 * q3 + q1 * q4)
  local a22 =   q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4
  local a31 =   2.0 * (q1 * q2 + q3 * q4)
  local a32 =   2.0 * (q2 * q4 - q1 * q3)
  local a33 =   q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4
  local pitch, roll, yaw
  pitch = asin(a32)
  roll = atan(a31 / a33)
  yaw = atan(a12 / a22)
  pitch = pitch * 180.0 / pi
  yaw = yaw * 180.0 / pi
  yaw = yaw --+ 13.8 -- Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  if yaw < 0 then yaw = yaw + 360.0 end -- Ensure yaw stays between 0 and 360
  roll = roll * 180.0 / pi

  readout.pitch, readout.roll, readout.yaw = pitch, roll, yaw

  readout.grav_x, readout.grav_y, readout.grav_z = -a31*1000, -a32*1000, a33*1000
  -- local lin_ax, lin_ay, lin_az
  -- lin_ax = ax + a31
  -- lin_ay = ay + a32
  -- lin_az = az - a33

  return readout
end

function mpu9250_readMagData(self)
  local mx, my, mz = 0, 0, 0
  local id = self.id
  local c = read_reg(id, AK8963_ADDRESS, AK8963_ST1)
  if  c and c:byte(1) & 0x01 ~= 0 then
    -- Read the six raw data and ST2 registers sequentially into data array
    local data = read_reg(id, AK8963_ADDRESS, AK8963_XOUT_L, 7)
    c = data:byte(7) -- End data read by reading ST2 register
    if (c & 0x08) == 0 then -- Check if magnetic sensor overflow set, if not then report data
      local MPU9250mRes = self.MPU9250mRes
      -- -- Calculate the magnetometer values in milliGauss
      -- -- Include factory calibration per data sheet and user environmental corrections
      -- -- get actual magnetometer value, this depends on scale being set
      mx = S16LE(data:byte(1, 2))*MPU9250mRes*self.magCalibration[1] - self.MPU9250magBias[1]
      my = S16LE(data:byte(3, 4))*MPU9250mRes*self.magCalibration[2] - self.MPU9250magBias[2]
      mz = S16LE(data:byte(5, 6))*MPU9250mRes*self.magCalibration[3] - self.MPU9250magBias[3]
    end
  end
  return mx, my, mz
end

function mpu9250_setup(self, id, callback, MPU9250Ascale, MPU9250Gscale, MPU9250Mscale)
  MPU9250Ascale, MPU9250Gscale, MPU9250Mscale = MPU9250Ascale or 0, MPU9250Gscale or 0, MPU9250Mscale or 0

  self = self  or {
    setup = mpu9250_setup,
    read = mpu9250_read,
    -- selftest = mpu9250_selftest,
    -- accelgyrocal = mpu9250_accelgyrocal,
    lastUpdate = 0,
  }
  self.id = id

  -- Possible accelerometer scales (and their register bit settings) are:
  -- 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  -- Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  if MPU9250Ascale == 0 then -- AFS_2G
    self.MPU9250aRes = 2.0/32768.0
  elseif MPU9250Ascale == 1 then -- AFS_4G
    self.MPU9250aRes = 4.0/32768.0
  elseif MPU9250Ascale == 2 then --AFS_8G
    self.MPU9250aRes = 8.0/32768.0
  elseif MPU9250Ascale == 3 then -- AFS_16G
    self.MPU9250aRes = 16.0/32768.0
  end

  -- Possible gyro scales (and their register bit settings) are:
  -- 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
  -- Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  if MPU9250Gscale == 0 then -- GFS_250DPS
    self.MPU9250gRes = 250.0/32768.0
  elseif MPU9250Gscale == 1 then -- GFS_500DPS
    self.MPU9250gRes = 500.0/32768.0
  elseif MPU9250Gscale == 2 then -- GFS_1000DPS
    self.MPU9250gRes = 1000.0/32768.0
  elseif MPU9250Gscale == 3 then -- GFS_2000DPS
    self.MPU9250gRes = 2000.0/32768.0
  end

  -- Possible magnetometer scales (and their register bit settings) are:
  -- 14 bit resolution (0) and 16 bit resolution (1)
  if MPU9250Mscale == 0 then -- MFS_14BITS
    self.MPU9250mRes = 10.*4912./8190. -- Proper scale to return milliGauss
  elseif  MPU9250Mscale == 1 then -- MFS_16BITS
    self.MPU9250mRes = 10.*4912./32760.0 -- Proper scale to return milliGauss
  end

  local c
  -- Read WHO_AM_I register for MPU-9250
  c = read_reg(id, MPU9250_ADDRESS, MPU9250_WHO_AM_I)
  if not c or  c:byte(1)  ~= 0x71 then
    debug("No MPU9250 detected.")
    return nil
  end
  debug("MPU9250 detected")

  local selftest
  selftest=mpu9250_selftest(self)
  -- debug("MPU9250 Self Test:")
  -- debug("x-axis self test: acceleration trim within %.1f%% of factory value", selftest[1])
  -- debug("y-axis self test: acceleration trim within %.1f%% of factory value", selftest[2])
  -- debug("z-axis self test: acceleration trim within %.1f%% of factory value", selftest[3])
  -- debug("x-axis self test: gyration trim within %.1f%% of factory value", selftest[4])
  -- debug("y-axis self test: gyration trim within %.1f%% of factory value", selftest[5])
  -- debug("z-axis self test: gyration trim within %.1f%% of factory value", selftest[6])

  debug("Calibrate MPU9250 gyro and accel")

  mpu9250_accelgyrocal(self, function (MPU9250gyroBias, MPU9250accelBias)
    self.MPU9250gyroBias, self.MPU9250accelBias = MPU9250gyroBias, MPU9250accelBias
    debug("accel biases (mg): x=%.2f, y=%.2f, z=%.2f",
      1000.*self.MPU9250accelBias[1], 1000.*self.MPU9250accelBias[2], 1000.*self.MPU9250accelBias[3])
    debug("gyro biases (dps): x=%.2f, y=%.2f, z=%.2f",
      self.MPU9250gyroBias[1], self.MPU9250gyroBias[2], self.MPU9250gyroBias[3])

    -- wake up device
    write_reg(id, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00) -- Clear sleep mode bit (6), enable all sensors
    -- delay(100000) -- Wait for all registers to reset
    tmr_create():alarm(100, tmr_ALARM_SINGLE, function()
      -- get stable time source
      -- Auto select clock source to be PLL gyroscope reference if ready else
      write_reg(id, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01)

      -- delay(200000)
      tmr_create():alarm(200, tmr_ALARM_SINGLE, function()
        -- Configure Gyro and Thermometer
        -- Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively
        -- minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        -- be higher than 1 / 0.0059 = 170 Hz
        -- DLPF_CFG = bits 2:0 = 011 this limits the sample rate to 1000 Hz for both
        -- With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        write_reg(id, MPU9250_ADDRESS, MPU9250_CONFIG, 0x03)

        -- Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        -- Use a 200 Hz rate a rate consistent with the filter update rate determined inset in CONFIG above
        write_reg(id, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04)

        -- Set gyroscope full scale range
        -- Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        c = read_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG):byte(1)
        --  writeRegister(GYRO_CONFIG, c & ~0xE0) -- Clear self-test bits [7:5]
        write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x03) -- Clear Fchoice bits [1:0]
        write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x18) -- Clear GFS bits [4:3]
        -- Set full scale range for the gyro
        write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c | MPU9250Gscale << 3)
        -- Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
        -- writeRegister(GYRO_CONFIG, c | 0x00)

        -- Set accelerometer full-scale range configuration
        c = read_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG):byte(1)
        --  writeRegister(ACCEL_CONFIG, c & ~0xE0) -- Clear self-test bits [7:5]
        write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c & ~0x18) -- Clear AFS bits [4:3]
        -- Set full scale range for the accelerometer
        write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c | MPU9250Ascale << 3)
        -- Set accelerometer sample rate configuration
        -- It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        -- accel_fchoice_b bit [3] in this case the bandwidth is 1.13 kHz
        c = read_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2):byte(1)
        -- Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c & ~0x0F)
        -- Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c | 0x03)

        -- The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        -- but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        -- Configure Interrupts and Bypass Enable
        -- Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        -- clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        -- can join the I2C bus and all can be controlled by the Arduino as master
        write_reg(id, MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x22)
        -- Enable data ready (bit 0) interrupt
        write_reg(id, MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01)
        -- delay(100000)

        debug("MPU9250 initialized for active data mode....")

        c = read_reg(id, AK8963_ADDRESS, WHO_AM_I_AK8963)  -- Read WHO_AM_I register for AK8963
        if not c or c:byte(1)~=0x48 then
          debug("No AK8963 detected")
          return nil
        end
        debug("AK8963 detected")

        -- First extract the factory calibration for each magnetometer axis
        write_reg(id, AK8963_ADDRESS, AK8963_CNTL, 0x00) -- Power down magnetometer
        delay(10000)
        write_reg(id, AK8963_ADDRESS, AK8963_CNTL, 0x0F) -- Enter Fuse ROM access mode
        delay(10000)
        local data = read_reg(id, AK8963_ADDRESS, AK8963_ASAX, 3)  -- Read the x-, y-, and z-axis calibration values
        data={data:byte(1,3)}
        -- x, y, z-axis sensitivity adjustment values, etc.
        self.magCalibration = {
          (data[1] - 128)/256. + 1.,
          (data[2] - 128)/256. + 1.,
          (data[3] - 128)/256. + 1.,
        }
        write_reg(id, AK8963_ADDRESS, AK8963_CNTL, 0x00) -- Power down magnetometer
        delay(10000)
        -- Configure the magnetometer for continuous read and highest resolution
        -- set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register, and enable continuous mode data
        -- acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        -- 2 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
        -- Set magnetometer data resolution and sample ODR
        write_reg(id, AK8963_ADDRESS, AK8963_CNTL, MPU9250Mscale << 4 | 0x06)
        delay(10000)
        debug("AK8963 initialized for active data mode....") -- Initialize device for active mode read of magnetometer

        mpu9250_magcal(self, function()
          if callback then return callback(selftest) end
        end)
      end)
    end)
  end)


  return self
end


-- Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
-- of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
function mpu9250_accelgyrocal(self, callback)
  local data
  local dest1, dest2 = {}, {}
  local id = self.id
  -- uint8_t data[12] -- data array to hold accelerometer and gyro x, y, z, data
  -- uint16_t ii, packet_count, fifo_count
  -- int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0}

  -- reset device
  -- Write a one to bit 7 reset bit toggle reset device
  write_reg(id, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80)

  tmr_create():alarm(100, tmr_ALARM_SINGLE, function()
    -- get stable time source Auto select clock source to be PLL gyroscope reference if ready
    -- else use the internal oscillator, bits 2:0 = 001
    write_reg(id, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01)

    write_reg(id, MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00)

    tmr_create():alarm(200, tmr_ALARM_SINGLE, function()
      -- Configure device for bias calculation
      write_reg(id, MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00)   -- Disable all interrupts
      write_reg(id, MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00)      -- Disable FIFO
      write_reg(id, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00)   -- Turn on internal clock source
      write_reg(id, MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00) -- Disable I2C master
      write_reg(id, MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00)    -- Disable FIFO and I2C master modes
      write_reg(id, MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C)    -- Reset FIFO and DMP
      delay(15000)

      -- Configure MPU6050 gyro and accelerometer for bias calculation
      -- Set low-pass filter to 188 Hz
      write_reg(id, MPU9250_ADDRESS, MPU9250_CONFIG, 0x01)
      -- Set sample rate to 1 kHz
      write_reg(id, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00)
      -- Set gyro full-scale to 250 degrees per second, maximum sensitivity
      write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00)
      -- Set accelerometer full-scale to 2 g, maximum sensitivity
      write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00)

      -- Configure FIFO to capture accelerometer and gyro data for bias calculation
      write_reg(id, MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40)   -- Enable FIFO

      -- Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
      write_reg(id, MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78)
      delay(40000) -- accumulate 40 samples in 40 milliseconds = 480 bytes

      -- At end of sample accumulation, turn off FIFO sensor read
      write_reg(id, MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00) -- Disable gyro and accelerometer sensors for FIFO
      data = read_reg(id, MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 2) -- read FIFO sample count
      local fifo_count = data:byte(1) << 8 | data:byte(2)
      local packet_count = fifo_count/12  -- How many sets of full gyro and accelerometer data for averaging

      local accel_bias, gyro_bias = {0, 0, 0}, {0, 0, 0}
      for _ = 1, packet_count do
        data = read_reg(id, MPU9250_ADDRESS, MPU9250_FIFO_R_W, 12) -- read data for averaging
        -- Form signed 16-bit integer for each sample in FIFO
        -- Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] = accel_bias[1] + S16BE(data:byte(1, 2))
        accel_bias[2] = accel_bias[2] + S16BE(data:byte(3, 4))
        accel_bias[3] = accel_bias[3] + S16BE(data:byte(5, 6))
        gyro_bias[1] = gyro_bias[1] + S16BE(data:byte(7, 8))
        gyro_bias[2] = gyro_bias[2] + S16BE(data:byte(9, 10))
        gyro_bias[3] = gyro_bias[3] + S16BE(data:byte(11, 12))
      end
      for i=1, 3 do
        -- Normalize sums to get average count biases
        accel_bias[i] = accel_bias[i] / packet_count
        gyro_bias[i] = gyro_bias[i] / packet_count
      end

      -- Remove gravity from the z-axis accelerometer bias calculation
      local gyrosensitivity  = 131   -- = 131 LSB/degrees/sec
      local accelsensitivity = 16384  -- = 16384 LSB/g
      if accel_bias[3] > 0 then
        accel_bias[3] = accel_bias[3] - accelsensitivity
      else
        accel_bias[3] = accel_bias[3] + accelsensitivity
      end

      -- Construct the gyro biases for push to the hardware gyro bias registers,
      -- which are reset to zero upon device startup
      data = {data:byte(1, 6)}
      -- Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
      data[1] = (-gyro_bias[1]//4  >> 8) & 0xFF
      -- Biases are additive, so change sign on calculated average gyro biases
      data[2] = (-gyro_bias[2]//4)       & 0xFF
      data[3] = (-gyro_bias[3]//4  >> 8) & 0xFF
      data[4] = (-gyro_bias[1]//4)       & 0xFF
      data[5] = (-gyro_bias[2]//4  >> 8) & 0xFF
      data[6] = (-gyro_bias[3]//4)       & 0xFF

      -- Push gyro biases to hardware registers
      -- write_reg(id, MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[1])
      -- write_reg(id, MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[2])
      -- write_reg(id, MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[3])
      -- write_reg(id, MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[4])
      -- write_reg(id, MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[5])
      -- write_reg(id, MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[6])
      debug("writing MPU9250_XG_OFFSET_H: %02x %02x %02x %02x %02x %02x", unpack(data))
      data = string_char(unpack(data))
      write_reg(id, MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data)

      -- Output scaled gyro biases for display in the main program
      dest1[1] = gyro_bias[1]/gyrosensitivity
      dest1[2] = gyro_bias[2]/gyrosensitivity
      dest1[3] = gyro_bias[3]/gyrosensitivity

      -- Construct the accelerometer biases for push to the hardware accelerometer bias registers.
      -- These registers contain factory trim values which must be added to the calculated accelerometer
      -- biases on boot up these registers will hold non-zero values. In addition, bit 0 of the lower byte must
      -- be preserved since it is used for temperature compensation calculations. Accelerometer bias registers
      -- expect bias input as 2048 LSB per g, so that the accelerometer biases calculated above must be divided by 8.
      local accel_bias_reg = {0, 0, 0} -- A place to hold the factory accelerometer trim biases
      data = read_reg(id, MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 9) -- Read factory accelerometer trim values
      accel_bias_reg[1] = S16BE(data:byte(1, 2))
      accel_bias_reg[2] = S16BE(data:byte(4, 5))
      accel_bias_reg[3] = S16BE(data:byte(7, 8))

      -- Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
      -- local mask = 0x01
      -- local mask_bit = {0, 0, 0} -- Define array to hold mask bit for each accelerometer bias axis

      -- for ii=1, 3 do
        -- -- If temperature compensation bit is set, record that fact in mask_bit
        -- if  accel_bias_reg[ii] & mask ~= 0 then mask_bit[ii] = 0x01 end
      -- end

      -- Construct total accelerometer bias, including calculated average accelerometer bias from above
      for i=1, 3 do
        -- Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        accel_bias_reg[i] = accel_bias_reg[i] - accel_bias[1]/8
        -- Output scaled accelerometer biases for display in the main program
        dest2[i] = accel_bias[i]/accelsensitivity
      end

      -- data = {}
      -- data[1] = (accel_bias_reg[1] >> 8) & 0xFF
      -- -- preserve temperature compensation bit when writing back to accelerometer bias registers
      -- data[2] = ((accel_bias_reg[1])      & 0xFF) | mask_bit[1]
      -- data[3] = (accel_bias_reg[2] >> 8) & 0xFF
      -- -- preserve temperature compensation bit when writing back to accelerometer bias registers
      -- data[4] = ((accel_bias_reg[2])      & 0xFF) | mask_bit[2]
      -- data[5] = (accel_bias_reg[3] >> 8) & 0xFF
      -- -- preserve temperature compensation bit when writing back to accelerometer bias registers
      -- data[6] = ((accel_bias_reg[3])      & 0xFF) | mask_bit[3]

      -- -- Apparently this is not working for the acceleration biases in the MPU-9250
      -- -- Are we handling the temperature correction bit properly?
      -- -- Push accelerometer biases to hardware registers
      -- data = struct.pack("BBBBBB", unpack(data))
      -- write_reg(id, MPU9250_ADDRESS, XA_OFFSET_H, data)
      -- -- write_reg(id, MPU9250_ADDRESS, XA_OFFSET_L, data[1])
      -- -- write_reg(id, MPU9250_ADDRESS, YA_OFFSET_H, data[2])
      -- -- write_reg(id, MPU9250_ADDRESS, YA_OFFSET_L, data[3])
      -- -- write_reg(id, MPU9250_ADDRESS, ZA_OFFSET_H, data[4])
      -- -- write_reg(id, MPU9250_ADDRESS, ZA_OFFSET_L, data[5])
      if callback then return callback(dest1, dest2) end
    end)
  end)
end

function mpu9250_magcal(self, callback)
  local ii = 0
  local mag_max = {-32767, -32767, -32767}
  local mag_min = {32767, 32767, 32767}

  debug("Mag Calibration: Wave device in a figure eight until done!")
  self.MPU9250magBias = {0, 0, 0}

  local tmr_calib = tmr_create()
  tmr_calib:alarm(135, tmr_ALARM_AUTO, function()
    local mag_temp = {mpu9250_readMagData(self)}
    for jj=1, 3 do
      if mag_temp[jj] > mag_max[jj] then mag_max[jj] = mag_temp[jj] end
      if mag_temp[jj] < mag_min[jj] then mag_min[jj] = mag_temp[jj] end
    end
    ii = ii + 1
    if ii>64 then
      tmr_calib:stop()
      tmr_calib = nil
      debug("mag x min/max: %.0f/%.0f", mag_max[1], mag_min[1])
      debug("mag y min/max: %.0f/%.0f", mag_max[2], mag_min[2])
      debug("mag z min/max: %.0f/%.0f", mag_max[3], mag_min[3])

      local mag_bias = {
        (mag_max[1] + mag_min[1])/2,  -- get average x mag bias in counts
        (mag_max[2] + mag_min[2])/2,  -- get average y mag bias in counts
        (mag_max[3] + mag_min[3])/2,  -- get average z mag bias in counts
      }

      -- save mag biases in G for main program
      self.MPU9250magBias = {
        mag_bias[1]*self.MPU9250mRes*self.magCalibration[1],
        mag_bias[2]*self.MPU9250mRes*self.magCalibration[2],
        mag_bias[3]*self.MPU9250mRes*self.magCalibration[3],
      }

      debug("Mag Calibration done!")
      debug("AK8963 mag biases (mG): %.0f, %.0f, %0.f",
        self.MPU9250magBias[1], self.MPU9250magBias[2], self.MPU9250magBias[3])

       if callback then return callback() end
    end
  end)
end

function mpu9250_selftest(self)
  local rawData
  local FS = 0

  local id = self.id

  write_reg(id, MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00)    -- Set gyro sample rate to 1 kHz
  write_reg(id, MPU9250_ADDRESS, MPU9250_CONFIG, 0x02) -- Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  -- Set full scale range for the gyro to 250 dps
  write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1<<FS)
  -- Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02)
  -- Set full scale range for the accelerometer to 2 g
  write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1<<FS)

  local aAvg, gAvg = {0, 0, 0}, {0, 0, 0}
  for _ = 1, 200 do -- get average current values of gyro and acclerometer
      --  Read the six raw data registers into data array
      rawData = read_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6)
      aAvg[1] = aAvg[1] + S16BE(rawData:byte(1, 2))
      aAvg[2] = aAvg[2] + S16BE(rawData:byte(3, 4))
      aAvg[3] = aAvg[3] + S16BE(rawData:byte(5, 6))

      -- Read the six raw data registers sequentially into data array
      rawData = read_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6)
      gAvg[1] = gAvg[1] + S16BE(rawData:byte(1, 2))
      gAvg[2] = gAvg[2] + S16BE(rawData:byte(3, 4))
      gAvg[3] = gAvg[3] + S16BE(rawData:byte(5, 6))
  end

  -- Get average of 200 values and store as average current readings
  for i = 1, 3 do
    aAvg[i]= aAvg[i]/200
    gAvg[i]= gAvg[i]/200
  end

  -- Configure the accelerometer for self-test
  -- Enable self test on all three axes and set accelerometer range to +/- 2 g
  write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0)
  -- Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0)
  -- tmr_create():alarm(25, tmr_ALARM_SINGLE, function() -- delay(25) -- Delay a while to let the device stabilize

  delay(25000) -- Delay a while to let the device stabilize

  local aSTAvg, gSTAvg = {0, 0, 0}, {0, 0, 0}
  for _ = 1, 200 do -- get average self-test values of gyro and acclerometer
      -- Read the six raw data registers into data array
      rawData = read_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6)
      aSTAvg[1] = aSTAvg[1] + S16BE(rawData:byte(1, 2))  -- Turn the MSB and LSB into a signed 16-bit value
      aSTAvg[2] = aSTAvg[2] + S16BE(rawData:byte(3, 4))
      aSTAvg[3] = aSTAvg[3] + S16BE(rawData:byte(5, 6))

      -- Read the six raw data registers sequentially into data array
      rawData = read_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6)
      gSTAvg[1] = gSTAvg[1] + S16BE(rawData:byte(1, 2))  -- Turn the MSB and LSB into a signed 16-bit value
      gSTAvg[2] = gSTAvg[2] + S16BE(rawData:byte(3, 4))
      gSTAvg[3] = gSTAvg[3] + S16BE(rawData:byte(5, 6))
    end

  for i = 1, 3 do  -- Get average of 200 values and store as average self-test readings
    aSTAvg[i]= aSTAvg[i]/200
    gSTAvg[i]= gSTAvg[i]/200
  end

  -- Configure the gyro and accelerometer for normal operation
  write_reg(id, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00)
  write_reg(id, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00)

  -- tmr_create():alarm(25, tmr_ALARM_SINGLE, function() -- delay(25)
  delay(25000) -- Delay a while to let the device stabilize
  -- Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  -- X, Y, Z-axis accel self-test results
  local selfTest1 = read_reg(id, MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL, 3)
  -- X, Y, Z-axis gyro self-test results
  local selfTest2 = read_reg(id, MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO, 3)

  -- Retrieve factory self-test value from self-test code reads
  local factoryTrim = {}
  factoryTrim[1] = (2620/1<<FS)*(1.01^(selfTest1:byte(1) - 1.0)) -- FT[Xa] factory trim calculation
  factoryTrim[2] = (2620/1<<FS)*(1.01^(selfTest1:byte(2) - 1.0)) -- FT[Ya] factory trim calculation
  factoryTrim[3] = (2620/1<<FS)*(1.01^(selfTest1:byte(3) - 1.0)) -- FT[Za] factory trim calculation
  factoryTrim[4] = (2620/1<<FS)*(1.01^(selfTest2:byte(1) - 1.0)) -- FT[Xg] factory trim calculation
  factoryTrim[5] = (2620/1<<FS)*(1.01^(selfTest2:byte(2) - 1.0)) -- FT[Yg] factory trim calculation
  factoryTrim[6] = (2620/1<<FS)*(1.01^(selfTest2:byte(3) - 1.0)) -- FT[Zg] factory trim calculation

  -- Report results as a ratio of (STR - FT)/FT the change from Factory Trim of the Self-Test Response
  -- To get percent, must multiply by 100
  local destination = {}
  for i = 1, 3 do
    destination[i]   = 100.0*((aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0   -- Report percent differences
    destination[i+3] = 100.0*((gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0 -- Report percent differences
  end
  return destination
end


-- Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
-- (see http:--www.x-io.co.uk/category/open-source/ for examples and more details)
-- which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
-- device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
-- The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
-- but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
function MadgwickQuaternionUpdate( q, ax,  ay,  az,  gx,  gy,  gz,  mx,  my,  mz, deltat)
    -- gyroscope measurement error in rads/s (start at 40 deg/s)
    local beta = sqrt(3.0 / 4.0) * pi * (40.0 / 180.0)
    -- gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    -- local zeta = sqrt(3.0 / 4.0) * pi * (0.0 / 180.0)
    -- compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

   local q1, q2, q3, q4 = q[1], q[2], q[3], q[4] --unpack(q)
   local norm
   local hx, hy, _2bx, _2bz
   local s1, s2, s3, s4
   local qDot1, qDot2, qDot3, qDot4

  -- Auxiliary variables to avoid repeated arithmetic
   local _2q1mx, _2q1my, _2q1mz, _2q2mx, _4bx, _4bz
   local _2q1 = 2.0 * q1
   local _2q2 = 2.0 * q2
   local _2q3 = 2.0 * q3
   local _2q4 = 2.0 * q4
   local _2q1q3 = 2.0 * q1 * q3
   local _2q3q4 = 2.0 * q3 * q4
   local q1q1 = q1 * q1
   local q1q2 = q1 * q2
   local q1q3 = q1 * q3
   local q1q4 = q1 * q4
   local q2q2 = q2 * q2
   local q2q3 = q2 * q3
   local q2q4 = q2 * q4
   local q3q3 = q3 * q3
   local q3q4 = q3 * q4
   local q4q4 = q4 * q4

  -- Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az)
  if (norm == 0.0) then debug("MadgwickQuaternionUpdate: exiting on NaN"); return end -- handle NaN
  norm = 1.0/norm
  ax = ax * norm
  ay = ay * norm
  az = az * norm

  -- Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz)
  if (norm == 0.0) then debug("MadgwickQuaternionUpdate: exiting on NaN"); return end-- handle NaN
  norm = 1.0/norm
  mx = mx * norm
  my = my * norm
  mz = mz * norm

  -- Reference direction of Earth's magnetic field
  _2q1mx = 2.0 * q1 * mx
  _2q1my = 2.0 * q1 * my
  _2q1mz = 2.0 * q1 * mz
  _2q2mx = 2.0 * q2 * mx
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
  _2bx = sqrt(hx * hx + hy * hy)
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
  _4bx = 2.0 * _2bx
  _4bz = 2.0 * _2bz

  -- Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) -
    _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
    (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
    _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
  s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) -
    4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) +
    _2bz * (q2q4 - q1q3) - mx) +
    (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
    (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
  s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 +
    _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) +
    (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
    (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
    (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
  s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) +
    (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) +
    _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
    _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    -- normalise step magnitude
  norm = 1.0/norm
  s1, s2, s3, s4 = s1 * norm, s2 * norm, s3 * norm, s4 * norm

  -- Compute rate of change of quaternion
  qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
  qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
  qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
  qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

  -- Integrate to yield quaternion
  q1 = q1 + qDot1 * deltat
  q2 = q2 + qDot2 * deltat
  q3 = q3 + qDot3 * deltat
  q4 = q4 + qDot4 * deltat
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)   -- normalise quaternion
  norm = 1.0/norm
  -- debug("MadgwickQuaternionUpdate: q = %f, %f, %f, %f", q1 * norm,  q2 * norm,  q3 * norm, q4 * norm);
  return {q1 * norm,  q2 * norm,  q3 * norm, q4 * norm}
end


function write_reg(id, dev_addr, reg_addr, data)
  -- debug("write_reg: %02x, %02x, %02x, %02x", id, dev_addr, reg_addr, data)
  i2c_start(id)
  if not i2c_address(id, dev_addr, i2c_TRANSMITTER) then
    debug("No ACK on address: %02x", dev_addr)
    return nil
  end
  i2c_write(id, reg_addr)
  local c = i2c_write(id, data)
  i2c_stop(id)
  return c
end

function read_reg(id, dev_addr, reg_addr, n)
  i2c_start(id)
  if not i2c_address(id, dev_addr, i2c_TRANSMITTER) then
    debug("No ACK on address: %02x", dev_addr)
    return nil
  end
  i2c_write(id, reg_addr)
  i2c_stop(id)
  i2c_start(id)
  i2c_address(id, dev_addr, i2c_RECEIVER)
  local c = i2c_read(id, n or 1)
  i2c_stop(id)
  return c
end

function S16BE(b1, b2)   -- convert unsigned 16-bit no. to signed 16-bit no.
  -- local num = b1 *256 + b2
  local num = b1 << 8 | b2
  if num > 32768 then
      num = num - 65536
  end
  return num
end
------------------------------------------------ -----------------------------
function S16LE(b1, b2)   -- convert unsigned 16-bit no. to signed 16-bit no.
  -- local num = b2 *256 + b1
  local num = b2 << 8 | b1
  if num > 32768 then
      num = num - 65536
  end
  return num
end

return mpu9250
