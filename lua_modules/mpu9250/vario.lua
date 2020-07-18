local sda, scl = 4, 3
local qfe_init = 0

local print, node_heap = print, node.heap
local function debug (fmt, ...)
  if (...) then fmt = fmt:format(...) end
  print("[vario]", node_heap(), fmt)
end

i2c.setup(0, sda, scl, i2c.SLOW) --i2c.FASTPLUS)
local calib_done = false
local s = require('bme280').setup(0) --, 0, 5, 0, 0, 0)
local g = require('mpu9250').setup(0, function(selftest)
  if selftest then
    debug("x-axis self test: acceleration trim within %.1f%% of factory value", selftest[1])
    debug("y-axis self test: acceleration trim within %.1f%% of factory value", selftest[2])
    debug("z-axis self test: acceleration trim within %.1f%% of factory value", selftest[3])
    debug("x-axis self test: gyration trim within %.1f%% of factory value", selftest[4])
    debug("y-axis self test: gyration trim within %.1f%% of factory value", selftest[5])
    debug("z-axis self test: gyration trim within %.1f%% of factory value", selftest[6])
  end
  calib_done = true
end, 0, 0, 0)

if not (s and g) then return end

local tmr_init = tmr.create()
local tmr_vario = tmr.create()

local measure = {}
debug("initializing")
tmr_init:alarm(250, tmr.ALARM_AUTO, function()
  if #measure >= 10 and calib_done then
    tmr_init:unregister()
    tmr_init=nil

    table.sort(measure)
    local mid = #measure//2
    if #measure % 2 == 0 then
      qfe_init = (measure[mid] + measure[mid+1]) / 2
    else
      qfe_init = measure[mid]
    end
    debug("initial QFE=%.2f", qfe_init)

    tmr_vario:start()
  else
      local _, P = s:read()
      if #measure<10 then measure[#measure+1] = P end
  end
end)

local function vario_handler()
  local _, P = s:read()
  local gyro = g:read()
  if P then
    local alt = s:altitude(P, qfe_init)
    debug("Altitude: %.3fm", alt)
  end
  if gyro then
    debug("Ax:%.3f Ay:%.3f Az:%.3f Gx:%.3f Gy:%.3f Gz:%.3f roll:%.1f pitch:%.1f yaw:%.1f",
      gyro.ax, gyro.ay, gyro.az, gyro.gx, gyro.gy, gyro.gz, gyro.roll, gyro.pitch, gyro.yaw)
  end
end

tmr_vario:register(250, tmr.ALARM_AUTO, vario_handler)
