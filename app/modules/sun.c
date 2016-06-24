// ***************************************************************************
// Sunrise/Sunset calculation module for ESP8266 with nodeMCU
// 
// Written by Lukas Voborsky, @voborsky
// 
// MIT license, http://opensource.org/licenses/MIT
// ***************************************************************************

//#define NODE_DEBUG

#include "module.h"
#include "flash_fs.h"
#include "lauxlib.h"
#include "c_math.h"

#include "sun.h"

#define cos(a) (sin(PI / 2 - a))
#define acos(a) (PI / 2 - asin(a))

#define JULIAN_OFFSET 2440587.5
#define SECDAY 86400.0

double sin(double a) {
  a = a - (int)(a / (2 * PI)) * PI * 2;
  if (a<0) {a += 2 * PI;}
  int8_t s = 1;
  if (a > PI) {
      s = -1;
      a = -a + 2 * PI;
  }
  if (a > PI / 2 & a <= PI) {
      a = PI - a;
  }
  double step = (PI / 2) / GRANULARITY;
  uint16_t offset = (int)(a / step); // * 4; // size_of(float)

  float sin1 = sin_table[offset] * s;
  float sin2 = sin_table[offset+1] * s;
  double sint = sin1 + (sin2 - sin1) / step * (a - offset * step);
  return sint;
}

double asin(double a) {
  int8_t s = 1;
  if (a < 0) {
      a *= -1;
      s = -1;
  }
  double step = 1.0 / GRANULARITY;
  uint16_t offset = (int)(a / step);

  float asin1 = asin_table[offset] * s;
  float asin2 = asin_table[offset+1] * s;
  double asint = asin1 + (asin2 - asin1) / step * (a - offset * step);
  return asint;
}

double mod(double a, double b){
  return a - ((int)(a / b)) * b;
}

#ifdef LUA_NUMBER_INTEGRAL
// Integer square root for integer version
static lua_Number isqrt(lua_Number x)
{
  lua_Number op, res, one;

  op = x; res = 0;

  /* "one" starts at the highest power of four <= than the argument. */
  one = 1 << 30;  /* second-to-top bit set */
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res +  2 * one;
    }
    res >>= 1;
    one >>= 2;
  }
  return(res);
}
#endif

#define JulianCycle(d, lon)(int32_t)(d - 2451545.0009 - lon / 360 + 0.5)
#define ApproximateSolarNoon(d, lon) (2451545.0009 + lon / 360 + JulianCycle(d, lon))

static double SolarMeanAnomaly(double d, double lon) {
  double res = mod((357.5291 + 0.98560028 * (ApproximateSolarNoon(d, lon) - 2451545)), 360.0);
  NODE_DBG("SolarMeanAnomaly: %d\n", (int)res);
  return res;
}

#define EquationOfCenter(M) (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M))

static double EclipticLongitude(double M) {
  double res = mod((M + 102.9372 + EquationOfCenter(M) + 180), 360);
  NODE_DBG("EclipticLongitude: %d\n", (int)res);
  return res;
}
// called from SolarTransit and HourAngle-SunDeclination -> remain a function
//#define EclipticLongitude(M) (mod((M + 102.9372 + EquationOfCenter(M) + 180), 360))

static double SolarTransit(double d, double lon) {
  double M = SolarMeanAnomaly(d, lon);
  double res = ApproximateSolarNoon(d, lon) + 0.0053 * sin(M) - 0.0069 * sin(2 * EclipticLongitude(M));
  NODE_DBG("SolarTransit: %d\n", (int)res);
  return res;
}

#define SunDeclination(M) (asin(sin(EclipticLongitude(M))*0.39794863130761038954479576746719))

static double HourAngle(double d, double lon, double lat, double elevation) {
  double M = SolarMeanAnomaly(d, lon);
#ifdef LUA_NUMBER_INTEGRAL
  double elevcorr = -2.076 * isqrt(elevation) / 60;
#else
  double elevcorr = -2.076 * sqrt(elevation) / 60;
#endif
  double delta = SunDeclination(M);
  double res = acos((sin((-0.83 + elevcorr)) - sin(lat) * sin(delta)) / (cos(lat) * cos(delta)));
  NODE_DBG("HourAngle: %d\n", (int)res);
  return res;
}

// static int sun_lua_sin(lua_State* L) {
  // float a = (float)luaL_checknumber(L, 1);
  // float s = sin(a);
  // lua_pushnumber(L, s);
  // return 1;
// }

// static int sun_lua_asin(lua_State* L) {
  // float a = (float)luaL_checknumber(L, 1);
  // if (a < -1 | a > 1) {
    // lua_pushnil(L);  /* error */
  // } else {
    // float sin = asin(a);
    // lua_pushnumber(L, sin);
  // }
  // return 1;
// }

static int sun_lua_SolarTransit(lua_State* L) {
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checknumber(L, 2)/1000.0;
  #else
  float lon = (float)luaL_checknumber(L, 2);
  #endif
  double res = (SolarTransit(d, lon) - JULIAN_OFFSET)*SECDAY;
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_HourAngle(lua_State* L) {
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  #else
  float lon = (float)luaL_checknumber(L, 2);
  float lat = (float)luaL_checknumber(L, 3);
  #endif
  float elevation = (float)luaL_optint(L, 4, 0);
  double res = (HourAngle(d, lon, lat, elevation) - JULIAN_OFFSET)*SECDAY;
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_Sunrise(lua_State* L) {
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  #else
  float lon = (float)luaL_checknumber(L, 2);
  float lat = (float)luaL_checknumber(L, 3);
  #endif
  float elevation = (float)luaL_optint(L, 4, 0);
  double res = ((SolarTransit(d, lon) - HourAngle(d, lon, lat, elevation) / 360) - JULIAN_OFFSET)*SECDAY; 
  NODE_DBG("Sunrise: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_Sunset(lua_State* L) {
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  #else
  float lon = (float)luaL_checknumber(L, 2);
  float lat = (float)luaL_checknumber(L, 3);
  #endif
  float elevation = (float)luaL_optint(L, 4, 0);
  double res = ((SolarTransit(d, lon) + HourAngle(d, lon, lat, elevation) / 360) - JULIAN_OFFSET)*SECDAY;
  NODE_DBG("Sunset: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_CivilTwilightEnd(lua_State* L) {
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  #else
  float lon = (float)luaL_checknumber(L, 2);
  float lat = (float)luaL_checknumber(L, 3);
  #endif
  float elevation = (float)luaL_optint(L, 4, 0);
  double res = ((SolarTransit(d, lon) - (HourAngle(d, lon, lat, elevation) + 6 + 0.83 * 2) / 360) - JULIAN_OFFSET)*SECDAY; 
  NODE_DBG("CivilTwilightEnd: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_CivilTwilightStart(lua_State* L) {
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  #else
  float lon = (float)luaL_checknumber(L, 2);
  float lat = (float)luaL_checknumber(L, 3);
  #endif
  float elevation = (float)luaL_optint(L, 4, 0);
  double res = ((SolarTransit(d, lon) + (HourAngle(d, lon, lat, elevation) + 6 + 0.83 * 2) / 360) - JULIAN_OFFSET)*SECDAY;
  NODE_DBG("CivilTwilightStart: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}


static const LUA_REG_TYPE sun_map[] = {
    // { LSTRKEY( "sin" ), LFUNCVAL(sun_lua_sin)},
    // { LSTRKEY( "asin" ), LFUNCVAL(sun_lua_asin)},
    { LSTRKEY( "solartransit" ), LFUNCVAL(sun_lua_SolarTransit)},
    { LSTRKEY( "hourangle" ), LFUNCVAL(sun_lua_HourAngle)},
    { LSTRKEY( "sunrise" ), LFUNCVAL(sun_lua_Sunrise)},
    { LSTRKEY( "sunset" ), LFUNCVAL(sun_lua_Sunset)},
    { LSTRKEY( "civiltwilightend" ), LFUNCVAL(sun_lua_CivilTwilightEnd)},
    { LSTRKEY( "civiltwilightstart" ), LFUNCVAL(sun_lua_CivilTwilightStart)},
    { LNILKEY, LNILVAL}
};

NODEMCU_MODULE(SUN, "sun", sun_map, NULL);
