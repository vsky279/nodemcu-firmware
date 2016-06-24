// ***************************************************************************
// Sunrise/Sunset calculation module for ESP8266 with nodeMCU
// 
// Written by Lukas Voborsky, @voborsky
// 
// MIT license, http://opensource.org/licenses/MIT
// ***************************************************************************

// #define NODE_DEBUG

#include "module.h"
#include "lauxlib.h"
#include "c_math.h"

#define cos(a) (sin(PI / 2 - a))
#define acos(a) (PI / 2 - asin(a))

static const short PI = 180;
static const double PIrad = 3.14159265358979323846;
static const float JULIAN_OFFSET = 2440587.5;
static const long SECDAY = 86400;

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
  a = a / PI * PIrad; // to radians
  double p = -a * a * a;
  double f = 6;
  double r = a + p/f;
  
  for (int j=1; j<=60; j++) {
    p = -p * a * a;
    f = f * 2 * (j+1)*(j+j+3);
    r = r + p/f;
  }  
  return r * s;
}

double asin(double a) {
  double p = a * a * a;
  double f = 0.5;
  double r = a + f * p/3;
  
  for (int j=1; j<=60; j++) {
    p = p * a * a;
    f = f * (2*j + 1) /(2 * (j+1));
    r = r + f * p / ((2*(j+1) + 1));
  }  
  return r * PI / PIrad;
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
  // #ifdef LUA_NUMBER_INTEGRAL
  // float a = (float)luaL_checknumber(L, 1)/1000.0;
  // float s = sin(a) * 1000.0;
  // lua_pushinteger(L, s);
  // #else
  // float a = (float)luaL_checknumber(L, 1);
  // float s = sin(a);
  // lua_pushnumber(L, s);
  // #endif
  // return 1;
// }

// static int sun_lua_asin(lua_State* L) {
  // #ifdef LUA_NUMBER_INTEGRAL
  // float a = (float)luaL_checknumber(L, 1)/1000.0;
  // #else
  // float a = (float)luaL_checknumber(L, 1);
  // #endif
  // if (a < -1 | a > 1) {
    // lua_pushnil(L);  /* error */
  // } else {
    // #ifdef LUA_NUMBER_INTEGRAL
    // float as = asin(a) * 1000.0;
    // lua_pushinteger(L, as);
    // #else
    // float as = asin(a);
    // lua_pushnumber(L, as);
    // #endif
  // }
  // return 1;
// }

// static int sun_lua_SolarTransit(lua_State* L) {
  // float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  // #ifdef LUA_NUMBER_INTEGRAL
  // float lon = (float)luaL_checknumber(L, 2)/1000.0;
  // #else
  // float lon = (float)luaL_checknumber(L, 2);
  // #endif
  // double res = (SolarTransit(d, lon) - JULIAN_OFFSET)*SECDAY;
  // lua_pushinteger(L, res);
  // return 1;
// }

// static int sun_lua_HourAngle(lua_State* L) {
  // float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  // #ifdef LUA_NUMBER_INTEGRAL
  // float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  // float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  // #else
  // float lon = (float)luaL_checknumber(L, 2);
  // float lat = (float)luaL_checknumber(L, 3);
  // #endif
  // float elevation = (float)luaL_optint(L, 4, 0);
  // double res = (HourAngle(d, lon, lat, elevation) - JULIAN_OFFSET)*SECDAY;
  // lua_pushinteger(L, res);
  // return 1;
// }

static int sun_lua_Sun(lua_State* L, int sign, float psunel) {
  float sunel;
  float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  #ifdef LUA_NUMBER_INTEGRAL
  float lon = (float)luaL_checkinteger(L, 2)/1000.0;
  float lat = (float)luaL_checkinteger(L, 3)/1000.0;
  if (psunel == 0) {sunel = (float)luaL_optint(L, 5, 0)/100.0;} else {sunel = psunel;}
  #else
  float lon = (float)luaL_checknumber(L, 2);
  float lat = (float)luaL_checknumber(L, 3);
  if (psunel == 0) {sunel = (float)luaL_optint(L, 5, 0);} else {sunel = psunel;}
  #endif
  float elevation = (float)luaL_optint(L, 4, 0);
  double res = ((SolarTransit(d, lon) + sign * (HourAngle(d, lon, lat, elevation) + sunel) / 360) - JULIAN_OFFSET)*SECDAY; 
  return res;
}

static int sun_lua_Sunrise(lua_State* L) {
  double res = sun_lua_Sun(L, -1, 0);
  NODE_DBG("Sunrise: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_Sunset(lua_State* L) {
  double res = sun_lua_Sun(L, 1, 0);
  NODE_DBG("Sunrise: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_CivilTwilightEnd(lua_State* L) {
  double res = sun_lua_Sun(L, 1, 6 + 0.83 * 2);
  NODE_DBG("Civil Twilight End: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static int sun_lua_CivilTwilightStart(lua_State* L) {
  double res = sun_lua_Sun(L, -1, 6 + 0.83 * 2);
  NODE_DBG("Civil Twilight Start: %d\n", (int)res);
  lua_pushinteger(L, res);
  return 1;
}

static const LUA_REG_TYPE sun_map[] = {
    // { LSTRKEY( "sin" ), LFUNCVAL(sun_lua_sin)},
    // { LSTRKEY( "asin" ), LFUNCVAL(sun_lua_asin)},
    // { LSTRKEY( "solartransit" ), LFUNCVAL(sun_lua_SolarTransit)},
    // { LSTRKEY( "hourangle" ), LFUNCVAL(sun_lua_HourAngle)},
    { LSTRKEY( "sunrise" ), LFUNCVAL(sun_lua_Sunrise)},
    { LSTRKEY( "sunset" ), LFUNCVAL(sun_lua_Sunset)},
    { LSTRKEY( "civiltwilightend" ), LFUNCVAL(sun_lua_CivilTwilightEnd)},
    { LSTRKEY( "civiltwilightstart" ), LFUNCVAL(sun_lua_CivilTwilightStart)},
    { LNILKEY, LNILVAL}
};

NODEMCU_MODULE(SUN, "sun", sun_map, NULL);
