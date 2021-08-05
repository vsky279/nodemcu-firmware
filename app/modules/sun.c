// ***************************************************************************
// Sunrise/Sunset calculation module for ESP8266 with nodeMCU
// based on https://en.wikipedia.org/wiki/Sunrise_equation
//
// Written by Lukas Voborsky, @voborsky
//
// MIT license, http://opensource.org/licenses/MIT
// ***************************************************************************

#define NODE_DEBUG

#include "module.h"
#include "lauxlib.h"
#include "math.h"

// #define cos(a) (sin(PI / 2 - a))
// #define acos(a) (PI / 2 - asin(a))

static const double JULIAN_OFFSET = 2440587.5;
static const long SECDAY = 86400;

// double sin(double a) {
  // a = a - (int)(a / (2 * PI)) * PI * 2;
  // if (a<0) {a += 2 * PI;}
  // int8_t s = 1;
  // if (a > PI) {
      // s = -1;
      // a = -a + 2 * PI;
  // }
  // if (a > PI / 2 & a <= PI) {
      // a = PI - a;
  // }
  // a = a / PI * PIrad; // to radians
  // double p = -a * a * a;
  // double f = 6;
  // double r = a + p/f;

  // for (int j=1; j<=60; j++) {
    // p = -p * a * a;
    // f = f * 2 * (j+1)*(j+j+3);
    // r = r + p/f;
  // }
  // return r * s;
// }

// double asin(double a) {
  // double p = a * a * a;
  // double f = 0.5;
  // double r = a + f * p/3;

  // for (int j=1; j<=60; j++) {
    // p = p * a * a;
    // f = f * (2*j + 1) /(2 * (j+1));
    // r = r + f * p / ((2*(j+1) + 1));
  // }
  // return r * PI / PIrad;
// }

double mod(double a, double b){
  return a - ((int)(a / b)) * b;
}

#define JulianCycle(d, lon)(int32_t)(d - 2451545.0009 - lon / 360 + 0.5)
#define ApproximateSolarNoon(d, lon) (2451545.0009 + lon / 360 + JulianCycle(d, lon))

static double SolarMeanAnomaly(double d, double lon) {
    NODE_DBG("d: %d; lon: %d\n", (int)d, (int)lon);
    NODE_DBG("JulianCycle: %d\n", (int)JulianCycle(d, lon));
    NODE_DBG("ApproximateSolarNoon: %d\n", (int)ApproximateSolarNoon(d, lon));
    NODE_DBG("before mod: %d\n", (int)(357.5291 + 0.98560028 * (ApproximateSolarNoon(d, lon) - 2451545)));
    double res = mod(357.5291 + 0.98560028 * (ApproximateSolarNoon(d, lon) - 2451545), 360); // in degrees
    NODE_DBG("SolarMeanAnomaly (deg): %d\n", (int)res);
    return res;
}

#define EquationOfCenter(M) (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M))

static double EclipticLongitude(double M) {
    double res = mod((M + 102.9372 + EquationOfCenter(M) + 180), 360);
    NODE_DBG("EclipticLongitude (deg): %d\n", (int)res);
    return res;
}
// called from SolarTransit and HourAngle-SunDeclination -> remain a function
//#define EclipticLongitude(M) (mod((M + 102.9372 + EquationOfCenter(M) + 180), 360))

static double SolarTransit(double d, double lon) {
    double M = SolarMeanAnomaly(d, lon) / 360 * M_TWOPI;
    double res = ApproximateSolarNoon(d, lon) + 0.0053 * sin(M) - 0.0069 * sin(2 * EclipticLongitude(M) / 360 * M_TWOPI);
    NODE_DBG("SolarTransit: %d\n", (int)res);
    return res;
}

#define SunDeclination(M) (asin(sin(EclipticLongitude(M))*0.39794863130761038954479576746719))

static double HourAngle(double d, double lon, double lat, double elevation) {
    double M = SolarMeanAnomaly(d, lon) / 360 * M_TWOPI;
    double elevcorr = -2.076 * sqrt(elevation) / 60;
    double delta = SunDeclination(M);
    double res = acos((sin((-0.83 + elevcorr)) - sin(lat) * sin(delta)) / (cos(lat) * cos(delta))) / M_TWOPI * 360;
    NODE_DBG("HourAngle: %d\n", (int)res);
    return res;
}

static double SolarAzimuth(double d, double lon )  {
    double sTransit = SolarTransit(d, lon);
    double res = M_PI + (d - sTransit) * M_TWOPI;
    res = res / M_TWOPI * 360;
    NODE_DBG("SolarAzimuth (deg): %d\n", (int)res);
    return res;
}

static double SolarElevation(double d, double lon, double lat) {
    double Az = SolarAzimuth(d, lon);
    double latrad = lat / 360 * M_TWOPI;
    double delta = SunDeclination(SolarMeanAnomaly(d, lon));

    double a = 1 - pow(sin(Az) * cos(latrad), 2);
    double b = 2 * sin(delta) * sin(latrad);
    double c = pow(sin(Az) * cos(latrad), 2) + pow(sin(delta) * sin(latrad), 2) - pow(cos(delta) * cos(latrad), 2);

    double DET = b * b - 4 * a * c;
    double r1 = (-b + sqrt(DET)) / (2 * a);
    double r2 = (-b - sqrt(DET)) / (2 * a);
    double res;
    if (Az < M_PI / 2 || Az > 3 *  M_PI / 2) {
        res = -asin(r1);
    } else {
        res  = -asin(r2);
    }
    res = res / M_TWOPI * 360;
    NODE_DBG("SolarElevation (deg): %d\n", (int)res);
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
    // float as = asin(a);
    // lua_pushnumber(L, as);
  // }
  // return 1;
// }

// static int sun_lua_SolarTransit(lua_State* L) {
  // float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  // float lon = (float)luaL_checknumber(L, 2);
  // double res = (SolarTransit(d, lon) - JULIAN_OFFSET)*SECDAY;
  // lua_pushinteger(L, res);
  // return 1;
// }

// static int sun_lua_HourAngle(lua_State* L) {
  // float d = (float)luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  // float lon = (float)luaL_checknumber(L, 2);
  // float lat = (float)luaL_checknumber(L, 3);
  // float elevation = (float)luaL_optint(L, 4, 0);
  // double res = (HourAngle(d, lon, lat, elevation) - JULIAN_OFFSET)*SECDAY;
  // lua_pushinteger(L, res);
  // return 1;
// }

static int sun_lua_Sun(lua_State* L, int sign, double psunel) {
    double sunel;
    double d = luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
    double lon = luaL_checknumber(L, 2);
    double lat = luaL_checknumber(L, 3);
    if (psunel == 0) {sunel = luaL_optint(L, 5, 0);} else {sunel = psunel;}
    double elevation = luaL_optint(L, 4, 0);
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

static int sun_lua_SolarAzimuth(lua_State* L) {
    double d = luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
    double lon = luaL_checknumber(L, 2);
    double res = SolarAzimuth(d, lon);
    lua_pushnumber(L, res);
    return 1;
}

static int sun_lua_SolarElevation(lua_State* L) {
    double d = luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
    double lon = luaL_checknumber(L, 2);
    double lat = luaL_checknumber(L, 3);
    double res = SolarElevation(d, lon, lat);
    lua_pushnumber(L, res);
    return 1;
}

// Module function map
LROT_BEGIN(sun, NULL, 0)
    // { LSTRKEY( "sin" ), LFUNCVAL(sun_lua_sin)},
    // { LSTRKEY( "asin" ), LFUNCVAL(sun_lua_asin)},
    // LROT_FUNCENTRY( solartransit, sun_lua_SolarTransit )
    // LROT_FUNCENTRY( hourangle, sun_lua_HourAngle )
    LROT_FUNCENTRY( solarazimuth, sun_lua_SolarAzimuth )
    LROT_FUNCENTRY( solarelevation, sun_lua_SolarElevation )
    LROT_FUNCENTRY( sunrise, sun_lua_Sunrise )
    LROT_FUNCENTRY( sunset, sun_lua_Sunset )
    LROT_FUNCENTRY( civiltwilightend, sun_lua_CivilTwilightEnd )
    LROT_FUNCENTRY( civiltwilightstart, sun_lua_CivilTwilightStart)
LROT_END(sun, NULL, 0)

NODEMCU_MODULE(SUN, "sun", sun, NULL);


// > =n:format(sun.sunrise(1628553099, -14.625, 50.013, 320))
// d: 2459436; lon: -14
// JulianCycle: 7892
// ApproximateSolarNoon: 2459436
// before mod: 8135
// SolarMeanAnomaly (deg): 215
// EclipticLongitude (deg): 285 <--------------- chyba
// SolarTransit: 2459436
// d: 2459436; lon: -14
// JulianCycle: 7892
// ApproximateSolarNoon: 2459436
// before mod: 8135
// SolarMeanAnomaly (deg): 215
// EclipticLongitude (deg): 285
// HourAngle: 2147483647
// Sunrise: 2147483647
// 2038/1/19 03:14:07
