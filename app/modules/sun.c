// ***************************************************************************
// Sunrise/Sunset calculation module for ESP8266 with nodeMCU
// based on https://en.wikipedia.org/wiki/Sunrise_equation
//
// Written by Lukas Voborsky, @voborsky
//
// MIT license, http://opensource.org/licenses/MIT
// ***************************************************************************

// #define NODE_DEBUG

#include "module.h"
#include "lauxlib.h"
#include "math.h"

double fmod(double x, double y) {
    return x - y * floor(x / y);
}

// all parameters and all function results are in _degrees_

#define JULIAN_OFFSET 2440587.5
#define SECDAY 86400.0

#define JulianCycle(d, lon)(int32_t)(d - 2451545.0009 - lon / 360 + 0.5)
#define ApproximateSolarNoon(d, lon) (2451545.0009 + lon / 360 + JulianCycle(d, lon))

static double SolarMeanAnomaly(double d, double lon) {
    double res = fmod(357.5291 + 0.98560028 * (ApproximateSolarNoon(d, lon) - 2451545), 360); // in degrees
    // NODE_DBG("SolarMeanAnomaly (deg): %d\n", (int)res);
    return res;
}

#define EquationOfCenter(M) (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M))

static double EclipticLongitude(double M) {
    double res = fmod((M + 102.9372 + EquationOfCenter(M / 360 * M_TWOPI) + 180), 360);
    // NODE_DBG("EclipticLongitude (deg): %d\n", (int)res);
    return res;
}

static double SolarTransit(double d, double lon) {
    double M = SolarMeanAnomaly(d, lon);
    double res = ApproximateSolarNoon(d, lon) + 0.0053 * sin(M / 360 * M_TWOPI) - 0.0069 * sin(2 * EclipticLongitude(M) / 360 * M_TWOPI);
    // NODE_DBG("SolarTransit: %d\n", (int)res);
    return res;
}

//#define SunDeclination(M)  (asin(sin(EclipticLongitude(M) / 360 * M_TWOPI) * sin(23.45 / 360 * M_TWOPI)) / M_TWOPI * 360
// sin(23.45 / 360 * M_TWOPI) = 0.39794863130761038954479576746719
#define SunDeclination(M)  (asin(sin(EclipticLongitude(M) / 360 * M_TWOPI) * 0.39794863130761038954479576746719) / M_TWOPI * 360)

static double HourAngle(double d, double lon, double lat, double elevation) {
    double M = SolarMeanAnomaly(d, lon);
    double latrad = lat / 360 * M_TWOPI;
    double elevcorr = -2.076 * sqrt(elevation) / 60;
    double delta = SunDeclination(M)  / 360 * M_TWOPI;
    double res = acos((sin((-0.83 + elevcorr) /360 * M_TWOPI) - sin(latrad) * sin(delta)) / (cos(latrad) * cos(delta))) / M_TWOPI * 360;
    // NODE_DBG("HourAngle: %d\n", (int)res);
    return res;
}

static double SolarAzimuth(double d, double lon )  {
    double sTransit = SolarTransit(d, lon);
    double res = (M_PI + (d - sTransit) * M_TWOPI) / M_TWOPI * 360;;
    // NODE_DBG("SolarAzimuth (deg): %d\n", (int)res);
    return res;
}

static double SolarElevation(double d, double lon, double lat, double Az) {
    // double Az = SolarAzimuth(d, lon) / 360 * M_TWOPI;
    double Azrad = Az / 360 * M_TWOPI;
    double latrad = lat / 360 * M_TWOPI;
    double delta = SunDeclination(SolarMeanAnomaly(d, lon)) / 360 * M_TWOPI;

    double a = 1 - pow(sin(Azrad) * cos(latrad), 2);
    double b = 2 * sin(delta) * sin(latrad);
    double c = pow(sin(Azrad) * cos(latrad), 2) + pow(sin(delta) * sin(latrad), 2) - pow(cos(delta) * cos(latrad), 2);
    // NODE_DBG("a=%d, b=%d, c=%d\n", (int)a, (int)b, (int)c);

    double DET = b * b - 4 * a * c;
    double r1 = (-b + sqrt(DET)) / (2 * a);
    double r2 = (-b - sqrt(DET)) / (2 * a);
    double res;
    if (Azrad < M_PI / 2 || Azrad > 3 *  M_PI / 2) {
        res = -asin(r1);
    } else {
        res  = -asin(r2);
    }
    res = res / M_TWOPI * 360;
    // NODE_DBG("SolarElevation (deg): %d\n", (int)res);
    return res;
}

// static int sun_lua_SolarTransit(lua_State* L) {
  // double d = luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  // double lon = luaL_checknumber(L, 2);
  // double res = (SolarTransit(d, lon) - JULIAN_OFFSET)*SECDAY;
  // lua_pushnumber(L, res);
  // return 1;
// }

// static int sun_lua_HourAngle(lua_State* L) {
  // double d = luaL_checknumber(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
  // double lon = luaL_checknumber(L, 2);
  // double lat = luaL_checknumber(L, 3);
  // double elevation = luaL_optint(L, 4, 0);
  // double res = (HourAngle(d, lon, lat, elevation) - JULIAN_OFFSET)*SECDAY;
  // lua_pushnumber(L, res);
  // return 1;
// }

static double SphereVectorsAngle(double theta1, double phi1, double theta2, double phi2) {
    double theta1rad = theta1/360*M_TWOPI;
    double phi1rad = phi1/360*M_TWOPI;
    double theta2rad = theta2/360*M_TWOPI;
    double phi2rad = phi2/360*M_TWOPI;

    double cosphi1rad = cos(phi1rad);
    double x1 = cos(theta1rad)*cosphi1rad;
    double y1 = sin(theta1rad)*cosphi1rad;
    double z1 =sin(phi1rad);
    
    double cosphi2rad = cos(phi2rad);
    double x2 = cos(theta2rad)*cosphi2rad;
    double y2 = sin(theta2rad)*cosphi2rad;
    double z2 =sin(phi2rad);

    double res = acos(x1 * x2 + y1 * y2 + z1 * z2) / M_TWOPI * 360;
    
    // NODE_DBG("PolarVectorsAngle (deg): %d, parameters: %d, %d, %d, %d\n", (int)res, (int)theta1, (int)phi1, (int)theta2, (int)phi2);
    return res;
}


static int sun_lua_Sun(lua_State* L, int sign, double psunel) {
    double sunel;
    double d = luaL_checkinteger(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
    double lon = luaL_checknumber(L, 2);
    double lat = luaL_checknumber(L, 3);
    if (psunel == 0) {sunel = luaL_optint(L, 5, 0);} else {sunel = psunel;}
    double elevation = luaL_optint(L, 4, 0);
    double res = (SolarTransit(d, lon) + sign * (HourAngle(d, lon, lat, elevation) + sunel) / 360 - JULIAN_OFFSET) * SECDAY;
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
    NODE_DBG("Sunset: %d\n", (int)res);
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

static int sun_lua_SolarPosition(lua_State* L) {
    double d = luaL_checkinteger(L, 1)/SECDAY + JULIAN_OFFSET; //timestamp to Julian date
    double lon = luaL_checknumber(L, 2);
    double lat = luaL_checknumber(L, 3);
    double az = SolarAzimuth(d, lon);
    double el = SolarElevation(d, lon, lat, az);
    lua_pushnumber(L, az);
    lua_pushnumber(L, el);
    return 2;
}

static int sun_lua_SphereVectorsAngle(lua_State* L) {
    double theta1 = luaL_checknumber(L, 1); //timestamp to Julian date
    double phi1 = luaL_checknumber(L, 2);
    double theta2 = luaL_checknumber(L, 3);
    double phi2 = luaL_checknumber(L, 4);
    double res = SphereVectorsAngle(theta1, phi1, theta2, phi2);
    lua_pushnumber(L, res);
    lua_pushnumber(L, cos(res / 360 * M_TWOPI));
    return 2;
}

#define MAX(a,b) (((a)>(b))?(a):(b))
static int sun_lua_Potential(lua_State* L) {
    uint32_t sec = fmod(luaL_checkinteger(L, 1), SECDAY); // seconds since utc midnight
    double mn = floor(luaL_checkinteger(L, 1)/SECDAY) + JULIAN_OFFSET; // utc midnight to Julian date
    
    double lon = luaL_checknumber(L, 2);
    double lat = luaL_checknumber(L, 3);
    double orientation = luaL_checknumber(L, 4);
    double declination = luaL_checknumber(L, 5);
    double kWp = luaL_checknumber(L, 6);
    double efficiency = luaL_optnumber(L, 7, 0.6);
    double basalconsumption = luaL_optnumber(L, 8, 0);
    double delta = luaL_optint(L, 9, 900);
    
    double end = sec + luaL_optint(L, 10, SECDAY);
    double potential = 0;
    double coef = kWp * efficiency;
    double deltahour = delta / 3600.0; // as a fraction of hour
    double d = mn + sec/SECDAY;
    double az = SolarAzimuth(d, lon);
    double el = SolarElevation(d, lon, lat, az);
    double w1 = (el>=6)?SphereVectorsAngle(az, el, orientation, declination):90;
    w1 = (w1>=-90 && w1<=90)?cos(w1/ 360 * M_TWOPI):0;
    double IAM = (w1>0)?(1-0.052*(1/w1-1)):0; // ASHRAE
    IAM = (IAM<=0)?0:IAM;
    NODE_DBG("sun position: %d, %d, eff: %d, IAM: %d\n", (int)(az), (int)(el), (int)(w1 *100), (int)(IAM*100));
    w1 = w1 * IAM;
    sec += delta;
    uint8_t aftersunrise = w1 > 0;
    NODE_DBG("potential: %d, eff: %d, aftersunrise: %d, deltahour: %d\n", (int)potential, (int)w1 *100, aftersunrise, (int)(deltahour*100));
    while (sec<=end && (w1 > 0 || !aftersunrise)) {
        d = mn + sec/SECDAY;
        az = SolarAzimuth(d, lon);
        el = SolarElevation(d, lon, lat, az);
        double w2 = (el>=6)?SphereVectorsAngle(az, el, orientation, declination):90;
        w2 = (w2>=-90 && w2<=90)?cos(w2/ 360 * M_TWOPI):0;
        IAM = (w2>0)?(1-0.052*(1/w2-1)):0; // ASHRAE
        IAM = (IAM<=0)?0:IAM;
        NODE_DBG("sun position: %d, %d, eff: %d, IAM: %d\n", (int)(az), (int)(el), (int)(w2 *100), (int)(IAM *100));
        w2 = w2 * IAM;
        potential += MAX((w1 + (w2 - w1)/2) * coef - basalconsumption, 0) * deltahour;
        NODE_DBG("potential: %d, eff: %d, potential+: %d, aftersunrise: %d\n", (int)potential, (int)(w1 *100), (int)(MAX((w1 + (w2 - w1)/2) * coef - basalconsumption, 0) * deltahour), aftersunrise);
        w1 = w2;
        aftersunrise |= (w1 > 0);
        sec += delta;
    }
    
    lua_pushnumber(L, potential);
    return 1;
}

// Module function map
LROT_BEGIN(sun, NULL, 0)
    // LROT_FUNCENTRY( solartransit, sun_lua_SolarTransit )
    // LROT_FUNCENTRY( hourangle, sun_lua_HourAngle )
    LROT_FUNCENTRY( position, sun_lua_SolarPosition )
    LROT_FUNCENTRY( sunrise, sun_lua_Sunrise )
    LROT_FUNCENTRY( sunset, sun_lua_Sunset )
    LROT_FUNCENTRY( civiltwilightend, sun_lua_CivilTwilightEnd )
    LROT_FUNCENTRY( civiltwilightstart, sun_lua_CivilTwilightStart)
    LROT_FUNCENTRY( spherevectorsangle, sun_lua_SphereVectorsAngle)
    LROT_FUNCENTRY( potential, sun_lua_Potential)
LROT_END(sun, NULL, 0)

NODEMCU_MODULE(SUN, "sun", sun, NULL);


// n=require("ntp");n:sync();s=tmr.create();
// d=n:time();s:alarm(100, tmr.ALARM_AUTO, function() print(n:format(d), sun.position(d, -14.625, 50.013)); d=d+600; end)
// =sun.spherevectorsangle(140,83,sun.position(rtctime.get(), -14.625, 50.013))
// =sun.potential((rtctime.get()//84600)*84600, -14.625, 50.013, 140, 83, 3600, 0.6, 0)
// d=(n:time()//86400) * 86400;st=d;s:alarm(100, tmr.ALARM_AUTO, function() dw=sun.potential(d, -14.625, 50.013, 140, 83, 3600, 1, 0, 1800); print(n:format(d), dw); if d==st+86400*365 then s:stop() end; d=d+86400; end) 
// sum=0;d=(n:time()//86400) * 86400;st=d;s:alarm(100, tmr.ALARM_AUTO, function() dw=sun.potential(d, -14.625, 50.013, 140, 83, 3600, 1, 0, 1800); sum=sum+dw;print(n:format(d), dw); if d==st+86400*365 then s:stop() end; d=d+86400; end)

// sum={0,0,0,0,0,0,0,0,0,0,0,0};d=(n:time()//86400) * 86400;st=d;
// s:alarm(100, tmr.ALARM_AUTO, function() dw=sun.potential(d, -14.625, 50.013, 140, 83, 3600, 1, 0, 1800); m=rtctime.epoch2cal(d)["mon"]; sum[m]=sum[m]+dw; print(n:format(d), m, dw); if d==st+86400*364 then s:stop() end; d=d+86400; end)  