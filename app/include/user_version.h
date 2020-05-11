#ifndef __USER_VERSION_H__
#define __USER_VERSION_H__

#define NODE_VERSION_MAJOR		2U
#define NODE_VERSION_MINOR		1U
#define NODE_VERSION_REVISION	0U
#define NODE_VERSION_INTERNAL   0U

#define NODE_VERSION	"NodeMCU 2.1.0" " built with Docker provided by frightanic.com\n\tbranch: marblem-ping\n\tcommit: 2d13bd0cf78dec2d904a4c65367be60d1a4d1305\n\tSSL: false\n\tBuild type: integer\n\tLFS: disabled\n\tmodules: cron,dht,enduser_setup,file,gpio,net,node,pwm,rtcmem,rtctime,sjson,sntp,tls,tmr,uart,wifi\n"
#ifndef BUILD_DATE
#define BUILD_DATE	  "created on 2020-05-11 18:03\n"
#endif

extern char SDK_VERSION[];

#endif	/* __USER_VERSION_H__ */
