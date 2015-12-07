/****************************************************************************
 # NMEA parser
 # Copyright (c) 2008-2012 Kjeld Jensen <kjeld@cetus.dk>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: nmeaparser.h
 # Purpose: NMEA parser
 # Project: DroneID
 # Author: Kjeld Jensen <kjeld@cetus.dk>
 # Created:  2008-10-05 Kjeld Jensen, Source written
 # Modified: 2004-12-02 Kjeld Jensen, 1.01, the port index is now passed from nmea_update to nmea_parse.
 # Modified: 2005-03-31 Kjeld Jensen, 1.02, added UTM conversion.
 # Modified: 2005-07-29 Kjeld Jensen, 1.03, updated handling of message count variables
 # Modified: 2008-06-06 Kjeld Jensen, 1.10, modified to support CASMOBOT
 # Modified: 2010-04-08 Kjeld Jensen, 1.20, modified to support a single GPS (JDAMS)
 # Modified: 2011-02-07 Kjeld Jensen, Released under MIT license
 # Modified: 2012-02-03 Kjeld Jensen, Modified to support the NMEAlog application
 # Modified: 2012-03-20 Kjeld Jensen, Added $GPGSV parsing
****************************************************************************/
#ifndef NMEA_H_
#define NMEA_H_

/* system includes */
#include <stdlib.h>

/***************************************************************************/
/* GPGGA */
typedef struct
{
	char time[11];					/* fix taken at utc */
	double lat;						/* position latitude [degrees] */
	double lon;						/* position longitude [degrees] */
	unsigned char fix;				/* fix quality status */
	unsigned char sat;				/* number of satellites being tracked */
	float hdop;						/* horizontal dilution of precision */
	double alt;						/* altitude above Mean Sea Level [m] */
	double geoid_height;			/* height of MSL (geoid) above WS84 ellipsoid */
	unsigned short dgps_time;		/* time since last dgps update */
	unsigned long dgps_stat_id;		/* dgps station id number */
} gpgga_t;

/* defines for GGA fix_quality */
#define GGA_FIX_INVALID			0	/* invalid fix */
#define GGA_FIX_GPS				1	/* GPS fix (SPS) */
#define GGA_FIX_DGPS		 	2	/* Differential GPS fix */
#define GGA_FIX_PPS			 	3	/* Precise Positioning Service */
#define GGA_FIX_RTK			 	4	/* real time kinematic */
#define GGA_FIX_RTK_FLOAT	 	5	/* float RTK */
/***************************************************************************/
/* GPVTG */
typedef struct
{
	double true_tmg;				/* true track made good [degrees] */
	double magnetic_tmg;			/* magnetic track made good [degrees] */
	double ground_speed;			/* ground speed [m/s] */
} gpvtg_t;
/***************************************************************************/
/** GPGSA */
typedef struct
{
	char fix;						/* GPGSA fix */
	float pdop;						/* positional dilution of precision */
	float vdop;						/* vertical dilution of precision */
	float hdop;						/* horizontal dilution of precision */
} gpgsa_t;

/* defines for GSA fix_quality */
#define GSA_FIX_INVALID			1	/* invalid fix */
#define GSA_FIX_2D				2	/* 2D GPS fix */
#define GSA_FIX_3D				3	/* 3D GPS fix */

/***************************************************************************/
typedef struct
{
	char prn;						/* satellite PRN number (-1 means no data) */
	short elevation;				/* satellite elevation (deg [0;90]), -1 means no info */
	short azimuth;					/* satellite azimuth (deg [0;359] True North), -1 means no info  */
	unsigned char snr;				/* satellite signal to noise ratio (dB Hz) */
} gpgsv_sat_t;

#define GSV_SAT_MAX				4	/* $GPGGA can hold info for up to 4 sats */

typedef struct
{
	unsigned char number_of_msgs;	/* number of messages required for all sats in view */
	unsigned char msg_number;		/* message number */
	unsigned char sats_in_view;		/* total number of satellites in view */
	gpgsv_sat_t sat[GSV_SAT_MAX];
} gpgsv_t;
/***************************************************************************/
typedef struct
{
	char zone;						/* UTM zone number */
	char letter;					/* UTM latitude letter */
	double n;		  	   			/* UTM Northing */
	double e;			 			/* UTM Easting */
} utm_t;
/***************************************************************************/
/* function prototypes */
int nmea_checksum (char *s);
int nmea_gpgga_parse(char *nmeastr, gpgga_t *gga);
int nmea_gpgsa_parse(char *nmeastr, gpgsa_t *gsa);
int nmea_gpvtg_parse(char *nmeastr, gpvtg_t *vtg);
int nmea_gpgsv_parse(char *nmeastr, gpgsv_t *gsv);

/***************************************************************************/
#endif /* NMEA_H_ */
