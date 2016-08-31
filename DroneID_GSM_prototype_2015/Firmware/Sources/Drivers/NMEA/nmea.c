/****************************************************************************
 # NMEA parser
 # Copyright (c) 2008-2013 Kjeld Jensen <kjeld@cetus.dk>
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
 # File: nmeaparser.c
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
 # Modified: 2011-05-31 Kjeld Jensen, Added time stamping of $GPGGA
 # Modified: 2012-03-20 Kjeld Jensen, Added $GPGSV parsing
 # Modified: 2012-05-24 Kjeld Jensen, added check for SNR 0xFF in $GPGSV parsing
 # Modified: 2013-05-02 Kjeld Jensen, Parser is now tolerant for problem with RTKlib $GPGSV messages
 # Modified: 2016-03-08 Martin Skriver, Added distance converter dd.dddd to m
 ****************************************************************************/
/* system includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/***************************************************************************/
/* application includes */
#include "nmea.h"

/***************************************************************************/
/* #defines */

#define false			 	0
#define true			 	1

#define OK				 	0
#define ERROR				-1

/* $GPGGA sentence positions */
#define GPGGA_TIME			1
#define GPGGA_LAT			2
#define GPGGA_LAT_H			3
#define GPGGA_LON			4
#define GPGGA_LON_H			5
#define GPGGA_FIX			6
#define GPGGA_SAT			7
#define GPGGA_HDOP			8
#define GPGGA_ALT			9
#define GPGGA_GEOID_HEIGHT	11
#define GPGGA_DGPS_UPDATE	13
#define GPGGA_DGPS_ID		14

/* $GPVTG sentence positions */
#define GPVTG_TRUE_TMG		1
#define GPVTG_MAGNETIC_TMG	3
#define GPVTG_GROUND_SPEED	7

#define GPGSA_FIX			2
#define GPGSA_PDOP			15
#define GPGSA_HDOP			16
#define GPGSA_VDOP			17

#define PI					3.14159
#define EARTH_RADIUS_M		6371000

/***************************************************************************/
/* function prototypes */
static int get_field (const char *s, int n, char *value);
static double nmea_latlon (char *s);

/***************************************************************************/
/* returns n'th comma-delimited field within s */
static int get_field (const char *s, int n, char *value)
{
 	int   i;

	/* skip n commas */
	for (i = 0; i < n; i++)
	{
		while (*s && *s != ',')
  			s++;
		if (*s == '\0')
        	return 0;
 		s++;
	}

	/* copy the field */
	while (*s && *s != ',' && *s != '*'  && *s != 13)
		*value++ = *s++;

	*value = '\0';

	return *s;
}
/***************************************************************************/
/* convert a nmea formatted coordinate dddmm.mmmmmmm to decimal degrees */
static double nmea_latlon (char *s)
{
	int i, dot, begin;
	double result, factor;

	/* set 'begin' to start of degrees */
	for (begin=0;s[begin] < '0' || s[begin] > '9';begin++)
		;

	/* set 'dot' to where the dot is placed */
	for (dot=begin+1; s[dot] != '.'; dot++)
		;

	/* retrieve minutes */
	result = (s[dot-2]-'0') * 10 + (s[dot-1]-'0');

	/* retrieve decimal minutes */
	for (i=dot+1, factor=10; i<=(dot+7);i++, factor *= 10)
		if (s[i]>='0' && s[i]<='9')
			result += (s[i]-'0')/factor;
		else
			break;

	/* convert decimal minutes to decimal degrees */
	result /= 60;

	/* retrieve degrees */
	for (i=dot-3, factor=1; i>=begin ;i--, factor *= 10)
		result += (s[i]-'0')*factor;

	/* set to negative if Southing or Westing */
	if (s[0] == 'S' || s[0] == 'W')
		result = -result;

	/* return the decimal degree coordinate */
	return result;
}
/***************************************************************************/
int nmea_gpgga_parse(char *nmeastr, gpgga_t *gga)
{
	char result = OK;
	char s[20];

	/* extract satellite fix quality from the nmea sentence */
	if (get_field(nmeastr, GPGGA_FIX, s))
	    gga->fix = atoi (s);
	else
		result = ERROR;

	/* if we got correct satellite fix info */
	if (result != ERROR)
	{
		/* if we have a satellite fix */
		if (gga->fix > GGA_FIX_INVALID)
		{
			/* extract all other data */
			if (get_field(nmeastr, GPGGA_LAT_H, s)
				&& get_field(nmeastr, GPGGA_LAT, s+1))
				gga->lat = nmea_latlon (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_LON_H, s)
				&& get_field(nmeastr, GPGGA_LON, s+1))
				gga->lon = nmea_latlon (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_TIME, s))
			{
				strncpy (gga->time, s, 10);
				gga->time[11] = 0;
			}
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_SAT, s))
				gga->sat = atoi (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_HDOP, s))
				gga->hdop = atof (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_ALT, s))
				gga->alt = atof (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_GEOID_HEIGHT, s))
				gga->geoid_height = atof (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_DGPS_UPDATE, s))
				gga->dgps_time = atoi (s);
			else
				result = ERROR;
			if (get_field(nmeastr, GPGGA_DGPS_ID, s))
				gga->dgps_stat_id = atoi (s);
			else
				result = ERROR;
		}
	}
	return result;
}
/***************************************************************************/
int nmea_gpvtg_parse(char *nmeastr, gpvtg_t *vtg)
{
	char result = OK;
	char s[20];

	if (get_field(nmeastr, GPVTG_TRUE_TMG, s))
	{
		if (s[0] != 0)
			vtg->true_tmg = atof (s);
		else
			vtg->true_tmg = -1;
	}
	else
		result = ERROR;

	if (get_field(nmeastr, GPVTG_MAGNETIC_TMG, s))
		if (s[0] != 0)
			vtg->magnetic_tmg = atof (s);
		else
			vtg->magnetic_tmg = -1;
	else
		result = ERROR;

	if (get_field(nmeastr, GPVTG_GROUND_SPEED, s))
		vtg->ground_speed = atof(s)/3.6;
	else
		result = ERROR;

	return (result);
}
/***************************************************************************/
int nmea_gpgsa_parse(char *nmeastr, gpgsa_t *gsa)
{
	char result = OK;
	char s[20];

	if (get_field(nmeastr, GPGSA_FIX, s))
	{
		gsa->fix = atoi (s);

		/* if we have a satellite fix */
		if (result != ERROR && gsa->fix > GSA_FIX_INVALID)
		{
			if (get_field(nmeastr, GPGSA_PDOP, s))
				gsa->pdop = atof (s);
			else
				result = ERROR;

			if (get_field(nmeastr, GPGSA_HDOP, s))
				gsa->hdop = atof (s);
			else
				result = ERROR;

			if (get_field(nmeastr, GPGSA_VDOP, s))
				gsa->vdop = atof (s);
			else
				result = ERROR;
		}
	}
	else
		result = ERROR;

	return (result);
}
/***************************************************************************/
int nmea_gpgsv_parse(char *nmeastr, gpgsv_t *gsv)
{
	char result = OK;
	char s[20];

	short i;

	for (i=0; i<4; i++)
		gsv->sat[i].prn = -1;

	/* number of messages required for all sats in view */
	if (get_field(nmeastr, 1, s))
    {
    	gsv->number_of_msgs = atoi(s);

		/* message number */
		if (get_field(nmeastr, 2, s))
    	{
    		gsv->msg_number = atoi(s);

			/* total number of satellites in view */
			if (get_field(nmeastr, 3, s))
			{
				char data_start = 4;

				gsv->sats_in_view = atoi(s);
				i = 0;

    			/* go through al satellite data sets in the sentence */
				while (get_field(nmeastr, data_start, s) && s[0] != 0)
				{
					char prn;
					unsigned short elevation, azimuth;
					unsigned char snr;

					/* save satellite PRN number */
					prn = atoi(s);

					/* save satellite elevation */
					gsv->sat[i].elevation = -1; /* u-blox receivers sometimes return this field as empty */
					if (get_field(nmeastr, data_start+1, s))
					{
						if (s[0] != 0)
							elevation = atoi(s);

						/* save satellite azimuth */
						gsv->sat[i].azimuth = -1; /* u-blox receivers sometimes return this field as empty */
						if (get_field(nmeastr, data_start+2, s))
						{
							if (s[0] != 0)
								azimuth = atoi(s);

							/* save satellite signal strength */
							snr = -1;
							if (get_field(nmeastr, data_start+3, s))
							{
								if (s[0] != 0)
									snr = atoi(s);
								if (snr == 0xff)
									snr = -1;

								gsv->sat[i].prn = prn;
								gsv->sat[i].elevation = elevation;
								gsv->sat[i].azimuth = azimuth;
								gsv->sat[i].snr = snr;
							}
						}
					}

					/* go to next satellite data set in the sentence */
					i++;
					data_start += 4;
				}
			}
		}
	}
	return result;
}
/***************************************************************************/
int nmea_checksum (char *s)
{
	unsigned char i, cs, hi, lo;
	int result = 0;

	/* calculate XOR checksum */
	for (i=0, cs=0; s[i] != 0 && s[i] != '*'; i++)
		cs = cs ^ s[i];

	/* retrieve transmitted checksum */
	if (s[i] == '*')
	{
		hi = s[i+1] - '0';
		if (hi > 9)	 /* compensate for hexadecimal letters */
			hi -= 7;
		lo = s[i+2] - '0';
		if (lo > 9)	 /* compensate for hexadecimal letters */
			lo -= 7;
		hi = (hi << 4) | lo;

		/* determine checksum result */
		if (cs != hi)
			result = -1;
	}

	return (result);
}
/***************************************************************************/
double degree_to_meter_conv(gpgga_t *gga1, gpgga_t *gga2)
{
	double lat1 = (gga1->lat*PI)/180;
	double lat2 = (gga2->lat*PI)/180;
	double delta_lat = ((gga2->lat-gga1->lat)*PI)/(double)180;
	double delta_lon = ((gga2->lon-gga1->lon)*PI)/(double)180;

	double a = sin(delta_lat/2) * sin(delta_lat/2) + sin(delta_lon/2) * sin(delta_lon/2) * cos(lat1) * cos(lat2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));

	return EARTH_RADIUS_M * c;

//	return 3.9;
}
/***************************************************************************/
