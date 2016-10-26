/****************************************************************************
* sprintf function
* File: float_to_string.c
* Purpose: Replacement of sprintf to change float format to string
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* Source copied from:
* "http://www.geeksforgeeks.org/convert-floating-point-number-string/"
* ****************************************************************************
* Log:
* Created:  2015-10-26 Martin Skriver,	File created
* Created:	2015-11-20 Martin Skriver, 	Changed to handle negative numbers
* Created:	2015-11-21 Martin Skriver, 	Changed change format from .5 to 0.5
****************************************************************************/

/***************************************************************************/
/* system includes */
#include <stdio.h>
#include <math.h>

/***************************************************************************/
/* application includes */
#include "float_to_string.h"

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[]. d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d, int n)
{
	int i = 0;
	if(x == 0)
	{
		str[i++] = '0';
	}
	else
	{
		while (x)
		{
			str[i++] = (x%10) + '0';
			x = x/10;
		}
	}

	// If number is negative
	if(n)
	{
		str[i++] = '-';
	}


	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
	// check for nevative value
	int negative = 0;
	if(n < 0.0)
	{
		negative = 1;
		n = fabs(n);
	}

	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = intToStr(ipart, res, 0, negative);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint, 0);
	}
}
