#include "utils.h"

/**
 ****************************************************************************************
 * @brief Following functions are the implementation of converting float to string.
 * @return void
 ****************************************************************************************
*/
// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
	int i = 0, j = len - 1, temp;
	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}
// Converts a floating-point/double number to a string.
void ftoa(float n_val, char* res, int afterpoint)
{
	// Convert float to double so we can have more precise calculation on power and divide.
	volatile double n = n_val;   
	// If n is a negative value, put a '-' symbol at first.
	if ( n < 0)
	{
		res[0] = '-';
		res = res + 1; 
		n = -n;
	}
	// Round n to its nearest float number with 'afterpoint' decimal places
	// Such if afterpoint = 2, it will round 15.638 to 15.64	
	n = round(n * pow(10, afterpoint));
  n = n / pow(10, afterpoint);

	// Extract integer part
	int ipart = (int)n;
	
	// Extract floating part
	float fpart = n - (float)ipart;
		
	// convert integer part to string
	int i = intToStr(ipart, res, 1);

	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter
		// is needed to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}
