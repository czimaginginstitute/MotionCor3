#pragma once

//=============================================================================
//
// This class parses string to integer and float.
//
// Shawn Zheng, 01/24/2003
//
//=============================================================================
class Util_Number
{
public:

	/*
	 * Parse a string made of digits into a number. If the string
	 * contains any letters except 0 to 9, +, -, and space, the 
	 * parsing fails.
	 *
	 * @return true if parsing success otherwise false.
	 */
	static bool ParseInt(char* pcVal, int& iOutVal);

	/*
	 * Parse a string made of digits into a float number. If the
	 * string contains any letters except 0 to 9, +, -, . and
	 * space, the parsing fails.
	 *
	 * @return true if parsing success otherwise false.
	 */
	static bool ParseFloat(char* pcVal, float& fOutVal);

	/*
	 * convert a real number into a string.
	 */
	static void ToString(double dNumber, char* pcStr);

	/*
	 * convert a int number to a string
	 */
	static void ToString(int iNumber, char* pcStr);

	static int Round(double dVal);

	static bool mCheckString(char* pcVal);

	static bool mIsDigit(char c);

	static bool mIsSign(char c);

	static bool mIsSpace(char c);
};
