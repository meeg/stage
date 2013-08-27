#include "utils.h"
#include "serial.h"


int parseCharReplyIntoArray( char* input, double* parsedReply ){
	//takes in a character array that has commands/params separated by spaces, and gives an equivalent floating	array.
	
	char localResultCopy[20];
	int numGroups, currentParsedIndex = 0;
	int numSpaces = findNumSpaces(input);
	double localParsedReply[20];
	
	numGroups = numSpaces + 1;
	//printf( "numGroups=%d\n", numGroups);

	string inputStr(input);
	int len = inputStr.length();
	int SPC_CHAR = 32, CR = 13, LF = 10;
	int prevSPCIndex = 0;
	
	
	//check special case:
	if ( (input[0] == 'e') & (input[1] == 'n') & (input[2] == 'd') ){
			return -1;
		}

			
			
	for (int i = 0; i < len; i++){
		//number groups are bounded by either spaces or the final carriage return
		if ( (input[i] == SPC_CHAR) | (input[i] == CR) | (input[i] == LF ) ){
			
			//convert only prevSPCIndex --> i;
			//printf("i=%d\n", i);
			char subArray[10];
			int status = copySubArray(subArray, input, prevSPCIndex, i);
			
			//for( int j = 0; j < 4; j++ ){
			//	printf( "subArray in Parse[%d]=%d\n", j, subArray[ j ] );
			//}	
			
			double number = getNumFromCharArray(subArray);
			//number = 0.0;	
			parsedReply[ currentParsedIndex ] = number;
			
			//printf("parsedReply[currentParsedIndex]=%f\n", parsedReply[ currentParsedIndex ] );
			prevSPCIndex = i + 1;
			currentParsedIndex++;
		}
		//printf("\n");
	}
	
	
	//debug
	//for( int j = 0; j < numGroups; j++ ){
	//	printf( "parsedReply[%d]=%f\n", j, parsedReply[ j ] );
	//}	
	
	
	return numGroups;
}


int copySubArray(char* subArray, char* input, int startIndex, int endIndex){
	//copy only a certain section of an input array into another array.
	
	char subset[20];
	
	int numCharsToCopy = endIndex - startIndex;
	
	for( int i = 0; i < numCharsToCopy; i++ ){
		subset[ i ] = input[ startIndex + i ];
		subset[ i + 1 ] = 0;		//prep the last element for when the for loop terminates
		//printf( "subset[%d]=%d\n", i, subset[ i ] );
	}
	
	
	strcpy(subArray, subset);
	//for( int i = 0; i < numCharsToCopy + 1; i++ ){
	//	printf( "subArray[%d]=%d\n", i, subArray[ i ] );
	//}		
	return 0;
}



double getNumFromCharArray(char* input){
	//takes in a character array (null terminated) and returns the floating point equivalent
	double valToReturn;
	char * pEnd;
	
	//valToReturn = strtol(input, &pEnd, 10);
	
	//special cases for letter/axis inputs:
	if ( input[0] > 97 ){
		//printf("detected letter\n");
		if ( input[0] == 120 ){
			valToReturn = X_AXIS_ID;
		}else if ( input[0] == 121 ){
			valToReturn = Y_AXIS_ID;
		}else	if ( input[0] == 122 ){
				valToReturn = Z_AXIS_ID;
		}else{
			valToReturn = 0.0;
		}
	}else{
		valToReturn = atof(input);
	}
	//printf("valToReturn=%f\n", valToReturn);
	return valToReturn;
}

int findNumSpaces(char* input){
	
	string inputStr(input);
	int len = inputStr.length();
	int numSpaces = 0;
	int SPC_CHAR = 32;
		
	for (int i = 0; i < len; i++){
		if ( input[i] == SPC_CHAR ) numSpaces++;	
		
	}
	
	return numSpaces;
}

int turnNumberIntoCharArray(int ID, char* resultingCharArray, double num ){
	//turn a number into a character array, with the first character being the device ID for easy parsing.
	ostringstream sout;
	sout << ID << num;

	char *buff = new char[sout.str().length() + 1];
	strcpy(buff, sout.str().c_str());
	
	
	// ... (use buff here)

	strcpy(resultingCharArray, buff);

	//printf("in turnNumberIntoCharArray()\n");
	//cout << resultingCharArray << endl; 
		
	delete [] buff;

	return 0;
}

int turnNumberIntoCharArray(int ID, char* resultingCharArray, int num ){
	ostringstream sout;
	sout << ID << num;

	char *buff = new char[sout.str().length() + 1];
	strcpy(buff, sout.str().c_str());
	
	
	// ... (use buff here)

	strcpy(resultingCharArray, buff);

	//printf("in turnNumberIntoCharArray()\n");
	//cout << resultingCharArray << endl; 
		
	delete [] buff;

	return 0;
}

int convertHexArrayToDec(char* hexArray, int* decResult){
	//takes in a character array holding hex values, and returns the decimal equivalent.  
	//It expects bunches of 4 digits separated by spaces.
	
	int hexGroupLen = 4;		//length of a hex group bunch
	
	//find number of characters:
	string lenArray(hexArray);
	int numChars = lenArray.length() + 1;
	//printf("numChars=%d\n", numChars);
	int numGroups;// = (numChars-1)/hexGroupLen;
	
	if ( numChars == 4 ) numGroups = 1;			//4 hex + cr
	if ( numChars == 10 ) numGroups = 2;		//8 hex + space + cr;
	if ( numChars == 15 ) numGroups = 3;		//12 hex + 2 spc + cr
	
	
	//printf("numGroups=%d, numChars=%d\n", numGroups, numChars);
/*	if (  ( numChars != hexGroupLen+1 ) & 
					( numChars != 2*hexGroupLen+numGroups) & 
						( numChars != 3*hexGroupLen+numGroups )  ){
		printf("ERR: finding number of hex groups; numChars = %d\n", numChars);
		return -1;
	}*/
		
	
	char * pEnd;
	int *decValue = decResult;		//decValue points to decResult, and any value put into decValue will be	returned from this sub
	decValue[ 0 ] = (int) strtol(hexArray, &pEnd, 16);
	
		
	for (int extraWords = 1; extraWords < numGroups; extraWords++){
		decValue[ extraWords ] = (int) strtol(pEnd, &pEnd, 16);
		//printf("%d\n", decValue[ extraWords ]);	
		//decValue[ extraWords + 1 ] = NULL;	//so when I leave the loop, the last element will be null
	}	

	
	if ( numGroups == 1 ){
		//printf("%d\n", decValue[ 0 ]);
		//printf("returned:%d\n", decValue[0] );
		return 0;
	}
	if ( numGroups == 2 ){
		//printf("%d\n", 65535*decValue[ 0 ]);
		//printf("%d\n", decValue[ 1 ]);
		decValue[0] = decValue[0]*65535 + decValue[1];
		//printf("returned:%d\n", decValue [0]);
		*decResult = decValue[0];
		return 0;
	}
	
	//printf("%d\n", decValue[ 0 ]);
	//printf("%d\n", decValue[ 1 ]);
		
	return 0;
}


void addArrayToArray(char* arrayToAddTo, char* arrayToAdd){
	//add array with spaces between calls
	
}

void decNumberToBinaryArray( int decVal, int* binaryArray){
	//this takes in a decimal character array, and outputs a binary array with the corresponding bits

	//string lenArray(hexArray);
	//int numChars = lenArray.length() + 1;
	//int decResult = 0;
	
	//printf("in dec to Bin:%d\n", decVal );

	char buffer[16];
	itoa (decVal, buffer, 2);		//convert decimal to base 2.
	int numBits = 0;
	
	//cout << "buffer=" << buffer << endl;
	//find how many binary characters there are.
	for(int i = 15; i >=0; i--){
		//printf( "buffer[%d]=%d\n", i, buffer[i]);
		if ( buffer[i] == 49 ){		//'1' ascii
			numBits = i + 1;
			break;
		}
	}
	//printf("numBits=%d\n",numBits);
	
	for(int i = 0; i < numBits; i++){
		binaryArray[ i ] = buffer[ i ] - NUMSTART;
		//printf("buffer=%d\n", binaryArray[ i ]);
	}
	
	
	//now fill in the remaining array with null
	for(int i = numBits; i < 16; i++){
		binaryArray[ i ] = 0;
		//printf("buffer=%d\n", binaryArray[ i ]);
	}
	
	if ( numBits > 16 ){
		printf("ERR: finding number of binary digis\n");
	}	/*else {
		printf("num bits:%d\n", numBits);
	}*/
		
			
	//assume a decimal input from a 4-digit hex value
	
	/*for(int i = 0; i < 16; i++){
		printf("binary[%d]=%d\n", i, binaryArray[ i ]);
	}*/

			
	//now reflip the bitstream
	for(int i = 0; i < 16; i++){
		buffer[ i ] = binaryArray[ 15 - i ];
	}
	for(int i = 0; i < 16; i++){
		binaryArray[ i ] = buffer[ i ];
	}	
				
}

	

int getHexArrayFromReply( char* inputArray, char* hexResult, int numWordsToGet){
	//this takes a reply from the motor controllers, 
	//and pulls out the 4 polling status word hex digits (16 bit binary).
	//optionally, grab more than one group of digits. 
	//Eg. if inputArray = "# 3 00B1 00A1" and numWordsToGet=2, then hexResult = "00B1 00A1"

	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
	//first find the carriage return, and then grab the last four next digits before that.
	int CRindex;
	string lenArray(inputArray);
	int numChars = lenArray.length() + 1;
	
	if (  (inputArray[ 0 ] != '#' ) & (inputArray[ 0 ] != '@' ) & (inputArray[ 0 ] != '!' ) & (inputArray[ 0 ]
			!= '*' )  ){
		printf("ERR seeing #, @,!, or * as first character\n");
		return -1;
	}
		
	//printf("replyString=%s\n", inputArray);

	//find character index in the string where CR is located
	for (int i = 0; i < numChars + 1; i++){
		//printf("replyString=%d\n", inputArray[i]);

		if ( inputArray[ i ] == 13 ){
			CRindex = i;
			//cout << "CRindex = " << CRindex << endl;
			break;
		}
	}
	
	//check for error
	if ( CRindex > numChars ){
		printf("ERR finding CR\n");
		return -1;		//error, couldn't find CR	
	}	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
	int hexGroupLen = 4;
	int startInputIndex = CRindex - hexGroupLen;						//the address of the INPUT to start pulling data from
	int replyGroupStartIndex = (numWordsToGet-1) * hexGroupLen;		/*the index to start writing the next group at.  This
	is changed with each group iteration.  For #words=2, start index for the first group to write = 4, the second
	group it's 0.*/
	
	char reversedHexResult[20];
	//printf("numWordsToGet=%d\n", numWordsToGet);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if ( numWordsToGet == 1 ){
		replyGroupStartIndex = 0;
		for(int dig = startInputIndex; dig < startInputIndex + hexGroupLen; dig++){
			//printf("inputArray[ %d ]=%c\n", dig, inputArray[ dig ]);
			//printf("dig=%d\n", dig);		
			//reversedHexResult[ replyGroupStartIndex ] = inputArray[ dig ];				
			hexResult[ replyGroupStartIndex ] = inputArray[ dig ];
			//printf("hexResult[ %d ]=%c\n", replyGroupStartIndex, hexResult[ replyGroupStartIndex ]);
			replyGroupStartIndex++;
		}	
		hexResult[ 4 ] = 0;	
		//printf("found PSW=%s\n", hexResult);
		return 0;
		
	}	
	
	
	//NOTE: the 2-word version still has a bug.  The group on the left is okay but the group on the right is off.
	
	if ( numWordsToGet == 2 ){
		
		replyGroupStartIndex = 5;
		for(int dig = startInputIndex; dig < startInputIndex + hexGroupLen; dig++){
			//printf("inputArray[ %d ]=%c\n", dig, inputArray[ dig ]);
			//printf("dig=%d\n", dig);		
			//reversedHexResult[ replyGroupStartIndex ] = inputArray[ dig ];				
			hexResult[ replyGroupStartIndex ] = inputArray[ dig ];
			//printf("replyGroupStartIndex=%d\n", replyGroupStartIndex);					
			//printf("hexResult[ %d ]=%c\n", replyGroupStartIndex, hexResult[ replyGroupStartIndex ]);
			replyGroupStartIndex++;
		}	
		hexResult[ 4 ] = 32; //space
		//printf("hexResult[ %d ]=%c\n", 4, hexResult[ 4 ]);
		
		startInputIndex = CRindex - ( 2*hexGroupLen  ) - 1;
		//printf("start reading input from here:%d\n", startInputIndex);
		replyGroupStartIndex = 0;
		for(int dig = startInputIndex; dig < startInputIndex + hexGroupLen; dig++){
				//printf("dig=%d\n", dig);
				//reversedHexResult[ replyGroupStartIndex ] = inputArray[ dig ];
				hexResult[ replyGroupStartIndex ] = inputArray[ dig ];
				//printf("replyGroupStartIndex=%d\n", replyGroupStartIndex);					
			//printf("hexResult[ %d ]=%c\n", replyGroupStartIndex, hexResult[ replyGroupStartIndex ]);
				replyGroupStartIndex++;
		}
		
		hexResult[ 9 ] = 0;		
		printf("found PSW=%s\n", hexResult);
		return 0;	
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
}




void strreverse(char* begin, char* end) {
	
	char aux;
	
	while(end>begin)
	
		aux=*end, *end--=*begin, *begin++=aux;
	
}
	
void itoa(int value, char* str, int base) {
	
	static char num[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	
	char* wstr=str;
	
	int sign;
	

	
	// Validate base
	
	if (base<2 || base>35){ *wstr='\0'; return; }
	

	
	// Take care of sign
	
	if ((sign=value) < 0) value = -value;
	

	
	// Conversion. Number is reversed.
	
	do *wstr++ = num[value%base]; while(value/=base);
	
	if(sign<0) *wstr++='-';
	
	*wstr='\0';
	

	
	// Reverse string
	
	strreverse(str,wstr-1);
	
}


void sleep(double timeInSeconds){
	//delay in seconds
	usleep( (int) (timeInSeconds * 1000000)  );
}
