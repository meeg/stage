#include "utils.h"

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

void sleep(double timeInSeconds){
	//delay in seconds
	usleep( (int) (timeInSeconds * 1000000)  );
}
