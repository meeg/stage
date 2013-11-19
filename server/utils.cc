#include "utils.h"

void sleep(double timeInSeconds){
	//delay in seconds
	usleep( (int) (timeInSeconds * 1000000)  );
}
