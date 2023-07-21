# ifndef FuzzyGbr_H  // include guard
# define FuzzyGbr_H

#include <Arduino.h>  // include the Arduino code

class FuzzyGbr {    // beginning of the class
	public:           // accessible for the user
		FuzzyGbr();   // constructor (set up the library)
		~FuzzyGbr();  // destructor (delete the library)
		
		/* declare all functions
		1st variable (crisp) - is the current input value 
		next variables - parameters of membership function
		*/
		float triMf(float crisp, float triParam[]);
		float trapMf (float crisp, float trapParam[]);
		float gaussMf (float crisp, float gaussParam[]);
    float singleMf (float crisp, float singleParam[]);
};                   // end of the class

#endif              // ends #ifndef preprocessor directive
