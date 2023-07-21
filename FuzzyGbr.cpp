//Gbr/03.11.2016

#include "FuzzyGbr.h" 
// constructor
FuzzyGbr::FuzzyGbr(){ //:: scop resolution operator 
                      //scop operator refers to class FuzzyGbr 
					  //declared in the class FuzzyGbr 
}
// <<destructor>> 
FuzzyGbr::~FuzzyGbr(){        
	// nothing to destruct
}
//singleton
float FuzzyGbr::singleMf(float crisp, float singleParam[]){ 
  float degree=0;
  if (crisp == singleParam[0]) {
    degree=1;
  }
  return degree;
}


// triangular membership function
 float FuzzyGbr::triMf(float crisp, float triParam[]){                     //scop operator refers to function on Gbr declared in the class FuzzyGbr 
	float degree=0;
    if ((crisp > triParam[0]) && (crisp < triParam[2])) 
    {
      if (crisp <= triParam[1])
        degree=(crisp-triParam[0])/(triParam[1]-triParam[0]);
      else 
        degree=(triParam[2]-crisp)/(triParam[2]-triParam[1]);
	}
	if ((crisp == triParam[0]) && (crisp == triParam[1]))
		degree = 1;
	if ((crisp == triParam[1]) && (crisp == triParam[2]))
		degree = 1;
   return degree;
}

// trapezoidal membership function
 float FuzzyGbr::trapMf(float crisp, float trapParam[]){
    float degree=0;
    if ((crisp > trapParam[0]) && (crisp < trapParam[3])) 
    {
      if (crisp < trapParam[1])
        degree=(crisp-trapParam[0])/(trapParam[1]-trapParam[0]);
      else if (crisp > trapParam[2])
        degree=(trapParam[3]-crisp)/(trapParam[3]-trapParam[2]);
      else
        degree=1;
    }
	if ((crisp == trapParam[0]) && (crisp == trapParam[1]))
		degree = 1;
	if ((crisp == trapParam[2]) && (crisp == trapParam[3]))
		degree = 1;
   return degree;
  }
  
    // gauss membership function defined by its parameters (sigma, center).   
    float FuzzyGbr::gaussMf(float crisp, float gaussParam[])
  {
  // exp(-(crisp - center)^2/(2*sigma^2))  
  float exponent = -(pow(crisp-gaussParam[1],2))/(2*pow(gaussParam[0],2));
  float degree = exp(exponent);
  return degree;
  }
  
