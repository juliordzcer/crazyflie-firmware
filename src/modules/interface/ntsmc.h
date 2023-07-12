#include <stdbool.h>
#include "filter.h"


typedef struct
{
  float des;      
  float des_p;      
  float e;        
  float integ;     
  float e_p;        
  float k0;          
  float k1;          
  float k2;           
  float iLimit;       
  float outputLimit;  
  float dt;           
  lpf2pData dFilter; 
  bool enableDFilter; 
} ntsmcObject;


 void ntsmcInit(ntsmcObject* ntsmc, const float des, const float des_p,
             const float k0, const float k1, const float k2, const float dt,
             const float samplingRate, const float cutoffFreq, bool enableDFilter);

void ntsmcReset(ntsmcObject* ntsmc)

float ntsmcUpdate(ntsmcObject* ntsmc, const float actual, const float vel);