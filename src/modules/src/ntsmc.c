
#include "ntsmc.h"
#include "num.h"
#include <math.h>
#include <float.h>

#include "math3d.h"

void NTSMCInit(const float des, const float des_p,
             const float k0, const float k1, const float k2, const float dt,
             const float samplingRate, const float cutoffFreq, bool enableDFilter)
{
  e             = 0;
  e_prev        = 0;
  integ         = 0;
  e_p           = 0;
  des           = des;
  des_p         = des_p;
  k0            = k0;
  k1            = k1;
  k2            = k2;
  iLimit        = 5000.0;
  outputLimit   = 0.0;
  dt            = dt;
  
  enableDFilter = enableDFilter;
  // if (enableDFilter)
  // {
  //   lpf2pInit(&dFilter, samplingRate, cutoffFreq);
  // }
}

float ntsmcUpdate( const float actual, const float vel)
{
    float output = 0.0f;

    e = des - actual;
    e_p = des_p - vel;

    if(iLimit != 0)
    {
    	integ = constrain(integ, -iLimit, iLimit);
    }

    float s = e + k0 * powf(fabsf(e_p),3.0f / 2.0f)*sign(e_p);
    integ += k2 * sign(s) * dt;
    output += integ + k1 * powf(fabsf(s),1.0f/3.0f)*sign(s);
    
      // if (enableDFilter)
      // {
      //   output = lpf2pApply(&dFilter, output);
      // }
      // else {
      //   output = output;
      // }
      // if (isnan(output)) {
      //   output = 0;
      // } 
    

    if(outputLimit != 0)
    {
      output = constrain(output, -outputLimit, outputLimit);
    }

    return output;
}

void ntsmcReset()
{
  e     = 0;
  integ     = 0;
  e_p     = 0;
}
