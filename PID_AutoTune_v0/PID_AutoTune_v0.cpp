#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_AutoTune_v0.h>


PID_ATune::PID_ATune(double* Input, double* Output)
{
	input = Input;
	output = Output;
	controlType = P_I; //default to PI
	noiseBand = 0.5;
	running = false;
	oStep = 30;
	SetLookbackSec(10);
	lastTime = millis();
	
}



void PID_ATune::Cancel()
{
	running = false;
} 
 
int PID_ATune::Runtime()
{
	justevaled=false;
	if(peakCount>9 && running)
	{
		running = false;
		FinishUp();
		return 1;
	}
	unsigned long now = millis();
	
	if((now-lastTime)<sampleTime) return false;
	lastTime = now;
	double refVal = *input;
	justevaled=true;
	if(!running)
	{ //initialize working variables the first time around
		peakType = 0;
		peakCount=0;
		justchanged=false;
		absMax=refVal;
		absMin=refVal;
		setpoint = refVal;
		running = true;
		outputStart = *output;
		*output = outputStart+oStep;
	}
	else
	{
		if(refVal>absMax)absMax=refVal;
		if(refVal<absMin)absMin=refVal;
	}
	
	//oscillate the output base on the input's relation to the setpoint
	
	if(refVal>setpoint+noiseBand) *output = outputStart-oStep;
	else if (refVal<setpoint-noiseBand) *output = outputStart+oStep;
	
	
  //bool isMax=true, isMin=true;
  isMax=true;isMin=true;
  //id peaks
  for(int i=nLookBack-1;i>=0;i--)
  {
    double val = lastInputs[i];
    if(isMax) isMax = refVal>val;
    if(isMin) isMin = refVal<val;
    lastInputs[i+1] = lastInputs[i];
  }
  lastInputs[0] = refVal;  
  if(nLookBack<9)
  {  //we don't want to trust the maxes or mins until the inputs array has been filled
	return 0;
	}
  
  if(isMax)
  {
    if(peakType==0)peakType=1;
    if(peakType==-1)
    {
      peakType = 1;
      justchanged=true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;
   
  }
  else if(isMin)
  {
    if(peakType==0)peakType=-1;
    if(peakType==1)
    {
      peakType=-1;
      peakCount++;
      justchanged=true;
    }
    
    if(peakCount<10)peaks[peakCount] = refVal;
  }
  
  if(justchanged && peakCount>2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    double avgSeparation = (abs(peaks[peakCount-1]-peaks[peakCount-2])+abs(peaks[peakCount-2]-peaks[peakCount-3]))/2;
    if( avgSeparation < 0.05*(absMax-absMin))
    {
		FinishUp();
      running = false;
	  return 1;
	 
    }
  }
   justchanged=false;
	return 0;
}
void PID_ATune::FinishUp()
{
	  *output = outputStart;
      //we can generate tuning parameters!
      Ku = 4*(2*oStep)/((absMax-absMin)*3.14159);
      Pu = (double)(peak1-peak2) / 1000;
}

double PID_ATune::GetKp()
{
	double kp = 0;
	switch (controlType) {
	case P:
		kp = 0.5 * Ku;
		break;
	case P_I:
		kp = 0.45 * Ku;
		break;
	case P_D:
		kp = 0.8 * Ku;
		break;
	case CLASSIC_PID:
		kp = 0.6 * Ku;
		break;
	case PESSEN_INTEGRAL_RULE:
		kp = 7 * Ku / 10;
		break;
	case SOME_OVERSHOOT:
		kp = Ku / 3;
		break;
	case NO_OVERSHOOT:
		kp = Ku / 5;
		break;
	}
	return kp;
}
//0=PI, 1=PID
double PID_ATune::GetKi()
{
	double ki = 0;
	switch (controlType) {
	case P:
		ki = 0;
		break;
	case P_I:
		ki = 0.54 * Ku / Pu;
		break;
	case P_D:
		break;
	case CLASSIC_PID:
		ki = 1.2 * Ku / Pu;
		break;
	case PESSEN_INTEGRAL_RULE:
		ki = 1.75 * Ku / Pu;
		break;
	case SOME_OVERSHOOT:
		ki = ki = 0.666 * Ku / Pu;
		break;
	case NO_OVERSHOOT:
		ki = ki = (2 / 5) * Ku / Pu;
		break;
	}
	return ki;
}

double PID_ATune::GetKd()
{
	double kd = 0;
		switch (controlType) {
		case P:
			kd = 0;
			break;
		case P_I:
			kd = 0;
			break;
		case P_D:
			kd = Ku * Pu / 10;
			break;
		case CLASSIC_PID:
			kd = 3 * Ku * Pu / 40;
			break;
		case PESSEN_INTEGRAL_RULE:
			kd = 21 * Ku * Pu / 200;
			break;
		case SOME_OVERSHOOT:
			kd = Ku * Pu / 9;
			break;
		case NO_OVERSHOOT:
			kd = Ku * Pu / 15;
			break;
		}
		return kd;
}

void PID_ATune::SetOutputStep(double Step)
{
	oStep = Step;
}

double PID_ATune::GetOutputStep()
{
	return oStep;
}

void PID_ATune::SetControlType(ControlType controlType)
{
	this->controlType = controlType;
}
int PID_ATune::GetControlType()
{
	return controlType;
}
	
void PID_ATune::SetNoiseBand(double Band)
{
	noiseBand = Band;
}

double PID_ATune::GetNoiseBand()
{
	return noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
    if (value<1) value = 1;
	
	if(value<25)
	{
		nLookBack = value * 4;
		sampleTime = 250;
	}
	else
	{
		nLookBack = 100;
		sampleTime = value*10;
	}
}

int PID_ATune::GetLookbackSec()
{
	return nLookBack * sampleTime / 1000;
}
