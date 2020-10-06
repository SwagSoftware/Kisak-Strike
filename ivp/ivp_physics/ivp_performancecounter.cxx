// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#	pragma implementation "ivp_performancecounter.hxx"
#endif
#include <ivp_performancecounter.hxx>



void IVP_PerformanceCounter_Simple::reset_and_print_performance_counters(IVP_Time current_time){
	IVP_DOUBLE diff = count_PSIs;
	if (diff == 0.0f) return;

	IVP_DOUBLE collision =  counter[IVP_PE_PSI_UNIVERSE][0]  + counter[IVP_PE_PSI_SHORT_MINDISTS][0] + 
							counter[IVP_PE_PSI_CRITICAL_MINDISTS][0]  + counter[IVP_PE_PSI_HULL][0] +
							counter[IVP_PE_AT_INIT][0];
	IVP_DOUBLE dynamics = counter[IVP_PE_PSI_CONTROLLERS][0] + counter[IVP_PE_PSI_INTEGRATORS][0] ;
	IVP_DOUBLE sum = collision + dynamics;

	IVP_DOUBLE factor = .001f / diff;

#if defined(PSX2)

	ivp_message(	"UNIV: %2.2f,%2.2f CONTR: %2.2f,%2.2f INTEGR: %2.2f,%2.2f "
				"HULL: %2.2f,%2.2f SHORT: %2.2f,%2.2f CRITIC: %2.2f,%2.2f "
				"USR: %2.2f,%2.2f\n",
		counter[IVP_PE_PSI_UNIVERSE][0] * factor,
		counter[IVP_PE_PSI_UNIVERSE][1] * factor,
		counter[IVP_PE_PSI_CONTROLLERS][0] * factor,
		counter[IVP_PE_PSI_CONTROLLERS][1] * factor,
		counter[IVP_PE_PSI_INTEGRATORS][0] * factor,
		counter[IVP_PE_PSI_INTEGRATORS][1] * factor,
		counter[IVP_PE_PSI_HULL][0] * factor,
		counter[IVP_PE_PSI_HULL][1] * factor,
		counter[IVP_PE_PSI_SHORT_MINDISTS][0] * factor,
		counter[IVP_PE_PSI_SHORT_MINDISTS][1] * factor,
		counter[IVP_PE_PSI_CRITICAL_MINDISTS][0] * factor,
		counter[IVP_PE_PSI_CRITICAL_MINDISTS][1] * factor,
		counter[IVP_PE_USR1][0] * factor,
		counter[IVP_PE_USR1][1] * factor);
#elif 1

		ivp_message(	"TOT %2.1f%% %2.2f COLL %2.2f  DYN %2.2f     det:  UNIV: %2.2f CONTR: %2.2f INTEGR: %2.2f "
				"HULL: %2.2f SHORT: %2.2f CRITIC: %2.2f AT %2.2f\n",
				sum * factor * 66.0f * (100.0 * 0.001),
				sum * factor, collision * factor , dynamics * factor,
		counter[IVP_PE_PSI_UNIVERSE][0] * factor,
		counter[IVP_PE_PSI_CONTROLLERS][0] * factor,
		counter[IVP_PE_PSI_INTEGRATORS][0] * factor,
		counter[IVP_PE_PSI_HULL][0] * factor,
		counter[IVP_PE_PSI_SHORT_MINDISTS][0] * factor,
		counter[IVP_PE_PSI_CRITICAL_MINDISTS][0] * factor,
		counter[IVP_PE_AT_INIT][0] * factor);

#endif
	P_MEM_CLEAR_M4(this);
	time_of_last_reset = current_time;
}


void IVP_PerformanceCounter_Simple::environment_is_going_to_be_deleted(IVP_Environment *){
	P_DELETE_THIS(this);
}

IVP_PerformanceCounter_Simple::~IVP_PerformanceCounter_Simple(){
	;
}

IVP_PerformanceCounter_Simple::IVP_PerformanceCounter_Simple(){
    P_MEM_CLEAR_M4(this);
}



#if defined(PSXII)

/*
 *	Include file for the performance counters. Link with libpc.a.
 *	Note that the performance counters will not be implemented
 *	in the retail version of the hardware so it sould not be
 *	compiled into the final release app.
 *
 *	Refer to the libpc.txt documentation for the defines to set
 *	up the performance counter. Below are some commonly used
 *	settings.
 */
#include	<libpc.h>
			 /*
			  * Count CPU cycles in Counter0 and DCache Misses
			  * in Counter1
			  */
			#define PROFILE_CPU_DCACHE													\
							(	SCE_PC_CTE |												\
																								\
								SCE_PC0_CPU_CYCLE |										\
								SCE_PC_U0|SCE_PC_S0|SCE_PC_K0|SCE_PC_EXL0 |		\
																								\
								SCE_PC1_DCACHE_MISS |									\
								SCE_PC_U1|SCE_PC_S1|SCE_PC_K1|SCE_PC_EXL1 		\
							)

			/*
			 * Count ICache misses in Counter0 and CPU cycles
			 * in Counter1.
			 */
			#define PROFILE_ICACHE_CPU													\
							(	SCE_PC_CTE |												\
																								\
								SCE_PC1_CPU_CYCLE |										\
								SCE_PC_U1|SCE_PC_S1|SCE_PC_K1|SCE_PC_EXL1 |		\
																								\
								SCE_PC0_ICACHE_MISS |									\
								SCE_PC_U0|SCE_PC_S0|SCE_PC_K0|SCE_PC_EXL0 		\
							)

			/*
			 * Count Address bus busy(0) and Data bus busy(1)
			 */
			#define PROFILE_ADDRBUS_DATABUS											\
							(	SCE_PC_CTE |												\
																								\
								SCE_PC0_ADDR_BUS_BUSY |									\
								SCE_PC_U0|SCE_PC_S0|SCE_PC_K0|SCE_PC_EXL0 |		\
																								\
								SCE_PC1_DATA_BUS_BUSY |									\
								SCE_PC_U1|SCE_PC_S1|SCE_PC_K1|SCE_PC_EXL1 		\
							)


/*
 *	Refer to the Sony libpc documentation for the flags.
 */
void IVP_PerformanceCounter_Simple::start_pcount(){
	int flags = PROFILE_CPU_DCACHE;
	count_PSIs++;
	counting = IVP_PE_PSI_START;
	scePcStart(flags,0,0);
}

void IVP_PerformanceCounter_Simple::stop_pcount(){
	scePcStart(SCE_PC0_NO_EVENT|SCE_PC1_NO_EVENT,0,0);
}

void IVP_PerformanceCounter_Simple::pcount( IVP_PERFORMANCE_ELEMENT el){
	/*
	 *	This could be += or = depending on how pcount() is called
	 */
	int c0 = scePcGetCounter0();
	int diff0 = c0 - ref_counter[0];
	ref_counter[0] = c0;

	int c1 = scePcGetCounter1();
	int diff1 = c1 - ref_counter[1];
	ref_counter[1] = c1;

	counter[counting][0] += diff0;
	counter[counting][1] += diff1;
	counting = el;
}

#elif defined(WIN32) 
#	ifndef _XBOX
#		ifndef WIN32_LEAN_AND_MEAN
#			define	WIN32_LEAN_AND_MEAN
#		endif
#		include	<windows.h>
#	else
#		ifndef WINVER
#			define WINVER 0x0500
#		endif
#		ifndef _X86_
#			define _X86_
#		endif  /* _X86_ */
#		include <excpt.h>
#		include <stdarg.h>
#		include <windef.h>
#		include <winbase.h>
#	endif

void IVP_PerformanceCounter_Simple::pcount( IVP_PERFORMANCE_ELEMENT el){
    __int64	Profile_Counter;
    __int64	Profile_Freq;

	if (el == IVP_PE_PSI_UNIVERSE ){
		count_PSIs++;
	}

    QueryPerformanceCounter((LARGE_INTEGER*)(&Profile_Counter));
	QueryPerformanceFrequency( (LARGE_INTEGER*) &Profile_Freq);   // address of current frequency


    int diff0 = Profile_Counter - ref_counter64;
    ref_counter64 = Profile_Counter;

    counter[counting][0] += 1e6 * double(diff0) / double (Profile_Freq);
    counting = el;
}

void IVP_PerformanceCounter_Simple::start_pcount(){
    	counting = IVP_PE_PSI_START;
}

void IVP_PerformanceCounter_Simple::stop_pcount(){ ; }

#else
	void IVP_PerformanceCounter_Simple::pcount( IVP_PERFORMANCE_ELEMENT ){;}
	void IVP_PerformanceCounter_Simple::start_pcount(){ ; }
	void IVP_PerformanceCounter_Simple::stop_pcount(){ ; }
#endif
