#include <math.h> // fabsf, sqrtf
#include <stdbool.h>
#include <stdint.h>

#include "angle_pid.h"
#include "config.h"
#include "drv_dshot.h"
#include "drv_time.h"
#include "filter.h"
#include "pid.h"
#include "rx.h"
#include "stick_vector.h"
#include "util.h"

#include "debug.h"

int onground = 1;

float thrsum;
float mixmax;
float throttle_reversing_kick;
float throttle_reversing_kick_sawtooth;
float throttle_reversing_kick_decrement;

float error[ PIDNUMBER ];
float setpoint[ PIDNUMBER ];

extern float vbattfilt; // battery.c
extern float vbatt_comp;
extern float battery_scale_factor;

extern float gyro[ 3 ]; // sixaxis.c

extern int pwmdir; // drv_dshot.c
extern float pidoutput[ PIDNUMBER ]; // pid.c
extern float ierror[ PIDNUMBER ]; // pid.c

extern bool failsafe; // rx.c
extern float rx[ 4 ]; // rx.c
extern char aux[ AUXNUMBER ]; // rx.c
extern float aux_analog[ 2 ];
extern int packet_period; // rx.c

extern bool ledcommand; // led.c

float rxcopy[ 4 ];
float bb_throttle;
float bb_mix[ 4 ];

void control( bool send_motor_values )
{
	// rates / expert mode
	float rate_multiplier = 1.0f;

	if ( aux[ RATES ] ) {
	} else {
		rate_multiplier = LOW_RATES_MULTI;
	}

#ifdef INVERTED_ENABLE
	const bool motor_direction_changed = ( aux[ FN_INVERTED ] && pwmdir != REVERSE ) || ( ! aux[ FN_INVERTED ] && pwmdir != FORWARD );
	if ( motor_direction_changed ) {
		ierror[ 0 ] = ierror[ 1 ] = ierror[ 2 ] = 0.0f;
		throttle_hpf_reset( 200 ); // ms
#ifdef THROTTLE_REVERSING_KICK
		throttle_reversing_kick = (float)THROTTLE_REVERSING_KICK * ( ( battery_scale_factor - 1.0f ) * 1.5f + 1.0f );
		#define TRKD 100000.0f // 100 ms throttle reversing kick duration
		throttle_reversing_kick_sawtooth = throttle_reversing_kick * ( TRKD + (float)THROTTLE_REVERSING_DEADTIME ) / TRKD;
		throttle_reversing_kick_decrement = throttle_reversing_kick_sawtooth * (float)LOOPTIME / ( TRKD + (float)THROTTLE_REVERSING_DEADTIME );
#endif // THROTTLE_REVERSING_KICK
	}

	if ( aux[ FN_INVERTED ] ) {
		pwmdir = REVERSE;
	} else {
		pwmdir = FORWARD;
	}
#endif // INVERTED_ENABLE

	for ( int i = 0; i < 3; ++i ) {
		rxcopy[ i ] = rx[ i ];

#ifdef STICKS_DEADBAND
		if ( fabsf( rxcopy[ i ] ) <= STICKS_DEADBAND ) {
			rxcopy[ i ] = 0.0f;
		} else {
			if ( rxcopy[ i ] >= 0 ) {
				rxcopy[ i ] = mapf( rxcopy[ i ], STICKS_DEADBAND, 1, 0, 1 );
			} else {
				rxcopy[ i ] = mapf( rxcopy[ i ], -STICKS_DEADBAND, -1, 0, -1 );
			}
		}
#endif // STICKS_DEADBAND

		rxcopy[ i ] *= rate_multiplier;
	}

	rxcopy[ 3 ] = rx[ 3 ]; // throttle

	const float mixRangeLimit = MIX_RANGE_LIMIT;
	rxcopy[ 3 ] *= mixRangeLimit;

#ifdef RX_SMOOTHING
	static float rxsmooth[ 4 ];
	static float lastRXcopy[ 4 ];
	static float stepRX[ 4 ];
	static int countRX[ 4 ];
	for ( int i = 0; i < 4; ++i ) {
		if ( rxcopy[ i ] != lastRXcopy[ i ] ) {
			const int step_count = packet_period / LOOPTIME; // Spread it evenly over e.g. 5 ms
			stepRX[ i ] = ( rxcopy[ i ] - lastRXcopy[ i ] ) / step_count;
			countRX[ i ] = step_count;
			rxsmooth[ i ] = lastRXcopy[ i ];
			lastRXcopy[ i ] = rxcopy[ i ];
		}
		if ( countRX[ i ] > 0 ) {
			--countRX[ i ];
			rxsmooth[ i ] += stepRX[ i ];
			rxcopy[ i ] = rxsmooth[ i ];
		} else {
			rxsmooth[ i ] = rxcopy[ i ];
		}
	}
#endif // RX_SMOOTHING

#ifdef STICK_VELOCITY_LIMIT
	static float rxcopy_limited[ 4 ];
	for ( int i = 0; i < 4; ++i ) {
		const float delta_limit = ( 1 + fabsf( rxcopy_limited[ i ] ) ) * (float)STICK_VELOCITY_LIMIT * LOOPTIME * 1e-6f;
		float abs_delta = fabsf( rxcopy[ i ] - rxcopy_limited[ i ] );
		if ( abs_delta > delta_limit ) {
			abs_delta = delta_limit;
		}
		if ( rxcopy[ i ] > rxcopy_limited[ i ] ) {
			rxcopy_limited[ i ] += abs_delta;
		} else {
			rxcopy_limited[ i ] -= abs_delta;
		}
		rxcopy[ i ] = rxcopy_limited[ i ];
	}
#endif

	pid_precalc();

	// flight control

#ifdef LEVELMODE
	if ( aux[ LEVELMODE ] ) { // level mode
		extern float angleerror[];
		extern float errorvect[]; // level mode angle error calculated by stick_vector.c
		extern float GEstG[ 3 ]; // gravity vector for yaw feedforward
		float yawerror[ 3 ] = { 0 }; // yaw rotation vector

		// calculate roll / pitch error
		stick_vector( rxcopy, 0 );

		float yawrate = rxcopy[ 2 ] * (float)MAX_RATEYAW * DEGTORAD;
		// apply yaw from the top of the quad
		yawerror[ 0 ] = GEstG[ 1 ] * yawrate;
		yawerror[ 1 ] = -GEstG[ 0 ] * yawrate;
		yawerror[ 2 ] = GEstG[ 2 ] * yawrate;

		// pitch and roll
		for ( int i = 0; i <= 1; ++i ) {
			angleerror[ i ] = errorvect[ i ];
			error[ i ] = angle_pid( i ) + yawerror[ i ] - gyro[ i ];
		}
		// yaw
		error[ 2 ] = yawerror[ 2 ] - gyro[ 2 ];

		// Set ierror to zero, otherwise it builds up and causes bounce back.
		ierror[ 0 ] = 0.0f; ierror[ 1 ] = 0.0f;

		// Nice for BB logging fun. Not needed otherwise.
		for ( int i = 0; i < 3; ++i ) {
			 setpoint[ i ] = error[ i ] + gyro[ i ];
		}
	} else
#endif // LEVELMODE
	{ // rate mode
		setpoint[ 0 ] = rxcopy[ 0 ] * (float)MAX_RATE * DEGTORAD;
		setpoint[ 1 ] = rxcopy[ 1 ] * (float)MAX_RATE * DEGTORAD;
		setpoint[ 2 ] = rxcopy[ 2 ] * (float)MAX_RATEYAW * DEGTORAD;

		for ( int i = 0; i < 3; ++i ) {
			error[ i ] = setpoint[ i ] - gyro[ i ];
		}
	}

#ifdef PID_ROTATE_ERRORS
	rotateErrors();
#endif

	pid( 0 );
	pid( 1 );
	pid( 2 );

	static unsigned long motors_failsafe_time = 1;
	bool motors_failsafe = false;
	if ( failsafe ) {
		if ( motors_failsafe_time == 0 ) {
			motors_failsafe_time = gettime();
		}
		if ( gettime() - motors_failsafe_time > MOTORS_FAILSAFETIME ) {
			motors_failsafe = true; // MOTORS_FAILSAFETIME after failsafe we turn off the motors.
		}
	} else {
		motors_failsafe_time = 0;
	}

	// Prevent startup if the TX is turned on with the THROTTLE_KILL_SWITCH not activated:
	static bool tx_just_turned_on = true;
	bool prevent_start = false;
	if ( tx_just_turned_on ) {
		if ( ! aux[ THROTTLE_KILL_SWITCH ] ) {
			prevent_start = true;
		} else {
			tx_just_turned_on = false;
		}
	} else {
		// if ( motors_failsafe ) {
		// 	tx_just_turned_on = true;
		// }
	}

	if ( motors_failsafe || aux[ THROTTLE_KILL_SWITCH ] || prevent_start ) {
		onground = 1; // This stops the motors.
		thrsum = 0.0f;
		mixmax = 0.0f;

		static int count;
		if ( count > 0 ) {
			--count;
		} else {
			// Call motorbeep() only every millisecond, otherwise the beeps get slowed down by the ESC.
			count = 500 / LOOPTIME - 1; // So we enter only every 0.5 ms and
			static bool send_beep; // alternate between pwm_set() and motorbeep().
			if ( send_beep ) {
#ifdef MOTOR_BEEPS
	#ifndef MOTOR_BEEPS_CHANNEL
		#define MOTOR_BEEPS_CHANNEL CH_OFF
	#endif
				motorbeep( motors_failsafe, MOTOR_BEEPS_CHANNEL );
#endif // MOTOR_BEEPS
			} else if ( send_motor_values ) {
				for ( int i = 0; i < 4; ++i ) {
					pwm_set( i, 0 );
				}
			}
			send_beep = ! send_beep;
		}
	} else { // motors on - normal flight
		onground = 0;

		float throttle = rxcopy[ 3 ];

#ifdef THROTTLE_TRANSIENT_COMPENSATION_FACTOR
		const float throttle_boost = throttle_hpf( throttle ); // Keep the HPF call in the loop to keep its state updated.
		extern bool lowbatt;
		if ( aux[ RATES ] && ! lowbatt ) {
			throttle += (float)THROTTLE_TRANSIENT_COMPENSATION_FACTOR * throttle_boost;
			if ( throttle < 0.0f ) {
				throttle = 0;
			} else if ( throttle > 1.0f ) {
				throttle = 1.0f;
			}
		}
#endif // THROTTLE_TRANSIENT_COMPENSATION_FACTOR

#ifdef THROTTLE_REVERSING_KICK
		if ( throttle_reversing_kick_sawtooth > 0.0f ) {
			if ( throttle_reversing_kick_sawtooth > throttle_reversing_kick ) {
				throttle = 0.0f;
				pidoutput[ 0 ] = pidoutput[ 1 ] = pidoutput[ 2 ] = 0.0f;
			} else {
				const float transitioning_factor = ( throttle_reversing_kick - throttle_reversing_kick_sawtooth ) / throttle_reversing_kick;
				throttle = throttle_reversing_kick_sawtooth + throttle * transitioning_factor;
				pidoutput[ 0 ] *= transitioning_factor;
				pidoutput[ 1 ] *= transitioning_factor;
				pidoutput[ 2 ] *= transitioning_factor;
			}
			throttle_reversing_kick_sawtooth -= throttle_reversing_kick_decrement;
		}
#endif // THROTTLE_REVERSING_KICK

#ifdef LVC_LOWER_THROTTLE
		static float throttle_i = 0.0f;
		float throttle_p = 0.0f;

		if ( vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW ) {
			throttle_p = ( (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt ) * (float)LVC_LOWER_THROTTLE_KP;
		}
		if ( vbatt_comp < (float)LVC_LOWER_THROTTLE_VOLTAGE ) {
			throttle_p = ( (float)LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp ) * (float)LVC_LOWER_THROTTLE_KP;
		}
		if ( throttle_p > 1.0f ) {
			throttle_p = 1.0f;
		}
		if ( throttle_p > 0 ) {
			throttle_i += throttle_p * 0.0001f; // ki
		} else {
			throttle_i -= 0.001f; // ki on release
		}

		if ( throttle_i > 0.5f ) {
			throttle_i = 0.5f;
		} else if ( throttle_i < 0.0f ) {
			throttle_i = 0.0f;
		}

		throttle -= throttle_p + throttle_i;
		if ( throttle < 0.0f ) {
			throttle = 0.0f;
		}
#endif // LVC_LOWER_THROTTLE

		bb_throttle = throttle;

#ifdef THRUST_LINEARIZATION
		#define AA_motorCurve (float)THRUST_LINEARIZATION // 0 .. linear, 1 .. quadratic
		const float aa = AA_motorCurve;
		throttle = throttle * ( throttle * aa + 1 - aa ); // invert the motor curve correction applied further below
#endif // THRUST_LINEARIZATION

#ifdef ALTERNATIVE_THRUST_LINEARIZATION
		// not really the inverse of ALTERNATIVE_THRUST_LINEARIZATION below, but a good enough approximation
		const float aa = 1.2f * (float)( ALTERNATIVE_THRUST_LINEARIZATION ); // use <1.2 for less compensation
		const float tr = 1.0f - throttle; // throttle reversed
		throttle = throttle / ( 1.0f + tr * tr * aa ); // compensate throttle for the linearization applied further below
#endif // ALTERNATIVE_THRUST_LINEARIZATION

#ifdef INVERT_YAW_PID
		pidoutput[ 2 ] = -pidoutput[ 2 ];
#endif

		float mix[ 4 ];

#ifdef INVERTED_ENABLE
		if ( pwmdir == REVERSE ) { // inverted flight
			mix[ MOTOR_FR - 1 ] = throttle + pidoutput[ ROLL ] + pidoutput[ PITCH ] - pidoutput[ YAW ]; // FR
			mix[ MOTOR_FL - 1 ] = throttle - pidoutput[ ROLL ] + pidoutput[ PITCH ] + pidoutput[ YAW ]; // FL
			mix[ MOTOR_BR - 1 ] = throttle + pidoutput[ ROLL ] - pidoutput[ PITCH ] + pidoutput[ YAW ]; // BR
			mix[ MOTOR_BL - 1 ] = throttle - pidoutput[ ROLL ] - pidoutput[ PITCH ] - pidoutput[ YAW ]; // BL
		} else
#endif // INVERTED_ENABLE
		{ // normal mixer
			mix[ MOTOR_FR - 1 ] = throttle - pidoutput[ ROLL ] - pidoutput[ PITCH ] + pidoutput[ YAW ]; // FR
			mix[ MOTOR_FL - 1 ] = throttle + pidoutput[ ROLL ] - pidoutput[ PITCH ] - pidoutput[ YAW ]; // FL
			mix[ MOTOR_BR - 1 ] = throttle - pidoutput[ ROLL ] + pidoutput[ PITCH ] - pidoutput[ YAW ]; // BR
			mix[ MOTOR_BL - 1 ] = throttle + pidoutput[ ROLL ] + pidoutput[ PITCH ] + pidoutput[ YAW ]; // BL
		}

#ifdef INVERT_YAW_PID
		// we invert again cause it's used by the pid internally (for limit)
		pidoutput[ 2 ] = -pidoutput[ 2 ];
#endif

#ifdef MIX_SCALING
	#ifdef TRANSIENT_MIX_INCREASING_HZ
		float maxSpeedRxcopy = 0.0f;
		static float lastRxcopy[ 4 ];
		for ( int i = 0; i < 4; ++i ) {
			const float absSpeedRxcopy = fabsf( rxcopy[ i ] - lastRxcopy[ i ] ) / LOOPTIME * 1e6f * 0.1f;
			lastRxcopy[ i ] = rxcopy[ i ];
			if ( absSpeedRxcopy > maxSpeedRxcopy ) {
				maxSpeedRxcopy = absSpeedRxcopy;
			}
		}
		static float transientMixIncreaseLimit;
		lpf( &transientMixIncreaseLimit, maxSpeedRxcopy, ALPHACALC( LOOPTIME, 1e6f / (float)( TRANSIENT_MIX_INCREASING_HZ ) ) );
		if ( transientMixIncreaseLimit > 1.0f ) {
			transientMixIncreaseLimit = 1.0f;
		}
	#endif // TRANSIENT_MIX_INCREASING_HZ

		float minMix = 1000.0f;
		float maxMix = -1000.0f;
		for ( int i = 0; i < 4; ++i ) {
			if ( mix[ i ] < minMix ) {
				minMix = mix[ i ];
			}
			if ( mix[ i ] > maxMix ) {
				maxMix = mix[ i ];
			}
		}
		const float mixRange = maxMix - minMix;
		float reduceAmount = 0.0f;
		if ( mixRange > mixRangeLimit ) {
			const float scale = mixRangeLimit / mixRange;
			for ( int i = 0; i < 4; ++i ) {
				mix[ i ] *= scale;
			}
			minMix *= scale;
			reduceAmount = minMix;
		} else {
			if ( maxMix > mixRangeLimit ) {
				reduceAmount = maxMix - mixRangeLimit;
			} else if ( minMix < 0.0f ) {
				reduceAmount = minMix;
			}
		}
	#ifdef ALLOW_MIX_INCREASING
		if ( reduceAmount != 0.0f ) {
	#else
		if ( reduceAmount > 0.0f ) {
	#endif // ALLOW_MIX_INCREASING
	#ifdef TRANSIENT_MIX_INCREASING_HZ
			if ( reduceAmount < -transientMixIncreaseLimit ) {
				reduceAmount = -transientMixIncreaseLimit;
			}
	#endif // TRANSIENT_MIX_INCREASING_HZ
			for ( int i = 0; i < 4; ++i ) {
				mix[ i ] -= reduceAmount;
			}
		}
#endif // MIX_SCALING

		thrsum = 0.0f;
		mixmax = 0.0f;

		for ( int i = 0; i < 4; ++i ) {

#if defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)

			extern int idle_offset;
			static int orig_idle_offset = 0;
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
			if ( orig_idle_offset == 0 ) {
				orig_idle_offset = idle_offset;
			}
			if ( aux[ MOTORS_TO_THROTTLE_MODE ] ) {
#endif
				mix[ i ] = throttle;
				if ( throttle > 0 ) {
					idle_offset = orig_idle_offset;
				}
				if ( ( i == MOTOR_FL - 1 && rxcopy[ ROLL ] < 0 && rxcopy[ PITCH ] > 0 ) ||
					( i == MOTOR_BL - 1 && rxcopy[ ROLL ] < 0 && rxcopy[ PITCH ] < 0 ) ||
					( i == MOTOR_FR - 1 && rxcopy[ ROLL ] > 0 && rxcopy[ PITCH ] > 0 ) ||
					( i == MOTOR_BR - 1 && rxcopy[ ROLL ] > 0 && rxcopy[ PITCH ] < 0 ) )
				{
					idle_offset = 0;
					mix[ i ] = fabsf( rxcopy[ ROLL ] * rxcopy[ PITCH ] );
#ifdef RPM_FILTER
					extern float motor_hz[ 4 ];
					notify_telemetry_value( motor_hz[ i ] );
#endif // RPM_FILTER
				}
				ledcommand = true;
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
			} else {
				idle_offset = orig_idle_offset;
			}
#endif

#endif // defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)

			if ( mix[ i ] < 0.0f ) {
				mix[ i ] = 0.0f;
			} else if ( mix[ i ] > 1.0f )	{
				mix[ i ] = 1.0f;
			}

#ifdef MOTOR_FILTER_A_HZ
			static float mix_filt_a[ 4 ];
			lpf( &mix_filt_a[ i ], mix[ i ], ALPHACALC( LOOPTIME, 1e6f / (float)( MOTOR_FILTER_A_HZ ) ) );
			mix[ i ] = mix_filt_a[ i ];
#endif // MOTOR_FILTER_A_HZ
#ifdef MOTOR_FILTER_B_HZ
			static float mix_filt_b[ 4 ];
			lpf( &mix_filt_b[ i ], mix[ i ], ALPHACALC( LOOPTIME, 1e6f / (float)( MOTOR_FILTER_B_HZ ) ) );
			mix[ i ] = mix_filt_b[ i ];
#endif // MOTOR_FILTER_B_HZ

#ifdef THRUST_LINEARIZATION
			// Computationally quite expensive:
			static float a, a_reci, b, b_sq;
			if ( a != AA_motorCurve ) {
				a = AA_motorCurve;
				if ( a > 0.0f ) {
					a_reci = 1 / a;
					b = ( 1 - a ) / ( 2 * a );
					b_sq = b * b;
				}
			}
			if ( mix[ i ] > 0.0f && a > 0.0f ) {
				mix[ i ] = sqrtf( mix[ i ] * a_reci + b_sq ) - b;
			}
#endif // THRUST_LINEARIZATION

#ifdef ALTERNATIVE_THRUST_LINEARIZATION
			const float mr = 1.0f - mix[ i ]; // mix reversed
			mix[ i ] *= 1.0f + mr * mr * (float)( ALTERNATIVE_THRUST_LINEARIZATION );
#endif // ALTERNATIVE_THRUST_LINEARIZATION

			if ( send_motor_values ) {
				pwm_set( i, mix[ i ] );
			}
			bb_mix[ i ] = mix[ i ];

			thrsum += mix[ i ];
			if ( mixmax < mix[ i ] ) {
				mixmax = mix[ i ];
			}
		}

		thrsum = thrsum / 4.0f;
	} // end motors on
}
