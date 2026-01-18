# PID Temperature Controller - Documentation


## Overview

The system now uses a complete PID controller (Proportional-Integral-Derivative) for hot-end temperature control. The old simple on-off controller has been replaced with a professional PID controller with auto-tuning function.

## Functions

### 1. PID Controller

The PID controller offers:
- **P component (Proportional)**: Responds directly to the deviation from target temperature
- **I component (Integral)**: Eliminates steady-state control errors
- **D component (Derivative)**: Dampens overshoot and improves stability
- **Anti-Windup**: Prevents integral saturation
- **Safety limits**: Automatic shutdown on over/undershoot

### 2. Auto-Tuning (Ziegler-Nichols Relay Method)

The system can automatically determine the optimal PID parameters for a given target temperature.

#### How it works:
1. The system generates an oscillation through relay control (on/off)
2. Measures the amplitude and period of oscillation
3. Calculates optimal Kp, Ki, Kd values according to Ziegler-Nichols
4. Saves the parameters automatically

## Usage

### Performing Auto-Tuning

To optimize the PID parameters for a specific temperature:

1. **Important**: Make sure NO measurement is running
2. Send the following command via serial interface:
   ```
   AUTOTUNE:<Temperature>
   ```
   Example for 200°C:
   ```
   AUTOTUNE:200
   ```

3. The process takes several minutes and provides continuous status messages
4. At the end, the optimized parameters are displayed:
   ```
   === New PID Parameters ===
   Kp: 8.5234
   Ki: 0.4123
   Kd: 2.1567
   === Auto-Tuning completed ===
   ```

### Setting PID Parameters Manually

If you want to use custom parameters:

```
SETPID:<Kp>,<Ki>,<Kd>
```

Example:
```
SETPID:10.0,0.5,2.0
```

### Querying Current PID Parameters

```
GETPID
```

Output:
```
=== Current PID Parameters ===
Kp: 8.0000
Ki: 0.5000
Kd: 2.0000
```

## Default Parameters

The default PID parameters at startup are:
- **Kp** = 8.0
- **Ki** = 0.5
- **Kd** = 2.0

These should already provide good results for most applications but can be optimized through auto-tuning.

## Tips for Best Results

1. **Auto-Tuning at operating temperature**: Perform auto-tuning at the temperature you use most frequently
2. **Environmental conditions**: Perform auto-tuning under similar conditions as your measurements (e.g., same fan setting)
3. **Patience**: Auto-tuning takes time (5-10 minutes) to deliver accurate results
4. **Repetition**: For very different temperature ranges (e.g., 180°C vs. 250°C), it may make sense to perform separate tunings

## Error Handling

### Auto-Tuning Fails
- Check if target temperature is in valid range (50-290°C)
- Make sure no measurement is running
- Check heater element and NTC wiring

### Temperature Oscillates Strongly
- Perform auto-tuning
- Reduce D component manually if necessary

### Temperature Doesn't Reach Setpoint
- Increase I component slightly
- Check heater power

### Overshoot During Heating
- Increase D component
- Reduce P component slightly

## Debug Output

During operation, the PID controller continuously outputs telemetry data:
```
>Temp:199.5
>Setpoint:200.0
>PWM:150
>Power:23.5
```

These can be visualized with the Arduino IDE Serial Plotter.

## Technical Details

- **Update rate**: 100ms (HEATER_DELAY)
- **Temperature range**: 0-290°C
- **PWM range**: 0-255
- **Anti-Windup**: Yes, with dynamic limiting
- **Safety shutdown**: At >290°C or <0°C
