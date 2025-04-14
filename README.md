This firmware is designed to run on a Raspberry Pi Pico and uses the MBed 
  RTos. It's written for a custom HRV/air exchanger which has 4 different control 
  axes: 
  - Intake, which applies a pressure gradient from an outside air intake, 
  across the intake side of the core assembly, to an inside vent. 
  - Exhaust, which applies a pressure gradient from the inside air across the 
  exhaust side of the core assembly, to an outside vent. 
  - Bypass, which applies a pressure gradient around the intake side of the core,
  bypassing it, bringing outside air directly inside without heat exchange
  - Core Assist, which increases air velocity through the intake side of the 
  core assembly in the direction of intake, and also applies a reverse flow 
  on the bypass axis, which must not exceed the intake pressure gradient. 

  Each axis has an "inlet" and "outlet" side, representing the source and 
  sink directions from the core. 

  Each axis has one or more PWM fans, Noctua Industrial are recommended, 
  controlled by the PWM channels via MOSFETS, and at least one mains powered 
  fan or "blower", which is engaged on the highest PWM level. Intake and Exhaust 
  blowers are centrifugal turbines, while bypass and core assist are either 
  axial turbines or large 120V server fans. 

  4 temperature sensors are used: 
  - Exhaust inlet, which is located anywhere on the room side of the exhaust 
  air flow path, directly in the air flow. 
  - Exhaust outlet, which is located very close to the outlet of the heat exchanger, 
  OR ideally, against the outlet side of the core. This temperature should 
  closely represent the actual core temperature. 
  - Intake inlet, which will be very close to the vent which admits outside air to the 
  system. 
  - Intake outlet, which should be somewhere in the air stream leaving the intake
  side, after the bypass air has been re-introduced. It should give an accurate 
  reading of the resulting temperature after the core air and bypass air are mixed, 
  which informs the ratio of bypass to core assist used to achieve the target 
  temperature.
