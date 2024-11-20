## Trinamic stepper driver plugin

This plugin adds settings, M-Code extensions and reports for TMC2130, TMC2209, TMC2660 and TMC5160 stepper drivers.

### M-codes

Some Marlin-style M-codes are supported: [M122](https://marlinfw.org/docs/gcode/M122.html), [M569](https://marlinfw.org/docs/gcode/M569.html), [M911](https://marlinfw.org/docs/gcode/M911.html),
 [M912](https://marlinfw.org/docs/gcode/M912.html), [M913](https://marlinfw.org/docs/gcode/M913.html), [M914](https://marlinfw.org/docs/gcode/M914.html)
 and [M919](https://marlinfw.org/docs/gcode/M919.html) - some with extensions and some with a sligthly different syntax.

#### M122 - output debug info or reset driver.

`M122 axisletters <H-> <S-> <Q-> <I>`

* `H` - 0 = SFILT off, 1 = SFILT on \(TMC2130\) only.
* `I` - reinitialize driver.
* `S` - 0 = disable StallGuard and live output of sg-value, 1 = enable StallGuard and live output. 
* `Q` - not yet enabled.

Examples:  
`M122` - output debug info for all enabled drivers. This is a plain text report, do not issue from a sender!  
`M122 Y` - output debug info for Y axis only. This is a plain text report, do not issue from a sender!  
`M122 I` - reset drivers.  
`M122 X S1` - enable live output of StallGuard value for tuning. Do not enable when running g-code jobs!  

#### M569 - Set chopper timing.

`M569 axisletters <S->`

Toggle between, or switch to, _StealthChop&trade;_ and/or _SpreadCycle&trade;_ mode. Available for TMC2130, TMC2209 and TMC5160 drivers.

* `S` - 0 = disable StealthChop, 1 = enable StealthChop.

Examples:  
`M569 X S1` - Set X-axis mode to _StealthChop_.  
`M569 X` - Toggle X-axis mode.  
`M569 XY S1` - Set X- and Y-axis mode to _StealthChop_.  
`M569` - output the current configuration.

#### M906 - set stepper current

`M906 axiswords`

Example:  
`M122 X700 Y950` - set stepper current for X-motors to 700mA RMS and Y-motors to 950mA.

> [!NOTE]
> Stepper current is not permanently stored.

#### M911 - Report prewarn flags

`M911`  

#### M912 - Clear prewarn flags

`M912`

#### M913 - Set hybrid threshold

`M913 axiswords`

Example:  
`M913 X31`

> [!NOTE]
> Hybrid threshold is not permanently stored.

 #### M914 - Set homing sensitivity.

`M914 axiswords`

Example:  
`M914 X31`

> [!NOTE]
> Homing sensitivity is not permanently stored.

#### M919 - Set chopper timing.

`M919 axisletters <O-> <P-> <S->`

* `O` - Time-off \(toff: 1..15\) value, if omitted reset to default value.
* `P` - Hysteresis end \(hend: -3..12\) value, if omitted reset to default value.
* `S` - Hysteresis start \(hstart: 1..8\) value, if omitted reset to default value.

Examples:  
`M919 X O4` - Set X-axis time-off to 4.  
`M919` - output the current configuration.

> [!NOTE]
> _hstart_ + _hend_ must be &le; 15.

> [!NOTE]
> Currently default values are common for all axes and changing the configuration via $-settings will overwrite any values set by M919.

> [!NOTE]
> Chopper timings are not permanently stored.


### Settings

Settings are provided for axis enable, homing, stepper current, microsteps and sensorless homing. More to follow.

#### $14x - Stepper current. x = 0 for X-axis, 1 for Y axis etc. 

#### $15x - Motor microsteps. x = 0 for X-axis, 1 for Y axis etc.

Valid values are 1, 2, 4, , 8, 16, 32, 64, 128 and 256.

#### $20x - StallGuard sensitivity, fast \(locate\) threshold. x = 0 for X-axis, 1 for Y axis etc.

Range is -64 to 63 for Stallguard2 \(TMC2130, TMC5160\), 0 - 255 for StallGuard4 \(TMC2209\).

#### $21x - Hold current as percentage of stepper current. x = 0 for X-axis, 1 for Y axis etc.

#### $22x - StallGuard sensitivity, slow \(seek\) threshold. x = 0 for X-axis, 1 for Y axis etc.

Range is -64 to 63 for Stallguard2 \(TMC2130, TMC5160\), 0 - 255 for StallGuard4 \(TMC2209\).

#### $338 - Driver enable

Parameter is a axismask where value is sum of X=1, Y=2, Z=4 etc.

> [!NOTE]
> Some boards does not allow mixed drivers, for these this setting is not available.

#### $339 - Sensorless homing enable

Parameter is a axismask where value is sum of X=1, Y=2, Z=4 etc.

Sensorless homing requires tuning of parameters, the datasheet has information about how to:

[TMC2130](https://www.analog.com/en/products/tmc2130.html)  
[TMC2209](https://www.analog.com/en/products/tmc2209.html)  
[TMC2660](https://www.analog.com/en/products/tmc2660.html)  
[TMC5160](https://www.analog.com/en/products/tmc5160.html)  

Some parameters are [hard coded](https://github.com/grblHAL/Plugins_motor/blob/master/trinamic.h) and require recompilation/reflashing to change.

Note that tuning is not trivial, for one-off machines it is likely that endstop switches will be the best/easiest option to implement.

### Advanced settings

It is possible to enable settings for some chopper mode and CoolStep parameters by adding/uncommenting this line in _my_machine.h_ and recompiling:

`#define TRINAMIC_EXTENDED_SETTINGS 1`

#### $651 - Chopper off time \(toff\).

Range is 1 to 15.

#### $652 - Chopper blanking time \(tbl\).

Range is 0 to 3.

#### $653 - Chopper mode \(chm\).

0 - Spreadcycle, 1 - Constant toff.

#### $654 - Chopper hysteresis start \(hstrt\).

Range is 1 to 8.

#### $655 - Chopper hysteresis end \(hend\).

Range is -3 to 12.

#### $656 - Chopper hysteresis interval decrement \(hdec\).

Range is 0 to 3.

#### $657 - Chopper random time off enable \(hdec\).

0 - fixed, 1 - random.

#### $659 - Lower CoolStep threshold \(semin\).

0 - CoolStep off, 1 - 15 threshold range.

#### $660 - CoolStep current increment \(seup\).

Range is 0 to 3.

#### $661 - Upper CoolStep threshold \(semax\).

Range is 0 to 15.

#### $662 - CoolStep sample count \(sedn\).

Range is 0 to 3.

#### $663 - CoolStep current factor \(seimin\).

0 - &frac12;, 1 - &frac14;.

> [!NOTE]
> These settings will be applied to all drivers, use M919 if you want to change some chopper parameters per driver.  
> If this option is enabled or disabled _all_ plugin settings will be reset to default values.

> [!NOTE]
> For a detailed description of what these settings applies to please refer to the relevant driver datasheet linked above.

---

The driver and driver configuration has to be extended to support these plugins.

Dependencies:

[Trinamic library](https://github.com/terjeio/Trinamic-library)

[Trinamic TMC2130 I2C<>SPI Bridge](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) \(optional\)

---
2024-11-18
