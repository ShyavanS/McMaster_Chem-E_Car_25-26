# McMaster_Chem-E_Car_25-26
The codebase for the McMaster Chem-E Car Team from 2025-2026.

This project uses an Adafruit Feather RP2040 Adalogger microcontroller programmed in the Arduino programming language. The car runs off a chemical reaction and is stopped by a chemical reaction as well. However, the code and the microcontroller are used to start the stopping reaction and stir the mixtures while monitoring it to identify when to stop based on a temperature threshold. It also uses an IMU to keep the car going straight, a servo for starting the stopping reaction, a switching circuit composed of solid-state relays to run the drive motors, and a motor driver for two auxiliary motors for stirring the braking reaction and operating accessories. The car's main drive system will be powered from a hydrogen fuel cell while the remaining auxiliary systems are powered from a smaller Li-Po battery.

The major components used are as follows:<br />
1 - N20 100 RPM Motor (for drive system)<br />
1 - N20 1000 RPM Motor (for reaction stirring mechanism)<br />
3 - SER0039 Servo Motor (for initially mixing reactants & steering)<br />
1 - DRV8847 Motor Driver (for auxiliary motors)<br />
1 - AMT103 Rotary Encoder (for datalogging to tune braking model only)<br />
1 - Adafruit 9-DOF Orientation IMU Fusion Breakout (BNO085 for logging/maintaining heading)<br /> 
1 - Adafruit Feather RP2040 Adalogger Microcontroller<br />
1 - 5000 mAh Li-Po Battery<br />

This repository also contains the custom-made PCB design for the component wiring we are using this year. The design was made using Altium, source files & gerber files are present in this repository.

The main objective of the competition is to design a small car to carry a certain amount of weight a set distance while powering and stopping the vehicle with a chemical reaction. The distance and weight vary each year. The compectition is hosted by the American Institute of Chemical Engineers (AIChE).

Makes use of [pimoroni's encoder-pio driver](https://github.com/pimoroni/pimoroni-pico/tree/encoder-pio/drivers/encoder-pio) to count encoder ticks on the PIOs of the RP2040 to prefvent lost ticks due to higher priority interrupts and increase accuracy while decreasing CPU load. Minor modifications were made to remove debouncing functionality & delays.
