# McMaster_Chem-E_Car_24-25
The codebase for the McMaster Chem-E Car Team from 2024-2025.

This project uses an Adafruit Feather RP2040 Adalogger microcontroller programmed in the Arduino programming language. The car runs off a chemical reaction and is stopped by a chemical reaction as well. However, the code and the microcontroller are used to start the stopping reaction and stir the mixtures while monitoring it to identify when to stop based on a temperature threshold. It also uses an IMU to keep the car going straight, a servo for starting the stopping reaction, two motor drivers to run the drive motors, and two motors for stirring the reactions. The car's main drive system will be powered from a custom-built alkaline battery while the remaining auxiliary systems are powered from a smaller Li-Po battery.

The major components used are as follows:<br />
2 - N20 100 RPM Motors (for drive system)<br />
2 - N20 1000 RPM Motor (for reaction stirring mechanisms)<br />
4 - SER0039 Servo Motor (for initially mixing reactants & steering)<br />
2 - DRV8847 Motor Drivers (only used as switches as per regulations)<br />
2 - AMT102 Rotary Encoders (for datalogging to tune braking model only)<br />
1 - DS18B20 Temperature Sensor (for measuring temperature of braking reaction)<br />
1 - Adafruit 9-DOF Orientation IMU Fusion Breakout (BNO085 for logging/maintaining heading)<br /> 
1 - Adafruit Feather RP2040 Adalogger Microcontroller<br />
1 - 5000 mAh Li-Po Battery<br />

This repository also contains the custom-made PCB design for the component wiring we are using this year. The design was made using Altium, source files & gerber files are present in this repository.

The main objective of the competition is to design a small car to carry a certain amount of weight a set distance while powering and stopping the vehicle with a chemical reaction. The distance and weight vary each year. The compectition is hosted by the American Institute of Chemical Engineers (AIChE).

Makes use of [pimoroni's encoder-pio driver](https://github.com/pimoroni/pimoroni-pico/tree/encoder-pio/drivers/encoder-pio) to count encoder ticks on the PIOs of the RP2040 to prefvent lost ticks due to higher priority interrupts and increase accuracy while decreasing CPU load. Minor modifications were made to remove debouncing functionality & delays.
