# Getting Started
[Homepage](https://www.imsl.fh-dortmund.de/mobile-roboter/edurob/)
## Initial Setup for Version 2

- Install Visual Studio Code: https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/#1
- Install PlatformIO extension: https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/#2
- Clone the Repository and open the "Edurob" folder with PlatformIO
- Depending on your hardware configuration (Mecanum, Differential drive, 3-Wheel Omni or 4-Wheel Omni) you have to select the corresponding software configuration by uncommenting **only one** of the following statements within the "parameter.h"

    `#define MECANUM`
    
    `#define DIFF`

    `#define OMNI4`

    `#define OMNI3`


## Next Steps
- Add your user code in the corresponding segment inside the loop-function
  
  `//#############-USER-CODE-START-#####################`
  
  `double tx = 0.0, ty = 0.0, theta = 0.0;`
  
  `robotSpeedSetpoint << tx, ty, theta;`

  `//#############-USER-CODE-END-#####################`
- Set the robot speed by changing the values inside the "robotSpeedSetpoint"-Vector