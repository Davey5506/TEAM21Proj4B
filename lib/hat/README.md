# Hat Controller
Hat Contoller is an abstraction layer intended to be used with PlatformIO and the CMSIS framework for interfacing between the Nucleo-F446RE and a Hat designed for the University of Delaware's course on Microprocessor Systems.
## Features of the Hat
The Hat that is this project is designed around contains several integrated circuits and modules that are connect to the Nucleo-F446RE through its morpho header pins.<br>
This library is able to:
- Control GPIO ports and pins.
- Control internal timers.
- Control a Seven Segment Display.
- Control continous and positional servos.
## Installation
To add Hat Controller to a PlatformIO project, create a folder in the lib directory and name it 'hat'. Download this repository and copy the files into that folder you created, as shown below.
```
|--MyProject
|  |
|  |--.pio
|  |--.vscode
|  |--include
|  |--lib
|  |  |--hat  <--COPY HERE
|  |
|  |--src
|  |--test
|
```
## Usage
To add the library to you source code add the line `#include "hat.h"` to the top of your main files as follows:
```
#include "hat.h"

int main(void){
  return 0;
}
```
## License
[AGPL-3.0 license](/LICENSE)