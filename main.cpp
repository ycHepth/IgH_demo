#include <iostream>
#include <errno.h>
#include "signal.h"
#include "stdio.h"
#include "sys/resource.h"
#include "sys/time.h"
#include "sys/types.h"
#include "unistd.h"
#include "sys/mman.h"

#include "ecrt.h"

#define TASK_FREQUENCY 30 // Hz
#define TARGET_VELOCITY 110000 // cnt per second
#define PROFILE_VELOCITY 3 // operation mode for 0x6060:0

// 1 -- profile position mode
// 3 -- profile velocity mode
// 4 -- Torque profile mode
// 8 -- cyclic sync position mode
// 9 -- cyclic sync velocity mode
// 10-- cyclic sync torque mode

// in profile mode, master does not send new calculated new value at each cycle, instead, the slave computes the
// intermediate position/velocity/torque itself directly at trajectory generator.



int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
