//
// Created by gabri on 2023-05-12.
//

#include "../../Inc/init.h"
#include "../../Inc/waves.h"

void init_waves(unsigned int amplitude){
    calcsin(amplitude);
    calcsaw(amplitude);
    calctri(amplitude);
    calcsquare(amplitude);
}
