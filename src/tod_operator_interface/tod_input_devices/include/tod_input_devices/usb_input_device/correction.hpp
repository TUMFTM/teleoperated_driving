/**
 * @file correction.hpp
 * @brief USB input device calibaration
 * @copyright Modified from https://github.com/flosse/linuxconsole/blob/master/utils/jscal.c
 **/

#pragma once

#include <cstring>
#include <iostream>
#include <unistd.h>
#include <linux/joystick.h>

namespace Correction{

inline bool set_correction(const char *p, int _js) {
    struct js_corr corr[ABS_MAX + 1];
    const char corr_coef_num[] = {0, 4};

    int i, j;
    int t = 0;



    char axes;
    if (ioctl(_js, JSIOCGAXES, &axes) < 0) {
        perror("UsbInputDevice::set_correction: error getting axes");
        return false;
    }
    if (axes > ABS_MAX + 1) {
        axes = ABS_MAX + 1;
    }
    if (!p) {
        fprintf(stderr, "UsbInputDevice::set_correction: missing number of axes\n");
        return false;
    }

    sscanf(p, "%d", &t);
    p = strstr(p, ",");

    if (t != axes) {
        std::cout << "UsbInputDevice::set_correction: joystick has different number of axes (" << int(axes)
            << ") than specified in command line: " << t << std::endl;
        return false;
    }
    for (i = 0; i < axes; i++) {
        if (!p) {
            fprintf(stderr, "UsbInputDevice::set_correction: missing correction type for axis %d\n", i);
            return false;
        }

        sscanf(++p, "%d", &t);
        p = strstr(p, ",");

        if (t != 1) {
            // correction is not broken line correction
            return false;
        }

        corr[i].type = t;

        if (!p) {
            fprintf(stderr, "UsbInputDevice::set_correction: missing precision for axis %d\n", i);
            return false;
        }

        sscanf(++p, "%d", &t);
        p = strstr(p, ",");

        corr[i].prec = t;

        for (j = 0; j < corr_coef_num[corr[i].type]; j++) {
            if (!p) {
                fprintf(stderr, "UsbInputDevice::set_correction: missing coefficient %d for axis %d\n", j, i);
                return false;
            }
            sscanf(++p, "%d", (int *)&corr[i].coef[j]);
            p = strstr(p, ",");
        }
    }

    if (p) {
        fprintf(stderr, "UsbInputDevice::set_correction: too many values\n");
        return false;
    }

    if (ioctl(_js, JSIOCSCORR, &corr) < 0) {
        perror("UsbInputDevice::set_correction: error setting correction");
        return false;
    }
    return true;
}
}; // namespace Correction
