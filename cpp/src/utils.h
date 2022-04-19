//
// Created by branislav on 17.04.2022.
//

#ifndef CPP_UTILS_UTILS_H
#define CPP_UTILS_UTILS_H

#include <iostream>
std::vector<int32_t> laser_angles_vlp16{-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

uint32_t
laser_id_to_color(int32_t laser_id) {
    auto color = (float_t) laser_id / laser_angles_vlp16.size();
    color = (uint32_t) (color * 255.0);
    return color;
}

uint8_t
angle_to_color(int32_t label) {
    auto angle = (int32_t) label;
    auto itr = std::find(laser_angles_vlp16.begin(), laser_angles_vlp16.end(), angle);
    if (itr != laser_angles_vlp16.cend()) {
        int32_t idx = std::distance(laser_angles_vlp16.begin(), itr);
        return laser_id_to_color(idx);
    }
    else {
        std::cout << "Element not found";
    }
}
#endif //CPP_UTILS_UTILS_H
