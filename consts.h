#pragma once

namespace consts {

    // torso
    constexpr double torso_height = 0.5;
    constexpr double torso_radius = 0.05;
    constexpr double torso_mass = 5.0;
    constexpr double torso_w = 8.0;

    // links for arms
    constexpr double link_length = 0.25;
    constexpr double link_radius = 0.03;
    constexpr double link_mass = 1.0;

    // cups
    constexpr double cup_radius = 0.37; // radius from the center of the cup
    constexpr double cup_z = torso_height + 0.10; // height from the ground
    constexpr double ball_cup_offset_z = 0.10;

    // objects
    constexpr double ball_radius = 0.04;
    constexpr double ball_mass = 0.1;

    // simulation
    constexpr double t_final = 20.0;
    constexpr double dt = 0.01;

    // juggling
    constexpr double hold_time = 2.0; // time to hold ball in cup before throw
    constexpr double catch_tolerance = 0.02; // high
    constexpr int num_rotations = 1;  // num rotations before catch

    // PID gains
    constexpr double Kp = 40.0;
    constexpr double Ki = 0.0;
    constexpr double Kd = 2.0;
}

