#pragma once

namespace consts {

    // torso
    constexpr double torso_height = 0.65;
    constexpr double torso_radius = 0.06;
    constexpr double torso_mass = 5.0;
    constexpr double torso_w = 7.0;

    // links for arms
    constexpr double link_length = 0.25;
    constexpr double link_radius = 0.025;
    constexpr double link_mass = 1.0;

    // cups
    constexpr double cup_radius = 0.37; // radius from the center of the cup
    constexpr double cup_z = torso_height + 0.10; // height from the ground
    constexpr double ball_cup_offset_z = 0.10;

    // objects
    constexpr int num_arms = 3; // number of arms equally spaced around torso, also number of ball
    constexpr double ball_radius = 0.05;
    constexpr double ball_mass = 0.1;

    // simulation
    constexpr double t_final = 60.0;
    constexpr double dt = 0.01;

    // juggling
    constexpr double hold_time = 2.0; // time to hold ball in cup before throw
    constexpr double catch_tolerance = 0.02; // high
    constexpr int num_rotations = 1;  // num rotations before catch
    constexpr double throw_motion_delta = 0.005; // time to move arm for throw motion, < hold_time/2

    // flight time is around 1.198s (calculated from: (2pi * num_rotations + 2pi/num_arms) / torso_w)
    // we want to space so balls overlap in air
    constexpr double first_hold_time[num_arms] = { // initial hold times for each ball to give juggling look
        0.5,
        0.9,
        1.3
    };

    // PID gains
    constexpr double Kp = 40.0;
    constexpr double Ki = 0.0;
    constexpr double Kd = 2.0;
}

