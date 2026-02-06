#pragma once

#include <mutex>
#include <cstdint>

typedef struct
{
    struct
    {
        struct
        {
            struct
            {
                double joint1;
                double joint2;
                double joint3;
                double Wheel;

            }Pos, Vel, Effort;
            
        }Real, Command;

    }Left, Right;

    double roll;
    double pitch;
    double yaw;

    struct
    {
        double target_j1;
        double target_j2;
        double target_j3;
        double duration_sec;
        double kp_j1;
        double kp_j2;
        double kp_j3;
        double kd_j1;
        double kd_j2;
        double kd_j3;
        std::uint64_t seq;
        std::uint64_t stop_seq;
    }Control;

} TitaState;

extern TitaState tita_state;
extern std::mutex tita_state_mutex;
