#pragma once

typedef struct
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

    }Left, Right;

    double roll;
    double pitch;
    double yaw;

} TitaState;

extern TitaState tita_state;
