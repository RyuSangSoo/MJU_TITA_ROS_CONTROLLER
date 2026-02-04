#ifndef STATE_HPP
#define STATE_HPP

typedef struct
{
    struct
    {
        struct
        {
            struct
            {
                double joint1 = 0.0;
                double joint2 = 0.0;
                double joint3 = 0.0;
                double joint4 = 0.0;
                
            }Pos, Vel, Effort;

        }right, left;
        
    }Ref, Act;
    
}Tita;

extern Tita tita_state;

#endif // STATE_HPP