#ifndef PARTICLE_H
#define PARTICLE_H

class Particle {
    private:
        float acc_x;
        float acc_y;

    public:
        float pos_x;
        float pos_y;
        float vel_x;
        float vel_y;

        void set_pos(float x, float y);
        void set_vel(float x, float y);
        void update(float timestep, int window_width, int window_height);

        Particle(
            float pos_x, 
            float pos_y,
            float vel_x,
            float vel_y,
            float acc_x,
            float acc_y
        ) :
            pos_x(pos_x),
            pos_y(pos_y),
            vel_x(vel_x),
            vel_y(vel_y),
            acc_x(acc_x),
            acc_y(acc_y)
        {}
    
};

#endif