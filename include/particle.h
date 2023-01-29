#ifndef PARTICLE_H
#define PARTICLE_H



class Particle {
    // private:
    //     float acc_x;
    //     float acc_y;

    public:
        // float pos_x;
        // float pos_y;
        // float vel_x;
        // float vel_y;


        struct vec {
            float x, y;
            float magnitude_limit {3.0e+038 };
            
            void add_vec(float x_to_add, float y_to_add) {
                x += x_to_add;
                y += y_to_add;
            }

            void update (vec vector, float timestep) {
                x += vector.x * timestep;
                y += vector.y * timestep;
            }

        } pos, vel, acc;

        void initialize_vectors(
            float pos_x, float pos_y,
            float vel_x, float vel_y,
            float acc_x, float acc_y
        );
        void update(float timestep, int window_width, int window_height);

        Particle() {}
        //     float pos_x, 
        //     float pos_y,
        //     float vel_x,
        //     float vel_y,
        //     float acc_x,
        //     float acc_y
        // ) :
        //     pos_x(pos_x),
        //     pos_y(pos_y),
        //     vel_x(vel_x),
        //     vel_y(vel_y),
        //     acc_x(acc_x),
        //     acc_y(acc_y)
        // {}
    
};

#endif