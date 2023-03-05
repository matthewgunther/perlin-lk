#include <iostream>
#include "../include/engine.h"
#include "../include/flow_field.h"
#include "../include/particle.h"

using namespace std;


// define macros for variables that stay constant
#define DOWNSAMPLE_SCALE 10
#define FLIP_IMAGE 1
#define LK_WINDOW_DIM 10
#define FLOW_THRESHOLD 10

#define PERLIN_ARR_SCALE 2
#define X_SCALAR 0.1
#define Y_SCALAR 0.1
#define Z_DELTA 0.01

int main () {
    Engine en;

    if (en.open_camera() == 0) {

        int num_of_bubbles { 1000 };
        Particle bubbles[num_of_bubbles];

        FlowField ff;
        ff.initialize_particles(bubbles, num_of_bubbles);

        while (1) {
            char key_press;
            // move these outside the while loop
            // keep as variables or make macro #defines at top of file
            // float downsample_scale { 10 };
            // int flip_image { 1 };
            // int lk_window_dim { 10 };
            // float flow_threshold { 10 };

            // int perlin_arr_scale { 2 };
            // float x_scalar { 0.1 };
            // float y_scalar { 0.1 };
            // float z_delta { 0.01 };

        
            en.get_current_frame(FLIP_IMAGE, DOWNSAMPLE_SCALE);
            en.compute_t_gradient();
            en.compute_x_gradient();
            en.compute_y_gradient();
            en.compute_lk_flow(LK_WINDOW_DIM);


            en.push_particles(
                bubbles,
                num_of_bubbles,
                DOWNSAMPLE_SCALE,
                FLOW_THRESHOLD
            );


            ff.move_particles(
                bubbles,
                num_of_bubbles,
                en.current_frame_color.rows,
                en.current_frame_color.cols,
                PERLIN_ARR_SCALE,
                X_SCALAR,
                Y_SCALAR,
                Z_DELTA,
                DOWNSAMPLE_SCALE
            );


            en.draw_particles(bubbles, num_of_bubbles);

            key_press = en.display_image("c", en.current_frame_color);
            if (key_press==27) {
                en.destroy_all_windows();
                en.release_cap();
                break;
            }
            en.store_previous_frame();
        }
    }
}
