#include <iostream>
#include "../include/engine.h"
#include "../include/flow_field.h"
#include "../include/particle.h"

using namespace std;

// define macros for variables that stay constant
#define DOWNSAMPLE_SCALE 10
#define FLIP_IMAGE 1
#define LK_WINDOW_DIM 10

#define PERLIN_ARR_SCALE 2
#define X_SCALAR 0.1
#define Y_SCALAR 0.1
#define Z_DELTA 0.01

#define NUM_OF_BUBBLES 1000




int main () {
    Engine en;

    if (en.open_camera() == 0) {

        Particle bubbles[NUM_OF_BUBBLES];        
        FlowField ff;
        ff.initialize_particles(bubbles, NUM_OF_BUBBLES);

        int64_t start_tick = getTickCount();
        int frame_counter = 0;

        while (1) {
            
            // compute optial flow vectors
            en.get_current_frame(FLIP_IMAGE, DOWNSAMPLE_SCALE);
            en.compute_t_gradient();
            en.compute_x_gradient();
            en.compute_y_gradient();
            en.compute_lk_flow(LK_WINDOW_DIM);
            
            
            en.push_particles(
                bubbles,
                NUM_OF_BUBBLES,
                DOWNSAMPLE_SCALE
            );


            ff.move_particles(
                bubbles,
                NUM_OF_BUBBLES,
                en.current_frame_color.rows,
                en.current_frame_color.cols,
                PERLIN_ARR_SCALE,
                X_SCALAR,
                Y_SCALAR,
                Z_DELTA,
                DOWNSAMPLE_SCALE
            );



            en.draw_particles(bubbles, NUM_OF_BUBBLES);

            frame_counter++;
            if (frame_counter == 30) {   // Calculate and print FPS every 30 frames
                int64_t end_tick = getTickCount();
                double fps = frame_counter / ((end_tick - start_tick) / getTickFrequency());
                cout << "FPS: " << fps << endl;

                // Reset counters
                start_tick = end_tick;
                frame_counter = 0;
            }


            char key_press;
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