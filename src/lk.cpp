#include <iostream>
#include "../include/engine.h"
#include "../include/flow_field.h"
#include "../include/particle.h"

using namespace std;

// define macros to be used across modules
// else macro should be defined in module header
#define DOWNSAMPLE_SCALE 10
#define NUM_OF_BUBBLES 1000



int main () {
    Engine en;
    
    if (en.open_camera() == 0) {

        en.get_current_frame(DOWNSAMPLE_SCALE);
        en.initialize_lk_arrays(DOWNSAMPLE_SCALE);
        en.initialize_kernels();


        // initialize particle properties
        Particle bubbles[NUM_OF_BUBBLES];        
        FlowField ff;
        ff.initialize_particles(bubbles, NUM_OF_BUBBLES);

        int64_t start_tick = getTickCount();
        int frame_counter = 0;

        while (1) {
            
            frame_counter++;
            if (frame_counter == 15) {   // Calculate and print FPS every 30 frames
                int64_t end_tick = getTickCount();
                double fps = frame_counter / ((end_tick - start_tick) / getTickFrequency());
                cout << "FPS: " << fps << endl;

                // Reset counters
                start_tick = end_tick;
                frame_counter = 0;
            }
            
            // compute optial flow vectors
            en.get_current_frame(DOWNSAMPLE_SCALE);
            en.compute_t_gradient();
            en.compute_x_gradient();
            en.compute_y_gradient();
            en.compute_lk_flow();

            en.visualize_lk_flow();
            
            // add optical flow acceleration to particles
            en.push_particles(
                bubbles,
                NUM_OF_BUBBLES,
                DOWNSAMPLE_SCALE
            );

            // move particles according to perlin noise flow field
            ff.move_particles(
                bubbles,
                NUM_OF_BUBBLES,
                &en.current_frame_color.rows,
                &en.current_frame_color.cols,
                DOWNSAMPLE_SCALE
            );

            en.draw_particles(bubbles, NUM_OF_BUBBLES);

            char key_press;
            key_press = en.display_image("Bubble Bender", en.current_frame_color);
            if (key_press==27) {
                en.destroy_all_windows();
                en.release_cap();
                break;
            }
            en.store_previous_frame();
        }
    }
}