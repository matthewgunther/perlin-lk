#include <iostream>
#include "../include/engine.h"
#include "../include/flow_field.h"
#include "../include/particle.h"

using namespace std;

// define macros to be used across modules
// else macro should be defined in module header





int main () {
    Engine en;
    
    if (en.open_camera() == 0) {

        en.get_current_frame(DOWNSAMPLE_SCALE);
        en.initialize_lk_arrays(DOWNSAMPLE_SCALE);
        en.initialize_kernels();


        unordered_map<int, vector<int>> bubble_hash;

        // initialize particle properties
        Particle bubbles[NUM_OF_BUBBLES];        
        FlowField ff;
        ff.initialize_particles(
            bubbles, 
            NUM_OF_BUBBLES, 
            bubble_hash, 
            en.current_frame_color.rows,
            en.current_frame_color.cols
        );

        int64_t start_tick = getTickCount();
        int frame_counter = 0;






    //     // to view hashmap
    //     for (const auto& p : bubble_hash) {
    //         cout << "Key: " << p.first << ", Value: ";
    //         for (const auto& value : p.second) {
    //             cout << value << " ";
    //         }
    //     cout << endl;
    //     }



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
            // en.compute_lk_flow();





            // const int& rows_c = en.current_frame_float.rows;
            // const int& cols_c = en.current_frame_float.cols;
            // double na[rows_c][cols_c];

            en.lk_hash(
                bubbles,
                NUM_OF_BUBBLES,
                bubble_hash, 
                en.current_frame_color.rows,
                en.current_frame_color.cols,
                DOWNSAMPLE_SCALE,
                &ff
            );





            // compute lk and perlin flow
            // en.image_operations(bubbles, NUM_OF_BUBBLES);

            
            // add optical flow acceleration to particles
            // en.push_particles(
            //     bubbles,
            //     NUM_OF_BUBBLES,
            //     DOWNSAMPLE_SCALE
            // );

            // move particles according to perlin noise flow field
            // ff.move_particles(
            //     bubbles,
            //     NUM_OF_BUBBLES,
            //     &en.current_frame_color.rows,
            //     &en.current_frame_color.cols,
            //     DOWNSAMPLE_SCALE
            // );

            en.draw_particles(bubbles, NUM_OF_BUBBLES);
            char key_press;
            key_press = en.display_image("Bubble Bender", en.current_frame_color);
            // en.visualize_lk_flow();
            // key_press = en.display_image("Flow", en.flow, DOWNSAMPLE_SCALE);
            if (key_press==27) {
                en.destroy_all_windows();
                en.release_cap();
                break;
            }
            en.store_previous_frame();
        }
    }
}