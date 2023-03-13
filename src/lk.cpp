#include <iostream>
#include "../include/engine.h"
#include "../include/flow_field.h"
#include "../include/particle.h"

using namespace std;



struct frame_counter {
    public:
        int frame_counter { 0 };
        int64_t start_tick { getTickCount() };
    
    void print_fps () {
        if (frame_counter == 30) {   // Calculate and print FPS every 30 frames
            int64_t end_tick = getTickCount();
            double fps = frame_counter / ((end_tick - start_tick) / getTickFrequency());
            cout << "FPS: " << fps << endl;

            // Reset counters
            start_tick = end_tick;
            frame_counter = 0;
        }
    }
};



int main () {
    Engine en;
    
    if (en.open_camera() == 0) {

        en.get_current_frame();
        en.initialize_lk_arrays();
        en.initialize_kernels();

        unordered_map<int, vector<int>> bubble_hash;

        // initialize particle properties
        Particle bubbles[NUM_OF_BUBBLES];        
        FlowField ff;
        ff.initialize_particles(
            bubbles, 
            bubble_hash, 
            en.current_frame_color.rows,
            en.current_frame_color.cols
        );

        frame_counter fc;

        while (1) {
            // print frames per second
            fc.frame_counter++;
            fc.print_fps();


            // compute optial flow vectors
            en.get_current_frame();



            en.compute_t_gradient();
            en.compute_x_gradient();
            en.compute_y_gradient();


            en.lk_hash(
                bubbles,
                bubble_hash, 
                &ff
            );


            en.draw_particles(bubbles);
            char key_press;
            key_press = en.display_image("Bubble Bender", en.current_frame_color, 0);


            if (key_press==27) {
                en.destroy_all_windows();
                en.release_cap();
                break;
            }
            en.store_previous_frame();
        }
    }
}