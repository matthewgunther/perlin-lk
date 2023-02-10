#include <iostream>
#include "../include/engine.h"
#include "../include/flow_field.h"
#include "../include/particle.h"

using namespace std;

int main () {
    Engine en;

    if (en.open_camera() == 0) {

        int num_of_bubbles { 1000 };
        Particle bubbles[num_of_bubbles];

        FlowField ff;
        ff.initialize_particles(bubbles, num_of_bubbles);

        while (1) {
            char key_press;
            float downsample_scale { 10 };
            int flip_image { 1 };
            int lk_window_dim { 10 };
            float flow_threshold { 10 };

            int perlin_arr_scale { 2 };
            float x_scalar { 0.1 };
            float y_scalar { 0.1 };
            float z_delta { 0.01 };

            

            en.get_current_frame(flip_image, downsample_scale);
            en.compute_t_gradient();
            en.compute_x_gradient();
            en.compute_y_gradient();
            en.compute_lk_flow(lk_window_dim);


            en.push_particles(
                bubbles,
                num_of_bubbles,
                downsample_scale,
                flow_threshold
            );


            ff.move_particles(
                bubbles,
                num_of_bubbles,
                en.current_frame_color.rows,
                en.current_frame_color.cols,
                perlin_arr_scale,
                x_scalar,
                y_scalar,
                z_delta,
                downsample_scale
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
