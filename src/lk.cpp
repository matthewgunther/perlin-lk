#include <iostream>
#include "../include/engine.h"

using namespace std;

int main () {
    Engine en;
    if (en.open_camera() == 0) {
        while (1) {
            char key_press;
            float downsample_scale { 10 };
            int flip_image { 1 };
            int lk_window_dim { 10 };

            en.get_current_frame(flip_image, downsample_scale);
            // key_press = en.display_image("color", en.current_frame_color);
            // key_press = en.display_image("bw", en.current_frame_float);


            en.compute_t_gradient();
            en.compute_x_gradient();
            en.compute_y_gradient();
            en.compute_lk_flow(lk_window_dim);

            en.visualize_lk_flow();



            key_press = en.display_image("flow", en.flow, downsample_scale);

            en.store_previous_frame();

            if (key_press==27) {
                en.destroy_all_windows();
                break;
            }

        }
    }
}
