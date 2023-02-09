#include <iostream>
#include "../include/engine.h"

using namespace std;

int main () {
    Engine en;
    if (en.open_camera() == 0) {
        while (1) {
            en.get_current_frame(1);
            char key_press;
            key_press = en.display_image("color", en.current_frame_color);
            key_press = en.display_image("bw", en.current_frame_bw);
            if (key_press==27)
                break;
        }
    }
}
