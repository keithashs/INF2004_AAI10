#include "ir_linefollow.h"

// ===== Helper: Check if raw reading indicates black =====
static inline bool is_black(int raw) {
    return IR_BLACK_IS_LOW ? (raw == 0) : (raw == 1);
}

// ===== Initialize GPIO pins =====
void ir_line_init(void) {
    gpio_init(IR_LEFT_PIN);
    gpio_set_dir(IR_LEFT_PIN, GPIO_IN);
    // Adjust pull resistor based on your sensor:
    // gpio_pull_up(IR_LEFT_PIN);   // if sensor is open-collector
    // gpio_disable_pulls(IR_LEFT_PIN); // if sensor has internal pull-up

    gpio_init(IR_RIGHT_PIN);
    gpio_set_dir(IR_RIGHT_PIN, GPIO_IN);
    // gpio_pull_up(IR_RIGHT_PIN);
}

// ===== Raw Reads =====
int ir_read_left_raw(void) {
    return gpio_get(IR_LEFT_PIN);
}

int ir_read_right_raw(void) {
    return gpio_get(IR_RIGHT_PIN);
}

// ===== Black Detection =====
bool ir_left_is_black(void) {
    return is_black(ir_read_left_raw());
}

bool ir_right_is_black(void) {
    return is_black(ir_read_right_raw());
}

// ===== High-Level Line Position =====
line_position_t ir_get_line_position(void) {
    bool left_black = ir_left_is_black();
    bool right_black = ir_right_is_black();

    if (left_black && right_black) {
        return LINE_CENTER;      // Both on line → centered
    } else if (left_black && !right_black) {
        return LINE_LEFT;        // Left on, right off → drifted right, steer left
    } else if (!left_black && right_black) {
        return LINE_RIGHT;       // Right on, left off → drifted left, steer right
    } else {
        return LINE_BOTH_OFF;    // Both off → lost or at junction
    }
}

// ===== String Conversion (for telemetry) =====
const char* ir_line_position_str(line_position_t pos) {
    switch (pos) {
        case LINE_CENTER:   return "CENTER";
        case LINE_LEFT:     return "LEFT";
        case LINE_RIGHT:    return "RIGHT";
        case LINE_BOTH_OFF: return "BOTH_OFF";
        case LINE_LOST:     return "LOST";
        default:            return "UNKNOWN";
    }
}