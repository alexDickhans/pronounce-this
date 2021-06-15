#pragma once

#include "api.h"

/**
 * Top left autonomous
 */
static int topLeft() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Top Left Auton");

    return 0;
}

/**
 * Top right autonomous
 */
static int topRight() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Top Right Auton");

    return 0;
}

/**
 * Misc left
 */
static int miscLeft() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Middle Left Auton");

    return 0;
}

/**
 * Misc right
 */
static int miscRight() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Middle Right Auton");

    return 0;
}

/**
 * Bottom left
 */
static int bottomLeft() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Bottom Left Auton");

    return 0;
}

/**
 * Bottom right
 */
static int bottomRight() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Bottom Left Auton");

    return 0;
}

/**
 * Skills Routine
 */
static int skills() {
    lv_obj_t* text = lv_label_create(lv_scr_act(), NULL);
    
    lv_label_set_text(text, "Skills Routine");

    return 0;
}