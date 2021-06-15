#pragma once

#include "api.h"

typedef int (*autonFunction)();

int nullAutonFunc();

#define AUTON_SIDE_COUNT 3
#define AUTON_POSITION_COUNT 4


// Current User Selection, Static for the moment 
// Because it needed a static function call
static int userSelection;
static const char *textMap[11];
static bool guiFinnished;

/**
 * A graphical interface for selecting a auton. Uses LVGL, so make sure that is included.
 */
class autonSelector
{
private:

    // The function map, used for graphical and execution.
    autonFunction functionMap[11];
    

    /*
    LVGL Widgets
    */

    // Pointer to Button Grid
    lv_obj_t* buttonMap;

    // How big the buttons are
    lv_obj_t* buttonMapRegion;

    /**
     * Delete all objects, clear up screen and memory
     */
    void del();

    void draw();

    void addCallbacks();

public:
    /**
     * Create an auton selector without autons configured.
     */
    autonSelector();

    /**
     * Create an auton selector without autons configured.
     */
    autonSelector(char* textMap[]);

    /**
     * Create an auton selector with all the auton functions already declared
     * ! <br> Not implemented yet 
     * 
     * @param textMap The map of different text.
     */
    autonSelector(char* textMap[], lv_obj_t* buttonMapRegion);
    
    /**
     * Run the user configuration with rendering.
     * 
     * @return The selected auton
     */
    int choose();

    /**
     * Run the user selection
     */
    int runSelection();

    /**
     * Function to handle button presses
     */
    void buttonPressed(int position) {
        userSelection = position;
    }

    /**
     * Replace the current function at a coordinate.
     * 
     * @param side The side the robot is on
     * @param position The position the robot is on
     * @param function The Function to replace it with
     */
    void setFunction(int position, autonFunction function);

    /**
     * Set the user selection
     * 
     * @param side Side to change to
     * @param position Position to change to
     */
    void setSelection(int position) {
        userSelection = position;
    }

    /**
     * Get the current user selection
     * 
     * @return A struct containing both objects.
     */
    int getSelection() { return userSelection; }

    /**
     * Temporary static solution, changes the userSelection
     * 
     */
    static lv_res_t pressHandler(lv_obj_t *btnm, const char *txt) {

        for (int i = 0; i < 11; i ++) {
            if (strcmp(txt, textMap[i]) == 0) {
                userSelection = i;
                break;
            }
        }

        guiFinnished = true;

        return LV_RES_OK;
    }
};