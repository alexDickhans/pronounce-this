#pragma once

#include "api.h"

/**
 * Just returns 0, just to have a function for the functionMap.
 */
int nullAutonFunc() { return 0; }

typedef int (*autonFunction)();

// Enums to make managing position easier.
enum sides
{
    Skills = 0,
    Red = 1,
    Blue = 2
};

enum positions
{
    Top = 0,
    Middle = 1,
    Bottom = 2,
    Misc = 3
};

struct selection
{
    sides side = Skills;
    positions position = Middle;
};

#define AUTON_SIDE_COUNT 3
#define AUTON_POSITION_COUNT 4

/**
 * A graphical interface for selecting a auton. Uses LVGL, so make sure that is included.
 */
class autonSelector
{
private:
    // Current User Selection
    selection userSelection;

    // The function map, used for graphical and execution.
    autonFunction functionMap[AUTON_SIDE_COUNT][AUTON_POSITION_COUNT];

    /*
    LVGL Widgets
    */

    // Pointer to tab view
    lv_obj_t* tabView;

    // Tabs
    lv_obj_t* skillsTab;
    lv_obj_t* blueTab;
    lv_obj_t* redTab;

    // Buttons
    // Kinda jank solution, but if you don't make them switch with the tabs then you can make only 3 buttons and save ram.
    lv_obj_t* topButton;
    lv_obj_t* midButton;
    lv_obj_t* bottomButton;
    lv_obj_t* miscButton;

    // Button visible on all tabs
    lv_obj_t* finnishedButton;

    bool finnished = true;

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
     * Create an auton selector with all the auton functions already declared
     * ! <br> Not implemented yet 
     * 
     * @param functionMap The map of different ports.
     */
    autonSelector(autonFunction* functionMap[AUTON_SIDE_COUNT][AUTON_POSITION_COUNT]) {}
    
    /**
     * Run the user configuration with rendering.
     * 
     * @return The selected auton
     */
    selection choose();

    /**
     * Run the user selection
     */
    int runSelection();

    /**
     * Function to handle button presses
     */
    void buttonPressed(positions position) {
        this->userSelection.position = position;
    }
    
    void topPressed() {
        buttonPressed(positions::Top);
    }

    void middlePressed() {
        buttonPressed(positions::Middle);
    }

    void bottomPressed() {
        buttonPressed(positions::Bottom);
    }

    void miscPressed() {
        buttonPressed(positions::Misc);
    }

    /**
     * Replace the current function at a coordinate.
     * 
     * @param side The side the robot is on
     * @param position The position the robot is on
     * @param function The Function to replace it with
     */
    void setFunction(sides side, positions position, autonFunction function);

    /**
     * Set the user selection
     * 
     * @param side Side to change to
     * @param position Position to change to
     */
    void setSelection(sides side, positions poisition) {
        this->userSelection.side = side;
        this->userSelection.position = poisition;
    }

    /**
     * Get the current user selection
     * 
     * @return A struct containing both objects.
     */
    selection getSelection() { return userSelection; }
};