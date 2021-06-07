#pragma once

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
    Misc = 2
};

struct selection
{
    sides side = Skills;
    positions position = Middle;
};

// Auton Selector Class
class autonSelector
{
private:
    // Current User Selection
    selection userSelection;

    // The function map, used for graphical and execution.
    autonFunction functionMap[3][3];

    /**
     * Render a new frame
     */
    void render();

    /**
     * Delete all objects, clear up screen and memory
     */
    void del();

public:
    /**
     * A graphical interface for selecting a auton.
     * 
     * @param functionMap The map of different ports.
     */
    autonSelector(autonFunction functionMap[3][3]);
    
    /**
     * Run the user configuration with rendering.
     * 
     */

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