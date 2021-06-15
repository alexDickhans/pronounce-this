
#include "autonSelector.hpp"

int nullAutonSelector() {
    return 0;
}

autonSelector::autonSelector() {
    // Nothing
}

autonSelector::autonSelector(const char* textMap[11]) {
    for (int i = 0; i < 11; i ++) {
        this->textMap[i] = textMap[i];
    }
}



int autonSelector::choose() {

    // Wait until button is pressed.
    this->finnished = false;

    // Draw the items
    this->draw();
    this->addCallbacks();

    // Wait until finnished.
    while (!finnished) {
        pros::delay(20);
    }

    this->del();

    // Return the selection.
    return userSelection;
}

int autonSelector::runSelection() {
    return this->functionMap[userSelection]();
}

void autonSelector::setFunction(int position, autonFunction function) {
    this->functionMap[position] = function;
}

void autonSelector::del() {
    lv_obj_del(buttonMap);
}

void autonSelector::draw() {
    buttonMap = lv_btnm_create(lv_scr_act(), NULL);
    lv_btnm_set_map(buttonMap, textMap);
}

void autonSelector::addCallbacks() {
    lv_btnm_set_action(buttonMap, pressHandler);
}
