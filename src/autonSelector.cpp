#include "autonSelector.hpp"

int nullAutonFunc() {
    return 0;
}

autonSelector::autonSelector() {
    // Nothing
}

autonSelector::autonSelector(char* inTextMap[AUTON_SELECTOR_COUNT]) {
    for (int i = 0; i < AUTON_SELECTOR_COUNT; i ++) {
        textMap[i] = inTextMap[i];
    }
}

autonSelector::autonSelector(char* inTextMap[AUTON_SELECTOR_COUNT], lv_obj_t* buttonMapRegion) {
    for (int i = 0; i < AUTON_SELECTOR_COUNT; i ++) {
        textMap[i] = inTextMap[i];
    }
    this->buttonMapRegion = buttonMapRegion;
}

int autonSelector::choose() {

    // Wait until button is pressed.
    guiFinnished = false;

    // Draw the items
    this->draw();
    this->addCallbacks();

    // Return the selection.
    return userSelection;
}

int autonSelector::runSelection() {
    this->preRun();
    this->functionMap[userSelection]();
    this->postAuton();
}

void autonSelector::setFunction(int position, autonFunction function) {
    this->functionMap[position] = function;
}

void autonSelector::del() {
    lv_obj_del(buttonMap);
}

void autonSelector::draw() {
    buttonMap = lv_btnm_create(buttonMapRegion, NULL);
    lv_btnm_set_map(buttonMap, textMap);
    lv_obj_set_size(buttonMap, lv_obj_get_width(buttonMapRegion),
      lv_obj_get_height(buttonMapRegion));
}

void autonSelector::addCallbacks() {
    lv_btnm_set_action(buttonMap, pressHandler);
}
