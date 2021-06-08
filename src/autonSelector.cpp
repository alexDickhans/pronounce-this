
#include "autonSelector.hpp"

autonSelector::autonSelector() {
    // Nothing
}

selection autonSelector::choose() {

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
    return this->functionMap[userSelection.side][userSelection.position]();
}

void autonSelector::del() {
    lv_obj_del(tabView);
    
    lv_obj_del(skillsTab);
    lv_obj_del(blueTab);
    lv_obj_del(redTab);

    lv_obj_del(topButton);
    lv_obj_del(midButton);
    lv_obj_del(bottomButton);
    lv_obj_del(miscButton);

    lv_obj_del(finnishedButton);
}

void autonSelector::draw() {
    tabView = lv_tabview_create(lv_scr_act(), NULL);

    skillsTab = lv_tabview_add_tab(tabView, "Skills");
    blueTab = lv_tabview_add_tab(tabView, "Blue");
    redTab = lv_tabview_add_tab(tabView, "Red");

}

void autonSelector::addCallbacks() {
    
}
