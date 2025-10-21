#include <iostream>
#include "DynamicBicycle.hpp"

int main() {
    try {
        VehicleParam params = VehicleParam::loadFromFile("../sim/src/vehicle/Configs/configDry.yaml");
        DynamicBicycle model(params);
        VehicleState state; // default zeros
        VehicleInput input(0.0,0.0,0.0);
        model.updateState(state, input, 0.1);
        std::cout << state.toString() << std::endl;
    } catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
