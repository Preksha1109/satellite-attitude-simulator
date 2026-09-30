#include <iostream>
#include <cassert>
#include "attitude/magnetorquer.hpp"
#include "attitude/reaction_wheel.hpp"

using namespace attitude;

int main() {
    std::cout << "\n=== Basic Magnetorquer and Reaction Wheel Tests ===\n\n";
    
    // Test 1: Create magnetorquer
    std::cout << "Test 1: Magnetorquer creation\n";
    Magnetorquer mag(0.5, 0.05, true);
    assert(mag.getMaxDipoleMoment() == 0.5);
    assert(mag.getBdotGain() == 0.05);
    std::cout << "  ✓ Magnetorquer created successfully\n";
    
    // Test 2: Compute torque
    std::cout << "Test 2: Magnetorquer torque computation\n";
    Vec3 B_field(0, 0, 25e-6);
    mag.updateBdot(B_field, 0.01);
    Vec3 tau = mag.computeTorque(B_field);
    std::cout << "  ✓ Torque computed\n";
    
    // Test 3: Create reaction wheel
    std::cout << "Test 3: Reaction wheel creation\n";
    ReactionWheel wheel(2, 0.1, 0.01, 0.2);
    assert(wheel.getAxis() == 2);
    assert(wheel.getMaxTorque() == 0.01);
    std::cout << "  ✓ Reaction wheel created successfully\n";
    
    // Test 4: Wheel torque
    std::cout << "Test 4: Reaction wheel torque\n";
    wheel.setTorqueCommand(0.005);
    Vec3 tau_wheel = wheel.getTorque(0.01);
    assert(tau_wheel.z == 0.005);
    std::cout << "  ✓ Wheel torque computed\n";
    
    // Test 5: Momentum integration
    std::cout << "Test 5: Momentum integration\n";
    double h = wheel.getMomentum();
    assert(h > 0);
    std::cout << "  ✓ Momentum integrated: " << h << " N·m·s\n";
    
    // Test 6: Wheel cluster
    std::cout << "Test 6: 3-axis reaction wheel cluster\n";
    ReactionWheelCluster cluster(3);
    assert(cluster.getNumWheels() == 3);
    cluster.setTorqueCommand(0.005, 0.007, 0.003);
    Vec3 tau_cluster = cluster.getTorque(0.01);
    std::cout << "  ✓ Cluster torque: [" << tau_cluster.x << ", " 
              << tau_cluster.y << ", " << tau_cluster.z << "] N·m\n";
    
    std::cout << "\n=== All tests passed! ===\n\n";
    return 0;
}
