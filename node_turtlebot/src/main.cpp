#include "node_turtlebot/motores.hpp"
#include "node_turtlebot/sensores.hpp"
#include "node_turtlebot/diff_drive_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Crear instancias de los nodos
    auto motores_node = std::make_shared<MotoresNode>();
    //motores_node->run();
    auto diff_drive_controller = std::make_shared<DiffDriveController>();
    //diff_drive_controller->run();

    // Crear un conjunto de nodos y ejecutar
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motores_node);
    executor.add_node(diff_drive_controller);

    // Esperar a que se cierre el nodo principal
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
