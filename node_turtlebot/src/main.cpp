#include "motores.hpp"
#include "sensores.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Crear instancias de los nodos
    auto motores_node = std::make_shared<MotoresNode>();
    auto sensores_node = std::make_shared<SensoresNode>();

    // Crear un conjunto de nodos y ejecutar
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(motores_node);
    executor.add_node(sensores_node);

    // Esperar a que se cierre el nodo principal
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
