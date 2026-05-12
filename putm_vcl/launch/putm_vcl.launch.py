import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                namespace="putm_vcl",
                package="putm_vcl",
                executable="can_rx_node",
                name="can_rx_node",
		        prefix=['taskset -c 2 chrt -f 90 ']
            ),
            launch_ros.actions.Node(
                namespace="putm_vcl",
                package="putm_vcl",
                executable="can_tx_node",
                name="can_tx_node",
		        prefix=['taskset -c 2 chrt -f 90 ']
            ),
            launch_ros.actions.Node(
                namespace="putm_vcl",
                package="putm_vcl",
                executable="amk_node",
                name="amk_node",
                prefix=['taskset -c 3 ']

            ),
            launch_ros.actions.Node(
                namespace="putm_vcl",
                package="putm_vcl",
                executable="rtd_node",
                name="rtd_node",
                prefix=['taskset -c 3 ']
            ),
            launch_ros.actions.Node(
                namespace="putm_vcl",
                package="putm_vcl",
                executable="rec_node",
                name="rec_node",
                prefix=['taskset -c 3 ']
            ),
        ]
    )
