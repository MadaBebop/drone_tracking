"""Avvio della catena di percezione, controllo e missione.

Tempo di simulazione. Tutti i nodi calcolano intervalli reali (`_calcola_dt`)
per non dipendere dalla frequenza dei topic; finora quegli intervalli venivano
misurati sull'orologio di parete, cosa che vale solo perche il SITL gira a
velocita 1 e non a lockstep - un'assunzione mai dichiarata da nessuna parte.
Con `/clock` pontato da Gazebo e `use_sim_time` attivo su ogni nodo,
`self.get_clock().now()` torna il tempo di simulazione e nessuna di quelle
funzioni va riscritta.

ATTENZIONE: con `use_sim_time:=true` i timer dei nodi non partono finche
`/clock` non pubblica, cioe finche Gazebo non e in esecuzione. Se serve
lanciare i nodi da soli (prove di rete, ispezione dei topic) si passa
`use_sim_time:=false`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    seed = ParameterValue(LaunchConfiguration('seed'), value_type=int)
    etichetta = LaunchConfiguration('etichetta_config')

    comune = [{'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Usa /clock di Gazebo invece dell orologio di parete. '
                        'Richiede Gazebo in esecuzione.'),
        DeclareLaunchArgument(
            'seed', default_value='42',
            description='Seme del disturbo di jammer_node. Fisso = prove '
                        'ripetibili; negativo = sequenza diversa a ogni run.'),
        DeclareLaunchArgument(
            'etichetta_config', default_value='',
            description='Etichetta aggiunta al nome del CSV di metrics_node, '
                        'per distinguere le configurazioni a confronto.'),
        # L'attacco al GNSS resta spento per default: accendendolo in ogni
        # prova non esisterebbe piu una linea di riferimento con cui
        # confrontarlo.
        DeclareLaunchArgument(
            'gnss_denial', default_value='false',
            description='Avvia gnss_denial_node, che attacca il ricevitore '
                        'satellitare del drone nel simulatore.'),
        DeclareLaunchArgument(
            'gnss_modo', default_value='jamming',
            description='jamming (fix intermittente), negazione (ricevitore '
                        'spento) o spoofing (fix falsificato).'),
        DeclareLaunchArgument(
            'gnss_sempre_attivo', default_value='true',
            description='true: attacco continuo per tutta la missione, come '
                        'serve al confronto A/B. false: cicli alternati.'),

        Node(
            package='drone_tracking',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=comune
        ),
        Node(
            package='drone_tracking',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=comune
        ),
        Node(
            package='drone_tracking',
            executable='jammer_node',
            name='jammer_node',
            output='screen',
            parameters=comune + [{'seed': seed}]
        ),
        Node(
            package='drone_tracking',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=comune
        ),
        Node(
            package='drone_tracking',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=comune
        ),
        Node(
            package='drone_tracking',
            executable='target_mover_node',
            name='target_mover_node',
            output='screen',
            parameters=comune
        ),
        Node(
            package='drone_tracking',
            executable='metrics_node',
            name='metrics_node',
            output='screen',
            parameters=comune + [{'etichetta_config': etichetta}]
        ),
        Node(
            package='drone_tracking',
            executable='gnss_denial_node',
            name='gnss_denial_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gnss_denial')),
            parameters=comune + [{
                'modo': LaunchConfiguration('gnss_modo'),
                'sempre_attivo': ParameterValue(
                    LaunchConfiguration('gnss_sempre_attivo'), value_type=bool),
            }]
        ),
        # Ponte Gazebo -> ROS 2. `[` significa unidirezionale verso ROS: la
        # camera e il clock si leggono, non si scrivono.
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/drone/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
    ])
