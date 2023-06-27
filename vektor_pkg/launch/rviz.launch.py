from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'vektor_pkg'
    package_path = get_package_share_directory(package_name)

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        description='Use RViz if true',
        choices=['true', 'false'],
        default_value='true',
        )
    
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        description='A display config file (.rviz) to load',
        default_value='vektor_rsp.rviz',
        )
    
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    rviz_config_file = PathJoinSubstitution([
        package_path, 'config', 'rviz', rviz_config_file
        ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        )
    
    # >>> Algoritmo para retorno automático de variáveis específicas do LaunchDescription <<<
    
    # Tags associadas às entidades que devem ser retornadas
    # Nem todos os objetos da função são retornados (ex.: LaunchConfiguration)
    tags = ['arg', 'node', 'launch']

    # Instancia o objeto de retorno
    ld = LaunchDescription()

    # Cópia independente do dicionário contendo as variáveis locais
    # As variáveis dentro do for alterarão o dicionário original, e se ele fosse usado ...
    # dispararia uma exceção (dictionary changed size during iteration)
    entities = locals().copy()

    # Para cada variável local é verificado se existe uma das tags listadas anteriormente
    for entity in entities:
        if any(tag in entity for tag in tags):
            # Em caso positivo, a variável é adicionada ao objeto de retorno
            ld.add_action(eval(entity, locals()))
    
    # Retorno com as variáveis selecionadas
    return ld
