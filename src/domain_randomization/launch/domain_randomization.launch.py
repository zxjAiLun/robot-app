from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    use_domain_randomizer = DeclareLaunchArgument(
        'use_domain_randomizer',
        default_value='true',
        description='是否启用域随机化器'
    )
    
    use_material_randomizer = DeclareLaunchArgument(
        'use_material_randomizer',
        default_value='true',
        description='是否启用材质随机化器'
    )
    
    use_dynamics_randomizer = DeclareLaunchArgument(
        'use_dynamics_randomizer',
        default_value='true',
        description='是否启用动力学随机化器'
    )
    
    # 创建节点
    domain_randomizer_node = Node(
        package='domain_randomization',
        executable='domain_randomizer',
        name='domain_randomizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_domain_randomizer'))
    )
    
    material_randomizer_node = Node(
        package='domain_randomization',
        executable='material_randomizer',
        name='material_randomizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_material_randomizer'))
    )
    
    dynamics_randomizer_node = Node(
        package='domain_randomization',
        executable='dynamics_randomizer',
        name='dynamics_randomizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_dynamics_randomizer'))
    )
    
    # 或者使用管理器节点
    randomization_manager_node = Node(
        package='domain_randomization',
        executable='randomization_manager',
        name='randomization_manager',
        output='screen',
        # 如果使用管理器，则不单独启动各个随机化器
        condition=IfCondition('false')
    )
    
    # 返回启动描述
    return LaunchDescription([
        use_domain_randomizer,
        use_material_randomizer,
        use_dynamics_randomizer,
        domain_randomizer_node,
        material_randomizer_node,
        dynamics_randomizer_node,
        randomization_manager_node
    ]) 