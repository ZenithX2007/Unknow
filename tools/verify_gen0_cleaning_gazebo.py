#!/usr/bin/env python3
"""Run the static-scene physics regression after sourcing ROS and the workspace.

Uses actual Gen0 collisions, masses, steering and road mesh, with rendering
sensors removed for speed. Does not test moving actors or perception. Uses a
separate ROS domain and Gazebo partition and only stops its own processes.
"""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET


def prepare_world(share, output):
    sdf = ET.Element('sdf', version='1.8')
    world = ET.SubElement(sdf, 'world', name='default')
    for filename, name in (
        ('ignition-gazebo-physics-system', 'gz::sim::systems::Physics'),
        ('ignition-gazebo-user-commands-system', 'gz::sim::systems::UserCommands'),
    ):
        ET.SubElement(world, 'plugin', filename=filename, name=name)
    physics = ET.SubElement(world, 'physics', name='default_physics', type='ignored')
    for name, value in (('max_step_size', '0.002'), ('real_time_factor', '4'),
                        ('real_time_update_rate', '2000')):
        ET.SubElement(physics, name).text = value
    source_world = ET.parse(share / 'worlds/my_map/my_map.sdf')
    terrain = copy.deepcopy(source_world.find(".//world/model[@name='my_map']"))
    for uri in terrain.findall('.//uri'):
        uri.text = str(share / 'worlds/my_map/my_map.obj')
    world.append(terrain)
    vehicle = ET.parse(share / 'urdf/gen0_model.sdf').find('model')
    ET.SubElement(vehicle, 'pose').text = source_world.find(".//world/model[@name='gen0_model']/pose").text
    # Keep all links, joints, masses, collision shapes and the steering plugin.
    for link in vehicle.findall('link'):
        for child in list(link):
            if child.tag in ('sensor', 'visual'):
                link.remove(child)
    for plugin in vehicle.findall('plugin'):
        if 'pose-publisher' in plugin.get('filename', ''):
            ET.SubElement(plugin, 'update_frequency').text = '50'
    world.append(vehicle)
    items = json.loads((share / 'worlds/trash_scenarios/my_map/small_trash_dense.json').read_text())
    for item in items:
        model = ET.parse(share / f"models/{item['model']}/model.sdf").find('model')
        model.set('name', item['name'])
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
        pose.text = ' '.join(map(str, item['pose']))
        for uri in model.findall('.//uri'):
            if uri.text.startswith('model://'):
                uri.text = str(share / 'models' / uri.text[len('model://'):])
        world.append(model)
    ET.ElementTree(sdf).write(output / 'world.sdf')
    bridge = [
        {'ros_topic_name': '/cmd_vel', 'gz_topic_name': '/cmd_vel',
         'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'ignition.msgs.Twist',
         'direction': 'ROS_TO_GZ'},
        {'ros_topic_name': '/gen0_model/links/poses', 'gz_topic_name': '/model/gen0_model/pose',
         'ros_type_name': 'geometry_msgs/msg/PoseArray', 'gz_type_name': 'ignition.msgs.Pose_V',
         'direction': 'GZ_TO_ROS'},
    ]
    (output / 'bridge.json').write_text(json.dumps(bridge))


def main():
    root = Path(__file__).resolve().parents[1]
    output = Path(os.environ.get('GEN0_VALIDATION_LOG_DIR', '/tmp/gen0_cleaning_gazebo_validation'))
    output.mkdir(parents=True, exist_ok=True)
    prepare_world(root / 'gen0_gz_sim_ros2/gen0_main', output)
    environment = os.environ.copy()
    partition = f'gen0_cleaning_validation_{os.getpid()}'
    environment.update(
        ROS_DOMAIN_ID=os.environ.get('GEN0_VALIDATION_ROS_DOMAIN_ID', '187'),
        IGN_PARTITION=partition, GZ_PARTITION=partition,
        ROS_LOG_DIR=str(output / 'ros'), OPENBLAS_NUM_THREADS='1',
    )
    processes = []

    def launch(name, command):
        with (output / f'{name}.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=environment, start_new_session=True)
        processes.append(process)

    try:
        launch('gazebo', ['ign', 'gazebo', '-s', '-r', str(output / 'world.sdf')])
        time.sleep(3)
        launch('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args',
                          '-p', f'config_file:={output}/bridge.json'])
        launch('cleanup', ['ros2', 'run', 'gen0_main', 'trash_cleanup_node', '--ros-args',
                           '-p', 'trash_scenario:=small_trash_dense', '-p', 'vehicle_length:=2.40',
                           '-p', 'vehicle_width:=1.65', '-p', 'vehicle_pose_index:=-1'])
        launch('controller', ['ros2', 'run', 'gen0_main', 'fixed_cleaning_controller',
                              '--ros-args', '-p', 'require_cleanup_confirmation:=true',
                              '-p', "actor_scenario:=''", '-p', 'scenario:=small_trash_dense'])
        deadline, previous = time.monotonic() + 600, ''
        while time.monotonic() < deadline:
            time.sleep(2)
            content = (output / 'controller.log').read_text()
            lines = [line for line in content.splitlines() if '"state":' in line]
            if lines and lines[-1] != previous:
                previous = lines[-1]
                status = json.loads(previous[previous.index('{'):])
                print(json.dumps({key: status[key] for key in
                                  ('state', 'covered', 'total', 'progress_m', 'reason')}), flush=True)
            if '"state": "failed"' in content:
                raise RuntimeError(f'Controller failed: see {output}/controller.log')
            if '"state": "completed"' in content:
                print('PHYSICS AND CLEANUP COMPLETE', flush=True)
                break
            if any(process.poll() is not None for process in processes):
                raise RuntimeError(f'Validation process exited: see {output}')
        else:
            raise RuntimeError(f'Validation deadline reached: see {output}')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        time.sleep(2)
        for process in processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            process.wait()


if __name__ == '__main__':
    main()
