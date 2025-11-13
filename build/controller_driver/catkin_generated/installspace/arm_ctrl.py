# -*- coding: utf-8 -*-
# @Author: Mingkai Qiu, Junkai Ma, Qiaoxin Zhang

import time
import os
import yaml
import threading
from concurrent.futures import ThreadPoolExecutor
from CANalyst2_canbus_handler import CanbusHandler 
from motor_controller import MotorController


def load_arm_config(file_name='configs/arm_bind.yaml'):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, file_name)
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Configuration file not found: {file_path}")
    with open(file_path, 'r', encoding='utf-8') as f:
        try:
            config = yaml.safe_load(f)
            print(f"Successfully loaded configuration from {file_name}")
            return config
        except yaml.YAMLError as e:
            raise IOError(f"Error parsing YAML file {file_path}: {e}")


class ArmController:
    def __init__(self, side_name='RIGHT_ARM', can_channel=0, baud_rate=1000, auto_enable=True):
        ARM_CONFIG = load_arm_config()
        self.config = ARM_CONFIG.get(side_name)
        if not self.config:
            raise ValueError(f"'{side_name}' is not a valid side_name in config file.")

        self.can_channel = can_channel
        self.baud_rate = baud_rate
        self.auto_enable = auto_enable
        self.can_handlers = {}
        self.motors = {}
        self.motor_id_list = []
        self.brake_motor_ids = set()
        self.motor_to_dev_idx_map = {}

        self.brkto_tasks = {}
        self.monitoring_thread = None
        self.monitoring_lock = threading.Lock()
        self.monitoring_stop_event = threading.Event()

        for group in self.config:
            self.motor_id_list.extend(group['motor_ids'])
            if 'brake_motor_ids' in group:
                self.brake_motor_ids.update(group['brake_motor_ids'])

        if self.brake_motor_ids:
            print(f"Brake-on-arrival enabled for motors: {sorted(list(self.brake_motor_ids))}")

        self._initialize_can_and_motors()

    def _initialize_can_and_motors(self):
        try:
            print("Scanning for CAN devices...")
            available_devices = CanbusHandler.scan_devices()
            if not available_devices:
                raise IOError("No CAN devices found.")

            print("-" * 70)
            print(f"{'Index':<6} | {'Serial Number':<25} | {'Hardware Type':<30}")
            print("-" * 70)
            for device in available_devices:
                print(f"{device['device_index']:<6} | {device['serial_number']:<25} | {device['hardware_type']:<30}")
            print("-" * 70)

            device_map = {dev['serial_number']: dev for dev in available_devices}

            for group in self.config:
                matcher = group['uuid_matcher']
                motor_ids = group['motor_ids']

                found_device = None
                if matcher in device_map:
                    found_device = device_map[matcher]
                else:
                    for sn, dev in device_map.items():
                        if sn.endswith(matcher):
                            found_device = dev
                            break

                if not found_device:
                    raise IOError(f"Could not find a CAN device matching UUID/suffix '{matcher}'")

                dev_idx = int(found_device['device_index'])

                if dev_idx not in self.can_handlers:
                    print(f"Initializing CAN device {dev_idx} (SN: {found_device['serial_number']})...")
                    can = CanbusHandler(can_channel=self.can_channel, baud_rate=self.baud_rate, device_index=dev_idx)
                    can.open()
                    self.can_handlers[dev_idx] = can
                    print(f"CAN device {dev_idx} opened on channel {self.can_channel} @ {self.baud_rate}kbps.")

                can_handler = self.can_handlers[dev_idx]

                for motor_id in motor_ids:
                    print(f"Initializing motor {motor_id} on CAN device {dev_idx}...")
                    self.motors[motor_id] = MotorController(can_handler, motor_id=motor_id)
                    self.motor_to_dev_idx_map[motor_id] = dev_idx
                    if self.auto_enable:
                        print(f"  Motor {motor_id} initialized and enabled.")
                        self.motors[motor_id].enable_motor()
                    else:
                        print(f"  Motor {motor_id} initialized (NOT enabled).")
                    self.motors[motor_id].clear_errors()

        except IOError as e:
            print(f"\nError: Could not communicate with CAN analyzer: {e}")
            raise
        except Exception as e:
            print(f"\nAn unexpected error occurred during initialization: {e}")
            raise

    def get_target_positions(self):
        try:
            positions = {}
            commands_by_device = self._group_motors_by_device(self.motor_id_list)

            def task_for_device(motor_ids):
                for motor_id in motor_ids:
                    positions[motor_id] = self.motors[motor_id].get_current_position()

            with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if m_ids]
                for future in futures: future.result()
            return [positions.get(motor_id) for motor_id in self.motor_id_list]
        except Exception as e:
            print(f"\n获取目标位置过程中发生错误: {e}")
            raise

    def set_target_positions(self, target_pose_list, is_check_move=True):
        if len(target_pose_list) != len(self.motor_id_list):
            raise ValueError('Length of target_pose_list does not match motor_id_list')
        try:
            print("Enabling all motors before collective movement...")
            self._enable_all_motors()
            target_pose_map = dict(zip(self.motor_id_list, target_pose_list))
            commands_by_device = self._group_motors_by_device(self.motor_id_list)

            def task_for_device(motor_ids):
                for motor_id in motor_ids:
                    self._send_motor_command(motor_id, target_pose_map[motor_id])

            with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if m_ids]
                for future in futures: future.result()

            if is_check_move:
                self._check_motor_notmove(self.motor_id_list)
        except Exception as e:
            print(f"\nAn error occurred during set_target_positions: {e}")
            raise

    def set_specific_motors_position(self, motor_pos_map: dict, is_check_move=True):
        """
        Enables and moves only a specific set of motors to their target positions.
        """
        if not isinstance(motor_pos_map, dict) or not motor_pos_map:
            print("Warning: motor_pos_map is empty or not a dict. No action taken.")
            return

        motor_ids_to_move = list(motor_pos_map.keys())
        print(f"Starting specific move for motors: {motor_ids_to_move}")

        try:
            self.enable_motors(motor_ids_to_move)
            commands_by_device = self._group_motors_by_device(motor_ids_to_move)

            def task_for_device(motor_ids):
                for motor_id in motor_ids:
                    self.motors[motor_id].set_target_position(motor_pos_map[motor_id])

            with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if m_ids]
                for future in futures: future.result()

            print(f"Move commands sent to motors: {motor_ids_to_move}")
            if is_check_move:
                self._check_motor_notmove(motor_ids_to_move)

        except Exception as e:
            print(f"\nAn error occurred during specific motor movement: {e}")
            raise

    def set_async_move(self, commands):
        motor_ids_involved = [cmd[0] for cmd in commands]
        print(f"Executing async move for motors: {motor_ids_involved}")
        self.enable_motors(motor_ids_involved)

        to_commands = []
        with self.monitoring_lock:
            for motor_id, cmd_type, target_pos in commands:
                if cmd_type in ['TO', 'BRKTO']:
                    to_commands.append((motor_id, target_pos))
                if cmd_type == 'BRKTO':
                    self.brkto_tasks[motor_id] = {'target': target_pos, 'in_range_since': None}

        if to_commands:
            commands_by_device = self._group_motors_by_device([cmd[0] for cmd in to_commands])

            def task_for_device(m_ids):
                for m_id in m_ids:
                    # Find the corresponding target_pos from the original to_commands list
                    target_pos = next(cmd[1] for cmd in to_commands if cmd[0] == m_id)
                    self.motors[m_id].set_target_position(target_pos)

            with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if m_ids]
                for future in futures: future.result()

        with self.monitoring_lock:
            if self.brkto_tasks and (self.monitoring_thread is None or not self.monitoring_thread.is_alive()):
                print("Starting BRKTO monitoring thread...")
                self.monitoring_stop_event.clear()
                self.monitoring_thread = threading.Thread(target=self._brkto_monitor_worker, daemon=True)
                self.monitoring_thread.start()

    def _brkto_monitor_worker(self):
        while not self.monitoring_stop_event.is_set():
            try:
                with self.monitoring_lock:
                    if not self.brkto_tasks:
                        print("No more BRKTO tasks. Stopping monitor.")
                        break
                    monitored_ids = list(self.brkto_tasks.keys())

                positions = {}
                commands_by_device = self._group_motors_by_device(monitored_ids)

                def task_for_device(motor_ids):
                    for motor_id in motor_ids:
                        positions[motor_id] = self.motors[motor_id].get_current_position()

                with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                    futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if
                               m_ids]
                    for future in futures: future.result()

                with self.monitoring_lock:
                    for motor_id in list(self.brkto_tasks.keys()):
                        task = self.brkto_tasks[motor_id]
                        current_pos = positions.get(motor_id)

                        if current_pos is None: continue

                        if abs(current_pos - task['target']) < 500:
                            if task['in_range_since'] is None:
                                task['in_range_since'] = time.time()
                            elif time.time() - task['in_range_since'] >= 2.0:
                                print(
                                    f"[Monitor] Motor {motor_id} reached target {task['target']}. Stopping for brake.")
                                self.stop_motors([motor_id])
                                del self.brkto_tasks[motor_id]
                        else:
                            task['in_range_since'] = None
            except Exception as e:
                print(f"[Monitor Thread Error] {e}")
            time.sleep(0.1)

        with self.monitoring_lock:
            self.monitoring_thread = None

    def _check_motor_notmove(self, motor_ids_to_check: list):
        if not motor_ids_to_check: return
        try:
            print(f"Waiting for motors {motor_ids_to_check} to stop...")
            start_time = time.time()
            timeout = 30
            all_stopped = False
            while not all_stopped and (time.time() - start_time < timeout):
                speeds = {}
                commands_by_device = self._group_motors_by_device(motor_ids_to_check)

                def task_for_device(motor_ids):
                    for motor_id in motor_ids:
                        speeds[motor_id] = self.motors[motor_id].get_current_speed()

                with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                    futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if
                               m_ids]
                    for future in futures: future.result()

                if len(speeds) == len(motor_ids_to_check):
                    all_stopped = all(speed == 0 for speed in speeds.values())
                else:
                    all_stopped = False
                if not all_stopped:
                    time.sleep(0.05)

            if all_stopped:
                print(f"Motors {motor_ids_to_check} have stopped.")
                brake_motors_to_stop = self.brake_motor_ids.intersection(motor_ids_to_check)
                if brake_motors_to_stop:
                    print(f"Stopping brake motors to save power: {list(brake_motors_to_stop)}")
                    self.stop_motors(list(brake_motors_to_stop))
            else:
                print(f"Warning: Timeout on waiting for motors {motor_ids_to_check} to stop.")
        except Exception as e:
            print(f"\nAn error occurred while checking motor speed: {e}")
            raise

    def _send_motor_command(self, motor_id, target_pose):
        self.motors[motor_id].set_target_position(target_pose)

    def _group_motors_by_device(self, motor_ids_to_group):
        grouped = {dev_idx: [] for dev_idx in self.can_handlers}
        for motor_id in motor_ids_to_group:
            if motor_id in self.motor_to_dev_idx_map:
                dev_idx = self.motor_to_dev_idx_map[motor_id]
                grouped[dev_idx].append(motor_id)
        return grouped

    def _execute_motor_command(self, motor_ids, command_func_name):
        if not motor_ids: return
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]

        try:
            def task_for_device(m_ids):
                for motor_id in m_ids:
                    if motor_id in self.motors:
                        getattr(self.motors[motor_id], command_func_name)()
                    else:
                        print(f"Warning: Motor ID {motor_id} not found.")

            commands_by_device = self._group_motors_by_device(motor_ids)
            with ThreadPoolExecutor(max_workers=len(self.can_handlers) or 1) as executor:
                futures = [executor.submit(task_for_device, m_ids) for _, m_ids in commands_by_device.items() if m_ids]
                for future in futures: future.result()
        except Exception as e:
            print(f"Error executing '{command_func_name}' on motors {motor_ids}: {e}")

    def enable_motors(self, motor_ids_to_enable):
        print(f"Enabling motors: {motor_ids_to_enable}")
        self._execute_motor_command(motor_ids_to_enable, 'enable_motor')
        time.sleep(0.1)

    def stop_motors(self, motor_ids_to_stop):
        print(f"Stopping motors: {motor_ids_to_stop}")
        self._execute_motor_command(motor_ids_to_stop, 'stop_motor')

    def _enable_all_motors(self):
        self.enable_motors(self.motor_id_list)

    def stop_all_motors(self):
        print("\nSending stop command to all motors.")
        self.stop_motors(self.motor_id_list)

    def __del__(self):
        print("Closing ArmController...")
        try:
            if self.monitoring_thread and self.monitoring_thread.is_alive():
                print("Stopping monitoring thread...")
                self.monitoring_stop_event.set()
                self.monitoring_thread.join(timeout=2)
            self.stop_all_motors()
            time.sleep(0.5)
            for dev_idx, handler in self.can_handlers.items():
                print(f"Closing CAN device {dev_idx}...")
                handler.close()
            print("All resources released.")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"An error occurred during cleanup: {e}")

