#!/usr/bin/env python3
import rospy
import yaml
import rospkg
from typing import Union
import os
import actionlib
import chargepal_actions.msg
from chargepal_services.srv import stopFreeDriveArm

from chargepal_services.srv import readRobotCharge

rospack = rospkg.RosPack()
gui_yaml_path = rospack.get_path("chargepal_monitor_gui") + "/cfg/gui.yaml"
loop_input = 0
block_action = False


def shutdown_orin():
    os.system("shutdown /s /t 5")


def read_config():
    with open(gui_yaml_path, "r") as file:
        config = yaml.safe_load(file)
    return config


def update_config(key: str, new_value: Union[int, bool]):
    config = read_config()
    config[key] = new_value
    with open(gui_yaml_path, "w") as file:
        yaml.dump(config, file)


def update_configs(updated_values):

    with open(gui_yaml_path, "r") as file:
        data = yaml.safe_load(file)

    # Update the specific values
    for key, value in updated_values.items():
        data[key] = value

    with open(gui_yaml_path, "w") as file:
        yaml.dump(data, file)


def execute_request(name):
    if name == "click_start":
        success = execute_start_demo()
    elif name == "click_stop":
        success = execute_stop_demo()
    elif name == "click_resume":
        success = execute_resume_demo()
    elif name == "free_arm":
        success = execute_free_arm()
    elif name == "stop_free_arm":
        success = execute_stop_free_arm()
    elif name == "move_arm_home":
        success = execute_move_arm_home()
    elif name == "restart_arm_node":
        success = execute_restart_arm_node()
    elif name == "update_statistics":
        success = execute_update_statistics()

    return success


def fetch_current_loop():
    config = read_config()
    return config["robot_current_loop"]


def fetch_ongoing_action():
    config = read_config()
    return config["ongoing_action"]


def fetch_loop_input():
    config = read_config()
    return config["gui_loop_input"]


def fetch_job_name():
    config = read_config()
    return config["ongoing_job"]


def publish_log(log):
    config = read_config()
    log.push(config["gui_log"])


def set_loop_count(value: str):
    global loop_input
    loop_input = int(value)
    update_config("gui_loop_input", loop_input)


def execute_update_statistics():
    return True


def execute_start_demo():
    if loop_input > 0:
        updated_config = {
            "gui_demo_stop": False,
            "gui_demo_resume": False,
            "gui_demo_start": True,
        }

        update_configs(updated_config)
        update_config("gui_log", "Start clicked!")
    else:
        update_config("gui_log", "Enter number of demo loops")

    return True


def execute_stop_demo():
    updated_config = {
        "gui_demo_stop": True,
        "gui_demo_resume": False,
        "gui_demo_start": False,
    }

    update_configs(updated_config)
    update_config(
        "gui_log", "Stop clicked. Demo stopped! Press Resume to resume the demo."
    )
    return True


def execute_resume_demo():
    if loop_input > 0:
        updated_config = {
            "gui_demo_stop": False,
            "gui_demo_resume": True,
            "gui_demo_start": False,
        }

        update_configs(updated_config)
        update_config("gui_log", "Resume clicked.")
    else:
        update_config("gui_log", "Enter number of demo loops")
    return True


def execute_free_arm():
    client = actionlib.SimpleActionClient(
        "free_drive_arm", chargepal_actions.msg.FreeDriveArmAction
    )
    client.wait_for_server()
    client.send_goal()
    client.wait_for_result()
    client.get_result()


def execute_stop_free_arm():
    try:
        service_proxy_stop_free_drive_arm = rospy.ServiceProxy(
            "/robot_arm/stop_free_drive_arm", stopFreeDriveArm
        )
        response = service_proxy_stop_free_drive_arm()
        success = response.success
        update_config("ongoing_action", f"Stop free drive arm is {success}")

    except rospy.ServiceException as e:
        update_config(
            "ongoing_action",
            f"ERROR:Unable to stop free drive arm. Error is {e}",
        )


def execute_move_arm_home():
    client = actionlib.SimpleActionClient(
        "move_home_arm", chargepal_actions.msg.MoveHomeArmAction
    )
    client.wait_for_server()
    client.send_goal()
    client.wait_for_result()
    result = client.get_result()
    update_config(
        "ongoing_action",
        f"move arm home is {result.success}",
    )


def execute_restart_arm_node():
    node_name = "your_node_name"
    package_name = "package_name"
    os.system("rosrun {} {}".format(package_name, node_name))


def fetch_battery_level():
    rospy.wait_for_service("/mir_rest_api/clear_error")

    try:
        service_proxy_clear_mir_error = rospy.ServiceProxy(
            "/mir_rest_api/robot_charge", readRobotCharge
        )
        response = service_proxy_clear_mir_error("ChargePal1")
        charge = response.robot_charge

    except rospy.ServiceException as e:
        print(e)

    return charge


def fetch_ec_aas():
    config = read_config()
    return config["error_count_arrive_at_station"]


def fetch_ec_gh():
    config = read_config()
    return config["error_count_go_home"]


def fetch_ec_pic():
    config = read_config()
    return config["error_count_pickup_cart"]


def fetch_ec_plc():
    config = read_config()
    return config["error_count_place_cart"]


def fetch_ec_piads():
    config = read_config()
    return config["error_count_plugin_ads"]


def fetch_ec_poads():
    config = read_config()
    return config["error_count_plugout_ads"]


def fetch_total_loop():
    config = read_config()
    return config["total_loop_count"]


def fetch_avg_loop_time():
    config = read_config()
    return config["avg_loop_time"]


def fetch_avg_loop_runtime():
    return 100.00


def fetch_loop_count():
    return 100.00


def perform_arrive_at_station(station_name):
    global block_action

    if not block_action:
        block_action = True
        client = actionlib.SimpleActionClient(
            "arrive_at_station", chargepal_actions.msg.ArriveAtStationAction
        )
        client.wait_for_server()
        goal = chargepal_actions.msg.ArriveAtStationGoal(target_station=station_name)
        client.send_goal(goal)
        client.wait_for_result()
        block_action = False
        client.get_result()


def perform_arrive_at_home():
    global block_action

    if not block_action:
        block_action = True
        client = actionlib.SimpleActionClient(
            "arrive_at_home", chargepal_actions.msg.ArriveAtHomeAction
        )
        client.wait_for_server()
        goal = chargepal_actions.msg.ArriveAtHomeGoal(target_station="RBS_1")
        client.send_goal(goal)
        client.wait_for_result()
        client.get_result()
        block_action = False


def perform_place_cart(cart_name):
    global block_action

    if not block_action:
        block_action = True
        client = actionlib.SimpleActionClient(
            "place_charger", chargepal_actions.msg.PlaceChargerAction
        )
        client.wait_for_server()
        goal = chargepal_actions.msg.PlaceChargerGoal(charger_name=cart_name)
        client.send_goal(goal)
        client.wait_for_result()
        client.get_result()
        block_action = False


def perform_pickup_cart(cart_name):
    global block_action

    if not block_action:
        block_action = True
        client = actionlib.SimpleActionClient(
            "pick_up_charger", chargepal_actions.msg.PickUpChargerAction
        )
        client.wait_for_server()
        goal = chargepal_actions.msg.PickUpChargerGoal(charger_name=cart_name)
        client.send_goal(goal)
        client.wait_for_result()
        client.get_result()
        block_action = False


def perform_plugin_ads_ac():
    global block_action

    if not block_action:
        block_action = True
        client = actionlib.SimpleActionClient(
            "plugin_ads_ac", chargepal_actions.msg.PlugInAdsAcAction
        )
        client.wait_for_server()
        client.send_goal()
        client.wait_for_result()
        client.get_result()
        block_action = False


def perform_plugout_ads_ac():
    global block_action

    if not block_action:
        block_action = True
        client = actionlib.SimpleActionClient(
            "plugout_ads_ac", chargepal_actions.msg.PlugOutAdsAcAction
        )
        client.wait_for_server()
        client.send_goal()
        client.wait_for_result()
        client.get_result()
        block_action = False


def perfrom_job(job):
    print(job)
