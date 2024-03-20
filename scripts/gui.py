#!/usr/bin/env python3
import rospy
import tkinter as tk
from tkinter import ttk
import socket
import json
from chargepal_monitor_gui.srv import (
    readRobotInfo,
    readRobotInfoResponse,
    updateRobotInfo,
    updateRobotInfoResponse,
)
from chargepal_monitor_gui.srv import (
    readCartInfo,
    readCartInfoResponse,
    updateCartInfo,
    updateCartInfoResponse,
)

# Initialize counters and data dictionaries
robotCounter = 1
cartCounter = 1
robotData = {}
cartData = {}
robot_info = {}
cart_info = {}


# Method to add a robot row
def addRobot():
    global robotCounter
    robot_info = [
        f"Chargepal{robotCounter}",
        "unknown",
        "unknown",
        "unknown",
        "unknown",
        "unknown",
        "unknown",
        "unknown",
    ]
    robotTable.insert("", "end", values=robot_info)
    robot_info_dict = dict(zip(robot_columns[1:], robot_info[1:]))
    robotCounter += 1


# Method to set the robot information
def setRobotInfo(req):

    robot_data = json.loads(req.robot_info)
    all_rows = robotTable.get_children()
    for row in all_rows:
        original_values = robotTable.item(row, "values")
        updated_values = []
        if robot_data["robot_name"] == original_values[0]:
            for column_name, value in zip(robotTable["columns"], original_values):

                if column_name not in robot_data:
                    updated_values.append(value)
                else:
                    updated_values.append(robot_data[column_name])

            robotTable.item(row, values=updated_values)
            return updateRobotInfoResponse(True)

    print("Unable to update robot value")
    return updateRobotInfoResponse(False)


# Method to fetch robot information
def getRobotData(req):
    name = req.robot_name
    get_robot_result = readRobotInfoResponse()
    all_rows = robotTable.get_children()
    for row in all_rows:
        robot_values = robotTable.item(row, "values")
        if name == robot_values[0]:
            robotData[robot_values[0]] = dict(zip(robot_columns[1:], robot_values[1:]))
            robot_information = json.dumps(robotData)
            get_robot_result.success = True
            get_robot_result.robot_info = robot_information
            return get_robot_result


# Method to delete a selected robot's information
def delRobot():
    global robotCounter

    selected_item = robotTable.selection()
    if selected_item:
        robotTable.delete(selected_item)
        robotCounter -= 1


# Method to add a cart row
def addCart():
    global cartCounter
    global cartData
    cart_info = [f"BAT_{cartCounter}", "BWS", "False", "unknown"]
    cartTable.insert("", "end", values=cart_info)
    cartData[f"BAT_{cartCounter}"] = dict(zip(cart_columns[1:], cart_info[1:]))
    cartCounter += 1


# Method to delete a selected cart's information
def delCart():
    global cartCounter
    global cartData

    selected_item = cartTable.selection()
    if selected_item:
        values = cartTable.item(selected_item, "values")
        cartTable.delete(selected_item)
        del cartData[values[0]]
        cartCounter -= 1


# Method to set the cart information
def setCartInfo(req):
    global cartData

    cart_data = json.loads(req.cart_info)
    all_rows = cartTable.get_children()
    for row in all_rows:
        original_values = cartTable.item(row, "values")
        updated_values = []
        if cart_data["cart_name"] == original_values[0]:
            for column_name, value in zip(cartTable["columns"], original_values):

                if column_name not in cart_data:
                    updated_values.append(value)
                else:
                    updated_values.append(cart_data[column_name])

            cartTable.item(row, values=updated_values)
            cartData[original_values[0]] = dict(
                zip(cart_columns[1:], updated_values[1:])
            )
            return updateCartInfoResponse(True)

    print("Unable to update cart value")
    return updateCartInfoResponse(False)


# Method to fetch cart information
def getCartData(req):
    cart_information = json.dumps(cartData)

    get_cart_result = readCartInfoResponse()
    get_cart_result.success = True
    get_cart_result.cart_info = cart_information
    return get_cart_result


if __name__ == "__main__":
    rospy.init_node("chargepal_gui")
    get_robotinfo = rospy.Service(
        "chargepal_services/get_robot_info", readRobotInfo, getRobotData
    )
    set_robotinfo = rospy.Service(
        "chargepal_services/set_robot_info", updateRobotInfo, setRobotInfo
    )
    get_cartinfo = rospy.Service(
        "chargepal_services/get_cart_info", readCartInfo, getCartData
    )
    set_cartinfo = rospy.Service(
        "chargepal_services/set_cart_info", updateCartInfo, setCartInfo
    )

    app = tk.Tk()
    app.title("Chargepal GUI")

    # Create the robot section
    robotSection = tk.Frame(app)
    robotSection.pack(padx=15, pady=15)
    robotLabel = tk.Label(robotSection, text="Robots")
    robotLabel.pack()

    addRobotButton = tk.Button(robotSection, text="Add Robot", command=addRobot)
    delRobotButton = tk.Button(robotSection, text="Delete Robot", command=delRobot)
    addRobotButton.pack()
    delRobotButton.pack()
    robotTable = ttk.Treeview(
        robotSection,
        columns=(
            "robot_name",
            "current_job",
            "ongoing_action",
            "previous_action",
            "robot_position",
            "cart_present",
            "charging",
            "battery",
        ),
    )
    robotTable.heading("#1", text="robot_name")
    robotTable.heading("#2", text="current_job")
    robotTable.heading("#3", text="ongoing_action")
    robotTable.heading("#4", text="previous_action")
    robotTable.heading("#5", text="robot_position")
    robotTable.heading("#6", text="cart_present")
    robotTable.heading("#7", text="charging")
    robotTable.heading("#8", text="battery")
    robotTable.pack()
    robot_columns = robotTable["columns"]

    # Create the cart section
    cartSection = tk.Frame(app)
    cartSection.pack(padx=15, pady=15)
    cartLabel = tk.Label(cartSection, text="Carts")
    cartLabel.pack()
    addCartButton = tk.Button(cartSection, text="Add Cart", command=addCart)
    delCartButton = tk.Button(cartSection, text="Delete Cart", command=delCart)
    addCartButton.pack()
    delCartButton.pack()
    cartTable = ttk.Treeview(
        cartSection,
        columns=("cart_name", "cart_position", "plug_connected", "robot_with_cart"),
    )
    cartTable.heading("#1", text="cart_name")
    cartTable.heading("#2", text="cart_position")
    cartTable.heading("#3", text="plug_connected")
    cartTable.heading("#4", text="robot_with_cart")
    cartTable.pack()
    cart_columns = cartTable["columns"]

    style = ttk.Style()
    style.configure("Treeview", rowheight=30)

    # Run the Tkinter main loop
    app.mainloop()
    rospy.spin()
