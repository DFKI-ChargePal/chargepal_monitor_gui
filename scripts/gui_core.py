#!/usr/bin/env python3
import rospy
from nicegui import ui
from nicegui.events import ValueChangeEventArguments
import asyncio
import gui_util as util
from concurrent.futures import ThreadPoolExecutor
import time


class Chargepal_GUI_Core:

    def run_gui(self):

        def show(event: ValueChangeEventArguments):
            name = type(event.sender).__name__
            ui.notify(f"{name}: {event.value}")

        def send_request(request_name):
            success = False
            n = ui.notification(timeout=None)

            n.type = "ongoing"

            while True:
                n.message = f"Executing"
                n.spinner = True
                success = util.execute_request(request_name)
                if success:
                    break

            n.message = "Done!"
            n.spinner = False
            n.dismiss()

        with ui.tabs().classes("w-full") as tabs:
            one = ui.tab("STATISTICS")
            two = ui.tab("ROBOT CONTROL")
        with ui.tab_panels(tabs, value=two).classes("w-full"):
            with ui.tab_panel(one):
                with ui.row():
                    ec_aas = ui.label()
                    ui.timer(
                        1.0,
                        lambda: ec_aas.set_text(
                            f"Error count arrive_at_station : {util.fetch_ec_aas()}"
                        ),
                    )
                    ui.separator()
                    ec_gh = ui.label()
                    ui.timer(
                        1.0,
                        lambda: ec_gh.set_text(
                            f"Error count go_home : {util.fetch_ec_gh()}"
                        ),
                    )
                    ui.separator()
                    ec_pic = ui.label()
                    ui.timer(
                        1.0,
                        lambda: ec_pic.set_text(
                            f"Error count pickup_cart : {util.fetch_ec_pic()}"
                        ),
                    )
                    ui.separator()
                    ec_plc = ui.label()
                    ui.timer(
                        1.0,
                        lambda: ec_plc.set_text(
                            f"Error count place_cart : {util.fetch_ec_plc()}"
                        ),
                    )
                    ui.separator()
                    ec_piads = ui.label()
                    ui.timer(
                        1.0,
                        lambda: ec_piads.set_text(
                            f"Error count plugin_ads : {util.fetch_ec_piads()}"
                        ),
                    )
                    ui.separator()
                    ec_poads = ui.label()
                    ui.timer(
                        1.0,
                        lambda: ec_poads.set_text(
                            f"Error count plugout_ads : {util.fetch_ec_poads()}"
                        ),
                    )
                    ui.separator()
                    total_loop = ui.label()
                    ui.timer(
                        1.0,
                        lambda: total_loop.set_text(
                            f"Total loop count: {util.fetch_total_loop()}"
                        ),
                    )
                    ui.separator()
                    avg_loop_time = ui.label()
                    ui.timer(
                        1.0,
                        lambda: avg_loop_time.set_text(
                            f"Average loop time: {util.fetch_avg_loop_time()} minutes"
                        ),
                    )

            with ui.tab_panel(two):
                with ui.row():

                    loop_label = ui.label()
                    ui.timer(
                        1.0,
                        lambda: loop_label.set_text(
                            f"Loop : {util.fetch_current_loop()} remaining!"
                        ),
                    )
                    ui.separator()
                    job_label = ui.label()
                    ui.timer(
                        1.0,
                        lambda: job_label.set_text(
                            f"Job name : {util.fetch_job_name()}"
                        ),
                    )
                    ui.separator()
                    status_label = ui.label()
                    ui.timer(
                        1.0,
                        lambda: status_label.set_text(
                            f"Status : {util.fetch_ongoing_action()}"
                        ),
                    )
                    ui.separator()
                    ui.label("Battery: ")
                    with ui.circular_progress(value=0.1, show_value=True) as progress:
                        ui.timer(
                            1.0, lambda: progress.set_value(util.fetch_battery_level())
                        )

                ui.separator()
                with ui.expansion("Demo Control", icon="work").classes("w-full"):

                    with ui.row():
                        ui.number(
                            label="Enter number of demo loops",
                            value=0,
                            format="%.2f",
                            on_change=lambda e: util.set_loop_count(e.value),
                        )
                    with ui.row():
                        with ui.row():
                            ui.button(
                                "Start",
                                on_click=lambda: send_request("click_start"),
                            )
                            ui.button(
                                "Stop",
                                on_click=lambda: send_request("click_stop"),
                            )
                            ui.button(
                                "Resume",
                                on_click=lambda: send_request("click_resume"),
                            )

                        log = ui.log(max_lines=1).classes("w-full h-30")
                        ui.timer(1.0, lambda: util.publish_log(log))

                with ui.expansion("Features", icon="star").classes("w-full"):
                    with ui.column():
                        with ui.dialog() as dialog_recover_arm, ui.card():
                            recover_arm_status_label = ui.label()
                            ui.timer(
                                1.0,
                                lambda: recover_arm_status_label.set_text(
                                    f"Status : {util.fetch_ongoing_action()}"
                                ),
                            )
                            with ui.stepper().props("vertical").classes(
                                "w-full"
                            ) as stepper:
                                with ui.step("Restart the Node"):
                                    ui.label("Restarts the manipulation node")
                                    ui.button(
                                        "Restart Node",
                                        on_click=lambda: send_request(
                                            "restart_arm_node"
                                        ),
                                    )

                                    ui.button("Next", on_click=stepper.next)
                                with ui.step("Free the Arm"):
                                    ui.label(
                                        "To free the arm, and move the arm out of failure position"
                                    )
                                    ui.button(
                                        "Free Arm",
                                        on_click=lambda: send_request("free_arm"),
                                    )
                                    ui.button(
                                        "Stop Free Arm",
                                        on_click=lambda: send_request("stop_free_arm"),
                                    )
                                    with ui.stepper_navigation():
                                        ui.button("Next", on_click=stepper.next)
                                with ui.step("Move to Home"):
                                    ui.label("The arm moves to the home position")
                                    ui.button(
                                        "Move Home",
                                        on_click=lambda: send_request("move_arm_home"),
                                    )
                                    with ui.stepper_navigation():
                                        ui.button(
                                            "Done",
                                            on_click=lambda: ui.notify(
                                                "Yay!", type="positive"
                                            ),
                                        )
                                        ui.button(
                                            "Back", on_click=stepper.previous
                                        ).props("flat")

                            ui.button("Close", on_click=dialog_recover_arm.close)

                        with ui.dialog() as dialog_shutdown_orin, ui.card():
                            ui.button(
                                "Confirm Shutdown",
                                on_click=lambda: util.shutdown_orin(),
                            )

                        ui.button("RECOVER ARM", on_click=dialog_recover_arm.open)
                        ui.button("SHUTDOWN ORIN", on_click=dialog_shutdown_orin.open)

        ui.run()


if __name__ in {"__main__", "__mp_main__"}:
    rospy.init_node("chargepal_gui")
    try:
        core = Chargepal_GUI_Core()
        core.run_gui()
    except rospy.ROSInterruptException:
        pass
