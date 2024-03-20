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

                error_count = util.fetch_error_count()
                avg_loop_runtime = util.fetch_avg_loop_runtime()
                loop_count = util.fetch_loop_count()

                grid = ui.aggrid(
                    {
                        "defaultColDef": {"flex": 1},
                        "columnDefs": [
                            {"headerName": "Variable", "field": "variable"},
                            {"headerName": "Value", "field": "value"},
                        ],
                        "rowData": [
                            {
                                "variable": "Error count",
                                "value": error_count,
                            },
                            {
                                "variable": "Average loop runtime",
                                "value": avg_loop_runtime,
                            },
                            {
                                "variable": "Number of loops",
                                "value": loop_count,
                            },
                        ],
                    }
                ).classes("max-h-40")
                ui.button("Update", on_click=lambda: send_request("update_statistics"))
            with ui.tab_panel(two):
                with ui.row():
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

                            with ui.stepper().props("vertical").classes(
                                "w-full"
                            ) as stepper:
                                with ui.step("Free the Arm"):
                                    ui.label(
                                        "To free the arm, and move the arm out of failure position"
                                    )
                                    ui.button(
                                        "Free Arm",
                                        on_click=lambda: send_request("free_arm"),
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
                                        ui.button("Next", on_click=stepper.next)
                                        ui.button(
                                            "Back", on_click=stepper.previous
                                        ).props("flat")
                                with ui.step("Restart the Node"):
                                    ui.label("Restarts the manipulation node")
                                    ui.button(
                                        "Restart Node",
                                        on_click=lambda: send_request(
                                            "restart_arm_node"
                                        ),
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
