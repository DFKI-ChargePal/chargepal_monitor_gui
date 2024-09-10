#!/usr/bin/env python3
from nicegui import ui
from env_table_components import create_tables, add_env_table_slots
from environment import save_environment
from database import (
    initialize_database,
    clear_database,
    fetch_table_names,
    fetch_table_data,
)
import config as cfg
from typing import List, Dict
import os


def create_header():
    with ui.header().style(
        "display: flex; flex-direction: column; align-items: center; padding: 10px 10px; background: linear-gradient(180deg, purple, white); color: white; box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);"
    ):
        ui.label("ChargePal Cockpit").style(
            "font-size: 50px; font-weight: bold; margin-bottom: 3px;"
        )
        with ui.row().style(
            "justify-content: space-between; width: 100%; max-width: 1200px;"
        ):
            ui.image(cfg.dfki_logo_path).style("height: 100px;width:120px;")
            ui.image(cfg.chargepal_logo_path).style("height: 70px;width:200px;")


@ui.refreshable
def display_logs(robot_name: str, file_path: str):
    with ui.expansion(
        f"Robot: {robot_name}", value=logs_expansion_states.get(robot_name, False)
    ).classes("w-full") as expansion:
        # Update the global state when the expansion state changes
        expansion.bind_value(logs_expansion_states, robot_name)

        with ui.scroll_area().classes("w-600 h-500"):
            with open(file_path, "r") as file:
                file_contents = file.read()
                ui.label(file_contents).style("white-space: pre-wrap")


@ui.refreshable
def display_table_data(db_path: str, table_name: str):
    columns, rows = fetch_table_data(db_path, table_name)

    with ui.expansion(
        f"Table: {table_name}", value=ldb_expansion_states.get(table_name, False)
    ) as expansion:
        # Update the global state when the expansion state changes
        expansion.bind_value(ldb_expansion_states, table_name)

        ui.table(
            rows=[dict(zip(columns, row)) for row in rows],
            columns=[{"name": col, "label": col, "field": col} for col in columns],
        )


with ui.tabs().classes("w-full") as main_tabs:
    env = ui.tab("SET ENVIRONMENT")
    ldb = ui.tab("LOCAL DATABASE")
    pdb = ui.tab("PLANNER DATABASE")
    logs = ui.tab("LOGS")


with ui.tab_panels(main_tabs, value=env).classes("w-full"):
    with ui.tab_panel(env):
        # Initialize database and clear existing data
        initialize_database()
        clear_database()
        tables = create_tables()
        add_env_table_slots(tables)

        ui.button(
            "Save Environment!",
            on_click=lambda: save_environment(list(tables.values())),
        )

    with ui.tab_panel(ldb):
        ldb_expansion_states: Dict[str, bool] = {}
        table_names = fetch_table_names(cfg.ldb_path)
        with ui.column():
            for table_name in table_names:
                display_table_data(cfg.ldb_path, table_name)

    with ui.tab_panel(pdb):
        pdb_expansion_states: Dict[str, bool] = {}
        table_names = fetch_table_names(cfg.pdb_path)
        with ui.column():
            for table_name in table_names:
                display_table_data(cfg.pdb_path, table_name)

    with ui.tab_panel(logs):
        logs_expansion_states: Dict[str, bool] = {}
        for file in os.listdir(cfg.logs_path):
            basename, extension = os.path.splitext(file)
            file_path = os.path.join(cfg.logs_path, file)
            if os.path.isfile(file_path):
                display_logs(basename, file_path)


create_header()
ui.timer(1.0, lambda: display_table_data.refresh())
ui.timer(1.0, lambda: display_logs.refresh())
ui.run(port=8081)
