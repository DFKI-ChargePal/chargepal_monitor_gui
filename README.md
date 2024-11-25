# chargepal_monitor_gui

The chargepal_monitor_gui allows users to configure ChargePal environment information and monitor databases on the server.

**Section: Set Environment**

The application interacts with the `ldb.db` database, specifically updating the `env_info`, `robot_info` and `cart_info` tables.

The GUI is divided into the two sections *Robot* and *Cart* for adding or updating respective entries.

**Section: Local Database**

Displays the local database `ldb.db`.

**Section: Planner Database**

Displays the planning database `pdb.db`.

## Installation

The GUI package is managed by [`uv`](https://docs.astral.sh/uv/). Therefore, clone the repository and execute `uv sync` for installing all necessary dependencies into a virtual environment.

## Execution

1. Activate the virtual environment.
2. Inside `chargepal_monitor_gui/chargepal_monitor_gui` run `python run_gui.py`.
3. Open http://localhost:8089 (or `http://<server-ip>:8089` if the GUI runs on a remote server) to view the GUI. The serving port can be configured in [`gui_config.toml`](./chargepal_monitor_gui/cfg/gui_config.toml).

## Dependency

Besides the dependencies defined in the [`pyproject.toml`](./pyproject.toml), the GUI needs [`chargepal_local_server`](https://github.com/DFKI-ChargePal/chargepal_local_server) with its `ldb.db`, `pdb.db` and robot logs. Configure the path of `chargepal_local_server` in the [`gui_config.toml`](./chargepal_monitor_gui/cfg/gui_config.toml).
