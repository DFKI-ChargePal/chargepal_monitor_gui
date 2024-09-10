from pathlib import Path
import toml

monitor_gui_path = Path(__file__).parent

# GUI configuration:
gui_config_file_path = monitor_gui_path / "cfg/gui_config.toml"

# Get chargepal_local_server directories:
try:
    with open(gui_config_file_path, "r") as f:
        gui_config = toml.load(f)
        server_dir = Path.home().joinpath(gui_config["server"]["server_dir"])
except FileNotFoundError as e:
    raise e
logs_path = server_dir / "src/chargepal_local_server/logs"
databases_path = server_dir / "src/chargepal_local_server/db"
ldb_path = databases_path / "ldb.db"
pdb_path = databases_path / "pdb.db"

envi_toml_file_path = monitor_gui_path / "cfg/envi.toml"
dfki_logo_path = monitor_gui_path / "img/dfki_logo.png"
chargepal_logo_path = monitor_gui_path / "img/chargepal_logo.svg"